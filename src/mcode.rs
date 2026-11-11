//! MCode over TCP (port 503) client for Novanta IMS LMD motors.
//!
//! Each command is ASCII text ended by CR. `connect` sets EM=2 (RAM only), since the motor may
//! have been left in another echo mode. With Echo Mode EM=2 the motor sends no
//! echo and no prompt: only `PR` (print) produces output, one line ended by CR LF. A rejected
//! command produces no output at all; it sets EF=1 and the error code in ER (reading ER clears EF).
//! Measured on the LMDCE571 (MCode 5.003): `PR P," ",V` -> "0 0\r\n", unknown command -> ER 30.
//!
//! Writes go through `command`/`set`, which follow each command with `PR EF` to see whether it
//! was accepted. Verified on the motor: `VI=1000` accepted and read back, `XYZ` -> ER 30.

use core::net::Ipv4Addr;

use defmt::Format;
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_time::{with_timeout, Duration};

pub const PORT: u16 = 503;
const LINE_LEN: usize = 128;
const REPLY_TIMEOUT: Duration = Duration::from_secs(1);

#[derive(Debug, Clone, Copy, Format)]
pub enum Error {
    Connect(embassy_net::tcp::ConnectError),
    /// TCP reset, timeout, or peer closed the connection.
    Io,
    /// No reply line within REPLY_TIMEOUT (the motor rejected the command, or the link is slow).
    NoReply,
    /// Reply too long, not ASCII, or not the expected number(s).
    Parse,
    /// The motor rejected a command: EF was set, this is the code read from ER
    /// (MCode manual error table, e.g. 30 = unknown command).
    Rejected(i32),
    /// A `set` was accepted but reading the variable back gave a different value.
    Verify { wrote: i32, read: i32 },
}

impl From<embassy_net::tcp::Error> for Error {
    fn from(_: embassy_net::tcp::Error) -> Self {
        Error::Io
    }
}

pub struct Client<'a> {
    sock: TcpSocket<'a>,
    line: [u8; LINE_LEN],
}

impl<'a> Client<'a> {
    pub async fn connect(stack: Stack<'a>, ip: Ipv4Addr, rx: &'a mut [u8], tx: &'a mut [u8]) -> Result<Self, Error> {
        let mut sock = TcpSocket::new(stack, rx, tx);
        // Drop the connection if the motor goes silent (the poll loop talks to it every 2 s).
        sock.set_timeout(Some(Duration::from_secs(10)));
        sock.connect((ip, PORT)).await.map_err(Error::Connect)?;
        let mut c = Client { sock, line: [0; LINE_LEN] };
        // Everything below relies on EM=2 (no echo, no prompt), but the motor may be set to
        // another mode (e.g. EM=0 echoes each command and prints a '>' prompt, which LMD Suite
        // leaves behind). Force EM=2 in RAM, then throw away the banner/echo/prompt it sent.
        c.send("").await?; // end any partial line
        c.send("EM=2").await?;
        c.drain().await?;
        Ok(c)
    }

    /// Discards incoming bytes until the motor has been silent for a short while.
    async fn drain(&mut self) -> Result<(), Error> {
        const QUIET: Duration = Duration::from_millis(200);
        loop {
            match with_timeout(QUIET, self.sock.read(&mut self.line)).await {
                Err(_) => return Ok(()), // quiet: done
                Ok(Ok(0)) => return Err(Error::Io),
                Ok(Ok(_)) => {}
                Ok(Err(e)) => return Err(e.into()),
            }
        }
    }

    pub async fn close(mut self) {
        self.sock.close();
        let _ = self.sock.flush().await;
    }

    async fn send(&mut self, cmd: &str) -> Result<(), Error> {
        let mut data = cmd.as_bytes();
        while !data.is_empty() {
            let n = self.sock.write(data).await?;
            data = &data[n..];
        }
        self.sock.write(b"\r").await?;
        self.sock.flush().await?;
        Ok(())
    }

    /// Sends `cmd` (normally a `PR ...`) and returns the reply line without CR LF.
    /// After `NoReply` the stream may be out of step; the caller should reconnect.
    pub async fn query(&mut self, cmd: &str) -> Result<&str, Error> {
        self.send(cmd).await?;
        let mut n = 0;
        let len = with_timeout(REPLY_TIMEOUT, async {
            loop {
                if n == LINE_LEN {
                    return Err(Error::Parse);
                }
                let got = self.sock.read(&mut self.line[n..]).await?;
                if got == 0 {
                    return Err(Error::Io);
                }
                n += got;
                if let Some(end) = self.line[..n].iter().position(|&b| b == b'\n') {
                    return Ok(end);
                }
            }
        })
        .await
        .map_err(|_| Error::NoReply)??;
        let s = core::str::from_utf8(&self.line[..len]).map_err(|_| Error::Parse)?;
        Ok(s.trim_end_matches('\r'))
    }

    /// Sends a command that prints nothing (e.g. `VM=50000`, `MR 200`), then reads EF to find
    /// out whether the motor accepted it. EF also covers earlier errors, so a stale flag makes
    /// this fail too; reading ER clears it either way.
    pub async fn command(&mut self, cmd: &str) -> Result<(), Error> {
        self.send(cmd).await?;
        if self.get("EF").await? != 0 {
            return Err(Error::Rejected(self.get("ER").await?));
        }
        Ok(())
    }

    /// `command` with one integer argument, e.g. `command_arg("MR ", 200)` sends `MR 200`.
    pub async fn command_arg(&mut self, cmd: &str, arg: i32) -> Result<(), Error> {
        let mut num = [0u8; 12];
        let num = fmt_i32(&mut num, arg);
        let mut buf = [0u8; 32];
        let line = concat(&mut buf, &[cmd, num])?;
        self.command(line).await
    }

    /// Sets an integer variable (`VAR=value`), checks it was accepted and reads it back.
    /// Settings live in RAM only; nothing here sends `S` (save to non-volatile memory).
    pub async fn set(&mut self, var: &str, value: i32) -> Result<(), Error> {
        let mut cmd = [0u8; 16];
        let cmd = concat(&mut cmd, &[var, "="])?;
        self.command_arg(cmd, value).await?;
        let read = self.get(var).await?;
        if read != value {
            return Err(Error::Verify { wrote: value, read });
        }
        Ok(())
    }

    /// Reads one integer variable, e.g. `get("P")` sends `PR P`.
    pub async fn get(&mut self, var: &str) -> Result<i32, Error> {
        let mut cmd = [0u8; 16];
        let cmd = concat(&mut cmd, &["PR ", var])?;
        let line = self.query(cmd).await?;
        line.trim().parse().map_err(|_| Error::Parse)
    }

    /// Reads several integer variables in one exchange (`PR A," ",B,...`).
    pub async fn get_many<const N: usize>(&mut self, vars: [&str; N]) -> Result<[i32; N], Error> {
        let mut cmd = [0u8; LINE_LEN];
        let mut n = copy(&mut cmd, 0, "PR ")?;
        for (i, v) in vars.iter().enumerate() {
            if i > 0 {
                n = copy(&mut cmd, n, ",\" \",")?;
            }
            n = copy(&mut cmd, n, v)?;
        }
        let cmd = core::str::from_utf8(&cmd[..n]).map_err(|_| Error::Parse)?;
        let line = self.query(cmd).await?;
        let mut out = [0i32; N];
        let mut it = line.split_ascii_whitespace();
        for o in out.iter_mut() {
            *o = it.next().and_then(|t| t.parse().ok()).ok_or(Error::Parse)?;
        }
        Ok(out)
    }
}

fn copy(buf: &mut [u8], at: usize, s: &str) -> Result<usize, Error> {
    let end = at + s.len();
    buf.get_mut(at..end).ok_or(Error::Parse)?.copy_from_slice(s.as_bytes());
    Ok(end)
}

/// Formats `v` in decimal into the end of `buf` and returns the text.
fn fmt_i32(buf: &mut [u8; 12], v: i32) -> &str {
    let mut u = v.unsigned_abs();
    let mut i = buf.len();
    loop {
        i -= 1;
        buf[i] = b'0' + (u % 10) as u8;
        u /= 10;
        if u == 0 {
            break;
        }
    }
    if v < 0 {
        i -= 1;
        buf[i] = b'-';
    }
    // Only ASCII digits and '-' were written, so this cannot fail.
    core::str::from_utf8(&buf[i..]).unwrap_or("")
}

fn concat<'b>(buf: &'b mut [u8], parts: &[&str]) -> Result<&'b str, Error> {
    let mut n = 0;
    for p in parts {
        n = copy(buf, n, p)?;
    }
    core::str::from_utf8(&buf[..n]).map_err(|_| Error::Parse)
}
