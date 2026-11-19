//! TI ADS1256 24-bit delta-sigma ADC (doc/ads1256.pdf) on SPI2: samples the ABP transducer.
//!
//! Wiring on the STM32746G-EVAL extension connectors. Each pin only goes to a connector that
//! is normally empty, so no board rework is needed as long as those stay empty:
//!
//! | ADS1256      | MCU  | Connector | Shared with (keep empty)       |
//! |--------------|------|-----------|--------------------------------|
//! | SCLK         | PD3  | CN6-23    | camera CN4                     |
//! | DIN          | PB15 | CN5-38    | USB FS2 CN14 (no cable)        |
//! | DOUT         | PB14 | CN5-35    | USB FS2 CN14 (no cable)        |
//! | CS           | PA4  | CN5-21    | camera CN4                     |
//! | DRDY         | PC11 | CN6-53    | microSD (no card)              |
//! | PDWN, RESET  | -    | +3V3      | tie high on the module         |
//! | 5V / GND     | -    | CN5-60 / CN5-59 |                          |
//!
//! Assumes the usual breakout module: 7.68 MHz crystal, 2.5 V reference, AINCOM tied to GND.
//! Full scale is +-5 V / gain; with the input buffer on, inputs must stay within 0..AVDD-2 V.
//!
//! Two modes, switched at run time from the ADC tab (`set_mux`):
//! - MUX OFF (default): only the transducer bridge, AIN0 (+) vs AIN1 (-), gain 64, converting
//!   continuously with the 1000 SPS filter. The registers are written once at start.
//! - MUX ON: one ADC behind the input multiplexer, so the channels take turns. A 1 kHz ticker
//!   starts every slot with SYNC + WAKEUP on AIN0-AIN1 (gain 64, 3750 SPS filter), so pressure
//!   samples are exactly 1 ms apart. In every 10th slot AIN0 vs AINCOM is converted right after
//!   it, and 5 slots later AIN1 vs AINCOM (gain 1, 15000 SPS filter): 100 SPS each.
//!   After a multiplexer change the ADS1256 settles in one conversion (datasheet "Settling Time
//!   Using the Input Multiplexer"): t18 = 0.44 ms at 3750 SPS and 0.25 ms at 15000 SPS
//!   (table 13), so both fit in one slot. The next channel is selected before the previous
//!   result is read (figure 19).
//!
//! Every register write is read back and repeated on mismatch: on the bench wiring ~8 % of
//! writes arrived with one bit doubled (SCLK glitch), e.g. DRATE 0xC0 -> 0x60, which stalls
//! the conversion. `View::write_retries` counts these.
//!
//! The task runs on the high-priority interrupt executor so the ~20 ms frame draws on the main
//! executor don't delay sampling.

use core::cell::Cell;
use core::convert::Infallible;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};

use defmt::{info, warn, Format};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::spi::{self, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{with_timeout, Duration, Instant, Ticker, Timer};

/// Crystal on the module. SCLK must stay below CLKIN / 4 = 1.92 MHz.
const CLKIN_HZ: u32 = 7_680_000;
pub const SPI_HZ: u32 = 1_800_000; // SPI2 on APB1 (54 MHz) / 32 = 1.69 MHz
/// Reference voltage in microvolts; full scale is +-2 * VREF / gain.
const VREF_UV: i64 = 2_500_000;

// Commands (datasheet table 24)
const CMD_RDATA: u8 = 0x01;
const CMD_SDATAC: u8 = 0x0F;
const CMD_RREG: u8 = 0x10;
const CMD_WREG: u8 = 0x50;
const CMD_SELFCAL: u8 = 0xF0;
const CMD_SYNC: u8 = 0xFC;
const CMD_WAKEUP: u8 = 0x00;
const CMD_RESET: u8 = 0xFE;

// Registers
const REG_STATUS: u8 = 0x00;
/// MUX, ADCON, DRATE are consecutive, so one WREG switches channel, gain and filter together.
const REG_MUX: u8 = 0x01;
/// Input buffer on, auto-calibration off, MSB first.
const STATUS_BUFEN: u8 = 1 << 1;
/// STATUS bits we write (ORDER, ACAL, BUFEN); bit 0 is DRDY and bits 7:4 the ID.
const STATUS_RW_MASK: u8 = 0x0E;
/// ID in STATUS[7:4] for the ADS1256.
const ID_ADS1256: u8 = 3;
/// Register write attempts before giving up.
const WRITE_TRIES: u32 = 4;

const AINCOM: u8 = 8;

/// One input setting: the MUX, ADCON and DRATE register values.
struct Channel {
    mux: u8,
    /// PGA code in ADCON[2:0]; CLKOUT and sensor detect are off.
    adcon: u8,
    drate: u8,
    gain: u16,
}

/// Transducer bridge, AIN0 (+) vs AIN1 (-), gain 64 (+-78 mV): continuous, 1000 SPS filter.
const DIFF_CONT: Channel = Channel { mux: 0 << 4 | 1, adcon: 6, drate: 0xA1, gain: 64 };
/// The same in MUX ON slots: 3750 SPS filter so a second conversion fits in the slot.
const DIFF_SLOT: Channel = Channel { mux: 0 << 4 | 1, adcon: 6, drate: 0xC0, gain: 64 };
/// Single-ended AIN0 / AIN1 vs AINCOM (GND): gain 1, 15000 SPS filter.
const AIN0_SE: Channel = Channel { mux: 0 << 4 | AINCOM, adcon: 0, drate: 0xE0, gain: 1 };
const AIN1_SE: Channel = Channel { mux: 1 << 4 | AINCOM, adcon: 0, drate: 0xE0, gain: 1 };

/// Differential (pressure) sample rate in both modes.
pub const DIFF_SPS: u32 = 1000;
/// MUX ON: each single-ended input is converted once every this many slots.
const SE_EVERY: u32 = 10;
/// Microvolts per code of the differential channel (gain 64 in both modes).
pub const DIFF_UV_PER_CODE: f32 = 2.0 * VREF_UV as f32 / (DIFF_CONT.gain as f32 * (1u32 << 23) as f32);

/// Channel indices in `View`.
pub const CH_DIFF: usize = 0;
pub const CH_AIN0: usize = 1;
pub const CH_AIN1: usize = 2;

/// For display, per channel index: inputs, gain, scheduled samples/s (0 = not sampled),
/// filter setting in SPS.
pub fn channel_info(mux: bool) -> [(&'static str, u16, u32, u32); 3] {
    if mux {
        [
            ("AIN0-AIN1", DIFF_SLOT.gain, DIFF_SPS, 3750),
            ("AIN0-AINCOM", AIN0_SE.gain, DIFF_SPS / SE_EVERY, 15000),
            ("AIN1-AINCOM", AIN1_SE.gain, DIFF_SPS / SE_EVERY, 15000),
        ]
    } else {
        [("AIN0-AIN1", DIFF_CONT.gain, DIFF_SPS, 1000), ("AIN0-AINCOM", 0, 0, 0), ("AIN1-AINCOM", 0, 0, 0)]
    }
}

static MUX_ON: AtomicBool = AtomicBool::new(false);

/// Selects MUX ON (true) or OFF; the ADC task restarts sampling in the new mode.
pub fn set_mux(on: bool) {
    MUX_ON.store(on, Ordering::Relaxed);
}

/// Mode requested with `set_mux` (`View::mux` is the mode in effect).
pub fn mux() -> bool {
    MUX_ON.load(Ordering::Relaxed)
}

#[derive(Clone, Copy, PartialEq, Format)]
pub enum Status {
    /// Not probed yet.
    Starting,
    /// STATUS did not show the ADS1256 ID (nothing wired, or wrong wiring).
    NotFound,
    /// The chip answered but DRDY never fell.
    NoDrdy,
    /// Register writes kept reading back wrong.
    WriteFail,
    Running,
}

#[derive(Clone, Copy)]
pub struct View {
    pub status: Status,
    /// Mode in effect: MUX ON (true) or OFF.
    pub mux: bool,
    /// Last result per channel (CH_*), in microvolts.
    pub uv: [i32; 3],
    /// Conversions read per second per channel, measured.
    pub sps: [u32; 3],
    /// MUX ON: longest slot in the last second, and slots that overran 1 ms since start.
    pub slot_us_max: u32,
    pub late: u32,
    /// Register writes that read back wrong and were repeated, since boot.
    pub write_retries: u32,
}

static VIEW: Mutex<CriticalSectionRawMutex, Cell<View>> = Mutex::new(Cell::new(View {
    status: Status::Starting,
    mux: false,
    uv: [0; 3],
    sps: [0; 3],
    slot_us_max: 0,
    late: 0,
    write_retries: 0,
}));

pub fn view() -> View {
    VIEW.lock(|v| v.get())
}

fn update_view(f: impl FnOnce(&mut View)) {
    VIEW.lock(|c| {
        let mut v = c.get();
        f(&mut v);
        c.set(v);
    });
}

/// Differential samples (raw codes), 8.2 s at 1000 SPS. Single writer (the ADC task), any
/// number of readers; a reader must stay well within RING_LEN of `head()`.
pub const RING_LEN: usize = 8192;
static RING: [AtomicI32; RING_LEN] = [const { AtomicI32::new(0) }; RING_LEN];
/// Number of samples written so far; sample `i` lives at RING[i % RING_LEN].
static HEAD: AtomicU32 = AtomicU32::new(0);

/// Samples written so far; the newest is `head() - 1`.
pub fn head() -> u32 {
    HEAD.load(Ordering::Acquire)
}

/// Differential sample `i` (raw code), for `i` in `head() - RING_LEN + margin .. head()`.
pub fn sample(i: u32) -> i32 {
    RING[i as usize % RING_LEN].load(Ordering::Relaxed)
}

fn push(code: i32) {
    let h = HEAD.load(Ordering::Relaxed);
    RING[h as usize % RING_LEN].store(code, Ordering::Relaxed);
    HEAD.store(h.wrapping_add(1), Ordering::Release);
}

/// Converts a signed 24-bit result to microvolts for the given gain.
pub fn to_uv(code: i32, gain: u16) -> i32 {
    (code as i64 * 2 * VREF_UV / (gain as i64 * (1 << 23))) as i32
}

#[derive(Clone, Copy, Format)]
enum Error {
    Spi,
    NotFound { status: u8 },
    DrdyTimeout,
    WriteVerify { reg: u8 },
    /// `set_mux` asked for the other mode.
    ModeChange,
}

impl From<spi::Error> for Error {
    fn from(_: spi::Error) -> Self {
        Error::Spi
    }
}

/// Busy-waits a number of CLKIN periods (the datasheet gives command timings in tCLKIN).
fn wait_clkin(n: u32) {
    // 216 MHz core: ceil(216e6 / CLKIN) = 29 cycles per tCLKIN.
    cortex_m::asm::delay(n * 216_000_000_u32.div_ceil(CLKIN_HZ));
}

pub struct Ads1256 {
    spi: Spi<'static, Blocking, spi::mode::Master>,
    cs: Output<'static>,
    drdy: ExtiInput<'static, Async>,
    /// Register writes repeated after a bad read-back.
    retries: u32,
}

impl Ads1256 {
    pub fn new(spi: Spi<'static, Blocking, spi::mode::Master>, cs: Output<'static>, drdy: ExtiInput<'static, Async>) -> Self {
        Ads1256 { spi, cs, drdy, retries: 0 }
    }

    /// SPI settings for the ADS1256: mode 1 (CPOL 0, CPHA 1), MSB first, below CLKIN / 4.
    pub fn spi_config() -> spi::Config {
        let mut c = spi::Config::default();
        c.mode = spi::MODE_1;
        c.frequency = embassy_stm32::time::Hertz(SPI_HZ);
        // Reads 0x00 instead of noise when nothing is connected, so probing fails cleanly.
        c.miso_pull = embassy_stm32::gpio::Pull::Down;
        // Slowest edges: the module hangs on jumper wires.
        c.gpio_speed = embassy_stm32::gpio::Speed::Low;
        c
    }

    /// Runs `f` with CS low; keeps CS low for t10 (8 tCLKIN) after the last clock.
    fn select<R>(&mut self, f: impl FnOnce(&mut Spi<'static, Blocking, spi::mode::Master>) -> Result<R, spi::Error>) -> Result<R, Error> {
        self.cs.set_low();
        let r = f(&mut self.spi);
        wait_clkin(8);
        self.cs.set_high();
        wait_clkin(4); // t11 before the next command
        Ok(r?)
    }

    fn command(&mut self, cmd: u8) -> Result<(), Error> {
        self.select(|s| s.blocking_write(&[cmd]))?;
        if cmd == CMD_SYNC {
            wait_clkin(24); // t11 for SYNC
        }
        Ok(())
    }

    fn write_regs(&mut self, first: u8, data: &[u8]) -> Result<(), Error> {
        self.select(|s| {
            s.blocking_write(&[CMD_WREG | first, data.len() as u8 - 1])?;
            s.blocking_write(data)
        })
    }

    fn read_regs(&mut self, first: u8, out: &mut [u8]) -> Result<(), Error> {
        self.select(|s| {
            s.blocking_write(&[CMD_RREG | first, out.len() as u8 - 1])?;
            wait_clkin(50); // t6
            s.blocking_read(out)
        })
    }

    /// Writes consecutive registers and reads them back (bits in `mask`), repeating on a
    /// mismatch.
    fn write_verified<const N: usize>(&mut self, first: u8, data: [u8; N], mask: u8) -> Result<(), Error> {
        for _ in 0..WRITE_TRIES {
            self.write_regs(first, &data)?;
            let mut got = [0u8; N];
            self.read_regs(first, &mut got)?;
            if got.iter().zip(&data).all(|(g, w)| g & mask == w & mask) {
                return Ok(());
            }
            self.retries += 1;
        }
        Err(Error::WriteVerify { reg: first })
    }

    fn read_data(&mut self) -> Result<i32, Error> {
        let mut b = [0u8; 3];
        self.select(|s| {
            s.blocking_write(&[CMD_RDATA])?;
            wait_clkin(50); // t6
            s.blocking_read(&mut b)
        })?;
        // Sign-extend 24 -> 32 bits.
        Ok(i32::from_be_bytes([b[0], b[1], b[2], 0]) >> 8)
    }

    fn set_channel(&mut self, ch: &Channel) -> Result<(), Error> {
        self.write_verified(REG_MUX, [ch.mux, ch.adcon, ch.drate], 0xFF)
    }

    /// Restarts the digital filter: the conversion starts now and settles in one cycle.
    fn start(&mut self) -> Result<(), Error> {
        self.command(CMD_SYNC)?;
        self.command(CMD_WAKEUP)
    }

    /// Waits for the next conversion result (falling edge of DRDY).
    async fn wait_conversion(&mut self, timeout: Duration) -> Result<(), Error> {
        with_timeout(timeout, self.drdy.wait_for_falling_edge()).await.map_err(|_| Error::DrdyTimeout)
    }

    /// Resets the chip and checks its ID. Returns the STATUS register.
    async fn probe(&mut self) -> Result<u8, Error> {
        self.command(CMD_RESET)?;
        Timer::after_millis(5).await;
        self.command(CMD_SDATAC)?;
        let mut status = [0u8];
        self.read_regs(REG_STATUS, &mut status)?;
        if status[0] >> 4 != ID_ADS1256 {
            return Err(Error::NotFound { status: status[0] });
        }
        Ok(status[0])
    }

    /// Buffer on, `ch` selected, self-calibration at its gain and filter. In MUX ON the
    /// single-ended channels (gain 1) reuse these offset/gain corrections: the residual error
    /// is far below what they are used for (checking the bridge voltages).
    async fn calibrate(&mut self, ch: &Channel) -> Result<(), Error> {
        self.write_verified(REG_STATUS, [STATUS_BUFEN], STATUS_RW_MASK)?;
        self.set_channel(ch)?;
        self.command(CMD_SELFCAL)?;
        // DRDY goes high while calibrating and low when done.
        let _ = with_timeout(Duration::from_millis(1), self.drdy.wait_for_high()).await;
        with_timeout(Duration::from_millis(500), self.drdy.wait_for_low())
            .await
            .map_err(|_| Error::DrdyTimeout)
    }
}

/// Finds the ADS1256 and samples in the selected mode. Retries every 2 s on errors.
#[embassy_executor::task]
pub async fn task(mut adc: Ads1256) {
    loop {
        let mux = mux();
        let Err(e) = run(&mut adc, mux).await;
        let status = match e {
            Error::ModeChange => {
                info!("ADS1256: switching to MUX {}", if mux { "OFF" } else { "ON" });
                update_view(|v| {
                    v.uv = [0; 3];
                    v.sps = [0; 3];
                });
                continue;
            }
            Error::NotFound { status } => {
                warn!("ADS1256 not found (STATUS {:#04x}); check wiring on SPI2", status);
                Status::NotFound
            }
            Error::DrdyTimeout => {
                warn!("ADS1256: DRDY did not fall (PC11)");
                Status::NoDrdy
            }
            Error::WriteVerify { reg } => {
                warn!("ADS1256: register {} reads back wrong after {} writes; check SCLK wiring", reg, WRITE_TRIES);
                Status::WriteFail
            }
            Error::Spi => {
                warn!("ADS1256: SPI error");
                Status::NotFound
            }
        };
        update_view(|v| {
            v.status = status;
            v.sps = [0; 3];
            v.write_retries = adc.retries;
        });
        Timer::after_secs(2).await;
    }
}

async fn run(adc: &mut Ads1256, mux: bool) -> Result<Infallible, Error> {
    let status = adc.probe().await?;
    info!("ADS1256 found, STATUS {:#04x}", status);
    adc.calibrate(if mux { &DIFF_SLOT } else { &DIFF_CONT }).await?;
    if mux {
        info!("ADS1256 MUX ON: AIN0-AIN1 gain 64 at {} SPS, AIN0/AIN1-AINCOM gain 1 at {} SPS", DIFF_SPS, DIFF_SPS / SE_EVERY);
    } else {
        info!("ADS1256 MUX OFF: AIN0-AIN1 gain 64, continuous at {} SPS", DIFF_SPS);
    }
    update_view(|v| {
        v.status = Status::Running;
        v.mux = mux;
        v.uv = [0; 3];
        v.slot_us_max = 0;
        v.late = 0;
    });
    if mux { run_slots(adc).await } else { run_continuous(adc).await }
}

/// Per-second rates for the view, and a log line every 5 s.
struct Stats {
    window: Instant,
    count: [u32; 3],
    slot_us_max: u32,
    late: u32,
    last_log: Instant,
}

impl Stats {
    fn new() -> Self {
        Stats { window: Instant::now(), count: [0; 3], slot_us_max: 0, late: 0, last_log: Instant::now() }
    }

    fn tick(&mut self, retries: u32, mux: bool) {
        let ms = self.window.elapsed().as_millis() as u32;
        if ms < 1000 {
            return;
        }
        let sps = self.count.map(|c| (c * 1000 + ms / 2) / ms);
        let (slot_max, late) = (self.slot_us_max, self.late);
        update_view(|v| {
            v.sps = sps;
            v.slot_us_max = slot_max;
            v.late = late;
            v.write_retries = retries;
        });
        if self.last_log.elapsed() >= Duration::from_secs(5) {
            let v = view();
            if mux {
                info!(
                    "ADS1256: diff {} uV ({} SPS), AIN0 {} uV, AIN1 {} uV ({}/{} SPS), slot max {} us, late {}, write retries {}",
                    v.uv[CH_DIFF], sps[CH_DIFF], v.uv[CH_AIN0], v.uv[CH_AIN1], sps[CH_AIN0], sps[CH_AIN1], slot_max, late, retries
                );
            } else {
                info!("ADS1256: diff {} uV ({} SPS), write retries {}", v.uv[CH_DIFF], sps[CH_DIFF], retries);
            }
            self.last_log = Instant::now();
        }
        self.window = Instant::now();
        self.count = [0; 3];
        self.slot_us_max = 0;
    }
}

/// MUX OFF: conversions run back to back at the filter rate; each is read after DRDY falls.
async fn run_continuous(adc: &mut Ads1256) -> Result<Infallible, Error> {
    let mut st = Stats::new();
    adc.start()?;
    loop {
        // One period is 1 ms; the first result after start takes t18 = 1.18 ms.
        adc.wait_conversion(Duration::from_millis(5)).await?;
        let code = adc.read_data()?;
        push(code);
        update_view(|v| v.uv[CH_DIFF] = to_uv(code, DIFF_CONT.gain));
        st.count[CH_DIFF] += 1;
        if mux() {
            return Err(Error::ModeChange);
        }
        st.tick(adc.retries, false);
    }
}

/// MUX ON: 1 kHz slots, see the module docs.
async fn run_slots(adc: &mut Ads1256) -> Result<Infallible, Error> {
    let mut st = Stats::new();
    let mut ticker = Ticker::every(Duration::from_hz(DIFF_SPS as u64));
    let mut slot = 0u32;
    let timeout = Duration::from_millis(2);
    loop {
        ticker.next().await;
        let t0 = Instant::now();

        // The multiplexer is on DIFF_SLOT here (set by calibrate or at the end of the last slot).
        adc.start()?;
        adc.wait_conversion(timeout).await?;
        let se = match slot % SE_EVERY {
            0 => Some((CH_AIN0, &AIN0_SE)),
            s if s == SE_EVERY / 2 => Some((CH_AIN1, &AIN1_SE)),
            _ => None,
        };
        let diff = match se {
            None => adc.read_data()?,
            Some((idx, ch)) => {
                // Switch first, then read the differential result (datasheet figure 19).
                adc.set_channel(ch)?;
                adc.start()?;
                let diff = adc.read_data()?;
                adc.wait_conversion(timeout).await?;
                adc.set_channel(&DIFF_SLOT)?;
                let uv = to_uv(adc.read_data()?, ch.gain);
                update_view(|v| v.uv[idx] = uv);
                st.count[idx] += 1;
                diff
            }
        };
        push(diff);
        update_view(|v| v.uv[CH_DIFF] = to_uv(diff, DIFF_SLOT.gain));
        st.count[CH_DIFF] += 1;
        slot = slot.wrapping_add(1);

        let us = t0.elapsed().as_micros() as u32;
        st.slot_us_max = st.slot_us_max.max(us);
        if us >= 1_000_000 / DIFF_SPS {
            st.late += 1;
        }
        if !mux() {
            return Err(Error::ModeChange);
        }
        st.tick(adc.retries, true);
    }
}
