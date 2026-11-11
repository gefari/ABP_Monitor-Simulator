//! Capacitive touch on the 5.7" LCD (MB1063): EETI EXC7200 on I2C1 (PB8 = SCL, PB9 = SDA),
//! 7-bit address 0x04. Polled (its INT line goes through the MFX expander, GPIO14).
//!
//! Protocol (from ST's exc7200 BSP driver): write command 0x09, read a 10-byte report.
//! byte 1 == 0x83 while touched; X = b3:b2[7:4], Y = b5:b4[7:4]. On this board both span 0..=2047.

use core::cell::Cell;

use defmt::{info, Format};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{I2C1, PB8, PB9};
use embassy_stm32::time::Hertz;
use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};

use crate::display::{HEIGHT, WIDTH};

pub type Bus = I2c<'static, Blocking, i2c::mode::Master>;

const ADDR: u8 = 0x04;
const CMD_READ: u8 = 0x09;
const TOUCHED: u8 = 0x83;
const RAW_MAX: u16 = 2047;
/// The controller returns an empty frame between reports (~20-30 ms apart), so a release is
/// only reported after this long without touch data.
const RELEASE_AFTER: Duration = Duration::from_millis(50);

#[derive(Clone, Copy, Format)]
pub struct Touch {
    /// Screen coordinates (pixels).
    pub x: u16,
    pub y: u16,
    /// Raw controller values (0..=2047), for calibration.
    pub raw_x: u16,
    pub raw_y: u16,
}

/// Latest touch state, written by `task`, read by the UI.
static CURRENT: Mutex<CriticalSectionRawMutex, Cell<Option<Touch>>> = Mutex::new(Cell::new(None));

pub fn current() -> Option<Touch> {
    CURRENT.lock(|c| c.get())
}

pub fn bus(i2c: Peri<'static, I2C1>, scl: Peri<'static, PB8>, sda: Peri<'static, PB9>) -> Bus {
    let mut cfg = i2c::Config::default();
    cfg.frequency = Hertz(100_000);
    I2c::new_blocking(i2c, scl, sda, cfg)
}

/// Probes every 7-bit address with a 1-byte read and logs the ones that ACK.
pub fn scan(bus: &mut Bus) {
    info!("I2C1 scan:");
    let mut found = 0;
    for addr in 0x03u8..0x78 {
        let mut b = [0u8; 1];
        if bus.blocking_read(addr, &mut b).is_ok() {
            info!("  {:#04x} ACK", addr);
            found += 1;
        }
    }
    info!("I2C1 scan done, {} device(s)", found);
}

fn read(bus: &mut Bus) -> Result<Option<Touch>, i2c::Error> {
    let mut f = [0u8; 10];
    bus.blocking_write_read(ADDR, &[CMD_READ], &mut f)?;
    if f[1] != TOUCHED {
        return Ok(None);
    }
    let raw_x = ((f[3] as u16) << 4) | ((f[2] as u16) >> 4);
    let raw_y = ((f[5] as u16) << 4) | ((f[4] as u16) >> 4);
    // Measured on this board: both axes span 0..=2047 (11-bit), same orientation as the LCD.
    let rx = raw_x.min(RAW_MAX);
    let ry = raw_y.min(RAW_MAX);
    let x = (rx as u32 * (WIDTH as u32 - 1) / RAW_MAX as u32) as u16;
    let y = (ry as u32 * (HEIGHT as u32 - 1) / RAW_MAX as u32) as u16;
    Ok(Some(Touch { x, y, raw_x, raw_y }))
}

/// Polls the controller every 10 ms, publishes the state and logs press/hold/release.
#[embassy_executor::task]
pub async fn task(mut bus: Bus) {
    let mut was_down = false;
    let mut last_log = Instant::now();
    let mut last_seen: Option<(Touch, Instant)> = None;
    loop {
        let sample = match read(&mut bus) {
            Ok(t) => t,
            Err(e) => {
                defmt::warn!("touch read error: {}", e);
                None
            }
        };
        if let Some(s) = sample {
            last_seen = Some((s, Instant::now()));
        }
        // Debounced state: keep the last report until RELEASE_AFTER has passed without one.
        let t = match last_seen {
            Some((s, at)) if at.elapsed() < RELEASE_AFTER => Some(s),
            _ => None,
        };
        CURRENT.lock(|c| c.set(t));
        match (t, was_down) {
            (Some(t), false) => {
                info!("touch DOWN {}", t);
                last_log = Instant::now();
            }
            (Some(t), true) if last_log.elapsed() >= Duration::from_millis(200) => {
                info!("touch HOLD {}", t);
                last_log = Instant::now();
            }
            (None, true) => info!("touch UP"),
            _ => {}
        }
        was_down = t.is_some();
        Timer::after_millis(10).await;
    }
}
