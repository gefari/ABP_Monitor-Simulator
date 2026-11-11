//! TI ADS1256 24-bit delta-sigma ADC (doc/ads1256.pdf) on SPI2.
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
//! Assumes the usual breakout module: 7.68 MHz crystal, 2.5 V reference, so the input range
//! is +-5 V / gain (and 0..AVDD-2 V absolute while the input buffer is on).
//!
//! Each conversion is read with RDATA after DRDY falls. The task runs on the high-priority
//! interrupt executor so the ~20 ms frame draws on the main executor don't make it miss
//! samples; every sample costs ~30 us of CPU, which is why the UI stops at 7500 SPS.

use core::cell::Cell;

use defmt::{info, warn, Format};
use embassy_futures::select::{select, Either};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::spi::{self, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration, Instant, Timer};

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
/// STATUS, MUX, ADCON, DRATE are consecutive, so one WREG writes them all.
const STATUS_BUFEN: u8 = 1 << 1;
/// ID in STATUS[7:4] for the ADS1256.
const ID_ADS1256: u8 = 3;

/// Inputs: 0..=7 = AIN0..AIN7, 8 = AINCOM.
pub const AINCOM: u8 = 8;
pub const GAINS: [u16; 7] = [1, 2, 4, 8, 16, 32, 64];

/// Data rates offered: (label, samples/s x10, DRATE code). 15k and 30k SPS are left out: at
/// ~30 us per sample they would take most of the CPU away from the UI.
pub const RATES: [(&str, u32, u8); 14] = [
    ("2.5", 25, 0x03),
    ("5", 50, 0x13),
    ("10", 100, 0x23),
    ("15", 150, 0x33),
    ("25", 250, 0x43),
    ("30", 300, 0x53),
    ("50", 500, 0x63),
    ("60", 600, 0x72),
    ("100", 1000, 0x82),
    ("500", 5000, 0x92),
    ("1000", 10000, 0xA1),
    ("2000", 20000, 0xB0),
    ("3750", 37500, 0xC0),
    ("7500", 75000, 0xD0),
];

#[derive(Clone, Copy, PartialEq, Format)]
pub struct Config {
    /// Positive input 0..=7 (AIN0..AIN7).
    pub pos: u8,
    /// Negative input 0..=8 (AIN0..AIN7, AINCOM).
    pub neg: u8,
    /// Index into `GAINS`.
    pub gain: u8,
    /// Index into `RATES`.
    pub rate: u8,
    /// Analog input buffer (high input impedance, but range limited to 0..AVDD-2 V).
    pub buffer: bool,
}

impl Config {
    pub const DEFAULT: Config = Config { pos: 0, neg: AINCOM, gain: 0, rate: 8, buffer: true };

    pub fn gain(&self) -> u16 {
        GAINS[self.gain as usize]
    }

    fn rate_x10(&self) -> u32 {
        RATES[self.rate as usize].1
    }
}

#[derive(Clone, Copy, PartialEq, Format)]
pub enum Status {
    /// Not probed yet.
    Starting,
    /// STATUS did not show the ADS1256 ID (nothing wired, or wrong wiring).
    NotFound,
    /// The chip answered but DRDY never fell.
    NoDrdy,
    Running,
}

#[derive(Clone, Copy)]
pub struct View {
    pub status: Status,
    /// Settings in effect (after self-calibration).
    pub config: Config,
    /// Last conversion result (signed 24-bit) and the same in microvolts.
    pub code: i32,
    pub uv: i32,
    /// Conversions read per second, measured.
    pub sps: u32,
}

static VIEW: Mutex<CriticalSectionRawMutex, Cell<View>> = Mutex::new(Cell::new(View {
    status: Status::Starting,
    config: Config::DEFAULT,
    code: 0,
    uv: 0,
    sps: 0,
}));
static CONFIG: Signal<CriticalSectionRawMutex, Config> = Signal::new();

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

/// Applies new settings; the ADC recalibrates and carries on. Only the latest one counts.
pub fn configure(cfg: Config) {
    CONFIG.signal(cfg);
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
}

impl Ads1256 {
    pub fn new(spi: Spi<'static, Blocking, spi::mode::Master>, cs: Output<'static>, drdy: ExtiInput<'static, Async>) -> Self {
        Ads1256 { spi, cs, drdy }
    }

    /// SPI settings for the ADS1256: mode 1 (CPOL 0, CPHA 1), MSB first, below CLKIN / 4.
    pub fn spi_config() -> spi::Config {
        let mut c = spi::Config::default();
        c.mode = spi::MODE_1;
        c.frequency = embassy_stm32::time::Hertz(SPI_HZ);
        // Reads 0x00 instead of noise when nothing is connected, so probing fails cleanly.
        c.miso_pull = embassy_stm32::gpio::Pull::Down;
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

    fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        self.select(|s| {
            s.blocking_write(&[CMD_RREG | reg, 0])?;
            wait_clkin(50); // t6
            let mut b = [0u8];
            s.blocking_read(&mut b)?;
            Ok(b[0])
        })
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

    async fn wait_drdy(&mut self, timeout: Duration) -> Result<(), Error> {
        with_timeout(timeout, self.drdy.wait_for_low()).await.map_err(|_| Error::DrdyTimeout)
    }

    /// Resets the chip and checks its ID. Returns the STATUS register.
    async fn probe(&mut self) -> Result<u8, Error> {
        self.command(CMD_RESET)?;
        Timer::after_millis(5).await;
        self.command(CMD_SDATAC)?;
        let status = self.read_reg(REG_STATUS)?;
        if status >> 4 != ID_ADS1256 {
            return Err(Error::NotFound { status });
        }
        Ok(status)
    }

    /// Writes STATUS/MUX/ADCON/DRATE, self-calibrates, and restarts conversions.
    async fn apply(&mut self, cfg: &Config) -> Result<(), Error> {
        let status = if cfg.buffer { STATUS_BUFEN } else { 0 }; // ACAL off, MSB first
        let mux = cfg.pos << 4 | cfg.neg;
        let adcon = cfg.gain; // CLKOUT off, sensor detect off, PGA
        let drate = RATES[cfg.rate as usize].2;
        self.write_regs(REG_STATUS, &[status, mux, adcon, drate])?;
        self.command(CMD_SELFCAL)?;
        // Self-calibration takes up to ~1.2 s at 2.5 SPS (datasheet table 21).
        self.wait_drdy(Duration::from_millis(1500)).await?;
        self.command(CMD_SYNC)?;
        self.command(CMD_WAKEUP)?;
        Ok(())
    }
}

/// Finds the ADS1256, applies settings from the UI and reads every conversion into the view.
/// Retries every 2 s while the chip is missing.
#[embassy_executor::task]
pub async fn task(mut adc: Ads1256) {
    let mut cfg = Config::DEFAULT;
    loop {
        let res = run(&mut adc, &mut cfg).await;
        let status = match res {
            Err(Error::NotFound { status }) => {
                warn!("ADS1256 not found (STATUS {:#04x}); check wiring on SPI2", status);
                Status::NotFound
            }
            Err(Error::DrdyTimeout) => {
                warn!("ADS1256: DRDY did not fall (PC11)");
                Status::NoDrdy
            }
            Err(Error::Spi) => {
                warn!("ADS1256: SPI error");
                Status::NotFound
            }
            Ok(()) => Status::Starting,
        };
        update_view(|v| {
            v.status = status;
            v.sps = 0;
        });
        Timer::after_secs(2).await;
    }
}

async fn run(adc: &mut Ads1256, cfg: &mut Config) -> Result<(), Error> {
    let status = adc.probe().await?;
    info!("ADS1256 found, STATUS {:#04x}", status);
    if let Some(c) = CONFIG.try_take() {
        *cfg = c;
    }
    loop {
        adc.apply(cfg).await?;
        info!("ADS1256: {}, gain {}, {} SPS, buffer {}", *cfg, cfg.gain(), RATES[cfg.rate as usize].0, cfg.buffer);
        update_view(|v| {
            v.status = Status::Running;
            v.config = *cfg;
        });

        // A missed DRDY is an error after 4 conversion periods (min 50 ms).
        let timeout = Duration::from_millis((40_000 / cfg.rate_x10()).max(50) as u64);
        let (mut window, mut count, mut last_log) = (Instant::now(), 0u32, Instant::now());
        loop {
            // DRDY goes high while a result is read, then low at the next conversion.
            match select(with_timeout(timeout, adc.drdy.wait_for_falling_edge()), CONFIG.wait()).await {
                Either::First(Ok(())) => {}
                Either::First(Err(_)) => return Err(Error::DrdyTimeout),
                Either::Second(c) => {
                    *cfg = c;
                    break; // re-apply
                }
            }
            let code = adc.read_data()?;
            let uv = to_uv(code, cfg.gain());
            count += 1;
            let elapsed = window.elapsed();
            let sps = (elapsed >= Duration::from_secs(1)).then(|| {
                let sps = (count as u64 * 1000 / elapsed.as_millis()) as u32;
                window = Instant::now();
                count = 0;
                sps
            });
            update_view(|v| {
                v.code = code;
                v.uv = uv;
                if let Some(s) = sps {
                    v.sps = s;
                }
            });
            if last_log.elapsed() >= Duration::from_secs(5) {
                info!("ADS1256: code {}, {} uV, {} SPS", code, uv, view().sps);
                last_log = Instant::now();
            }
        }
    }
}
