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
//! The ADC tab selects (`set_settings`) the differential input pair (default AIN0 (+) vs AIN1
//! (-), the transducer bridge), its gain (1..64, default 64), the data rate (MUX OFF only,
//! 5..7500 SPS, default 1000) and one of two modes:
//! - MUX OFF (default): only the differential pair, converting continuously at the selected
//!   rate. The registers are written once at start.
//! - MUX ON: one ADC behind the input multiplexer, so the channels take turns. A 1 kHz ticker
//!   starts every slot with SYNC + WAKEUP on the pair (3750 SPS filter), so pressure samples are
//!   exactly 1 ms apart. In every 10th slot the positive input vs AINCOM is converted right
//!   after it, and 5 slots later the negative input vs AINCOM (gain 1, 15000 SPS filter): 100
//!   SPS each.
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
use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

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
/// Selectable SCLK rates: (requested Hz, label of what SPI2 really makes). SPI2 runs from APB1
/// (54 MHz) through a power-of-two divider, so the requests land on /32, /64, /128, /256.
/// /16 (3.4 MHz) would exceed CLKIN / 4. Slower clocks lengthen every transfer: a 32-bit read
/// takes ~19 us at 1.69 MHz and ~150 us at 211 kHz, which limits the usable data rate. MUX ON
/// always uses the fastest (see `Settings::spi_speed`).
pub const SPI_SPEEDS: [(u32, &str); 4] =
    [(1_800_000, "1688k"), (900_000, "844k"), (450_000, "422k"), (225_000, "211k")];
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


/// One input setting: the MUX, ADCON and DRATE register values.
#[derive(Clone, Copy)]
struct Channel {
    mux: u8,
    /// PGA code in ADCON[2:0]; CLKOUT and sensor detect are off.
    adcon: u8,
    drate: u8,
}

/// MUX ON: differential filter, 3750 SPS so a second conversion fits in the 1 ms slot.
const DRATE_SLOT: u8 = 0xC0;
/// MUX ON: single-ended filter (gain 1, 15000 SPS).
const DRATE_SE: u8 = 0xE0;

/// Selectable differential data rates in MUX OFF: (samples/s, DRATE code), datasheet table 13.
/// Above 7500 SPS one read per DRDY no longer fits comfortably with SCLK at 1.69 MHz.
pub const RATES: [(u32, u8); 13] = [
    (5, 0x13),
    (10, 0x23),
    (15, 0x33),
    (25, 0x43),
    (30, 0x53),
    (50, 0x63),
    (60, 0x72),
    (100, 0x82),
    (500, 0x92),
    (1000, 0xA1),
    (2000, 0xB0),
    (3750, 0xC0),
    (7500, 0xD0),
];
const RATE_DEFAULT: u8 = 9; // 1000 SPS

/// MUX ON: slots per second (= differential samples/s) and single-ended rate divider.
pub const SLOT_HZ: u32 = 1000;
const SE_EVERY: u32 = 10;

/// Input code for AINCOM in the MUX register; 0..7 are AIN0..AIN7.
pub const AINCOM: u8 = 8;
/// Highest PGA code: gain 2^6 = 64.
pub const PGA_MAX: u8 = 6;

/// Differential samples are stored scaled to gain 64 (`code << (6 - pga)`, at most 2^29), so
/// the pressure scale and the ZERO offset stay valid when the gain changes. Microvolts per
/// stored unit:
pub const DIFF_UV_PER_CODE: f32 = 2.0 * VREF_UV as f32 / (64.0 * (1u32 << 23) as f32);

/// Channel indices in `View` and `RINGS`: the differential pair, then its positive and its
/// negative input vs AINCOM (MUX ON only).
pub const CH_DIFF: usize = 0;
pub const CH_AIN0: usize = 1;
pub const CH_AIN1: usize = 2;

/// Everything the ADC tab can change. Changing any of it restarts sampling (with a new
/// self-calibration).
#[derive(Clone, Copy, PartialEq, Eq, Format)]
pub struct Settings {
    /// MUX ON (true): 1 kHz slots with the single-ended inputs; OFF: differential only.
    pub mux: bool,
    /// Differential inputs, 0..7 = AIN0..AIN7, 8 = AINCOM; never equal.
    pub pos: u8,
    pub neg: u8,
    /// Differential PGA code 0..=6 (gain 1..64).
    pub pga: u8,
    /// Index into RATES (MUX OFF only; MUX ON always runs at SLOT_HZ).
    pub rate: u8,
    /// Index into SPI_SPEEDS.
    pub spi: u8,
}

impl Settings {
    const DEFAULT: Settings = Settings { mux: false, pos: 0, neg: 1, pga: PGA_MAX, rate: RATE_DEFAULT, spi: 0 };

    const fn pack(self) -> u32 {
        self.mux as u32 | (self.pos as u32) << 4 | (self.neg as u32) << 8 | (self.pga as u32) << 12 | (self.rate as u32) << 16 | (self.spi as u32) << 20
    }

    fn unpack(v: u32) -> Self {
        Settings {
            mux: v & 1 != 0,
            pos: (v >> 4 & 0xF) as u8,
            neg: (v >> 8 & 0xF) as u8,
            pga: (v >> 12 & 0x7) as u8,
            rate: (v >> 16 & 0xF) as u8,
            spi: (v >> 20 & 0x3) as u8,
        }
    }

    pub fn gain(&self) -> u32 {
        1 << self.pga
    }

    /// Differential samples per second.
    /// SCLK in use: the selected one in MUX OFF; MUX ON always runs at the fastest, because
    /// its 1 ms slots already overrun at 844 kHz (measured).
    pub fn spi_speed(&self) -> (u32, &'static str) {
        SPI_SPEEDS[if self.mux { 0 } else { self.spi as usize }]
    }

    pub fn diff_sps(&self) -> u32 {
        if self.mux { SLOT_HZ } else { RATES[self.rate as usize].0 }
    }

    /// Input (0..7, AINCOM) measured vs AINCOM on single-ended channel `ch` (CH_AIN0/1), or None
    /// if that channel is not sampled (MUX OFF, or the input is AINCOM itself).
    pub fn se_input(&self, ch: usize) -> Option<u8> {
        let input = if ch == CH_AIN0 { self.pos } else { self.neg };
        (self.mux && input != AINCOM).then_some(input)
    }

    fn diff_channel(&self) -> Channel {
        let drate = if self.mux { DRATE_SLOT } else { RATES[self.rate as usize].1 };
        Channel { mux: self.pos << 4 | self.neg, adcon: self.pga, drate }
    }
}

/// "AIN0".."AIN7" or "COM".
pub fn input_name(input: u8) -> &'static str {
    ["AIN0", "AIN1", "AIN2", "AIN3", "AIN4", "AIN5", "AIN6", "AIN7", "COM"][input.min(AINCOM) as usize]
}

static SETTINGS: AtomicU32 = AtomicU32::new(Settings::DEFAULT.pack());

/// Settings requested from the UI (`View::cfg` is what is in effect).
pub fn settings() -> Settings {
    Settings::unpack(SETTINGS.load(Ordering::Relaxed))
}

/// Requests new settings; the ADC task restarts with them. Invalid ones are ignored.
pub fn set_settings(s: Settings) {
    if s.pos <= AINCOM && s.neg <= AINCOM && s.pos != s.neg && s.pga <= PGA_MAX && (s.rate as usize) < RATES.len()
        && (s.spi as usize) < SPI_SPEEDS.len()
    {
        SETTINGS.store(s.pack(), Ordering::Relaxed);
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
    /// Register writes kept reading back wrong.
    WriteFail,
    Running,
}

#[derive(Clone, Copy)]
pub struct View {
    pub status: Status,
    /// Settings in effect (sampling restarts after `set_settings`).
    pub cfg: Settings,
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
    cfg: Settings::DEFAULT,
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

/// History of raw conversion codes for one channel. Single writer (the ADC task), any number of
/// readers; a reader must stay well within `len()` of `head()` (the writer keeps going while it
/// reads).
pub struct Ring {
    buf: &'static [AtomicI32],
    /// Number of samples written so far; sample `i` lives at buf[i % len].
    head: AtomicU32,
}

impl Ring {
    const fn new(buf: &'static [AtomicI32]) -> Self {
        Ring { buf, head: AtomicU32::new(0) }
    }

    /// Samples written so far; the newest is `head() - 1`.
    pub fn head(&self) -> u32 {
        self.head.load(Ordering::Acquire)
    }

    /// Sample `i` (raw code), for `i` in `head() - len() + margin .. head()`.
    pub fn get(&self, i: u32) -> i32 {
        self.buf[i as usize % self.buf.len()].load(Ordering::Relaxed)
    }

    pub fn len(&self) -> usize {
        self.buf.len()
    }

    fn push(&self, code: i32) {
        let h = self.head.load(Ordering::Relaxed);
        self.buf[h as usize % self.buf.len()].store(code, Ordering::Relaxed);
        self.head.store(h.wrapping_add(1), Ordering::Release);
    }
}

/// 16.4 s of the differential channel at 1000 SPS (the UI shows up to 12 s; less at higher
/// rates).
static DIFF_BUF: [AtomicI32; 16384] = [const { AtomicI32::new(0) }; 16384];
/// 20 s of each single-ended channel at 100 SPS (MUX ON only).
static AIN0_BUF: [AtomicI32; 2048] = [const { AtomicI32::new(0) }; 2048];
static AIN1_BUF: [AtomicI32; 2048] = [const { AtomicI32::new(0) }; 2048];

/// Sample history per channel index (CH_*).
pub static RINGS: [Ring; 3] = [Ring::new(&DIFF_BUF), Ring::new(&AIN0_BUF), Ring::new(&AIN1_BUF)];

/// Differential samples written so far (shorthand for `RINGS[CH_DIFF].head()`).
pub fn head() -> u32 {
    RINGS[CH_DIFF].head()
}

/// Differential sample `i` (raw code).
pub fn sample(i: u32) -> i32 {
    RINGS[CH_DIFF].get(i)
}

/// Nominal samples per second of channel `ch` with the settings in effect.
pub fn rate(ch: usize) -> u32 {
    if ch == CH_DIFF { view().cfg.diff_sps() } else { SLOT_HZ / SE_EVERY }
}

/// Converts a stored sample of channel `ch` (`RINGS`) to nanovolts (exact integer maths).
/// Differential samples are stored at the gain-64 scale, single-ended ones at gain 1.
pub fn code_to_nv(ch: usize, code: i64) -> i64 {
    let gain = if ch == CH_DIFF { 64 } else { 1 };
    code * 2 * VREF_UV * 1000 / (gain * (1 << 23))
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
    /// `set_settings` asked for something else.
    ConfigChange,
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
    /// SCLK currently configured.
    spi_hz: u32,
    /// Data rate the chip was last set to (its power-on default is 30000 SPS); sets the serial
    /// interface timeout below.
    chip_sps: u32,
}

impl Ads1256 {
    pub fn new(spi: Spi<'static, Blocking, spi::mode::Master>, cs: Output<'static>, drdy: ExtiInput<'static, Async>) -> Self {
        Ads1256 { spi, cs, drdy, retries: 0, spi_hz: SPI_SPEEDS[0].0, chip_sps: 30_000 }
    }

    /// SPI settings for the ADS1256 at `hz`: mode 1 (CPOL 0, CPHA 1), MSB first, below CLKIN / 4.
    pub fn spi_config(hz: u32) -> spi::Config {
        let mut c = spi::Config::default();
        c.mode = spi::MODE_1;
        c.frequency = embassy_stm32::time::Hertz(hz);
        // Reads 0x00 instead of noise when nothing is connected, so probing fails cleanly.
        c.miso_pull = embassy_stm32::gpio::Pull::Down;
        // Slowest edges: the module hangs on jumper wires.
        c.gpio_speed = embassy_stm32::gpio::Speed::Low;
        c
    }

    /// Changes SCLK, then clocks one dummy byte with CS high (ignored by the ADS1256) so the
    /// SPI peripheral settles at the new rate before the next real transfer.
    /// Returns whether the rate changed.
    fn set_spi_hz(&mut self, hz: u32) -> Result<bool, Error> {
        if hz == self.spi_hz {
            return Ok(false);
        }
        self.spi.set_config(&Ads1256::spi_config(hz)).map_err(|_| Error::Spi)?;
        self.spi.blocking_write(&[0u8])?;
        self.spi_hz = hz;
        Ok(true)
    }

    /// Waits until the ADS1256 serial interface has timed out and resynchronised: SCLK idle
    /// low for 32 DRDY periods (datasheet "Serial Clock (SCLK)"). After an SCLK rate change
    /// the chip was seen one bit out of step (STATUS 0x30 read as 0x60).
    async fn resync(&mut self) {
        Timer::after_micros(32 * 1_000_000 / self.chip_sps as u64 + 5_000).await;
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
        self.write_verified(REG_MUX, [ch.mux, ch.adcon, ch.drate], 0xFF)?;
        self.chip_sps = match ch.drate {
            DRATE_SE => 15_000,
            d => RATES.iter().find(|r| r.1 == d).map_or(30_000, |r| r.0),
        };
        Ok(())
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
        self.chip_sps = 30_000; // RESET restores DRATE 0xF0
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
        // Self-calibration takes up to ~1 s at the slowest data rates (datasheet table 21).
        with_timeout(Duration::from_millis(2000), self.drdy.wait_for_low())
            .await
            .map_err(|_| Error::DrdyTimeout)
    }
}

/// Finds the ADS1256 and samples with the requested settings. Retries every 2 s on errors.
#[embassy_executor::task]
pub async fn task(mut adc: Ads1256) {
    loop {
        let s = settings();
        let Err(e) = run(&mut adc, s).await;
        let status = match e {
            Error::ConfigChange => {
                info!("ADS1256: settings changed to {}", settings());
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

async fn run(adc: &mut Ads1256, s: Settings) -> Result<Infallible, Error> {
    if adc.set_spi_hz(s.spi_speed().0)? {
        adc.resync().await;
    }
    // After an SCLK change the probe or the first register writes can fail (interface out of
    // step): wait for the interface timeout and retry, instead of the task's 2 s retry.
    let mut tries = 0;
    loop {
        let r = async {
            let status = adc.probe().await?;
            info!("ADS1256 found, STATUS {:#04x}", status);
            adc.calibrate(&s.diff_channel()).await
        }
        .await;
        match r {
            Err(Error::NotFound { .. } | Error::WriteVerify { .. }) if tries < 3 => {
                tries += 1;
                adc.resync().await;
            }
            r => break r?,
        }
    }
    info!(
        "ADS1256 MUX {}: {}-{} gain {} at {} SPS, SCLK {}",
        if s.mux { "ON" } else { "OFF" },
        input_name(s.pos),
        input_name(s.neg),
        s.gain(),
        s.diff_sps(),
        s.spi_speed().1
    );
    update_view(|v| {
        v.status = Status::Running;
        v.cfg = s;
        v.uv = [0; 3];
        v.slot_us_max = 0;
        v.late = 0;
    });
    if s.mux { run_slots(adc, s).await } else { run_continuous(adc, s).await }
}

/// Stores a differential result: ring scaled to gain 64, view in microvolts.
fn store_diff(code: i32, s: &Settings) {
    RINGS[CH_DIFF].push(code << (PGA_MAX - s.pga));
    update_view(|v| v.uv[CH_DIFF] = to_uv(code, s.gain() as u16));
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
                    "ADS1256: diff {} uV ({} SPS), SE {} uV, {} uV ({}/{} SPS), slot max {} us, late {}, write retries {}",
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
async fn run_continuous(adc: &mut Ads1256, s: Settings) -> Result<Infallible, Error> {
    let mut st = Stats::new();
    // The first result after start takes t18 (a few periods at the slow rates); allow 5
    // periods plus margin.
    let timeout = Duration::from_micros(5_000_000 / s.diff_sps() as u64 + 5_000);
    adc.start()?;
    loop {
        adc.wait_conversion(timeout).await?;
        let code = adc.read_data()?;
        store_diff(code, &s);
        st.count[CH_DIFF] += 1;
        if settings() != s {
            return Err(Error::ConfigChange);
        }
        st.tick(adc.retries, false);
    }
}

/// MUX ON: 1 kHz slots, see the module docs.
async fn run_slots(adc: &mut Ads1256, s: Settings) -> Result<Infallible, Error> {
    let mut st = Stats::new();
    let mut ticker = Ticker::every(Duration::from_hz(SLOT_HZ as u64));
    let mut slot = 0u32;
    let timeout = Duration::from_millis(2);
    let diff_ch = s.diff_channel();
    let se_ch = |ch: usize| s.se_input(ch).map(|input| (ch, Channel { mux: input << 4 | AINCOM, adcon: 0, drate: DRATE_SE }));
    let se = [se_ch(CH_AIN0), se_ch(CH_AIN1)];
    loop {
        ticker.next().await;
        let t0 = Instant::now();

        // The multiplexer is on the differential pair here (set by calibrate or at the end of
        // the last slot).
        adc.start()?;
        adc.wait_conversion(timeout).await?;
        let this_se = match slot % SE_EVERY {
            0 => se[0],
            n if n == SE_EVERY / 2 => se[1],
            _ => None,
        };
        let diff = match this_se {
            None => adc.read_data()?,
            Some((idx, ch)) => {
                // Switch first, then read the differential result (datasheet figure 19).
                adc.set_channel(&ch)?;
                adc.start()?;
                let diff = adc.read_data()?;
                adc.wait_conversion(timeout).await?;
                adc.set_channel(&diff_ch)?;
                let code = adc.read_data()?;
                RINGS[idx].push(code);
                update_view(|v| v.uv[idx] = to_uv(code, 1));
                st.count[idx] += 1;
                diff
            }
        };
        store_diff(diff, &s);
        st.count[CH_DIFF] += 1;
        slot = slot.wrapping_add(1);

        let us = t0.elapsed().as_micros() as u32;
        st.slot_us_max = st.slot_us_max.max(us);
        if us >= 1_000_000 / SLOT_HZ {
            st.late += 1;
        }
        if settings() != s {
            return Err(Error::ConfigChange);
        }
        st.tick(adc.retries, true);
    }
}
