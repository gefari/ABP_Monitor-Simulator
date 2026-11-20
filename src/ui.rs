//! Screen: a header with three tabs.
//! - Monitor: scrolling arterial-pressure waveform measured by the ADS1256 (AIN0-AIN1, mmHg),
//!   SYS/DIA, MAP and HR from the last 4 s, the AIN0/AIN1 voltages (MUX ON only), ZERO, a time
//!   window selector (100 ms .. 12 s, time axis under the trace) and AUTO (autoscale on/off).
//! - Motor Test: speed setpoint with -/+ (tap = 10 rpm, hold to repeat), Start and Stop.
//! - ADC: input pair, gain, data rate and MUX ON/OFF; ADS1256 channels, measured rates, last
//!   readings with peak-peak and std dev over the Monitor time window, slot timing and register
//!   write retries.

use core::fmt::Write;

use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle};
use embedded_graphics::text::{Alignment, Text};
use embassy_time::{Duration, Instant};

use crate::ads1256::{self, CH_AIN0, CH_AIN1, CH_DIFF};
use crate::display::WIDTH;
use crate::motor::{self, Command, MAX_RPM, MIN_RPM};
use crate::touch::Touch;

const BG: Rgb565 = Rgb565::new(1, 2, 3);
const HEADER: Rgb565 = Rgb565::new(2, 10, 14);
const GRID: Rgb565 = Rgb565::new(5, 11, 6);
const TRACE: Rgb565 = Rgb565::new(31, 10, 6);
const LABEL: Rgb565 = Rgb565::new(20, 42, 22);
const ACCENT: Rgb565 = Rgb565::new(6, 30, 26);
const DISABLED: Rgb565 = Rgb565::new(6, 12, 7);
const GREEN: Rgb565 = Rgb565::new(4, 40, 8);
const RED: Rgb565 = Rgb565::new(24, 8, 4);

const HEADER_H: u32 = 44;

// Waveform panel
const PLOT_X: i32 = 50;
const PLOT_Y: i32 = 64;
const PLOT_W: usize = 570;
const PLOT_H: i32 = 240;
const P_MAX: f32 = 200.0; // mmHg at top of panel when autoscale is off
/// Autoscale: smallest grid step (so the full panel is never less than ~5 mmHg and sensor noise
/// is not blown up) and the number of grid divisions aimed for.
const AUTO_MIN_STEP: f32 = 1.0;
const AUTO_DIVS: f32 = 5.0;
/// Monitor time windows: (width ms, button label). At high data rates the trace is limited to
/// what the sample ring holds (`window_span`).
const WINDOWS: [(u32, &str); 5] = [(100, "100 ms"), (1000, "1 s"), (2000, "2 s"), (4000, "4 s"), (12000, "12 s")];
const WINDOW_DEFAULT: usize = 3;

// Pressure scaling: disposable ABP transducers are 5 uV/V/mmHg (AAMI BP22), excited at 5 V.
const UV_PER_MMHG: f32 = 5.0 * 5.0;
const CODES_PER_MMHG: f32 = UV_PER_MMHG / ads1256::DIFF_UV_PER_CODE;
/// SYS/DIA/MAP/HR window.
const STATS_MS: u32 = 4000;
/// Below this pulse pressure no heart rate is shown.
const MIN_PULSE_MMHG: f32 = 10.0;
/// ZERO takes the mean of the last second.
const ZERO_MS: u32 = 1000;

// Monitor readouts and ZERO button
const READOUT_Y: i32 = 322;
const READOUT_W: u32 = 116;
const READOUT_H: u32 = 104;
const BTN_ZERO: Rectangle = Rectangle::new(Point::new(8, 436), Size::new(120, 38));
const BTN_WIN: Rectangle = Rectangle::new(Point::new(380, 436), Size::new(120, 38));
const BTN_AUTO: Rectangle = Rectangle::new(Point::new(512, 436), Size::new(120, 38));

// ADC page: settings row
const SET_Y: i32 = 266;
const BTN_POS: Rectangle = Rectangle::new(Point::new(8, SET_Y), Size::new(99, 44));
const BTN_NEG: Rectangle = Rectangle::new(Point::new(113, SET_Y), Size::new(99, 44));
const BTN_GAIN: Rectangle = Rectangle::new(Point::new(218, SET_Y), Size::new(99, 44));
const BTN_RATE: Rectangle = Rectangle::new(Point::new(323, SET_Y), Size::new(99, 44));
const BTN_SPI: Rectangle = Rectangle::new(Point::new(428, SET_Y), Size::new(99, 44));
const BTN_MUX: Rectangle = Rectangle::new(Point::new(533, SET_Y), Size::new(99, 44));

// Header tabs
const TAB_MONITOR: Rectangle = Rectangle::new(Point::new(222, 6), Size::new(110, 32));
const TAB_MOTOR: Rectangle = Rectangle::new(Point::new(342, 6), Size::new(110, 32));
const TAB_ADC: Rectangle = Rectangle::new(Point::new(462, 6), Size::new(110, 32));

/// Firmware version (Cargo.toml) and git commit (build.rs), top right of the header.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
pub const GIT_VERSION: &str = env!("GIT_VERSION");

// Motor Test page
const BTN_MINUS: Rectangle = Rectangle::new(Point::new(130, 130), Size::new(90, 90));
const RPM_BOX: Rectangle = Rectangle::new(Point::new(235, 130), Size::new(170, 90));
const BTN_PLUS: Rectangle = Rectangle::new(Point::new(420, 130), Size::new(90, 90));
const BTN_START: Rectangle = Rectangle::new(Point::new(130, 330), Size::new(180, 90));
const BTN_STOP: Rectangle = Rectangle::new(Point::new(330, 330), Size::new(180, 90));

const RPM_STEP: i32 = 10;
const RPM_DEFAULT: i32 = 60;
const REPEAT_DELAY: Duration = Duration::from_millis(500);
const REPEAT_EVERY: Duration = Duration::from_millis(100);

#[derive(Clone, Copy, PartialEq)]
enum Tab {
    Monitor,
    Motor,
    Adc,
}

pub struct Ui {
    tab: Tab,
    rpm: i32,
    /// ADC code of the bridge output taken as 0 mmHg (ZERO button); None = not zeroed yet.
    zero: Option<i32>,
    /// Monitor vertical scale follows the visible trace (true) or is fixed 0..P_MAX.
    autoscale: bool,
    /// Index into WINDOWS: time span of the Monitor trace and of the ADC tab statistics.
    window: usize,
    /// Current touch point, if the screen is touched.
    touch: Option<Point>,
    /// When the -/+ button under the finger steps again.
    next_repeat: Option<Instant>,
}

fn hit(r: &Rectangle, p: Option<Point>) -> bool {
    p.is_some_and(|p| r.contains(p))
}

/// ADC tab settings row: the left half of a setting steps down, the right half up.
fn adc_touch(p: Point) {
    let mut s = ads1256::settings();
    let step = |r: &Rectangle| if p.x < r.center().x { -1 } else { 1 };
    // Inputs wrap around AIN0..AIN7, COM, skipping the other input of the pair.
    let next_input = |cur: u8, other: u8, dir: i32| {
        let mut v = cur as i32;
        loop {
            v = (v + dir).rem_euclid(ads1256::AINCOM as i32 + 1);
            if v != other as i32 {
                return v as u8;
            }
        }
    };
    if BTN_POS.contains(p) {
        s.pos = next_input(s.pos, s.neg, step(&BTN_POS));
    } else if BTN_NEG.contains(p) {
        s.neg = next_input(s.neg, s.pos, step(&BTN_NEG));
    } else if BTN_GAIN.contains(p) {
        s.pga = (s.pga as i32 + step(&BTN_GAIN)).clamp(0, ads1256::PGA_MAX as i32) as u8;
    } else if BTN_RATE.contains(p) && !s.mux {
        s.rate = (s.rate as i32 + step(&BTN_RATE)).clamp(0, ads1256::RATES.len() as i32 - 1) as u8;
    } else if BTN_SPI.contains(p) && !s.mux {
        // Left half = slower, right half = faster (SPI_SPEEDS runs fast to slow).
        s.spi = (s.spi as i32 - step(&BTN_SPI)).clamp(0, ads1256::SPI_SPEEDS.len() as i32 - 1) as u8;
    } else if BTN_MUX.contains(p) {
        s.mux = !s.mux;
    } else {
        return;
    }
    ads1256::set_settings(s);
}

/// Samples of channel `ch` covering `window_ms` at its current rate, limited to 7/8 of its ring
/// (the writer keeps going while the UI reads) and at least 2; and the ms they really span.
fn window_span(ch: usize, window_ms: u32) -> (u32, u32) {
    let rate = ads1256::rate(ch).max(1);
    let cap = ads1256::RINGS[ch].len() as u32 / 8 * 7;
    let n = (window_ms * rate / 1000).clamp(2, cap);
    (n, n * 1000 / rate)
}

/// "250 ms", "2 s" or "2.1 s".
fn write_ms<const N: usize>(b: &mut Buf<N>, ms: u32) {
    let _ = if ms < 1000 {
        write!(b, "{} ms", ms)
    } else if ms % 1000 < 100 {
        write!(b, "{} s", ms / 1000)
    } else {
        write!(b, "{}.{} s", ms / 1000, ms % 1000 / 100)
    };
}

/// Smallest 1-2-5 x 10^n step >= raw.
fn nice_step(raw: f32) -> f32 {
    let mag = libm::powf(10.0, libm::floorf(libm::log10f(raw)));
    [1.0, 2.0, 5.0, 10.0].iter().map(|m| m * mag).find(|s| *s >= raw).unwrap_or(10.0 * mag)
}

impl Ui {
    pub fn new() -> Self {
        Ui { tab: Tab::Monitor, rpm: RPM_DEFAULT, zero: None, autoscale: true, window: WINDOW_DEFAULT, touch: None, next_repeat: None }
    }

    /// Handles the latest touch state; call once per frame.
    pub fn update(&mut self, touch: Option<Touch>) {
        let p = touch.map(|t| Point::new(t.x as i32, t.y as i32));
        let pressed = p.is_some() && self.touch.is_none();
        self.touch = p;
        if p.is_none() {
            self.next_repeat = None;
            return;
        }

        if pressed {
            if hit(&TAB_MONITOR, p) {
                self.tab = Tab::Monitor;
            } else if hit(&TAB_MOTOR, p) {
                self.tab = Tab::Motor;
            } else if hit(&TAB_ADC, p) {
                self.tab = Tab::Adc;
            }
        }
        if self.tab == Tab::Monitor {
            if pressed && hit(&BTN_ZERO, p) {
                self.zero = mean_code(window_span(CH_DIFF, ZERO_MS).0).or(self.zero);
            } else if pressed && hit(&BTN_AUTO, p) {
                self.autoscale = !self.autoscale;
            } else if pressed && hit(&BTN_WIN, p) {
                self.window = (self.window + 1) % WINDOWS.len();
            }
            return;
        }
        if self.tab == Tab::Adc {
            if let (true, Some(p)) = (pressed, p) {
                adc_touch(p);
            }
            return;
        }

        if pressed && motor::view().connected {
            if hit(&BTN_START, p) {
                motor::command(Command::Run { rpm: self.rpm });
            } else if hit(&BTN_STOP, p) {
                motor::command(Command::Stop);
            }
        }

        // -/+ : step on press, then auto-repeat while held on the button.
        let step = if hit(&BTN_MINUS, p) {
            -RPM_STEP
        } else if hit(&BTN_PLUS, p) {
            RPM_STEP
        } else {
            self.next_repeat = None;
            return;
        };
        let now = Instant::now();
        let fire = if pressed {
            self.next_repeat = Some(now + REPEAT_DELAY);
            true
        } else if self.next_repeat.is_some_and(|t| now >= t) {
            self.next_repeat = Some(now + REPEAT_EVERY);
            true
        } else {
            false
        };
        if fire {
            let rpm = (self.rpm + step).clamp(MIN_RPM, MAX_RPM);
            if rpm != self.rpm {
                self.rpm = rpm;
                // While running, apply the new speed straight away.
                if motor::view().commanded_rpm.is_some() {
                    motor::command(Command::Run { rpm });
                }
            }
        }
    }

    pub fn draw<D: DrawTarget<Color = Rgb565>>(&self, d: &mut D) -> Result<(), D::Error> {
        d.clear(BG)?;
        Rectangle::new(Point::zero(), Size::new(WIDTH as u32, HEADER_H))
            .into_styled(PrimitiveStyle::with_fill(HEADER))
            .draw(d)?;
        Text::new("ABP Simulator", Point::new(14, 29), MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE)).draw(d)?;
        // Version, two small lines: "v0.1.0" and the commit ("ffbcdd2", "*" = uncommitted changes).
        let ver_style = MonoTextStyle::new(&FONT_6X10, LABEL);
        let mut v = Buf::<12>::new();
        let _ = write!(v, "v{}", VERSION);
        Text::with_alignment(v.as_str(), Point::new(WIDTH as i32 - 6, 19), ver_style, Alignment::Right).draw(d)?;
        let mut g = Buf::<12>::new();
        let _ = match GIT_VERSION.strip_suffix("-dirty") {
            Some(hash) => write!(g, "{}*", hash),
            None => g.write_str(GIT_VERSION),
        };
        Text::with_alignment(g.as_str(), Point::new(WIDTH as i32 - 6, 33), ver_style, Alignment::Right).draw(d)?;
        self.tab_button(d, &TAB_MONITOR, "Monitor", Tab::Monitor)?;
        self.tab_button(d, &TAB_MOTOR, "Motor Test", Tab::Motor)?;
        self.tab_button(d, &TAB_ADC, "ADC", Tab::Adc)?;
        match self.tab {
            Tab::Monitor => self.draw_monitor(d),
            Tab::Motor => self.draw_motor(d),
            Tab::Adc => self.draw_adc(d),
        }
    }

    fn draw_adc<D: DrawTarget<Color = Rgb565>>(&self, d: &mut D) -> Result<(), D::Error> {
        let av = ads1256::view();
        let running = av.status == ads1256::Status::Running;
        let small = MonoTextStyle::new(&FONT_10X20, LABEL);
        let white = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);

        let mut hdr = Buf::<32>::new();
        let _ = write!(hdr, "ADS1256  SPI2  SCLK {}Hz", av.cfg.spi_speed().1);
        Text::new(hdr.as_str(), Point::new(10, 80), small).draw(d)?;
        let warn = Rgb565::new(31, 40, 0);
        let (state, color) = match av.status {
            ads1256::Status::Starting => ("STARTING", LABEL),
            ads1256::Status::NotFound => ("NOT FOUND", warn),
            ads1256::Status::NoDrdy => ("NO DRDY", warn),
            ads1256::Status::WriteFail => ("WRITE FAIL", warn),
            ads1256::Status::Running => ("RUNNING", Rgb565::new(8, 60, 8)),
        };
        Text::with_alignment(state, Point::new(600, 80), MonoTextStyle::new(&FONT_10X20, color), Alignment::Right).draw(d)?;

        // One row per channel: inputs, gain, measured rate, last value, peak-to-peak (max - min)
        // and standard deviation over the Monitor time window. All voltages in mV.
        let window_ms = WINDOWS[self.window].0;
        let mut t = Buf::<64>::new();
        let _ = t.write_str("Diff in uV, single-ended in mV; stats over last ");
        write_ms(&mut t, window_span(CH_DIFF, window_ms).1);
        Text::new(t.as_str(), Point::new(10, 112), small).draw(d)?;
        const X_INPUTS: i32 = 10;
        const X_GAIN: i32 = 130;
        const X_READ: i32 = 170;
        // Right edges of Value, Peak-Peak, StdDev.
        const X_NUM: [i32; 3] = [360, 500, 630];
        for (x, h) in [(X_INPUTS, "Inputs"), (X_GAIN, "Gain"), (X_READ, "Read")] {
            Text::new(h, Point::new(x, 140), small).draw(d)?;
        }
        for (x, h) in X_NUM.iter().zip(["Value", "Peak-Peak", "StdDev"]) {
            Text::with_alignment(h, Point::new(*x, 140), small, Alignment::Right).draw(d)?;
        }
        let cfg = av.cfg; // in effect; the buttons below show the requested settings
        for i in 0..3 {
            let y = 174 + i as i32 * 32;
            let mut label = Buf::<16>::new();
            let sampled = if i == CH_DIFF {
                let _ = write!(label, "{}-{}", ads1256::input_name(cfg.pos), ads1256::input_name(cfg.neg));
                true
            } else {
                let input = if i == CH_AIN0 { cfg.pos } else { cfg.neg };
                let _ = write!(label, "{}-COM", ads1256::input_name(input));
                cfg.se_input(i).is_some()
            };
            Text::new(label.as_str(), Point::new(X_INPUTS, y), if sampled { white } else { small }).draw(d)?;
            if !sampled {
                Text::new(if cfg.mux { "n/a (input is COM)" } else { "off (MUX OFF)" }, Point::new(X_GAIN, y), small).draw(d)?;
                continue;
            }
            let mut g = Buf::<8>::new();
            let _ = write!(g, "x{}", if i == CH_DIFF { cfg.gain() } else { 1 });
            Text::new(g.as_str(), Point::new(X_GAIN, y), white).draw(d)?;
            if !running {
                Text::new("-", Point::new(X_READ, y), white).draw(d)?;
                continue;
            }
            let mut m = Buf::<8>::new();
            let _ = write!(m, "{}/s", av.sps[i]);
            Text::new(m.as_str(), Point::new(X_READ, y), white).draw(d)?;
            // Diff in uV with 2 decimals (0.0093 uV/code at gain 64); single-ended in mV with 3.
            let (unit_nv, dec) = if i == CH_DIFF { (1_000, 2) } else { (1_000_000, 3) };
            let mut vals = [Buf::<12>::new(), Buf::<12>::new(), Buf::<12>::new()];
            // Latest stored sample (the View value is rounded to whole uV).
            let ring = &ads1256::RINGS[i];
            let last = if ring.head() > 0 { ring.get(ring.head() - 1) } else { 0 };
            write_fixed(&mut vals[0], ads1256::code_to_nv(i, last as i64), unit_nv, dec);
            match ring_stats(i, window_ms) {
                Some(st) => {
                    write_fixed(&mut vals[1], ads1256::code_to_nv(i, st.max as i64 - st.min as i64), unit_nv, dec);
                    // Scale in f32: std can be a fraction of a code.
                    let nv_per_code = ads1256::code_to_nv(i, 1 << 20) as f32 / (1 << 20) as f32;
                    write_fixed(&mut vals[2], libm::roundf(st.std * nv_per_code) as i64, unit_nv, dec);
                }
                None => {
                    for v in &mut vals[1..] {
                        let _ = v.write_str("-");
                    }
                }
            }
            for (x, v) in X_NUM.iter().zip(&vals) {
                Text::with_alignment(v.as_str(), Point::new(*x, y), white, Alignment::Right).draw(d)?;
            }
        }

        // Settings row (requested values; tap the left half to step down, the right half up).
        let want = ads1256::settings();
        let mut rate = Buf::<8>::new();
        let _ = write!(rate, "{}/s", want.diff_sps());
        let mut gain = Buf::<8>::new();
        let _ = write!(gain, "x{}", want.gain());
        self.stepper(d, &BTN_POS, "IN+", ads1256::input_name(want.pos), true)?;
        self.stepper(d, &BTN_NEG, "IN-", ads1256::input_name(want.neg), true)?;
        self.stepper(d, &BTN_GAIN, "GAIN (diff)", gain.as_str(), true)?;
        self.stepper(d, &BTN_RATE, if want.mux { "RATE (fixed)" } else { "RATE" }, rate.as_str(), !want.mux)?;
        self.stepper(d, &BTN_SPI, if want.mux { "SCLK (fixed)" } else { "SPI SCLK" }, want.spi_speed().1, !want.mux)?;
        Text::new("MODE", Point::new(BTN_MUX.top_left.x, BTN_MUX.top_left.y - 6), MonoTextStyle::new(&FONT_6X10, LABEL))
            .draw(d)?;
        self.button(d, &BTN_MUX, if want.mux { "MUX ON" } else { "MUX OFF" }, if want.mux { GREEN } else { ACCENT })?;

        let y = 340;
        let hint = if want != cfg {
            "Applying: restart and self-calibration..."
        } else if cfg.mux {
            "MUX ON: 1 kHz slots; IN+ and IN- also vs COM at 100/s"
        } else {
            "MUX OFF: differential pair only, continuous"
        };
        Text::new(hint, Point::new(10, y), small).draw(d)?;
        if running {
            let mut s = Buf::<48>::new();
            if cfg.mux {
                let _ = write!(s, "Slot time max {} us, late slots {}", av.slot_us_max, av.late);
            } else {
                let _ = s.write_str("Continuous conversion, no slots");
            }
            Text::new(s.as_str(), Point::new(10, y + 28), small).draw(d)?;
        } else {
            Text::new("No ADC: see src/ads1256.rs for wiring", Point::new(10, y + 28), small).draw(d)?;
        }
        let mut w = Buf::<40>::new();
        let _ = write!(w, "Register write retries {}", av.write_retries);
        Text::new(w.as_str(), Point::new(10, y + 56), if av.write_retries > 0 { MonoTextStyle::new(&FONT_10X20, warn) } else { small })
            .draw(d)?;

        let note = MonoTextStyle::new(&FONT_6X10, LABEL);
        let mut mode = Buf::<112>::new();
        let _ = if cfg.mux {
            mode.write_str("MUX ON: 1 kHz slots convert the pair (3750 SPS filter); every 10th also IN+ vs COM, 5 later IN- (15000).")
        } else {
            write!(mode, "MUX OFF: the pair converts continuously with the {} SPS filter; registers written once.", cfg.diff_sps())
        };
        for (k, line) in [
            mode.as_str(),
            "Buffer on: inputs 0..3 V. AINCOM is tied to GND. Self-calibrated at the selected gain and rate.",
            "Pressure 25 uV/mmHg. Slow SCLK lengthens reads (211k: up to ~2000 SPS). Retries > 0: check SCLK.",
        ]
        .iter()
        .enumerate()
        {
            Text::with_alignment(line, Point::new(WIDTH as i32 / 2, 436 + k as i32 * 16), note, Alignment::Center).draw(d)?;
        }
        Ok(())
    }

    /// A setting button with a title above it and < > marks for the two tap halves.
    fn stepper<D: DrawTarget<Color = Rgb565>>(&self, d: &mut D, r: &Rectangle, title: &str, value: &str, enabled: bool) -> Result<(), D::Error> {
        let note = MonoTextStyle::new(&FONT_6X10, LABEL);
        Text::new(title, Point::new(r.top_left.x, r.top_left.y - 6), note).draw(d)?;
        self.button(d, r, value, if enabled { ACCENT } else { DISABLED })?;
        if enabled {
            let arrow = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
            let y = r.center().y + 6;
            Text::new("<", Point::new(r.top_left.x + 6, y), arrow).draw(d)?;
            Text::with_alignment(">", Point::new(r.top_left.x + r.size.width as i32 - 6, y), arrow, Alignment::Right).draw(d)?;
        }
        Ok(())
    }

    fn tab_button<D: DrawTarget<Color = Rgb565>>(&self, d: &mut D, r: &Rectangle, label: &str, tab: Tab) -> Result<(), D::Error> {
        let active = self.tab == tab;
        let style = if active {
            PrimitiveStyleBuilder::new().fill_color(ACCENT).build()
        } else {
            PrimitiveStyleBuilder::new().stroke_color(LABEL).stroke_width(1).build()
        };
        RoundedRectangle::with_equal_corners(*r, Size::new(8, 8)).into_styled(style).draw(d)?;
        centered(d, r, label, if active { Rgb565::WHITE } else { LABEL })
    }

    fn draw_motor<D: DrawTarget<Color = Rgb565>>(&self, d: &mut D) -> Result<(), D::Error> {
        let mv = motor::view();
        let small = MonoTextStyle::new(&FONT_10X20, LABEL);
        let white = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);

        // Status line
        let (state, color) = match (mv.connected, mv.commanded_rpm) {
            (false, _) => ("OFFLINE", Rgb565::new(31, 40, 0)),
            (true, Some(_)) => ("RUNNING", Rgb565::new(8, 60, 8)),
            (true, None) => ("STOPPED", Rgb565::WHITE),
        };
        let mut s = Buf::<48>::new();
        let _ = write!(s, "LMDCE571  {}", crate::net::MOTOR_IP);
        Text::new(s.as_str(), Point::new(40, 80), small).draw(d)?;
        Text::with_alignment(state, Point::new(600, 80), MonoTextStyle::new(&FONT_10X20, color), Alignment::Right).draw(d)?;
        if mv.error != 0 {
            let mut e = Buf::<24>::new();
            let _ = write!(e, "motor error ER {}", mv.error);
            Text::with_alignment(e.as_str(), Point::new(600, 104), MonoTextStyle::new(&FONT_10X20, TRACE), Alignment::Right)
                .draw(d)?;
        }

        // Speed setpoint with -/+
        Text::new("Speed setpoint", Point::new(130, 120), small).draw(d)?;
        self.button(d, &BTN_MINUS, "", if self.rpm <= MIN_RPM { DISABLED } else { ACCENT })?;
        self.button(d, &BTN_PLUS, "", if self.rpm >= MAX_RPM { DISABLED } else { ACCENT })?;
        bar(d, BTN_MINUS.center(), 40, 8)?;
        bar(d, BTN_PLUS.center(), 40, 8)?;
        bar(d, BTN_PLUS.center(), 8, 40)?;
        RoundedRectangle::with_equal_corners(RPM_BOX, Size::new(10, 10))
            .into_styled(PrimitiveStyleBuilder::new().stroke_color(LABEL).stroke_width(2).fill_color(HEADER).build())
            .draw(d)?;
        let mut r = Buf::<12>::new();
        let _ = write!(r, "{}", self.rpm);
        Text::with_alignment(r.as_str(), RPM_BOX.center(), white, Alignment::Center).draw(d)?;
        Text::with_alignment("rpm", RPM_BOX.center() + Point::new(0, 28), small, Alignment::Center).draw(d)?;

        // Measured values
        let mut a = Buf::<32>::new();
        let _ = write!(a, "Actual speed  {} rpm", mv.rpm);
        Text::new(a.as_str(), Point::new(130, 262), white).draw(d)?;
        let mut p = Buf::<48>::new();
        let rev10 = mv.position as i64 * 10 / (200 * 256);
        let sign = if rev10 < 0 { "-" } else { "" };
        let _ = write!(p, "Position      {} steps ({}{}.{} rev)", mv.position, sign, rev10.abs() / 10, rev10.abs() % 10);
        Text::new(p.as_str(), Point::new(130, 292), white).draw(d)?;

        // Start / Stop
        let online = mv.connected;
        self.button(d, &BTN_START, "START", if online { GREEN } else { DISABLED })?;
        self.button(d, &BTN_STOP, "STOP", if online { RED } else { DISABLED })?;
        if mv.commanded_rpm.is_some() {
            // Ring around START while running.
            RoundedRectangle::with_equal_corners(BTN_START.offset(5), Size::new(14, 14))
                .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 2))
                .draw(d)?;
        }
        Text::with_alignment(
            "STOP decelerates over the network; it is not an emergency stop.",
            Point::new(WIDTH as i32 / 2, 462),
            MonoTextStyle::new(&FONT_6X10, LABEL),
            Alignment::Center,
        )
        .draw(d)?;
        Ok(())
    }

    /// Filled rounded button with a white outline while the finger is on it.
    fn button<D: DrawTarget<Color = Rgb565>>(&self, d: &mut D, r: &Rectangle, label: &str, fill: Rgb565) -> Result<(), D::Error> {
        let pressed = hit(r, self.touch) && fill != DISABLED;
        let style = PrimitiveStyleBuilder::new()
            .fill_color(fill)
            .stroke_color(if pressed { Rgb565::WHITE } else { fill })
            .stroke_width(3)
            .build();
        RoundedRectangle::with_equal_corners(*r, Size::new(12, 12)).into_styled(style).draw(d)?;
        centered(d, r, label, Rgb565::WHITE)
    }
}


/// Nanovolts in units of `unit_nv` (1000 = uV, 1_000_000 = mV) with `decimals` digits,
/// e.g. "216.04" uV or "2420.384" mV.
fn write_fixed<const N: usize>(b: &mut Buf<N>, nv: i64, unit_nv: u64, decimals: u32) {
    let (sign, a) = (if nv < 0 { "-" } else { "" }, nv.unsigned_abs());
    let frac = (a % unit_nv) * 10u64.pow(decimals) / unit_nv;
    let _ = write!(b, "{}{}.{:0w$}", sign, a / unit_nv, frac, w = decimals as usize);
}

/// Min / max / standard deviation of a channel's raw codes.
struct RingStats {
    min: i32,
    max: i32,
    std: f32,
}

/// Statistics of channel `ch` over its last `window_ms` (fewer samples right after boot).
fn ring_stats(ch: usize, window_ms: u32) -> Option<RingStats> {
    let ring = &ads1256::RINGS[ch];
    let head = ring.head();
    let n = window_span(ch, window_ms).0.min(head);
    if n == 0 {
        return None;
    }
    let range = head - n..head;
    let (mut min, mut max, mut sum) = (i32::MAX, i32::MIN, 0i64);
    for i in range.clone() {
        let c = ring.get(i);
        min = min.min(c);
        max = max.max(c);
        sum += c as i64;
    }
    // Second pass around the mean: no overflow and no loss of the small noise figure.
    let mean = (sum / n as i64) as i32;
    let var = range.map(|i| { let e = (ring.get(i) - mean) as f32; e * e }).sum::<f32>() / n as f32;
    Some(RingStats { min, max, std: libm::sqrtf(var) })
}

/// Microvolts as volts with `decimals` (4 or 6) digits, e.g. "2.5012" or "-0.000123".
fn write_volts<const N: usize>(b: &mut Buf<N>, uv: i32, decimals: u32) {
    let (sign, a) = (if uv < 0 { "-" } else { "" }, uv.unsigned_abs());
    let frac = (a % 1_000_000) / 10u32.pow(6 - decimals);
    let _ = write!(b, "{}{}.{:0w$}", sign, a / 1_000_000, frac, w = decimals as usize);
}

/// Text centred in a rectangle (FONT_10X20).
fn centered<D: DrawTarget<Color = Rgb565>>(d: &mut D, r: &Rectangle, s: &str, c: Rgb565) -> Result<(), D::Error> {
    Text::with_alignment(s, r.center() + Point::new(0, 6), MonoTextStyle::new(&FONT_10X20, c), Alignment::Center).draw(d)?;
    Ok(())
}

/// White filled bar centred on `c` (for the - and + symbols).
fn bar<D: DrawTarget<Color = Rgb565>>(d: &mut D, c: Point, w: u32, h: u32) -> Result<(), D::Error> {
    d.fill_solid(&Rectangle::with_center(c, Size::new(w, h)), Rgb565::WHITE)
}

/// Fixed-capacity string for formatting numbers without an allocator.
struct Buf<const N: usize> {
    b: [u8; N],
    len: usize,
}
impl<const N: usize> Buf<N> {
    fn new() -> Self {
        Self { b: [0; N], len: 0 }
    }
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.b[..self.len]).unwrap_or("")
    }
}
impl<const N: usize> Write for Buf<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let n = s.len().min(N - self.len);
        self.b[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

// ---------------------------------------------------------------------------------------------
// Monitor tab

/// Mean code of the last `n` pressure samples, if the ADC is running and has that many.
fn mean_code(n: u32) -> Option<i32> {
    let head = ads1256::head();
    if ads1256::view().status != ads1256::Status::Running || head < n {
        return None;
    }
    let sum: i64 = (head - n..head).map(|i| ads1256::sample(i) as i64).sum();
    Some((sum / n as i64) as i32)
}

/// Pressure figures over the last `n` samples (mmHg, bpm).
struct Vitals {
    sys: f32,
    dia: f32,
    /// Mean of the waveform over the window (the window is not a whole number of beats, so
    /// this moves a little with the beat phase).
    map: f32,
    hr: Option<u32>,
}

fn vitals(head: u32, n: u32, rate: u32, mmhg: &impl Fn(u32) -> f32) -> Option<Vitals> {
    if head < n {
        return None;
    }
    let start = head - n;
    let (mut sys, mut dia, mut sum) = (f32::MIN, f32::MAX, 0.0f32);
    for i in start..head {
        let p = mmhg(i);
        sys = sys.max(p);
        dia = dia.min(p);
        sum += p;
    }
    // Heart rate: rising crossings of 60 % of the pulse pressure, re-armed below 40 %.
    let pp = sys - dia;
    let mut hr = None;
    if pp >= MIN_PULSE_MMHG {
        let (hi, lo) = (dia + 0.6 * pp, dia + 0.4 * pp);
        let (mut armed, mut first, mut last, mut beats) = (false, 0u32, 0u32, 0u32);
        for i in start..head {
            let p = mmhg(i);
            if p < lo {
                armed = true;
            } else if armed && p > hi {
                armed = false;
                if beats == 0 {
                    first = i;
                }
                last = i;
                beats += 1;
            }
        }
        if beats >= 2 {
            hr = Some((beats - 1) * 60 * rate / (last - first));
        }
    }
    Some(Vitals { sys, dia, map: sum / n as f32, hr })
}

/// Vertical scale of the waveform panel: `lo`..`hi` mmHg with a grid line every `step`.
#[derive(Clone, Copy)]
struct Scale {
    lo: f32,
    hi: f32,
    step: f32,
}

impl Scale {
    const FIXED: Scale = Scale { lo: 0.0, hi: P_MAX, step: 40.0 };

    /// Range covering `min..max` with 1-2-5 grid steps, limits on grid lines. Snapping to the
    /// grid keeps the scale from jumping every frame as the trace scrolls.
    fn fit(min: f32, max: f32) -> Scale {
        let raw = ((max - min) / AUTO_DIVS).max(AUTO_MIN_STEP);
        let step = nice_step(raw);
        let mut lo = libm::floorf(min / step) * step;
        let mut hi = libm::ceilf(max / step) * step;
        if hi - lo < step * 2.0 {
            // Flat trace: keep it in the middle of a two-division panel.
            lo -= step;
            hi = lo + step * 3.0;
        }
        Scale { lo, hi, step }
    }

    fn y(&self, p: f32) -> i32 {
        let f = (p - self.lo) / (self.hi - self.lo);
        (PLOT_Y + PLOT_H - (f * PLOT_H as f32) as i32).clamp(PLOT_Y, PLOT_Y + PLOT_H)
    }
}

/// Readout box number `slot` (0..5) along the bottom of the Monitor tab.
fn readout<D: DrawTarget<Color = Rgb565>>(d: &mut D, slot: i32, label: &str, value: &str, unit: &str) -> Result<(), D::Error> {
    let x = 8 + slot * (READOUT_W as i32 + 10);
    let cx = x + READOUT_W as i32 / 2;
    let style = PrimitiveStyleBuilder::new().stroke_color(LABEL).stroke_width(2).fill_color(HEADER).build();
    RoundedRectangle::with_equal_corners(Rectangle::new(Point::new(x, READOUT_Y), Size::new(READOUT_W, READOUT_H)), Size::new(12, 12))
        .into_styled(style)
        .draw(d)?;
    let small = MonoTextStyle::new(&FONT_10X20, LABEL);
    let big = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    Text::new(label, Point::new(x + 10, READOUT_Y + 26), small).draw(d)?;
    Text::with_alignment(value, Point::new(cx, READOUT_Y + 62), big, Alignment::Center).draw(d)?;
    Text::with_alignment(unit, Point::new(cx, READOUT_Y + 92), small, Alignment::Center).draw(d)?;
    Ok(())
}

impl Ui {
    fn draw_monitor<D: DrawTarget<Color = Rgb565>>(&self, d: &mut D) -> Result<(), D::Error> {
        let av = ads1256::view();
        let running = av.status == ads1256::Status::Running;
        let head = ads1256::head();
        let zero = self.zero.unwrap_or(0);
        let mmhg = |i: u32| (ads1256::sample(i) - zero) as f32 / CODES_PER_MMHG;
        let small = MonoTextStyle::new(&FONT_10X20, LABEL);
        let (span, window_ms) = window_span(CH_DIFF, WINDOWS[self.window].0);
        let tick_ms = (nice_step(window_ms as f32 / 5.0) as u32).max(1);
        let have_trace = running && head > span;

        // Vertical scale: fixed 0..200 mmHg, or fitted to what is on screen.
        let scale = if self.autoscale && have_trace {
            let (min, max) = (head - span - 1..head)
                .map(mmhg)
                .fold((f32::MAX, f32::MIN), |(lo, hi), p| (lo.min(p), hi.max(p)));
            Scale::fit(min, max)
        } else {
            Scale::FIXED
        };

        // Grid with labels
        let label_style = MonoTextStyle::new(&FONT_6X10, LABEL);
        let n = libm::roundf((scale.hi - scale.lo) / scale.step) as i32;
        for k in 0..=n {
            let p = scale.lo + k as f32 * scale.step;
            let y = scale.y(p);
            Line::new(Point::new(PLOT_X, y), Point::new(PLOT_X + PLOT_W as i32 - 1, y))
                .into_styled(PrimitiveStyle::with_stroke(GRID, 1))
                .draw(d)?;
            let mut s = Buf::<6>::new();
            let _ = write!(s, "{}", libm::roundf(p) as i32);
            Text::with_alignment(s.as_str(), Point::new(PLOT_X - 8, y + 4), label_style, Alignment::Right).draw(d)?;
        }
        Text::new("mmHg", Point::new(8, PLOT_Y - 8), label_style).draw(d)?;

        // Time axis: vertical grid line and label every tick, "now" at the right edge.
        let ticks = window_ms / tick_ms;
        for k in 0..=ticks {
            let t_ms = (ticks - k) * tick_ms; // time before now
            let x = PLOT_X + ((PLOT_W as u32 - 1) * (window_ms - t_ms) / window_ms) as i32;
            Line::new(Point::new(x, PLOT_Y), Point::new(x, PLOT_Y + PLOT_H))
                .into_styled(PrimitiveStyle::with_stroke(GRID, 1))
                .draw(d)?;
            let mut s = Buf::<12>::new();
            let sign = if t_ms > 0 { "-" } else { "" };
            let _ = if window_ms < 1000 {
                write!(s, "{}{}", sign, t_ms)
            } else if tick_ms % 1000 != 0 {
                write!(s, "{}{}.{}", sign, t_ms / 1000, t_ms % 1000 / 100)
            } else {
                write!(s, "{}{}", sign, t_ms / 1000)
            };
            let (pos, align) = if t_ms == 0 {
                let _ = s.write_str(if window_ms < 1000 { " ms" } else { " s" });
                (Point::new(x + 6, PLOT_Y + PLOT_H + 12), Alignment::Right)
            } else {
                (Point::new(x, PLOT_Y + PLOT_H + 12), Alignment::Center)
            };
            Text::with_alignment(s.as_str(), pos, label_style, align).draw(d)?;
        }

        // Scrolling trace, rightmost column is "now". Each column covers span / PLOT_W samples
        // (0.2 at 100 ms .. 21 at 12 s and 1000 SPS) and is drawn as one 2-px-wide vertical span from the
        // previous column's last sample through its own min..max (much cheaper than a thick
        // Polyline). Columns with no sample of their own repeat the previous level.
        if have_trace {
            let first = |col: u32| head - span + col * span / PLOT_W as u32;
            let mut prev = scale.y(mmhg(first(0) - 1));
            for col in 0..PLOT_W as u32 {
                let (mut top, mut bot) = (prev, prev);
                for i in first(col)..first(col + 1) {
                    prev = scale.y(mmhg(i));
                    top = top.min(prev);
                    bot = bot.max(prev);
                }
                d.fill_solid(
                    &Rectangle::new(Point::new(PLOT_X + col as i32, top), Size::new(2, (bot - top + 2) as u32)),
                    TRACE,
                )?;
            }
        } else {
            let msg = if running { "Collecting samples..." } else { "No pressure signal: ADC not running (see ADC tab)" };
            Text::with_alignment(msg, Point::new(PLOT_X + PLOT_W as i32 / 2, PLOT_Y + PLOT_H / 2), small, Alignment::Center)
                .draw(d)?;
        }

        // Readouts
        let vit = if running { vitals(head, window_span(CH_DIFF, STATS_MS).0, ads1256::rate(CH_DIFF), &mmhg) } else { None };
        let mut hr = Buf::<8>::new();
        let mut sd = Buf::<16>::new();
        let mut map = Buf::<8>::new();
        match &vit {
            Some(v) => {
                let _ = match v.hr {
                    Some(b) => write!(hr, "{}", b),
                    None => hr.write_str("--"),
                };
                let _ = write!(sd, "{}/{}", libm::roundf(v.sys) as i32, libm::roundf(v.dia) as i32);
                let _ = write!(map, "{}", libm::roundf(v.map) as i32);
            }
            None => {
                let _ = hr.write_str("--");
                let _ = sd.write_str("--/--");
                let _ = map.write_str("--");
            }
        }
        // Single-ended readouts: the pair's + and - inputs vs COM (MUX ON).
        let mut se = [Buf::<12>::new(), Buf::<12>::new()];
        for (b, ch) in se.iter_mut().zip([CH_AIN0, CH_AIN1]) {
            match (running, av.cfg.se_input(ch)) {
                (true, Some(_)) => write_volts(b, av.uv[ch], 4),
                (true, None) => {
                    let _ = b.write_str("off");
                }
                (false, _) => {
                    let _ = b.write_str("--");
                }
            }
        }
        readout(d, 0, "HR", hr.as_str(), "bpm")?;
        readout(d, 1, "SYS/DIA", sd.as_str(), "mmHg")?;
        readout(d, 2, "MAP", map.as_str(), "mmHg")?;
        readout(d, 3, ads1256::input_name(av.cfg.pos), se[0].as_str(), "V")?;
        readout(d, 4, ads1256::input_name(av.cfg.neg), se[1].as_str(), "V")?;

        // Beat indicator in the HR box: lit while the pressure is above the pulse midpoint.
        let beat = vit.as_ref().is_some_and(|v| v.hr.is_some() && mmhg(head - 1) > (v.sys + v.dia) / 2.0);
        Circle::new(Point::new(8 + READOUT_W as i32 - 30, READOUT_Y + 12), 18)
            .into_styled(PrimitiveStyle::with_fill(if beat { TRACE } else { GRID }))
            .draw(d)?;

        // ZERO
        self.button(d, &BTN_ZERO, "ZERO", if running { ACCENT } else { DISABLED })?;
        let mut z = Buf::<48>::new();
        let _ = match self.zero {
            None => z.write_str("Vent, press ZERO"),
            Some(c) => write!(z, "Zero {} uV", (c as f32 * ads1256::DIFF_UV_PER_CODE) as i32),
        };
        Text::new(z.as_str(), Point::new(BTN_ZERO.top_left.x + 136, BTN_ZERO.center().y + 6), small).draw(d)?;

        // Time window (cycles through WINDOWS)
        self.button(d, &BTN_WIN, WINDOWS[self.window].1, ACCENT)?;

        // Autoscale toggle
        let (label, fill) = if self.autoscale { ("AUTO", GREEN) } else { ("0-200", ACCENT) };
        self.button(d, &BTN_AUTO, label, fill)?;
        Ok(())
    }
}
