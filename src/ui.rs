//! Screen: a header with three tabs.
//! - Monitor: scrolling arterial-pressure waveform measured by the ADS1256 (AIN0-AIN1, mmHg),
//!   SYS/DIA, MAP and HR from the last 4 s, the AIN0/AIN1 voltages (MUX ON only), ZERO, and AUTO
//!   (autoscale on/off).
//! - Motor Test: speed setpoint with -/+ (tap = 10 rpm, hold to repeat), Start and Stop.
//! - ADC: MUX ON/OFF switch, ADS1256 channels, measured rates, last readings, slot timing and
//!   register write retries.

use core::fmt::Write;

use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle};
use embedded_graphics::text::{Alignment, Text};
use embassy_time::{Duration, Instant};

use crate::ads1256::{self, CH_AIN0, CH_AIN1};
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
const SWEEP_PX_PER_S: u32 = 150;

// Pressure scaling: disposable ABP transducers are 5 uV/V/mmHg (AAMI BP22), excited at 5 V.
const UV_PER_MMHG: f32 = 5.0 * 5.0;
const CODES_PER_MMHG: f32 = UV_PER_MMHG / ads1256::DIFF_UV_PER_CODE;
/// SYS/DIA/MAP/HR window: 4 s at 1000 SPS (well inside ads1256::RING_LEN).
const STATS_SAMPLES: u32 = 4000;
/// Below this pulse pressure no heart rate is shown.
const MIN_PULSE_MMHG: f32 = 10.0;
/// ZERO takes the mean of the last second.
const ZERO_SAMPLES: u32 = 1000;

// Monitor readouts and ZERO button
const READOUT_Y: i32 = 322;
const READOUT_W: u32 = 116;
const READOUT_H: u32 = 104;
const BTN_ZERO: Rectangle = Rectangle::new(Point::new(8, 436), Size::new(120, 38));
const BTN_AUTO: Rectangle = Rectangle::new(Point::new(512, 436), Size::new(120, 38));

// ADC page
const BTN_MUX: Rectangle = Rectangle::new(Point::new(40, 270), Size::new(160, 46));

// Header tabs
const TAB_MONITOR: Rectangle = Rectangle::new(Point::new(280, 6), Size::new(110, 32));
const TAB_MOTOR: Rectangle = Rectangle::new(Point::new(400, 6), Size::new(110, 32));
const TAB_ADC: Rectangle = Rectangle::new(Point::new(520, 6), Size::new(110, 32));

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
    /// Current touch point, if the screen is touched.
    touch: Option<Point>,
    /// When the -/+ button under the finger steps again.
    next_repeat: Option<Instant>,
}

fn hit(r: &Rectangle, p: Option<Point>) -> bool {
    p.is_some_and(|p| r.contains(p))
}

impl Ui {
    pub fn new() -> Self {
        Ui { tab: Tab::Monitor, rpm: RPM_DEFAULT, zero: None, autoscale: true, touch: None, next_repeat: None }
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
                self.zero = mean_code(ZERO_SAMPLES).or(self.zero);
            } else if pressed && hit(&BTN_AUTO, p) {
                self.autoscale = !self.autoscale;
            }
            return;
        }
        if self.tab == Tab::Adc {
            if pressed && hit(&BTN_MUX, p) {
                ads1256::set_mux(!ads1256::mux());
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

        Text::new("ADS1256  SPI2", Point::new(40, 80), small).draw(d)?;
        let warn = Rgb565::new(31, 40, 0);
        let (state, color) = match av.status {
            ads1256::Status::Starting => ("STARTING", LABEL),
            ads1256::Status::NotFound => ("NOT FOUND", warn),
            ads1256::Status::NoDrdy => ("NO DRDY", warn),
            ads1256::Status::WriteFail => ("WRITE FAIL", warn),
            ads1256::Status::Running => ("RUNNING", Rgb565::new(8, 60, 8)),
        };
        Text::with_alignment(state, Point::new(600, 80), MonoTextStyle::new(&FONT_10X20, color), Alignment::Right).draw(d)?;

        // One row per channel: inputs, gain, scheduled rate, measured rate, last value.
        const COLS: [i32; 4] = [40, 200, 270, 360];
        for (x, h) in COLS.iter().zip(["Inputs", "Gain", "Rate", "Read"]) {
            Text::new(h, Point::new(*x, 130), small).draw(d)?;
        }
        Text::with_alignment("Value", Point::new(600, 130), small, Alignment::Right).draw(d)?;
        for (i, (inputs, gain, rate, _)) in ads1256::channel_info(av.mux).iter().enumerate() {
            let y = 168 + i as i32 * 34;
            let on = *rate > 0;
            Text::new(inputs, Point::new(COLS[0], y), if on { white } else { small }).draw(d)?;
            if !on {
                Text::new("off (MUX OFF)", Point::new(COLS[1], y), small).draw(d)?;
                continue;
            }
            let mut g = Buf::<8>::new();
            let _ = write!(g, "x{}", gain);
            Text::new(g.as_str(), Point::new(COLS[1], y), white).draw(d)?;
            let mut r = Buf::<8>::new();
            let _ = write!(r, "{}/s", rate);
            Text::new(r.as_str(), Point::new(COLS[2], y), white).draw(d)?;
            let mut m = Buf::<8>::new();
            let mut v = Buf::<16>::new();
            if running {
                let _ = write!(m, "{}/s", av.sps[i]);
                write_volts(&mut v, av.uv[i], 6);
                let _ = v.write_str(" V");
            } else {
                let _ = m.write_str("-");
                let _ = v.write_str("-");
            }
            Text::new(m.as_str(), Point::new(COLS[3], y), white).draw(d)?;
            Text::with_alignment(v.as_str(), Point::new(600, y), white, Alignment::Right).draw(d)?;
        }

        // MUX switch (shows the requested mode; the rows above show the one in effect).
        let want = ads1256::mux();
        self.button(d, &BTN_MUX, if want { "MUX ON" } else { "MUX OFF" }, if want { GREEN } else { ACCENT })?;
        let hint = if want != av.mux {
            "switching..."
        } else if want {
            "AIN0/AIN1-AINCOM sampled too"
        } else {
            "AIN0-AIN1 only, continuous"
        };
        Text::new(hint, Point::new(BTN_MUX.top_left.x + BTN_MUX.size.width as i32 + 16, BTN_MUX.center().y + 6), small)
            .draw(d)?;

        let y = 350;
        if running {
            let mut s = Buf::<48>::new();
            if av.mux {
                let _ = write!(s, "Slot time max {} us, late slots {}", av.slot_us_max, av.late);
            } else {
                let _ = s.write_str("Continuous conversion, no slots");
            }
            Text::new(s.as_str(), Point::new(40, y), small).draw(d)?;
        } else {
            Text::new("No ADC: see src/ads1256.rs for wiring", Point::new(40, y), small).draw(d)?;
        }
        let mut w = Buf::<40>::new();
        let _ = write!(w, "Register write retries {}", av.write_retries);
        Text::new(w.as_str(), Point::new(40, y + 30), if av.write_retries > 0 { MonoTextStyle::new(&FONT_10X20, warn) } else { small })
            .draw(d)?;

        let note = MonoTextStyle::new(&FONT_6X10, LABEL);
        let mode = if av.mux {
            "MUX ON: 1 kHz slots convert AIN0-AIN1 (3750 SPS filter); every 10th also AIN0, 5 later AIN1 (15000)."
        } else {
            "MUX OFF: AIN0-AIN1 converts continuously with the 1000 SPS filter; registers written once."
        };
        for (k, line) in [
            mode,
            "Buffer on: inputs 0..3 V. AINCOM is tied to GND. Self-calibrated at gain 64.",
            "Pressure: 25 uV/mmHg (5 uV/V/mmHg at 5 V). Write retries > 0: check the SCLK wiring.",
        ]
        .iter()
        .enumerate()
        {
            Text::with_alignment(line, Point::new(WIDTH as i32 / 2, 430 + k as i32 * 16), note, Alignment::Center).draw(d)?;
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

/// Pressure figures over the last STATS_SAMPLES samples (mmHg, bpm).
struct Vitals {
    sys: f32,
    dia: f32,
    /// Mean of the waveform over the window (the window is not a whole number of beats, so
    /// this moves a little with the beat phase).
    map: f32,
    hr: Option<u32>,
}

fn vitals(head: u32, mmhg: &impl Fn(u32) -> f32) -> Option<Vitals> {
    if head < STATS_SAMPLES {
        return None;
    }
    let start = head - STATS_SAMPLES;
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
            hr = Some((beats - 1) * 60 * ads1256::DIFF_SPS / (last - first));
        }
    }
    Some(Vitals { sys, dia, map: sum / STATS_SAMPLES as f32, hr })
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
        let mag = libm::powf(10.0, libm::floorf(libm::log10f(raw)));
        let step = [1.0, 2.0, 5.0, 10.0].iter().map(|m| m * mag).find(|s| *s >= raw).unwrap_or(10.0 * mag);
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
        let span = PLOT_W as u32 * ads1256::DIFF_SPS / SWEEP_PX_PER_S;
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

        // Scrolling trace, rightmost column is "now". Each column covers 1000 / 150 = 6-7
        // samples and is drawn as one 2-px-wide vertical span from the previous column's last
        // sample through its own min..max (much cheaper than a thick Polyline).
        if have_trace {
            let first = |col: u32| head - (PLOT_W as u32 - col) * ads1256::DIFF_SPS / SWEEP_PX_PER_S;
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
        let vit = if running { vitals(head, &mmhg) } else { None };
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
        let mut a0 = Buf::<12>::new();
        let mut a1 = Buf::<12>::new();
        if running && av.mux {
            write_volts(&mut a0, av.uv[CH_AIN0], 4);
            write_volts(&mut a1, av.uv[CH_AIN1], 4);
        } else if running {
            let _ = a0.write_str("off");
            let _ = a1.write_str("off");
        } else {
            let _ = a0.write_str("--");
            let _ = a1.write_str("--");
        }
        readout(d, 0, "HR", hr.as_str(), "bpm")?;
        readout(d, 1, "SYS/DIA", sd.as_str(), "mmHg")?;
        readout(d, 2, "MAP", map.as_str(), "mmHg")?;
        readout(d, 3, "AIN0", a0.as_str(), "V")?;
        readout(d, 4, "AIN1", a1.as_str(), "V")?;

        // Beat indicator in the HR box: lit while the pressure is above the pulse midpoint.
        let beat = vit.as_ref().is_some_and(|v| v.hr.is_some() && mmhg(head - 1) > (v.sys + v.dia) / 2.0);
        Circle::new(Point::new(8 + READOUT_W as i32 - 30, READOUT_Y + 12), 18)
            .into_styled(PrimitiveStyle::with_fill(if beat { TRACE } else { GRID }))
            .draw(d)?;

        // ZERO
        self.button(d, &BTN_ZERO, "ZERO", if running { ACCENT } else { DISABLED })?;
        let mut z = Buf::<48>::new();
        let _ = match self.zero {
            None => z.write_str("Not zeroed: vent to air, press ZERO"),
            Some(c) => write!(z, "Zero offset {} uV", (c as f32 * ads1256::DIFF_UV_PER_CODE) as i32),
        };
        Text::new(z.as_str(), Point::new(BTN_ZERO.top_left.x + 136, BTN_ZERO.center().y + 6), small).draw(d)?;

        // Autoscale toggle
        let (label, fill) = if self.autoscale { ("AUTO", GREEN) } else { ("0-200", ACCENT) };
        self.button(d, &BTN_AUTO, label, fill)?;
        Ok(())
    }
}
