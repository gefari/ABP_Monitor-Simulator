//! Screen: a header with two tabs.
//! - Monitor: scrolling arterial-pressure waveform with grid, numeric readouts. The waveform is
//!   a placeholder shape (two Gaussians), not a physiological model yet.
//! - Motor Test: speed setpoint with -/+ (tap = 10 rpm, hold to repeat), Start and Stop.
//! - ADC: ADS1256 settings (inputs, gain, data rate, buffer) with -/+, and the live reading.

use core::fmt::Write;

use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle};
use embedded_graphics::text::{Alignment, Text};
use embassy_time::{Duration, Instant};

use crate::ads1256::{self, AINCOM, GAINS, RATES};
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
const P_MAX: f32 = 200.0; // mmHg at top of panel
const SWEEP_PX_PER_S: f32 = 150.0;

// Header tabs
const TAB_MONITOR: Rectangle = Rectangle::new(Point::new(280, 6), Size::new(110, 32));
const TAB_MOTOR: Rectangle = Rectangle::new(Point::new(400, 6), Size::new(110, 32));
const TAB_ADC: Rectangle = Rectangle::new(Point::new(520, 6), Size::new(110, 32));

// ADC page: one row per setting, each with - value +
const ADC_ROWS: usize = 5;
const ADC_ROW_Y: i32 = 96;
const ADC_ROW_PITCH: i32 = 56;
const ADC_BTN: Size = Size::new(60, 46);
const ADC_LABELS: [&str; ADC_ROWS] = ["Input +", "Input -", "Gain", "Data rate", "Buffer"];

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

pub struct Vitals {
    pub hr_bpm: f32,
    pub sys: f32,
    pub dia: f32,
}

#[derive(Clone, Copy, PartialEq)]
enum Tab {
    Monitor,
    Motor,
    Adc,
}

pub struct Ui {
    tab: Tab,
    rpm: i32,
    /// ADS1256 settings as chosen here (sent to the ADC task on every change).
    adc: ads1256::Config,
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
        Ui { tab: Tab::Monitor, rpm: RPM_DEFAULT, adc: ads1256::Config::DEFAULT, touch: None, next_repeat: None }
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
        if self.tab == Tab::Adc {
            if pressed {
                self.adc_press(p);
            }
            return;
        }
        if self.tab != Tab::Motor {
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

    pub fn draw<D: DrawTarget<Color = Rgb565>>(&self, d: &mut D, v: &Vitals, t: f32) -> Result<(), D::Error> {
        d.clear(BG)?;
        Rectangle::new(Point::zero(), Size::new(WIDTH as u32, HEADER_H))
            .into_styled(PrimitiveStyle::with_fill(HEADER))
            .draw(d)?;
        Text::new("ABP Simulator", Point::new(14, 29), MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE)).draw(d)?;
        self.tab_button(d, &TAB_MONITOR, "Monitor", Tab::Monitor)?;
        self.tab_button(d, &TAB_MOTOR, "Motor Test", Tab::Motor)?;
        self.tab_button(d, &TAB_ADC, "ADC", Tab::Adc)?;
        match self.tab {
            Tab::Monitor => draw_monitor(d, v, t),
            Tab::Motor => self.draw_motor(d),
            Tab::Adc => self.draw_adc(d),
        }
    }

    /// Steps the setting under the finger (- or + of its row) and sends the new settings.
    fn adc_press(&mut self, p: Option<Point>) {
        let Some((row, step)) = (0..ADC_ROWS).find_map(|r| {
            let (minus, _, plus) = adc_row(r);
            if hit(&minus, p) {
                Some((r, -1))
            } else if hit(&plus, p) {
                Some((r, 1))
            } else {
                None
            }
        }) else {
            return;
        };
        let stepped = |v: u8, count: usize| (v as i32 + step).clamp(0, count as i32 - 1) as u8;
        let old = self.adc;
        let c = &mut self.adc;
        match row {
            0 => c.pos = stepped(c.pos, 8),
            1 => c.neg = stepped(c.neg, AINCOM as usize + 1),
            2 => c.gain = stepped(c.gain, GAINS.len()),
            3 => c.rate = stepped(c.rate, RATES.len()),
            _ => c.buffer = !c.buffer,
        }
        if self.adc != old {
            ads1256::configure(self.adc);
        }
    }

    fn draw_adc<D: DrawTarget<Color = Rgb565>>(&self, d: &mut D) -> Result<(), D::Error> {
        let av = ads1256::view();
        let small = MonoTextStyle::new(&FONT_10X20, LABEL);
        let white = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);

        Text::new("ADS1256  SPI2", Point::new(40, 80), small).draw(d)?;
        let (state, color) = match av.status {
            ads1256::Status::Starting => ("STARTING", LABEL),
            ads1256::Status::NotFound => ("NOT FOUND", Rgb565::new(31, 40, 0)),
            ads1256::Status::NoDrdy => ("NO DRDY", Rgb565::new(31, 40, 0)),
            ads1256::Status::Running => ("RUNNING", Rgb565::new(8, 60, 8)),
        };
        Text::with_alignment(state, Point::new(600, 80), MonoTextStyle::new(&FONT_10X20, color), Alignment::Right).draw(d)?;

        let c = &self.adc;
        for (r, label) in ADC_LABELS.iter().enumerate() {
            let (minus, value, plus) = adc_row(r);
            Text::new(label, Point::new(40, minus.top_left.y + 30), small).draw(d)?;
            let mut v = Buf::<16>::new();
            let (at_min, at_max) = match r {
                0 => {
                    write_input(&mut v, c.pos);
                    (c.pos == 0, c.pos == 7)
                }
                1 => {
                    write_input(&mut v, c.neg);
                    (c.neg == 0, c.neg == AINCOM)
                }
                2 => {
                    let _ = write!(v, "x{}", c.gain());
                    (c.gain == 0, c.gain as usize == GAINS.len() - 1)
                }
                3 => {
                    let _ = write!(v, "{} SPS", RATES[c.rate as usize].0);
                    (c.rate == 0, c.rate as usize == RATES.len() - 1)
                }
                _ => {
                    let _ = v.write_str(if c.buffer { "ON" } else { "OFF" });
                    (false, false)
                }
            };
            self.button(d, &minus, "", if at_min { DISABLED } else { ACCENT })?;
            self.button(d, &plus, "", if at_max { DISABLED } else { ACCENT })?;
            bar(d, minus.center(), 24, 6)?;
            bar(d, plus.center(), 24, 6)?;
            bar(d, plus.center(), 6, 24)?;
            RoundedRectangle::with_equal_corners(value, Size::new(8, 8))
                .into_styled(PrimitiveStyleBuilder::new().stroke_color(LABEL).stroke_width(2).fill_color(HEADER).build())
                .draw(d)?;
            centered(d, &value, v.as_str(), Rgb565::WHITE)?;
        }

        // Live reading
        let y = ADC_ROW_Y + ADC_ROWS as i32 * ADC_ROW_PITCH + 24;
        if av.status == ads1256::Status::Running {
            let mut s = Buf::<24>::new();
            let (sign, uv) = (if av.uv < 0 { "-" } else { "+" }, av.uv.unsigned_abs());
            let _ = write!(s, "{}{}.{:06} V", sign, uv / 1_000_000, uv % 1_000_000);
            Text::new(s.as_str(), Point::new(40, y), white).draw(d)?;
            let mut k = Buf::<40>::new();
            let _ = write!(k, "code {}  {} SPS read", av.code, av.sps);
            Text::with_alignment(k.as_str(), Point::new(600, y), small, Alignment::Right).draw(d)?;
            if av.config != self.adc {
                Text::new("applying...", Point::new(40, y + 28), small).draw(d)?;
            }
        } else {
            Text::new("No ADC: see src/ads1256.rs for wiring", Point::new(40, y), small).draw(d)?;
        }
        Text::with_alignment(
            "Range +-5 V / gain. Buffer on: inputs 0..3 V. Each change re-runs self-calibration.",
            Point::new(WIDTH as i32 / 2, 470),
            MonoTextStyle::new(&FONT_6X10, LABEL),
            Alignment::Center,
        )
        .draw(d)?;
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

/// The -, value and + rectangles of ADC settings row `r`.
fn adc_row(r: usize) -> (Rectangle, Rectangle, Rectangle) {
    let y = ADC_ROW_Y + r as i32 * ADC_ROW_PITCH;
    (
        Rectangle::new(Point::new(250, y), ADC_BTN),
        Rectangle::new(Point::new(320, y), Size::new(160, ADC_BTN.height)),
        Rectangle::new(Point::new(490, y), ADC_BTN),
    )
}

/// ADS1256 input name: AIN0..AIN7 or AINCOM.
fn write_input<const N: usize>(b: &mut Buf<N>, i: u8) {
    let _ = if i == AINCOM { b.write_str("AINCOM") } else { write!(b, "AIN{}", i) };
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

fn gauss(x: f32, c: f32, w: f32) -> f32 {
    let d = (x - c) / w;
    libm::expf(-0.5 * d * d)
}

/// Placeholder ABP shape: systolic peak + dicrotic wave, scaled between dia and sys.
fn pressure(v: &Vitals, t: f32) -> (f32, f32) {
    let phase = libm::fmodf(t * v.hr_bpm / 60.0, 1.0);
    let shape = gauss(phase, 0.15, 0.06) + 0.35 * gauss(phase, 0.40, 0.05);
    (v.dia + (v.sys - v.dia) * shape.min(1.0), phase)
}

fn y_of(p: f32) -> i32 {
    PLOT_Y + PLOT_H - (p / P_MAX * PLOT_H as f32) as i32
}

fn readout<D: DrawTarget<Color = Rgb565>>(d: &mut D, x: i32, label: &str, value: &str, unit: &str) -> Result<(), D::Error> {
    let style = PrimitiveStyleBuilder::new().stroke_color(LABEL).stroke_width(2).fill_color(HEADER).build();
    RoundedRectangle::with_equal_corners(Rectangle::new(Point::new(x, 330), Size::new(180, 120)), Size::new(12, 12))
        .into_styled(style)
        .draw(d)?;
    let small = MonoTextStyle::new(&FONT_10X20, LABEL);
    let big = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    Text::new(label, Point::new(x + 14, 360), small).draw(d)?;
    Text::with_alignment(value, Point::new(x + 90, 400), big, Alignment::Center).draw(d)?;
    Text::with_alignment(unit, Point::new(x + 90, 430), small, Alignment::Center).draw(d)?;
    Ok(())
}

fn draw_monitor<D: DrawTarget<Color = Rgb565>>(d: &mut D, v: &Vitals, t: f32) -> Result<(), D::Error> {
    // Grid every 40 mmHg with labels
    let label_style = MonoTextStyle::new(&FONT_6X10, LABEL);
    for p in (0..=200).step_by(40) {
        let y = y_of(p as f32);
        Line::new(Point::new(PLOT_X, y), Point::new(PLOT_X + PLOT_W as i32 - 1, y))
            .into_styled(PrimitiveStyle::with_stroke(GRID, 1))
            .draw(d)?;
        let mut s = Buf::<4>::new();
        let _ = write!(s, "{}", p);
        Text::with_alignment(s.as_str(), Point::new(PLOT_X - 8, y + 4), label_style, Alignment::Right).draw(d)?;
    }
    Text::new("mmHg", Point::new(8, PLOT_Y - 8), label_style).draw(d)?;

    // Scrolling trace, rightmost column is "now". Drawn as one 2-px-wide vertical span per
    // column joining the previous sample (much cheaper than a thick Polyline).
    let mut prev = y_of(pressure(v, t - PLOT_W as f32 / SWEEP_PX_PER_S).0);
    for i in 0..PLOT_W {
        let ts = t - (PLOT_W - 1 - i) as f32 / SWEEP_PX_PER_S;
        let y = y_of(pressure(v, ts).0);
        let (top, bot) = (prev.min(y), prev.max(y));
        d.fill_solid(
            &Rectangle::new(Point::new(PLOT_X + i as i32, top), Size::new(2, (bot - top + 2) as u32)),
            TRACE,
        )?;
        prev = y;
    }

    // Readouts
    let map = v.dia + (v.sys - v.dia) / 3.0;
    let mut hr = Buf::<8>::new();
    let _ = write!(hr, "{}", v.hr_bpm as u32);
    let mut sd = Buf::<12>::new();
    let _ = write!(sd, "{}/{}", v.sys as u32, v.dia as u32);
    let mut m = Buf::<8>::new();
    let _ = write!(m, "{}", map as u32);
    readout(d, 20, "HR", hr.as_str(), "bpm")?;
    readout(d, 230, "SYS/DIA", sd.as_str(), "mmHg")?;
    readout(d, 440, "MAP", m.as_str(), "mmHg")?;

    // Beat indicator: lit during systole.
    let (_, phase) = pressure(v, t);
    let heart = if phase < 0.3 { TRACE } else { GRID };
    Circle::new(Point::new(170, 342), 18)
        .into_styled(PrimitiveStyle::with_fill(heart))
        .draw(d)?;
    Ok(())
}
