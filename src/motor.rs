//! Novanta IMS LMDCE571 (Liberty MDrive, NEMA 23) over MCode/TCP.
//!
//! The UI sends `Command`s (run at a speed, stop); this task turns them into MCode slew
//! commands (`SL`) and publishes a `View` of the motor state. Settings written here (A, D, VM)
//! live in the motor's RAM only; a power cycle restores the saved values.
//! Speeds and positions on the wire are in microsteps (MS=256 -> 51200 per revolution).
//!
//! `stop` is a normal decelerated stop over the network, not an emergency stop.

use core::cell::Cell;

use defmt::{info, warn, Format};
use embassy_futures::select::{select, Either};
use embassy_net::Stack;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

use crate::mcode::{Client, Error};
use crate::net::MOTOR_IP;

/// Microsteps per revolution at the factory MS=256 (1.8 degree motor, 200 full steps).
const STEPS_PER_REV: i32 = 200 * 256;
/// Speed range offered by the UI.
pub const MIN_RPM: i32 = 10;
pub const MAX_RPM: i32 = 300;
/// Accel = decel: 5 rev/s^2, so 0 -> 300 rpm takes 1 s.
const ACCEL: i32 = 5 * STEPS_PER_REV;
const POLL: Duration = Duration::from_millis(250);

#[derive(Clone, Copy, PartialEq, Format)]
pub enum Command {
    /// Turn continuously at this speed (positive = clockwise as seen by the motor's convention).
    Run { rpm: i32 },
    Stop,
}

/// Motor state for the UI.
#[derive(Clone, Copy, Default)]
pub struct View {
    pub connected: bool,
    /// Speed last commanded (None = stopped).
    pub commanded_rpm: Option<i32>,
    /// Measured speed and position from the motor.
    pub rpm: i32,
    pub position: i32,
    /// Last error code the motor reported (ER), 0 if none since connecting.
    pub error: i32,
}

static VIEW: Mutex<CriticalSectionRawMutex, Cell<View>> = Mutex::new(Cell::new(View {
    connected: false,
    commanded_rpm: None,
    rpm: 0,
    position: 0,
    error: 0,
}));
static COMMAND: Signal<CriticalSectionRawMutex, Command> = Signal::new();

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

/// Sends a command to the motor task. Ignored while the motor is offline; if several arrive
/// before the task gets to them, only the latest counts.
pub fn command(cmd: Command) {
    if view().connected {
        COMMAND.signal(cmd);
    }
}

fn rpm_to_steps(rpm: i32) -> i32 {
    rpm * STEPS_PER_REV / 60
}

fn steps_to_rpm(steps: i32) -> i32 {
    // Round to nearest.
    (steps * 60 + steps.signum() * STEPS_PER_REV / 2) / STEPS_PER_REV
}

/// Logs the motor's identity and motion settings.
async fn identify(m: &mut Client<'_>) -> Result<(), Error> {
    for (label, cmd) in [
        ("part number PN", "PR PN"),
        ("serial SN", "PR SN"),
        ("firmware VR", "PR VR"),
        ("temperature IT (C)", "PR IT"),
        ("supply VT", "PR VT"),
    ] {
        let line = m.query(cmd).await?;
        info!("  {=str}: {=str}", label, line);
    }
    let [rc, hc, ms, de, em] = m.get_many(["RC", "HC", "MS", "DE", "EM"]).await?;
    info!("  run current RC {}%, hold current HC {}%, microsteps MS {}, drive enable DE {}, echo mode EM {}", rc, hc, ms, de, em);
    if ms != 256 {
        warn!("motor: MS is {}, speeds shown assume 256", ms);
    }
    Ok(())
}

/// Brings the motor to a known state on (re)connect: stopped, with our accel/decel.
async fn setup(m: &mut Client<'_>) -> Result<(), Error> {
    m.command("SL 0").await?;
    m.set("A", ACCEL).await?;
    m.set("D", ACCEL).await
}

async fn execute(m: &mut Client<'_>, cmd: Command) -> Result<(), Error> {
    info!("motor: {}", cmd);
    match cmd {
        Command::Run { rpm } => {
            let rpm = rpm.clamp(-MAX_RPM, MAX_RPM);
            let steps = rpm_to_steps(rpm);
            // VM caps the slew speed; keep it at (or above) the requested speed.
            m.set("VM", steps.abs().max(rpm_to_steps(MIN_RPM))).await?;
            m.command_arg("SL ", steps).await?;
            update_view(|v| v.commanded_rpm = Some(rpm));
        }
        Command::Stop => {
            m.command("SL 0").await?;
            update_view(|v| v.commanded_rpm = None);
        }
    }
    Ok(())
}

/// Reads speed, position and the error flag into the view.
async fn poll(m: &mut Client<'_>) -> Result<(), Error> {
    let [p, v, ef] = m.get_many(["P", "V", "EF"]).await?;
    update_view(|view| {
        view.position = p;
        view.rpm = steps_to_rpm(v);
    });
    if ef != 0 {
        let er = m.get("ER").await?; // reading ER clears EF
        warn!("motor: error flag set, ER = {}", er);
        update_view(|view| view.error = er);
    }
    Ok(())
}

/// Keeps an MCode/TCP connection to the motor and carries out UI commands. On every
/// (re)connect the motor is stopped first. Reconnects on failure.
#[embassy_executor::task]
pub async fn task(stack: Stack<'static>) {
    let (mut rx, mut tx) = ([0u8; 1024], [0u8; 1024]);
    loop {
        stack.wait_link_up().await;
        match Client::connect(stack, MOTOR_IP, &mut rx, &mut tx).await {
            Ok(mut m) => {
                info!("motor: MCode/TCP connected to {}:{}", MOTOR_IP, crate::mcode::PORT);
                let mut res = match identify(&mut m).await {
                    Ok(()) => setup(&mut m).await,
                    e => e,
                };
                COMMAND.reset(); // drop commands made while offline
                if res.is_ok() {
                    update_view(|v| *v = View { connected: true, ..View::default() });
                }
                let mut last_log = Instant::now();
                while res.is_ok() {
                    res = match select(Timer::after(POLL), COMMAND.wait()).await {
                        Either::First(()) => poll(&mut m).await,
                        Either::Second(cmd) => execute(&mut m, cmd).await,
                    };
                    // A rejected command is reported, not a reason to drop the connection.
                    if let Err(Error::Rejected(er)) = res {
                        warn!("motor: command rejected, ER = {}", er);
                        update_view(|v| v.error = er);
                        res = Ok(());
                    }
                    let v = view();
                    if v.commanded_rpm.is_some() && last_log.elapsed() >= Duration::from_secs(2) {
                        info!("motor: {} rpm, P {}", v.rpm, v.position);
                        last_log = Instant::now();
                    }
                }
                update_view(|v| v.connected = false);
                warn!("motor: connection lost: {}", res.unwrap_err());
                m.close().await;
            }
            Err(e) => warn!("motor: connect to {} failed: {}", MOTOR_IP, e),
        }
        Timer::after_secs(3).await;
    }
}
