# ABP Simulator – notes for Claude

Arterial blood pressure (ABP) simulator for testing patient monitors. Rust + Embassy firmware on the
STM32746G-EVAL (STM32F746NG). It drives a Novanta LMDCE571 stepper motor (pressure source) over
Ethernet/MCode and will read an ABP pressure transducer through a TI ADS1256 ADC.
Board setup, jumpers, wiring and toolchain: see `README.md`.

## Working on the hardware

- Build: `cargo build` (first full build takes ~6.5 min on this PC; incremental is seconds).
- Flash + logs: `cargo run` (probe-rs, onboard ST-LINK). PowerShell, not Git Bash, for probe-rs installs.
- **Capturing logs by killing probe-rs freezes the firmware a few seconds later**: probe-rs leaves defmt
  RTT in blocking mode and the log buffer fills (`DEFMT_LOG=debug`). After any capture that kills
  probe-rs, run `probe-rs reset --chip STM32F746NGHx`. The user sees this as "firmware is halt".
- If probe-rs says "No connected probes", the ST-LINK USB is unplugged or unpowered; ask the user.
- Motor log shows `connection lost: Parse` with echoed commands → the motor's Echo Mode is not 2.
  `mcode::Client::connect` now forces `EM=2` on every connect (LMD Suite can change it).
  Don't keep LMD Suite connected to the motor at the same time.

## Current state (2026-11-11)

- Working on hardware: display, touch, Ethernet, motor run/stop (verified at 10 rpm).
- ADS1256 driver (`src/ads1256.rs`) + ADC tab in the UI: **built, never run on hardware** (probe was
  disconnected). Pinout in `src/ads1256.rs` and README. Runs on a high-priority `InterruptExecutor`
  (UART4 IRQ) because frame draws block the main executor ~20 ms.
- Monitor tab waveform is a placeholder (two Gaussians, fixed 72 bpm 120/80), not driven by the ADC.
- **Next step from the user**: they will say which ADS1256 input the ABP transducer is on; then set
  inputs/gain/scaling (mmHg) and show the measured pressure on the Monitor tab.

## Regulatory documentation (IEC 62304)

- Software Architecture Document draft 0.1 lives online as a Claude Doc:
  https://claude.ai/artifact/BWkcnFKXnUmPFXYZ2ug1zm (tab "Proposed requirements" holds 37 draft SRS
  requirements traced to the architecture). Local exports in `doc/SAD/` (.md with `images/`, .docx, .pdf);
  the online doc is the master, re-export after edits.
- IDs used there: software items SI-01..SI-11 (one per module, SI-11 = waveform in `ui.rs`),
  interfaces IF-xx, risk controls RC-01..RC-10, requirements SRS-MOT/ADC/WAV/UI/PLT-nn.
  Keep code changes and these IDs consistent; when adding a module, add a software item.
- Proposed safety class B (not yet confirmed). Missing: SRS, risk analysis, watchdog, motor fail-safe on
  comms loss, calibration, pressure control loop (see "Gaps in the architecture" in the SAD).

## Conventions

- Comments explain hardware facts with sources (UM1902 table, datasheet section, measured on the board).
- No `unsafe` without a `// SAFETY:` comment.
