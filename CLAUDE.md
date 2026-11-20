# ABP Simulator – notes for Claude

Arterial blood pressure (ABP) simulator for testing patient monitors. Rust + Embassy firmware on the
STM32746G-EVAL (STM32F746NG). It drives a Novanta LMDCE571 stepper motor (pressure source) over
Ethernet/MCode and will read an ABP pressure transducer through a TI ADS1256 ADC.
Board setup, jumpers, wiring and toolchain: see `README.md`.

Git remote: `origin` = https://github.com/gefari/ABP_Monitor-Simulator (branch `main`, HTTPS).
The GitHub CLI `gh` is not installed on this PC; use plain `git push`/`git pull`.

## Working on the hardware

- Build: `cargo build` (first full build takes ~6.5 min on this PC; incremental is seconds).
- Flash + logs: `cargo run` (probe-rs, onboard ST-LINK). PowerShell, not Git Bash, for probe-rs installs.
- **Capturing logs by killing probe-rs freezes the firmware a few seconds later**: probe-rs leaves defmt
  RTT in blocking mode and the log buffer fills (`DEFMT_LOG=debug`). After any capture that kills
  probe-rs, run `probe-rs reset --chip STM32F746NGHx`. The user sees this as "firmware is halt".
- If probe-rs says "No connected probes", the ST-LINK USB is unplugged or unpowered; ask the user.
  A charge-only USB cable gives the same symptom while the ST-LINK LED is lit: check that
  `Get-PnpDevice -PresentOnly` lists VID_0483. "Target voltage (VAPP) is 0.00 V" = main board unpowered.
- probe-rs here is the prebuilt GitHub release (`cargo install probe-rs-tools` needs MSYS2 `dlltool`,
  which can't be installed because pacman fails TLS checks on this network). Smart App Control is off
  on this PC; it blocked cargo build scripts (os error 4551).
- To see logs without reflashing: `probe-rs attach --chip STM32F746NGHx target/thumbv7em-none-eabihf/debug/abp-simulator`
  (then reset after killing it, as above).
- Motor log shows `connection lost: Parse` with echoed commands → the motor's Echo Mode is not 2.
  `mcode::Client::connect` now forces `EM=2` on every connect (LMD Suite can change it).
  Don't keep LMD Suite connected to the motor at the same time.

## Current state (2026-11-19)

- Working on hardware: display, touch, Ethernet, motor run/stop (verified at 10 rpm).
- ADS1256 driver (`src/ads1256.rs`) + ADC tab in the UI: **verified on hardware 2026-11-19** (ID read,
  self-cal, steady 1000 SPS with no missed DRDY; input still unconnected, reads a few mV of drift).
  Pinout in `src/ads1256.rs` and README. Runs on a high-priority `InterruptExecutor`
  (UART4 IRQ) because frame draws block the main executor ~20 ms.
  `ADS1256 not found (STATUS 0x00)` means DOUT never driven: module unpowered (5 V from CN5-60) or wiring.
- ABP transducer on AIN0-AIN1 (gain 64), AINCOM tied to GND. Scaling 5 uV/V/mmHg at 5 V excitation
  = 25 uV/mmHg; ZERO button on the Monitor tab. Monitor shows the measured waveform (autoscale with
  AUTO button, or fixed 0-200 mmHg), time window 100 ms..12 s with time axis, SYS/DIA, MAP, HR
  (4 s window), voltages of the pair's inputs vs COM.
- ADC settings, changed at runtime on the ADC tab (`ads1256::set_settings`; any change restarts with a
  new self-cal): differential pair IN+/IN- (AIN0..7, COM; default AIN0-AIN1), gain 1..64 (default 64),
  rate 5..7500 SPS (MUX OFF only, default 1000), SPI SCLK 1688k/844k/422k/211k (MUX OFF only; MUX ON
  forced to 1688k because its slots overrun at 844k), and the mode:
  - MUX OFF: the pair only, continuous conversion at the selected rate (registers written once).
  - MUX ON: 1 kHz slots, the pair at DRATE 3750 + IN+/IN- vs AINCOM at 100 SPS each.
  Verified on hardware 2026-11-20: 5, 1000, 7500 SPS (7500: UI drops to ~12 fps), MUX ON slot max
  ~956 us of 1000. Diff samples are stored scaled to gain 64 (`code << (6 - pga)`), so pressure
  scale and ZERO survive gain changes. ADC tab shows Value / Peak-Peak / StdDev (diff in uV,
  single-ended in mV) over the Monitor window.
- Hardware issue: ~7.5 % of SPI register writes get corrupted (SCLK ringing on jumper wires), so writes
  are verified and retried (counter on the ADC tab). Hurts MUX ON only. Fix: shorter wires, GND next to
  SCLK, 33-100 ohm series resistor at CN6-23. After a later re-wiring a 40 s run showed 0 retries.
- Change ADC settings without touching the screen: `probe-rs write --chip STM32F746NGHx b32 <addr> <word>`
  with the address of `ads1256::SETTINGS` from `llvm-nm` (in `~/.rustup/toolchains/*/lib/rustlib/*/bin`).
  Word = mux | pos<<4 | neg<<8 | pga<<12 | rate_index<<16 | spi_index<<20; default 0x00096100. `probe-rs reset`
  did NOT restore the default here (value survived), so write the default back after testing.
- SCLK speed findings (2026-11-20): slower SCLK did NOT reduce write corruption (more retries at 844k
  than 1688k). After an SCLK change the chip is sometimes a bit out of step (STATUS 0x30 read as 0x60);
  `run()` waits 32 DRDY periods (datasheet serial-interface timeout) and retries, but switching to 211k
  still took up to ~10 s once. Rates verified: 844k @ 7500 SPS, 211k @ 2000 SPS.
- DMA evaluated and not used: transfers are 1-5 bytes, t6/t10/t11 waits stay busy-waits, MUX ON slots
  (~956 us of 1000) would get slower, and D-cache is on with no cache maintenance in embassy SPI DMA
  (buffers would need DTCM or non-cacheable SDRAM). Worth it only for 15-30 kSPS: RDATAC + 3-byte DMA
  read started from the DRDY EXTI handler.
- Firmware version (Cargo.toml) + git commit (`build.rs`, `GIT_VERSION`; `*` = dirty) shown top right
  and in the boot log. build.rs finds git at `C:/Program Files/Git/cmd/git.exe` (not on PowerShell PATH).
- `JtagGetIdcodeError` with the board powered (LCD blank, chip not running) cleared after the user
  reworked the ADS1256 wiring/power; check the module wiring and power-up order first when it appears.

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
