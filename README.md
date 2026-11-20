# ABP Simulator

Firmware in Rust ([Embassy](https://embassy.dev)) for the **STM32746G-EVAL** board (STM32F746NGH6, Cortex-M7, 216 MHz).

## Toolchain (Windows)

| Tool | Notes |
|---|---|
| rustup, stable, **`x86_64-pc-windows-gnu` host** | No MSVC Build Tools are installed, so the GNU host is used. C code in build scripts is compiled with MSYS2 `C:\msys64\ucrt64\bin\gcc.exe`. |
| target `thumbv7em-none-eabihf` | Pinned in `rust-toolchain.toml` |
| `probe-rs` | `cargo install probe-rs-tools --locked` (run from PowerShell, not Git Bash). If that fails for lack of MSYS2 `dlltool`, use the prebuilt Windows release from GitHub. |
| `flip-link` | `cargo install flip-link --locked`. Stack overflow protection, used as the linker |

## Build and run

```sh
cargo build            # debug build
cargo run              # flash + reset + stream defmt logs (probe-rs)
cargo run --release
```

Log verbosity is set by `DEFMT_LOG` in `.cargo/config.toml`.

If you stop `cargo run` / `probe-rs` by killing it, the firmware freezes a few seconds later (defmt RTT is left in blocking mode and the log buffer fills). Run `probe-rs reset --chip STM32F746NGHx` afterwards. To read logs without reflashing: `probe-rs attach --chip STM32F746NGHx target/thumbv7em-none-eabihf/debug/abp-simulator`.

## Board setup

- **Probe:** onboard ST-LINK/V2-1 (CN21). It had been converted to J-Link firmware; the original ST-LINK firmware was restored with SEGGER STLinkReflash, so probe-rs, STM32CubeProgrammer and the Virtual COM Port all work.
- **Power:** ST-LINK USB power (JP13 = STlk) only works **without the 5.7" LCD**. With the LCD connected, the board draws more than the USB limit, LD11 (overcurrent) turns on, and SWD fails. With the LCD, use 5 V into CN17, JP13 = PSU, JP15 removed.
- **Boot:** SW1 = 0 boots from Flash (normal). SW1 = 1 boots the system bootloader.
- **Ethernet (CN9):** JP6 must be fitted in the X4 position (25 MHz crystal for the DP83848 PHY). Without it the PHY still answers on MDIO but never gets a link. JP21/JP22 stay at their default (audio); the firmware bit-bangs MDIO on PG6/PG7.
- **Motor:** Novanta LMDCE571, factory IP 192.168.33.1/16, cabled straight to CN9. The board uses 192.168.33.10 and talks MCode over TCP port 503 (the motor ships with Modbus/TCP + MCode/TCP enabled; EtherNet/IP is not needed).
  The firmware expects Echo Mode EM=2 (no echo, no `>` prompt) and sets `EM=2` (RAM only) on every connect, because the motor may be left in another mode (e.g. by LMD Suite). With echo on, replies are out of step and the motor shows OFFLINE with `connection lost: Parse` in the log. Don't keep LMD Suite connected at the same time: it may switch the echo mode back.
- **ADC (ADS1256 breakout, 7.68 MHz, VREF 2.5 V) on SPI2:** SCLK PD3 (CN6-23), DIN PB15 (CN5-38), DOUT PB14 (CN5-35), CS PA4 (CN5-21), DRDY PC11 (CN6-53), PDWN and RESET tied to +3V3, 5 V from CN5-60. These pins are shared only with connectors that must stay empty: camera CN4, USB FS2 CN14 (no cable), microSD (no card). Configure it on the ADC tab.
  - Pressure transducer: bridge output on AIN0 (+) / AIN1 (-), AINCOM tied to GND. With the input buffer on, the inputs must stay within 0..3 V (AVDD - 2 V); a 5 V-excited bridge sits at about 2.4 V.
  - Keep the SPI wires short. On jumper wires, ringing on SCLK corrupts some register writes (the firmware reads every write back and retries; the count is on the ADC tab). Helps: a GND wire next to SCLK, a 33-100 Ω series resistor at CN6-23.
  - If SWD stops connecting (`JtagGetIdcodeError`) while the LCD stays blank, check the ADC module wiring first: a misplaced jumper wire can stop the MCU.
- **LEDs used by the blinky:** LD1 green on PF10 (JP24 at 2-3), LD3 red on PB7 (JP23 at 2-3). Both are active-low.

## Screen

Three tabs in the header. The firmware version and git commit are shown at the top right (`*` after the commit = built with uncommitted changes).

### Monitor

- Scrolling arterial pressure waveform from the ADS1256 differential input, in mmHg. Scaling: 5 µV/V/mmHg at 5 V excitation = 25 µV/mmHg (standard disposable transducers, AAMI BP22).
- Time axis under the trace. The **window** button cycles 100 ms → 1 s → 2 s → 4 s → 12 s. At data rates above ~1000 SPS the window is limited to what the sample buffer holds (16384 samples); the axis shows the real span.
- **AUTO**: vertical scale follows the visible trace (1-2-5 grid, at least 1 mmHg per division). Tap for the fixed 0-200 mmHg scale.
- Readouts: HR, SYS/DIA, MAP (from the last 4 s), and the voltages of the pair's two inputs vs GND (MUX ON only).
- **ZERO**: vent the transducer to air and tap; the mean of the last second becomes 0 mmHg.

### Motor Test

Speed setpoint with − / + (tap = 10 rpm, hold to repeat), Start and Stop.

### ADC

- Settings row. Tap the left half of a button to step down, the right half to step up. Any change restarts the ADC with a new self-calibration ("Applying..." meanwhile).

  | Button | Range | Default |
  |---|---|---|
  | IN+ / IN- | AIN0..AIN7, COM (never the same) | AIN0 / AIN1 |
  | GAIN (diff) | x1..x64 | x64 (±78 mV full scale) |
  | RATE | 5..7500 SPS, MUX OFF only | 1000 SPS |
  | SPI SCLK | 1688k, 844k, 422k, 211k; MUX OFF only (MUX ON uses 1688k) | 1688k |
  | MUX | OFF / ON | OFF |

- **MUX OFF**: only the differential pair, converting continuously at the selected rate.
- **MUX ON**: 1 kHz slots. Every slot converts the pair (3750 SPS filter, so samples are exactly 1 ms apart); every 10th slot also IN+ vs COM, 5 slots later IN- vs COM (gain 1, 100 SPS each). Slot timing and late slots are shown.
- Table per channel: inputs, gain, measured rate, last value, **Peak-Peak** and **StdDev** over the Monitor window. Differential values in µV, single-ended in mV.
- At 7500 SPS the screen drops to about 12 fps; up to 1000 SPS it runs at 30 fps.

## Clocks

HSE 25 MHz (X1), PLL to SYSCLK 216 MHz, AHB 216, APB1 54, APB2 108, PLLQ 48 MHz.

## Docs

- `doc/um1902-...pdf`: board user manual (UM1902)
- `doc/stm327x6g-eval_sch/`: Altium schematics and layout (MB1167 main board, MB1063 LCD, MB1183 camera)
- `doc/IMS-PO-LMDE-57R_A.pdf`: Liberty MDrive 57 mm (LMD 57) Ethernet product overview
- `doc/LMD-EIP_REV-F.pdf`: LMD Ethernet TCP/IP fieldbus software manual (MCode/TCP, Modbus/TCP, EtherNet/IP)
- `doc/ads1256.pdf`: TI ADS1255/ADS1256 24-bit delta-sigma ADC datasheet (SPI, 8 single-ended / 4 differential inputs)
