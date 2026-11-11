# ABP Simulator

Firmware in Rust ([Embassy](https://embassy.dev)) for the **STM32746G-EVAL** board (STM32F746NGH6, Cortex-M7, 216 MHz).

## Toolchain (Windows)

| Tool | Notes |
|---|---|
| rustup, stable, **`x86_64-pc-windows-gnu` host** | No MSVC Build Tools are installed, so the GNU host is used. C code in build scripts is compiled with MSYS2 `C:\msys64\ucrt64\bin\gcc.exe`. |
| target `thumbv7em-none-eabihf` | Pinned in `rust-toolchain.toml` |
| `probe-rs` | `cargo install probe-rs-tools --locked` (run from PowerShell, not Git Bash) |
| `flip-link` | `cargo install flip-link --locked`. Stack overflow protection, used as the linker |

## Build and run

```sh
cargo build            # debug build
cargo run              # flash + reset + stream defmt logs (probe-rs)
cargo run --release
```

Log verbosity is set by `DEFMT_LOG` in `.cargo/config.toml`.

## Board setup

- **Probe:** onboard ST-LINK/V2-1 (CN21). It had been converted to J-Link firmware; the original ST-LINK firmware was restored with SEGGER STLinkReflash, so probe-rs, STM32CubeProgrammer and the Virtual COM Port all work.
- **Power:** ST-LINK USB power (JP13 = STlk) only works **without the 5.7" LCD**. With the LCD connected, the board draws more than the USB limit, LD11 (overcurrent) turns on, and SWD fails. With the LCD, use 5 V into CN17, JP13 = PSU, JP15 removed.
- **Boot:** SW1 = 0 boots from Flash (normal). SW1 = 1 boots the system bootloader.
- **Ethernet (CN9):** JP6 must be fitted in the X4 position (25 MHz crystal for the DP83848 PHY). Without it the PHY still answers on MDIO but never gets a link. JP21/JP22 stay at their default (audio); the firmware bit-bangs MDIO on PG6/PG7.
- **Motor:** Novanta LMDCE571, factory IP 192.168.33.1/16, cabled straight to CN9. The board uses 192.168.33.10 and talks MCode over TCP port 503 (the motor ships with Modbus/TCP + MCode/TCP enabled; EtherNet/IP is not needed).
  The firmware expects Echo Mode EM=2 (no echo, no `>` prompt) and sets `EM=2` (RAM only) on every connect, because the motor may be left in another mode (e.g. by LMD Suite). With echo on, replies are out of step and the motor shows OFFLINE with `connection lost: Parse` in the log. Don't keep LMD Suite connected at the same time: it may switch the echo mode back.
- **ADC (ADS1256 breakout, 7.68 MHz, VREF 2.5 V) on SPI2:** SCLK PD3 (CN6-23), DIN PB15 (CN5-38), DOUT PB14 (CN5-35), CS PA4 (CN5-21), DRDY PC11 (CN6-53), PDWN and RESET tied to +3V3, 5 V from CN5-60. These pins are shared only with connectors that must stay empty: camera CN4, USB FS2 CN14 (no cable), microSD (no card). Configure it on the ADC tab.
- **LEDs used by the blinky:** LD1 green on PF10 (JP24 at 2-3), LD3 red on PB7 (JP23 at 2-3). Both are active-low.

## Clocks

HSE 25 MHz (X1), PLL to SYSCLK 216 MHz, AHB 216, APB1 54, APB2 108, PLLQ 48 MHz.

## Docs

- `doc/um1902-...pdf`: board user manual (UM1902)
- `doc/stm327x6g-eval_sch/`: Altium schematics and layout (MB1167 main board, MB1063 LCD, MB1183 camera)
- `doc/IMS-PO-LMDE-57R_A.pdf`: Liberty MDrive 57 mm (LMD 57) Ethernet product overview
- `doc/LMD-EIP_REV-F.pdf`: LMD Ethernet TCP/IP fieldbus software manual (MCode/TCP, Modbus/TCP, EtherNet/IP)
- `doc/ads1256.pdf`: TI ADS1255/ADS1256 24-bit delta-sigma ADC datasheet (SPI, 8 single-ended / 4 differential inputs)
