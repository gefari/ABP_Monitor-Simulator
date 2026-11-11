//! MFX I/O expander (STM32L152 running ST's MFX firmware) on I2C1 at 0x42.
//! Register map from ST's mfxstm32l152 BSP driver. GPIO13 is the DP83848 PWR_DOWN/INT pin
//! (MII_INT): held low, the PHY stays powered down.

use defmt::{info, warn};
use embassy_time::{block_for, Duration};

use crate::touch::Bus;

const ADDR: u8 = 0x42;
const REG_ID: u8 = 0x00;
const REG_SYS_CTRL: u8 = 0x40;
const REG_GPIO_STATE1: u8 = 0x10;
const REG_GPIO_DIR1: u8 = 0x60;
const REG_GPIO_TYPE1: u8 = 0x64;
const REG_GPO_SET1: u8 = 0x6C;
const SYS_CTRL_GPIO_EN: u8 = 0x01;
const MFX_ID: u8 = 0x7B;
/// GPIO13 is bit 5 of the second byte in each 3-byte GPIO register group.
const PHY_PWR: (u8, u8) = (1, 1 << 5);

fn wr(bus: &mut Bus, reg: u8, val: u8) {
    let _ = bus.blocking_write(ADDR, &[reg, val]);
}

fn rd(bus: &mut Bus, reg: u8) -> u8 {
    let mut b = [0u8; 1];
    let _ = bus.blocking_write_read(ADDR, &[reg], &mut b);
    b[0]
}

fn modify(bus: &mut Bus, reg: u8, f: impl FnOnce(u8) -> u8) {
    let v = rd(bus, reg);
    wr(bus, reg, f(v));
}

/// Drives the PHY's PWR_DOWN line high (push-pull output) so the PHY is never left powered down
/// by whatever state the expander comes up in.
pub fn phy_power_up(bus: &mut Bus) {
    let id = rd(bus, REG_ID);
    if id != MFX_ID {
        warn!("MFX: unexpected ID {:#04x}, PHY power pin left alone", id);
        return;
    }
    modify(bus, REG_SYS_CTRL, |v| v | SYS_CTRL_GPIO_EN);
    block_for(Duration::from_millis(5));

    let (byte, bit) = PHY_PWR;
    wr(bus, REG_GPO_SET1 + byte, bit);
    modify(bus, REG_GPIO_TYPE1 + byte, |v| v & !bit); // push-pull
    modify(bus, REG_GPIO_DIR1 + byte, |v| v | bit); // output
    block_for(Duration::from_millis(1));
    let high = rd(bus, REG_GPIO_STATE1 + byte) & bit != 0;
    info!("MFX: PHY PWR_DOWN driven {}", if high { "high (PHY on)" } else { "LOW?" });
}
