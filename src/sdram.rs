//! External SDRAM on the STM32746G-EVAL: IS42S32800G-6 (8M x 32 = 32 MiB),
//! wired to FMC SDRAM bank 1 (SDNE0/SDCKE0), mapped at 0xC000_0000.
//! Pinout: UM1902 tables 26/27/37, schematic MB1167 sheet 4.

use embassy_stm32::fmc::Fmc;
use embassy_stm32::peripherals::*;
use embassy_stm32::Peri;
use embassy_time::Delay;
use stm32_fmc::devices::is42s32800g_6::Is42s32800g;

pub const BASE: usize = 0xC000_0000;
pub const SIZE_BYTES: usize = 32 * 1024 * 1024;
pub const SIZE_WORDS: usize = SIZE_BYTES / 4;

/// All FMC pins used by the SDRAM. Grouped so `main` can hand them over in one go.
pub struct SdramPins {
    pub fmc: Peri<'static, FMC>,
    // A0..A11
    pub a: (
        Peri<'static, PF0>, Peri<'static, PF1>, Peri<'static, PF2>, Peri<'static, PF3>,
        Peri<'static, PF4>, Peri<'static, PF5>, Peri<'static, PF12>, Peri<'static, PF13>,
        Peri<'static, PF14>, Peri<'static, PF15>, Peri<'static, PG0>, Peri<'static, PG1>,
    ),
    // BA0, BA1
    pub ba: (Peri<'static, PG4>, Peri<'static, PG5>),
    // D0..D15
    pub d_lo: (
        Peri<'static, PD14>, Peri<'static, PD15>, Peri<'static, PD0>, Peri<'static, PD1>,
        Peri<'static, PE7>, Peri<'static, PE8>, Peri<'static, PE9>, Peri<'static, PE10>,
        Peri<'static, PE11>, Peri<'static, PE12>, Peri<'static, PE13>, Peri<'static, PE14>,
        Peri<'static, PE15>, Peri<'static, PD8>, Peri<'static, PD9>, Peri<'static, PD10>,
    ),
    // D16..D31
    pub d_hi: (
        Peri<'static, PH8>, Peri<'static, PH9>, Peri<'static, PH10>, Peri<'static, PH11>,
        Peri<'static, PH12>, Peri<'static, PH13>, Peri<'static, PH14>, Peri<'static, PH15>,
        Peri<'static, PI0>, Peri<'static, PI1>, Peri<'static, PI2>, Peri<'static, PI3>,
        Peri<'static, PI6>, Peri<'static, PI7>, Peri<'static, PI9>, Peri<'static, PI10>,
    ),
    // NBL0..NBL3
    pub nbl: (Peri<'static, PE0>, Peri<'static, PE1>, Peri<'static, PI4>, Peri<'static, PI5>),
    // SDCKE0, SDCLK, SDNCAS, SDNE0, SDNRAS, SDNWE
    pub ctrl: (
        Peri<'static, PH2>, Peri<'static, PG8>, Peri<'static, PG15>,
        Peri<'static, PH3>, Peri<'static, PF11>, Peri<'static, PH5>,
    ),
}

/// Configures the FMC and runs the SDRAM power-up sequence.
/// Returns the whole SDRAM as a word slice.
pub fn init(p: SdramPins) -> &'static mut [u32] {
    let (a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11) = p.a;
    let (ba0, ba1) = p.ba;
    let (d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15) = p.d_lo;
    let (d16, d17, d18, d19, d20, d21, d22, d23, d24, d25, d26, d27, d28, d29, d30, d31) = p.d_hi;
    let (nbl0, nbl1, nbl2, nbl3) = p.nbl;
    let (sdcke, sdclk, sdncas, sdne, sdnras, sdnwe) = p.ctrl;

    let mut sdram = Fmc::sdram_a12bits_d32bits_4banks_bank1(
        p.fmc,
        a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11,
        ba0, ba1,
        d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15,
        d16, d17, d18, d19, d20, d21, d22, d23, d24, d25, d26, d27, d28, d29, d30, d31,
        nbl0, nbl1, nbl2, nbl3,
        sdcke, sdclk, sdncas, sdne, sdnras, sdnwe,
        Is42s32800g {},
    );

    let ptr = sdram.init(&mut Delay);
    defmt::assert_eq!(ptr as usize, BASE);
    // SAFETY: the FMC now maps SIZE_BYTES of SDRAM at `ptr`; nothing else aliases it.
    unsafe { core::slice::from_raw_parts_mut(ptr, SIZE_WORDS) }
}

#[cfg(feature = "sdram-test")]
#[derive(defmt::Format)]
pub enum TestError {
    DataBus { pattern: u32, read: u32 },
    AddressBus { offset_words: usize, expected: u32, read: u32 },
    ByteLane { lane: usize, read: u32 },
    Pattern { pass: &'static str, index: usize, expected: u32, read: u32 },
}

#[cfg(feature = "sdram-test")]
fn wr(mem: &mut [u32], i: usize, v: u32) {
    unsafe { core::ptr::write_volatile(&mut mem[i], v) }
}
#[cfg(feature = "sdram-test")]
fn rd(mem: &[u32], i: usize) -> u32 {
    unsafe { core::ptr::read_volatile(&mem[i]) }
}

/// Classic memory test: data-bus walking ones, address-bus power-of-two offsets,
/// byte-lane (NBL0..3) writes, then two full-array passes (address-as-data and its inverse).
#[cfg(feature = "sdram-test")]
pub fn self_test(mem: &mut [u32]) -> Result<(), TestError> {
    // 1. Data bus: walking 1 on all 32 data lines (D0..D31).
    for bit in 0..32 {
        let pattern = 1u32 << bit;
        wr(mem, 0, pattern);
        let read = rd(mem, 0);
        if read != pattern {
            return Err(TestError::DataBus { pattern, read });
        }
    }

    // 2. Address bus: each power-of-two word offset must hit a distinct location
    //    (catches stuck/shorted row, column and bank address lines).
    let n = mem.len();
    let mut off = 1;
    while off < n {
        wr(mem, off, 0xAAAA_AAAA);
        off <<= 1;
    }
    wr(mem, 0, 0x5555_5555);
    let mut off = 1;
    while off < n {
        let read = rd(mem, off);
        if read != 0xAAAA_AAAA {
            return Err(TestError::AddressBus { offset_words: off, expected: 0xAAAA_AAAA, read });
        }
        off <<= 1;
    }

    // 3. Byte lanes: single-byte writes must only touch their own lane (NBL0..NBL3).
    let bytes = mem.as_mut_ptr() as *mut u8;
    for lane in 0..4 {
        wr(mem, 1, 0);
        unsafe { core::ptr::write_volatile(bytes.add(4 + lane), 0xA5) };
        let read = rd(mem, 1);
        if read != 0xA5u32 << (8 * lane) {
            return Err(TestError::ByteLane { lane, read });
        }
    }

    // 4. Full array: address-as-data, then inverted (each cell holds both 0 and 1 per bit).
    for (pass, invert) in [("addr", 0u32), ("~addr", u32::MAX)] {
        for i in 0..n {
            wr(mem, i, (i as u32).wrapping_mul(4) ^ invert);
        }
        for i in 0..n {
            let expected = (i as u32).wrapping_mul(4) ^ invert;
            let read = rd(mem, i);
            if read != expected {
                return Err(TestError::Pattern { pass, index: i, expected, read });
            }
        }
    }
    Ok(())
}
