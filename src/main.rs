#![no_std]
#![no_main]

mod ads1256;
mod display;
mod mcode;
mod mfx;
mod motor;
mod net;
mod sdram;
mod touch;
mod ui;

use defmt::{info, warn};
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::interrupt::{self, InterruptExt, Priority};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, Peri};
use embassy_time::{Duration, Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
});

/// Runs time-critical tasks (ADC sampling) above the main executor, whose frame draws block
/// it for ~20 ms. Driven by the otherwise unused UART4 interrupt.
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[cortex_m_rt::interrupt]
unsafe fn UART4() {
    // SAFETY: the interrupt is only pended by EXECUTOR_HIGH, which was started with it.
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

/// System clock: 25 MHz HSE (crystal X1) -> PLL -> 216 MHz SYSCLK, 48 MHz PLLQ.
/// LCD pixel clock: PLLSAI 1 MHz x151 = 151 MHz, /R3 = 50.3 MHz, /2 (PLLSAIDIVR) = 25.17 MHz.
fn clock_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::*;

    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(25_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV25,  // 1 MHz VCO input (shared by all PLLs on F7)
        mul: PllMul::MUL432,       // 432 MHz VCO
        divp: Some(PllPDiv::DIV2), // 216 MHz SYSCLK
        divq: Some(PllQDiv::DIV9), // 48 MHz (USB/SDMMC)
        divr: None,
    });
    config.rcc.pllsai = Some(Pll {
        prediv: PllPreDiv::DIV25,
        mul: PllMul::MUL151,       // 151 MHz VCO
        divp: None,
        divq: None,
        divr: Some(PllRDiv::DIV3), // 50.3 MHz -> LTDC /2 -> 25.17 MHz pixel clock
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1; // HCLK 216 MHz
    config.rcc.apb1_pre = APBPrescaler::DIV4; // 54 MHz
    config.rcc.apb2_pre = APBPrescaler::DIV2; // 108 MHz
    config.rcc.sys = Sysclk::PLL1_P;
    config
}

/// Enables the Cortex-M7 L1 I/D caches and the Flash ART accelerator + prefetch.
/// SDRAM (framebuffers) is marked non-cacheable by the MPU, so LTDC always sees CPU writes.
fn enable_caches() {
    // SAFETY: called once at startup before anything relies on cache state.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.SCB.enable_icache();
    cp.SCB.enable_dcache(&mut cp.CPUID);
    embassy_stm32::pac::FLASH.acr().modify(|w| {
        w.set_arten(true);
        w.set_prften(true);
    });
}

/// Blinks an active-low LED (anode on +3V3 on STM32746G-EVAL).
#[embassy_executor::task(pool_size = 2)]
async fn blink(pin: Peri<'static, embassy_stm32::gpio::AnyPin>, period_ms: u64) {
    let mut led = Output::new(pin, Level::High, Speed::Low); // High = off
    loop {
        led.toggle();
        Timer::after_millis(period_ms).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(clock_config());
    enable_caches();
    info!("ABP simulator v{} ({}) starting on STM32F746NG", ui::VERSION, ui::GIT_VERSION);

    let sdram = sdram::init(sdram::SdramPins {
        fmc: p.FMC,
        a: (p.PF0, p.PF1, p.PF2, p.PF3, p.PF4, p.PF5, p.PF12, p.PF13, p.PF14, p.PF15, p.PG0, p.PG1),
        ba: (p.PG4, p.PG5),
        d_lo: (
            p.PD14, p.PD15, p.PD0, p.PD1, p.PE7, p.PE8, p.PE9, p.PE10,
            p.PE11, p.PE12, p.PE13, p.PE14, p.PE15, p.PD8, p.PD9, p.PD10,
        ),
        d_hi: (
            p.PH8, p.PH9, p.PH10, p.PH11, p.PH12, p.PH13, p.PH14, p.PH15,
            p.PI0, p.PI1, p.PI2, p.PI3, p.PI6, p.PI7, p.PI9, p.PI10,
        ),
        nbl: (p.PE0, p.PE1, p.PI4, p.PI5),
        ctrl: (p.PH2, p.PG8, p.PG15, p.PH3, p.PF11, p.PH5),
    });
    display::mpu_sdram_normal();
    info!("SDRAM ready: {} MiB at {:#010x}", sdram::SIZE_BYTES / (1024 * 1024), sdram::BASE);

    #[cfg(feature = "sdram-test")]
    {
        let t0 = embassy_time::Instant::now();
        match sdram::self_test(sdram) {
            Ok(()) => info!("SDRAM self-test PASSED in {} ms", t0.elapsed().as_millis()),
            Err(e) => defmt::error!("SDRAM self-test FAILED: {}", e),
        }
    }

    // SDRAM layout: two RGB565 framebuffers (600 KiB each), then the Ethernet DMA packet queue
    // (needs non-cacheable memory; the MPU makes all of SDRAM non-cacheable).
    // SAFETY: SDRAM is large enough, the ranges do not overlap, and the queue offset
    // (2 * 614400 bytes) is a multiple of its 8-byte alignment.
    let (fb0, fb1, eth_queue) = unsafe {
        let base = sdram.as_mut_ptr() as *mut u16;
        let fb0 = core::slice::from_raw_parts_mut(base, display::FB_PIXELS);
        let fb1 = core::slice::from_raw_parts_mut(base.add(display::FB_PIXELS), display::FB_PIXELS);
        fb0.fill(0);
        let q = base.add(2 * display::FB_PIXELS) as *mut core::mem::MaybeUninit<net::Queue>;
        defmt::assert!(q as usize % core::mem::align_of::<net::Queue>() == 0);
        (fb0, fb1, &mut *q)
    };

    let mut disp = display::init(
        display::LcdPins {
            ltdc: p.LTDC,
            clk: p.PI14,
            hsync: p.PI12,
            vsync: p.PI13,
            de: p.PK7,
            r: (p.PI15, p.PJ0, p.PJ1, p.PJ2, p.PJ3, p.PJ4, p.PJ5, p.PJ6),
            g: (p.PJ7, p.PJ8, p.PJ9, p.PJ10, p.PJ11, p.PK0, p.PK1, p.PK2),
            b: (p.PJ12, p.PJ13, p.PJ14, p.PJ15, p.PK3, p.PK4, p.PK5, p.PK6),
        },
        fb0,
        fb1,
    );
    info!("LTDC on: {}x{} RGB565, double-buffered", display::WIDTH, display::HEIGHT);

    let mut i2c1 = touch::bus(p.I2C1, p.PB8, p.PB9);
    touch::scan(&mut i2c1);
    mfx::phy_power_up(&mut i2c1);
    spawner.spawn(touch::task(i2c1).unwrap());

    let stack = net::init(
        spawner,
        net::EthPins {
            eth: p.ETH,
            rx_clk: p.PA1,
            tx_clk: p.PC3,
            rx_dv: p.PA7,
            rxd: (p.PC4, p.PC5, p.PH6, p.PH7),
            txd: (p.PG13, p.PG14, p.PC2, p.PE2),
            tx_en: p.PG11,
            mdio: p.PG6,
            mdc: p.PG7,
        },
        eth_queue,
    );
    spawner.spawn(motor::task(stack).unwrap());

    // ADS1256 on SPI2 (pinout in ads1256.rs), sampled on the high-priority executor.
    let adc = ads1256::Ads1256::new(
        Spi::new_blocking(p.SPI2, p.PD3, p.PB15, p.PB14, ads1256::Ads1256::spi_config(ads1256::SPI_SPEEDS[0].0)),
        Output::new(p.PA4, Level::High, Speed::Medium),
        ExtiInput::new(p.PC11, p.EXTI11, Pull::Up, Irqs),
    );
    interrupt::UART4.set_priority(Priority::P6);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::UART4);
    spawner_high.spawn(ads1256::task(adc).unwrap());

    // LD1 (green) on PF10 with JP24 at 2-3; LD3 (red) on PB7 with JP23 at 2-3 (defaults).
    spawner.spawn(blink(p.PF10.into(), 500).unwrap());
    spawner.spawn(blink(p.PB7.into(), 1000).unwrap());

    let mut window = Instant::now();
    let (mut frames, mut draw_us_max) = (0u32, 0u64);
    let mut screen = ui::Ui::new();
    loop {
        screen.update(touch::current());

        let t_draw = Instant::now();
        let _ = screen.draw(&mut disp.back());
        draw_us_max = draw_us_max.max(t_draw.elapsed().as_micros());
        if let Err(e) = disp.swap().await {
            warn!("LTDC swap error: {}", e);
        }
        frames += 1;
        if window.elapsed() >= Duration::from_secs(10) {
            info!("{} fps, max draw time {} us", frames / 10, draw_us_max);
            frames = 0;
            draw_us_max = 0;
            window = Instant::now();
        }
    }
}
