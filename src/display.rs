//! 5.7" Ampire AM640480G5TNQW-T00H (640x480, parallel RGB888) on CN20, driven by the LTDC.
//! Framebuffer: RGB565 in external SDRAM. Timings from ST's ampire640480 BSP driver.

use embassy_stm32::ltdc::{
    self, Ltdc, LtdcConfiguration, LtdcLayer, LtdcLayerConfig, PixelFormat, PolarityActive, PolarityEdge,
};
use core::convert::Infallible;

use embassy_stm32::pac;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embassy_stm32::peripherals::*;
use embassy_stm32::{bind_interrupts, Peri};

pub const WIDTH: usize = 640;
pub const HEIGHT: usize = 480;
pub const FB_PIXELS: usize = WIDTH * HEIGHT;

bind_interrupts!(struct Irqs {
    LTDC => ltdc::InterruptHandler<LTDC>;
});

/// LTDC pins on the STM32746G-EVAL (UM1902 table 14).
pub struct LcdPins {
    pub ltdc: Peri<'static, LTDC>,
    pub clk: Peri<'static, PI14>,
    pub hsync: Peri<'static, PI12>,
    pub vsync: Peri<'static, PI13>,
    pub de: Peri<'static, PK7>,
    pub r: (
        Peri<'static, PI15>, Peri<'static, PJ0>, Peri<'static, PJ1>, Peri<'static, PJ2>,
        Peri<'static, PJ3>, Peri<'static, PJ4>, Peri<'static, PJ5>, Peri<'static, PJ6>,
    ),
    pub g: (
        Peri<'static, PJ7>, Peri<'static, PJ8>, Peri<'static, PJ9>, Peri<'static, PJ10>,
        Peri<'static, PJ11>, Peri<'static, PK0>, Peri<'static, PK1>, Peri<'static, PK2>,
    ),
    pub b: (
        Peri<'static, PJ12>, Peri<'static, PJ13>, Peri<'static, PJ14>, Peri<'static, PJ15>,
        Peri<'static, PK3>, Peri<'static, PK4>, Peri<'static, PK5>, Peri<'static, PK6>,
    ),
}

const PANEL: LtdcConfiguration = LtdcConfiguration {
    active_width: WIDTH as u16,
    active_height: HEIGHT as u16,
    h_sync: 30,
    h_back_porch: 114,
    h_front_porch: 16,
    v_sync: 3,
    v_back_porch: 32,
    v_front_porch: 10,
    h_sync_polarity: PolarityActive::ActiveLow,
    v_sync_polarity: PolarityActive::ActiveLow,
    data_enable_polarity: PolarityActive::ActiveLow,
    pixel_clock_polarity: PolarityEdge::RisingEdge, // "IPC" in ST's HAL (PCPOL = 0)
};

/// Marks the SDRAM window (0xC000_0000, 32 MiB) as Normal, non-cacheable memory.
/// The Cortex-M7 default map treats 0xC000_0000 as Device memory, where unaligned
/// accesses (e.g. from memcpy or graphics code) fault.
pub fn mpu_sdram_normal() {
    // SAFETY: single-threaded early init; region 0 is not used elsewhere.
    unsafe {
        let mpu = &*cortex_m::peripheral::MPU::PTR;
        cortex_m::asm::dmb();
        mpu.ctrl.write(0);
        mpu.rnr.write(0);
        mpu.rbar.write(crate::sdram::BASE as u32);
        const XN: u32 = 1 << 28;
        const AP_FULL: u32 = 0b011 << 24;
        const TEX_NORMAL_NC: u32 = 0b001 << 19; // TEX=001, C=0, B=0: Normal, non-cacheable
        const SIZE_32MB: u32 = 24 << 1; // 2^(24+1) bytes
        mpu.rasr.write(XN | AP_FULL | TEX_NORMAL_NC | SIZE_32MB | 1);
        mpu.ctrl.write((1 << 2) | 1); // PRIVDEFENA | ENABLE
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}

/// An RGB565 framebuffer that `embedded-graphics` can draw on.
pub struct Framebuffer<'a> {
    buf: &'a mut [u16],
}

impl OriginDimensions for Framebuffer<'_> {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl DrawTarget for Framebuffer<'_> {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(p, c) in pixels {
            if (0..WIDTH as i32).contains(&p.x) && (0..HEIGHT as i32).contains(&p.y) {
                self.buf[p.y as usize * WIDTH + p.x as usize] = c.into_storage();
            }
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let area = area.intersection(&self.bounding_box());
        if let Some(br) = area.bottom_right() {
            let (x0, x1) = (area.top_left.x as usize, br.x as usize);
            let v = color.into_storage();
            for y in area.top_left.y as usize..=br.y as usize {
                self.buf[y * WIDTH + x0..=y * WIDTH + x1].fill(v);
            }
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.buf.fill(color.into_storage());
        Ok(())
    }
}

/// Double-buffered display: draw into `back()`, then `swap().await` shows it at the next
/// vertical blanking, so frames never tear.
pub struct Display {
    ltdc: Ltdc<'static, LTDC>,
    bufs: [&'static mut [u16]; 2],
    front: usize,
}

impl Display {
    pub fn back(&mut self) -> Framebuffer<'_> {
        Framebuffer { buf: &mut *self.bufs[1 - self.front] }
    }

    /// Presents the back buffer. Resolves after the LTDC has latched it (at vertical blanking),
    /// after which the previous front buffer is free to draw into.
    pub async fn swap(&mut self) -> Result<(), ltdc::Error> {
        let back = 1 - self.front;
        let ptr = self.bufs[back].as_ptr() as *const ();
        let mut res = self.ltdc.set_buffer(LtdcLayer::Layer1, ptr).await;
        if res.is_err() {
            // A stale error flag makes set_buffer return without swapping; flags are now
            // cleared, so retry once.
            res = self.ltdc.set_buffer(LtdcLayer::Layer1, ptr).await;
        }
        if res.is_ok() {
            self.front = back;
        }
        res
    }
}

/// Configures the LTDC for the panel and shows `fb0` on layer 1; `fb1` is the back buffer.
pub fn init(p: LcdPins, fb0: &'static mut [u16], fb1: &'static mut [u16]) -> Display {
    defmt::assert!(fb0.len() >= FB_PIXELS && fb1.len() >= FB_PIXELS);
    let fb = &*fb0;
    let (r0, r1, r2, r3, r4, r5, r6, r7) = p.r;
    let (g0, g1, g2, g3, g4, g5, g6, g7) = p.g;
    let (b0, b1, b2, b3, b4, b5, b6, b7) = p.b;

    let mut ltdc = Ltdc::new_with_pins(
        p.ltdc, Irqs, p.clk, p.hsync, p.vsync,
        b0, b1, b2, b3, b4, b5, b6, b7,
        g0, g1, g2, g3, g4, g5, g6, g7,
        r0, r1, r2, r3, r4, r5, r6, r7,
    );
    // The driver does not take the DE pin; configure PK7 as LTDC_DE (AF14) ourselves.
    // (Owning `p.de` guarantees nothing else uses the pin; embassy init already clocks GPIOK.)
    let _de = p.de;
    pac::GPIOK.moder().modify(|w| w.set_moder(7, pac::gpio::vals::Moder::ALTERNATE));
    pac::GPIOK.afr(0).modify(|w| w.set_afr(7, 14));
    pac::GPIOK.ospeedr().modify(|w| w.set_ospeedr(7, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED));

    ltdc.init(&PANEL);
    ltdc.init_layer(
        &LtdcLayerConfig {
            layer: LtdcLayer::Layer1,
            pixel_format: PixelFormat::RGB565,
            window_x0: 0,
            window_x1: WIDTH as u16,
            window_y0: 0,
            window_y1: HEIGHT as u16,
        },
        None,
    );
    // Point layer 1 at the framebuffer and reload immediately. (`Ltdc::set_buffer` skips the
    // write if an error flag is already pending, e.g. from fetching before a buffer was set.)
    pac::LTDC.layer(0).cfbar().write(|w| w.set_cfbadd(fb.as_ptr() as u32));
    pac::LTDC.srcr().write(|w| w.set_imr(pac::ltdc::vals::Imr::RELOAD));
    pac::LTDC.icr().write(|w| {
        w.set_cfuif(pac::ltdc::vals::Cfuif::CLEAR);
        w.set_cterrif(pac::ltdc::vals::Cterrif::CLEAR);
    });
    Display { ltdc, bufs: [fb0, fb1], front: 0 }
}

