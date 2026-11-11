//! Ethernet on the STM32746G-EVAL: DP83848CVV PHY (U6) on MII, RJ45 CN9 (UM1902 section 5.13).
//!
//! - MII data pins: TX_CLK PC3, TX_EN PG11, TXD0..3 PG13 PG14 PC2 PE2,
//!   RX_CLK PA1, RX_DV PA7, RXD0..3 PC4 PC5 PH6 PH7. PHY clock from crystal X4 (JP6 default).
//! - Management (MDIO/MDC) is wired to plain GPIOs by default (PG6 = MDIO, PG7 = MDC), not to the
//!   MAC's SMI pins (those need JP21/JP22 moved, which disconnects audio), so it is bit-banged here.
//! - JP6 must be fitted in the X4 position: without it the PHY has no 25 MHz reference, still
//!   answers on MDIO, but never sees a link.
//! - The Ethernet DMA does no cache maintenance, so the packet buffers live in SDRAM, which the
//!   MPU marks non-cacheable.

use core::mem::MaybeUninit;
use core::net::Ipv4Addr;
use core::pin::pin;
use core::task::Context;

use defmt::{info, warn};
use embassy_net::{Ipv4Cidr, Stack, StackResources, StaticConfigV4};
use embassy_stm32::eth::{self, Ethernet, PacketQueue, Phy};
use embassy_stm32::gpio::{Flex, Pull, Speed};
use embassy_stm32::peripherals::*;
use embassy_stm32::{bind_interrupts, pac, Peri};
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;

/// This board's address. The LMD factory default is 192.168.33.1 / 255.255.0.0.
pub const OUR_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 33, 10);
pub const PREFIX_LEN: u8 = 16;
pub const MOTOR_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 33, 1);

pub type Queue = PacketQueue<4, 4>;
type Device = Ethernet<'static, ETH, Dp83848>;

bind_interrupts!(struct Irqs {
    ETH => eth::InterruptHandler;
});

// ---------------------------------------------------------------------------------------------
// MDIO bit-bang (IEEE 802.3 clause 22)

struct Mdio {
    mdio: Flex<'static>,
    mdc: Flex<'static>,
}

impl Mdio {
    /// About 1 us per half period at 216 MHz -> ~500 kHz MDC (PHY max is 25 MHz, MDIO needs < 2.5 MHz).
    fn half() {
        cortex_m::asm::delay(216);
    }

    fn send_bit(&mut self, bit: bool) {
        if bit {
            self.mdio.set_high();
        } else {
            self.mdio.set_low();
        }
        Self::half();
        self.mdc.set_high();
        Self::half();
        self.mdc.set_low();
    }

    fn get_bit(&mut self) -> bool {
        Self::half();
        self.mdc.set_high();
        Self::half();
        self.mdc.set_low();
        self.mdio.is_high()
    }

    fn send_bits(&mut self, val: u32, n: u32) {
        for i in (0..n).rev() {
            self.send_bit(val >> i & 1 != 0);
        }
    }

    fn cmd(&mut self, op: u32, phy: u8, reg: u8) {
        self.mdio.set_high();
        self.mdio.set_as_output(Speed::Low);
        self.send_bits(0xFFFF_FFFF, 32); // preamble
        self.send_bits(0b01, 2); // start
        self.send_bits(op, 2);
        self.send_bits(phy as u32, 5);
        self.send_bits(reg as u32, 5);
    }

    fn read(&mut self, phy: u8, reg: u8) -> Option<u16> {
        self.cmd(0b10, phy, reg);
        self.mdio.set_as_input(Pull::Up);
        // Turnaround: the PHY drives the second bit low; a high level means nobody answered.
        let ack = !self.get_bit();
        let mut v = 0u16;
        for _ in 0..16 {
            v = v << 1 | self.get_bit() as u16;
        }
        self.get_bit(); // idle clock
        ack.then_some(v)
    }

    fn write(&mut self, phy: u8, reg: u8, val: u16) {
        self.cmd(0b01, phy, reg);
        self.send_bits(0b10, 2); // turnaround
        self.send_bits(val as u32, 16);
        self.mdio.set_as_input(Pull::Up);
    }
}

// ---------------------------------------------------------------------------------------------
// DP83848 PHY

const BMCR: u8 = 0x00;
const ID1: u8 = 0x02;
const ID2: u8 = 0x03;
const PHYSTS: u8 = 0x10;
const BMCR_RESET: u16 = 1 << 15;
const BMCR_AN_ENABLE: u16 = 1 << 12;
const BMCR_AN_RESTART: u16 = 1 << 9;
const PHYSTS_LINK: u16 = 1 << 0;
const PHYSTS_10M: u16 = 1 << 1;
const PHYSTS_FULL_DUPLEX: u16 = 1 << 2;

#[derive(Clone, Copy, PartialEq, defmt::Format)]
pub enum Link {
    Down,
    Up { mbps: u8, full_duplex: bool },
}

pub struct Dp83848 {
    smi: Mdio,
    addr: u8,
    link: Link,
    last_poll: Option<Instant>,
}

impl Dp83848 {
    const POLL: Duration = Duration::from_millis(500);

    fn read(&mut self, reg: u8) -> u16 {
        self.smi.read(self.addr, reg).unwrap_or(0xFFFF)
    }
}

impl Phy for Dp83848 {
    fn phy_reset(&mut self) {
        // Board straps give address 1, but scan so a different strap still works.
        let found = (0..32u8).find(|&a| matches!(self.smi.read(a, ID1), Some(id) if id != 0xFFFF && id != 0));
        match found {
            Some(a) => {
                self.addr = a;
                let (id1, id2) = (self.read(ID1), self.read(ID2));
                info!("ETH PHY at MDIO address {}: ID {:#06x}:{:#06x} (DP83848 = 0x2000:0x5c9x)", a, id1, id2);
            }
            None => {
                warn!("ETH PHY not answering on MDIO (PG6/PG7); check JP21/JP22 and SB10/SB11");
                return;
            }
        }
        self.smi.write(self.addr, BMCR, BMCR_RESET);
        let t0 = Instant::now();
        while self.read(BMCR) & BMCR_RESET != 0 {
            if t0.elapsed() > Duration::from_millis(500) {
                warn!("ETH PHY reset timed out");
                break;
            }
        }
    }

    fn phy_init(&mut self) {
        self.smi.write(self.addr, BMCR, BMCR_AN_ENABLE | BMCR_AN_RESTART);
    }

    fn poll_link(&mut self, cx: &mut Context) -> bool {
        // Bit-banged MDIO costs ~150 us, so only read the PHY every POLL and cache the result.
        // Schedule a wake-up so the stack polls again (same approach as embassy's GenericPhy).
        let due = self.last_poll.is_none_or(|t| t.elapsed() >= Self::POLL);
        let _ = core::future::Future::poll(pin!(Timer::after(Self::POLL)), cx);
        if !due {
            return self.link != Link::Down;
        }
        self.last_poll = Some(Instant::now());

        let sts = self.read(PHYSTS);
        let link = if sts != 0xFFFF && sts & PHYSTS_LINK != 0 {
            Link::Up {
                mbps: if sts & PHYSTS_10M != 0 { 10 } else { 100 },
                full_duplex: sts & PHYSTS_FULL_DUPLEX != 0,
            }
        } else {
            Link::Down
        };
        if link != self.link {
            info!("ETH link {}", link);
            if let Link::Up { mbps, full_duplex } = link {
                // The driver starts the MAC at 100M full duplex; follow what was negotiated.
                use pac::eth::vals::{Dm, Fes};
                pac::ETH.ethernet_mac().maccr().modify(|w| {
                    w.set_fes(if mbps == 100 { Fes::FES100 } else { Fes::FES10 });
                    w.set_dm(if full_duplex { Dm::FULL_DUPLEX } else { Dm::HALF_DUPLEX });
                });
            }
            self.link = link;
        }
        self.link != Link::Down
    }
}

// ---------------------------------------------------------------------------------------------

pub struct EthPins {
    pub eth: Peri<'static, ETH>,
    pub rx_clk: Peri<'static, PA1>,
    pub tx_clk: Peri<'static, PC3>,
    pub rx_dv: Peri<'static, PA7>,
    pub rxd: (Peri<'static, PC4>, Peri<'static, PC5>, Peri<'static, PH6>, Peri<'static, PH7>),
    pub txd: (Peri<'static, PG13>, Peri<'static, PG14>, Peri<'static, PC2>, Peri<'static, PE2>),
    pub tx_en: Peri<'static, PG11>,
    pub mdio: Peri<'static, PG6>,
    pub mdc: Peri<'static, PG7>,
}

/// Locally administered MAC derived from the MCU's unique ID.
fn mac_addr() -> [u8; 6] {
    let uid = embassy_stm32::uid::uid();
    let h = uid.iter().fold(0x811c_9dc5u32, |h, &b| (h ^ b as u32).wrapping_mul(0x0100_0193));
    let [a, b, c, d] = h.to_be_bytes();
    [0x02, 0x80, a, b, c, d]
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}

/// Brings up the MAC, PHY and IP stack. `queue` must be in non-cacheable memory.
pub fn init(spawner: embassy_executor::Spawner, p: EthPins, queue: &'static mut MaybeUninit<Queue>) -> Stack<'static> {
    Queue::init(queue);
    // SAFETY: `init` zero-fills the queue, which is its valid initial state.
    let queue = unsafe { queue.assume_init_mut() };

    let mut mdio = Flex::new(p.mdio);
    mdio.set_as_input(Pull::Up);
    let mut mdc = Flex::new(p.mdc);
    mdc.set_low();
    mdc.set_as_output(Speed::Low);
    let phy = Dp83848 { smi: Mdio { mdio, mdc }, addr: 1, link: Link::Down, last_poll: None };

    let mac = mac_addr();
    let (rxd0, rxd1, rxd2, rxd3) = p.rxd;
    let (txd0, txd1, txd2, txd3) = p.txd;
    let device = Ethernet::new_mii_with_phy(
        queue, p.eth, Irqs, p.rx_clk, p.tx_clk, p.rx_dv, rxd0, rxd1, rxd2, rxd3, txd0, txd1, txd2, txd3,
        p.tx_en, mac, phy,
    );
    info!("ETH MAC {:02x}", mac);

    let config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(OUR_IP, PREFIX_LEN),
        gateway: None,
        dns_servers: Default::default(),
    });
    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let seed = u64::from_le_bytes(embassy_stm32::uid::uid()[..8].try_into().unwrap());
    let (stack, runner) = embassy_net::new(device, config, RESOURCES.init(StackResources::new()), seed);
    spawner.spawn(net_task(runner).unwrap());
    info!("IP {} /{}", OUR_IP, PREFIX_LEN);
    stack
}
