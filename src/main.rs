//#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m;
use rtic::app;

#[macro_use]
#[allow(unused)]
mod utilities;
use stm32h7xx_hal::time::Hertz;
use cortex_m_rt::{ExceptionFrame, exception};
use core::panic::PanicInfo;
use core::sync::atomic;
use core::sync::atomic::Ordering;
use rtt_target::{rtt_init_print, rprintln};
use tim_systick_monotonic;
use rtic::rtic_monotonic::Milliseconds;
use core::convert::TryFrom;

mod network;

pub const CLOCKS_FREQ_HZ: u32 = 200_000_000; //200 MHz
pub type TimMono = tim_systick_monotonic::TimSystickMonotonic<CLOCKS_FREQ_HZ>;

pub fn time_now() -> Milliseconds {
    let now: rtic::rtic_monotonic::Instant<TimMono> = app::monotonics::now();
    Milliseconds::<u32>::try_from(now.duration_since_epoch()).unwrap_or(Milliseconds(0))
}

#[app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [FLASH])]
mod app {
    use crate::{rtt_init_print, rprintln};
    use stm32h7xx_hal::rcc::CoreClocks;
    use stm32h7xx_hal::{prelude::*, stm32, gpio, pac, ethernet};
    use embedded_hal::digital::v2::OutputPin;
    use embedded_hal::digital::v2::ToggleableOutputPin;
    use rtic::rtic_monotonic::Extensions;
    use crate::{utilities};
    use stm32h7xx_hal::time::MegaHertz;
    use cortex_m::asm::delay;
    use rtic::Monotonic;
    use stm32h7xx_hal::gpio::Speed::VeryHigh;
    use stm32h7xx_hal::ethernet::PHY;
    use smoltcp::socket::UdpSocket;
    use crate::network::net;
    use core::fmt::Write;

    type LinkLed = gpio::gpiob::PB0<gpio::Output<gpio::PushPull>>;
    type EthEventLed = gpio::gpioe::PE1<gpio::Output<gpio::PushPull>>;
    type SystemLed = gpio::gpiob::PB14<gpio::Output<gpio::PushPull>>;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = crate::TimMono;

    #[local]
    struct Local {
        link_led: LinkLed,
        eth_led: EthEventLed,
        sys_led: SystemLed,
        lan8742a: ethernet::phy::LAN8742A<ethernet::EthernetMAC>,
    }
    #[shared]
    struct Shared {
        network: net::Net<'static>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        let mut dp: pac::Peripherals = cx.device;
        let pwr = dp.PWR.constrain();
        let pwrcfg = example_power!(pwr).freeze();

        // Link the SRAM3 power state to CPU1
        dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

        // Initialise clocks...
        let rcc = dp.RCC.constrain();
        let ccdr = rcc
            .sys_ck(crate::CLOCKS_FREQ_HZ.hz())
            .hclk(crate::CLOCKS_FREQ_HZ.hz())
            .freeze(pwrcfg, &dp.SYSCFG);

        // Initialise ethernet...
        assert_eq!(ccdr.clocks.hclk().0, 200_000_000); // HCLK 200MHz
        assert_eq!(ccdr.clocks.pclk1().0, 100_000_000); // PCLK 100MHz
        assert_eq!(ccdr.clocks.pclk2().0, 100_000_000); // PCLK 100MHz
        assert_eq!(ccdr.clocks.pclk4().0, 100_000_000); // PCLK 100MHz

        let mut core: cortex_m::Peripherals = cx.core;
        core.SCB.enable_icache();
        core.DWT.enable_cycle_counter();

        let systick = core.SYST;
        let mono = tim_systick_monotonic::TimSystickMonotonic::new(systick, dp.TIM15, dp.TIM17, crate::CLOCKS_FREQ_HZ);

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

        let mut link_led: LinkLed = gpiob.pb0.into_push_pull_output();
        link_led.set_high().ok();

        let mut eth_led: EthEventLed = gpioe.pe1.into_push_pull_output();
        eth_led.set_high().ok();

        let mut sys_led: SystemLed = gpiob.pb14.into_push_pull_output();
        sys_led.set_high().ok();

        let _rmii_ref_clk = gpioa.pa1.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_mdio = gpioa.pa2.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_mdc = gpioc.pc1.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_crs_dv = gpioa.pa7.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_rxd0 = gpioc.pc4.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_rxd1 = gpioc.pc5.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_tx_en = gpiog.pg11.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_txd0 = gpiog.pg13.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_txd1 = gpiob.pb13.into_alternate_af11().set_speed(VeryHigh);

        let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&net::config::MAC_ADDRESS);
        let (eth_dma, eth_mac) = unsafe {
            ethernet::new_unchecked(
                dp.ETHERNET_MAC,
                dp.ETHERNET_MTL,
                dp.ETHERNET_DMA,
                &mut net::DES_RING,
                mac_addr.clone(),
                ccdr.peripheral.ETH1MAC,
                &ccdr.clocks,
            )
        };

        // Initialise ethernet PHY...
        let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac);
        lan8742a.phy_reset();
        lan8742a.phy_init();
        // The eth_dma should not be used until the PHY reports the link is up
        rprintln!("lan8742a init");
        unsafe {
            ethernet::enable_interrupt();
        }
        rprintln!("enable irq");
        // unsafe: mutable reference to static storage, we only do this once
        let store = unsafe { &mut net::STORE };
        let mut network = net::Net::new(store, eth_dma, mac_addr);


        link_led.set_low().ok();
        eth_led.set_low().ok();
        sys_led.set_low().ok();

        sys_led_blink::spawn_after(1.seconds()).unwrap();
        (
            Shared {
                network,
            },
            Local {
                link_led,
                eth_led,
                sys_led,
                lan8742a
            },
            init::Monotonics(mono)
        )
    }

    #[task(local = [sys_led])]
    fn sys_led_blink(cx: sys_led_blink::Context) {
        cx.local.sys_led.toggle().ok();
        sys_led_blink::spawn_after(1.seconds()).unwrap();
    }

    /*#[task(shared = [network])]
    fn try_poll_task(mut cx: try_poll_task::Context) {
        let now: rtic::rtic_monotonic::Instant<crate::TimMono> = monotonics::now();
        let now = Milliseconds::<u32>::try_from(now.duration_since_epoch()).unwrap_or(Milliseconds(0));
        cx.shared.network.lock(|n|{
            match n.try_poll(now.0 as i64) {
                Ok(_) => {
                    n.poll(now.0 as i64);
                }
                Err(duration) => {
                    try_poll_task::spawn_after((duration.millis() as u32).milliseconds()).unwrap();
                }
            }
        });
    }*/

    #[idle(local = [link_led, lan8742a])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            match cx.local.lan8742a.poll_link() {
                true =>  cx.local.link_led.set_low(),
                _ => cx.local.link_led.set_high(),
            }
                .ok();
        }
    }

    #[task(binds = ETH, shared = [network], local = [eth_led])]
    fn ethernet_event(mut cx: ethernet_event::Context) {
        unsafe { ethernet::interrupt_handler() }
        cx.local.eth_led.toggle().unwrap();
        cx.shared.network.lock(|n|{
            n.poll(crate::time_now().0 as i64);
            {
                let mut t_s: net::SocketRef<net::TcpSocket> = n.sockets.get::<net::TcpSocket>(n.tcp_handle);
                if !t_s.is_open() {
                    t_s.listen(1234).unwrap();
                }
                if t_s.may_recv() {
                    let data = t_s.recv(|buffer| {
                        let len = buffer.len();
                        let mut data: [u8; net::config::TCP_SERVER_RX_SIZE] = [0; net::config::TCP_SERVER_RX_SIZE];
                        data[0..len].copy_from_slice(&buffer[0..len]);
                        (len, (data, len))
                    }).unwrap();
                    if t_s.can_send() {
                        t_s.send_slice(&data.0[0..data.1]).unwrap();
                    }
                }
                else if t_s.may_send() {
                    t_s.close();
                }
            }
            n.poll(crate::time_now().0 as i64)
        });
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    rprintln!("Panic {:?}", _info);
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
