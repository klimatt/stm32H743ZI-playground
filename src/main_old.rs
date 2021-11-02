//#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m;
use rtic::app;

#[macro_use]
#[allow(unused)]
mod utilities;
use log::info;



use gpio::Speed::*;
use stm32h7xx_hal::gpio;
use stm32h7xx_hal::hal::digital::v2::OutputPin;
use stm32h7xx_hal::rcc::CoreClocks;

use stm32h7xx_hal::{prelude::*, stm32};
use core::sync::atomic::{AtomicU32, Ordering};

use stm32h7xx_hal::hal::digital::v2::ToggleableOutputPin;
/// Configure SYSTICK for 1ms timebase
fn systick_init(mut syst: stm32::SYST, clocks: CoreClocks) {
    let c_ck_mhz = clocks.c_ck().0 / 1_000_000;

    let syst_calib = 0x3E8;

    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload((syst_calib * c_ck_mhz) - 1);
    syst.enable_interrupt();
    syst.enable_counter();
}











use core::fmt::Write;

#[app(device = stm32h7xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        net: Net<'static>,
        lan8742a: ethernet::phy::LAN8742A<ethernet::EthernetMAC>,
        link_led: gpio::gpiob::PB0<gpio::Output<gpio::PushPull>>,
        usr_led: gpio::gpioe::PE1<gpio::Output<gpio::PushPull>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> init::LateResources {
        utilities::logger::init();
        //rtt_init_print!();
        rprintln!("Start");
        info!("Start               ");
        // Initialise power...
        let pwr = ctx.device.PWR.constrain();
        let pwrcfg = example_power!(pwr).freeze();
        //rprintln!("Pwr Init");
        // Link the SRAM3 power state to CPU1
        ctx.device.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

        // Initialise clocks...
        let rcc = ctx.device.RCC.constrain();
        let ccdr = rcc
            .sys_ck(200.mhz())
            .hclk(200.mhz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);
        // Initialise system...
        ctx.core.SCB.enable_icache();
        // TODO: ETH DMA coherence issues
        // ctx.core.SCB.enable_dcache(&mut ctx.core.CPUID);
        ctx.core.DWT.enable_cycle_counter();

        // Initialise IO...
        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiog = ctx.device.GPIOG.split(ccdr.peripheral.GPIOG);
        let gpioe = ctx.device.GPIOE.split(ccdr.peripheral.GPIOE);
        //let gpioi = ctx.device.GPIOI.split(ccdr.peripheral.GPIOI);
        let mut link_led = gpiob.pb0.into_push_pull_output(); // LED3
        link_led.set_high().ok();

        let mut usr_led = gpioe.pe1.into_push_pull_output(); // LED2
        usr_led.set_high().ok();

        let _rmii_ref_clk = gpioa.pa1.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_mdio = gpioa.pa2.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_mdc = gpioc.pc1.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_crs_dv = gpioa.pa7.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_rxd0 = gpioc.pc4.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_rxd1 = gpioc.pc5.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_tx_en = gpiog.pg11.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_txd0 = gpiog.pg13.into_alternate_af11().set_speed(VeryHigh);
        let _rmii_txd1 = gpiob.pb13.into_alternate_af11().set_speed(VeryHigh);

        // Initialise ethernet...
        assert_eq!(ccdr.clocks.hclk().0, 200_000_000); // HCLK 200MHz
        assert_eq!(ccdr.clocks.pclk1().0, 100_000_000); // PCLK 100MHz
        assert_eq!(ccdr.clocks.pclk2().0, 100_000_000); // PCLK 100MHz
        assert_eq!(ccdr.clocks.pclk4().0, 100_000_000); // PCLK 100MHz

        rprintln!("Mac adddr");

        let store = unsafe { &mut net::STORE };
        let mut net = net::Net::new(store, eth_dma, mac_addr);
        {
            let mut u_s = net.sockets.get::<UdpSocket>(net.udp_handle);
            if !u_s.is_open() {
                info!("UDP_Socket: {:?}", u_s.bind(6969));
            }
        }
        // 1ms tick
        systick_init(ctx.core.SYST, ccdr.clocks);

        //network.poll( 0);
        init::LateResources {
            net,
            lan8742a,
            link_led,
            usr_led
        }
    }

    #[idle(resources = [lan8742a, link_led])]
    fn idle(ctx: idle::Context) -> ! {
        loop {
            match ctx.resources.lan8742a.poll_link() {
                true => ctx.resources.link_led.set_low(),
                _ => ctx.resources.link_led.set_high(),
            }
                .ok();
        }
    }

    #[task(binds = ETH, resources = [net, usr_led])]
    fn ethernet_event(ctx: ethernet_event::Context) {
        unsafe { ethernet::interrupt_handler() }
        ctx.resources.usr_led.toggle().unwrap();
        ctx.resources.net.poll();
        {
            let mut u_s: SocketRef<UdpSocket> = ctx.resources.net.sockets.get::<UdpSocket>(ctx.resources.net.udp_handle);
            if u_s.is_open(){
                let client = match u_s.recv() {
                    Ok((data, endpoint)) => {
                        //info!("u_rx: {:?}, from: {:?}", data, endpoint);
                        Some(endpoint)
                    }
                    Err(_) => None,
                };
                if let Some(endpoint) = client {
                    let data = b"UDP Hello from STM32H7!\n";
                    u_s.send_slice(data, endpoint).unwrap();
                }
            }
        }
        //ctx.resources.network.poll();
        {
            let mut t_s: SocketRef<TcpSocket> = ctx.resources.net.sockets.get::<TcpSocket>(ctx.resources.net.tcp_handle);
            if !t_s.is_open() {
                t_s.listen(1234).unwrap();
            }
            if t_s.can_recv() {
                match t_s.recv(|buffer| {
                    //info!("t_rx: {:?}", buffer);
                    let len = buffer.len();
                    (len, buffer)
                }){
                    Ok(_) => {
                        if t_s.can_send() {
                            t_s.write_str("TCP Hello from STM32H7!").unwrap();
                        }
                        t_s.close();
                    }
                    Err(_) => {}
                }
            }
        }
        ctx.resources.net.poll();
    }

    #[task(binds = SysTick, priority=15)]
    fn systick_tick(_: systick_tick::Context) {
        TIME.fetch_add(1, Ordering::Relaxed);
    }
};