//#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m;
use rtic::app;

#[macro_use]
#[allow(unused)]
mod utilities;
use log::info;

use smoltcp::iface::{
    EthernetInterface, EthernetInterfaceBuilder, Neighbor, NeighborCache,
    Route, Routes,
};
use smoltcp::socket::{SocketSet, SocketSetItem, UdpSocket, UdpSocketBuffer, UdpPacketMetadata, TcpSocketBuffer, TcpSocket, SocketHandle, SocketRef};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv6Cidr, IpEndpoint};

use gpio::Speed::*;
use stm32h7xx_hal::gpio;
use stm32h7xx_hal::hal::digital::v2::OutputPin;
use stm32h7xx_hal::rcc::CoreClocks;
use stm32h7xx_hal::{ethernet, ethernet::PHY};
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

/// TIME is an atomic u32 that counts milliseconds.
static TIME: AtomicU32 = AtomicU32::new(0);

/// Locally administered MAC address
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];

/// Ethernet descriptor rings are a global singleton
#[link_section = ".sram3.eth"]
static mut DES_RING: ethernet::DesRing = ethernet::DesRing::new();

/// Net storage with static initialisation - another global singleton
pub struct NetStorageStatic<'a> {
    ip_addrs: [IpCidr; 1],
    socket_set_entries: [Option<SocketSetItem<'a>>; 8],
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 8],
    routes_storage: [Option<(IpCidr, Route)>; 1],
}
static mut STORE: NetStorageStatic = NetStorageStatic {
    // Garbage
    ip_addrs: [IpCidr::Ipv6(Ipv6Cidr::SOLICITED_NODE_PREFIX)],
    socket_set_entries: [None, None, None, None, None, None, None, None],
    neighbor_cache_storage: [None; 8],
    routes_storage: [None; 1],
};

pub struct Net<'a> {
    iface: EthernetInterface<'a, ethernet::EthernetDMA<'a>>,
    sockets: SocketSet<'a>,
    udp_handle: SocketHandle,
    tcp_handle: SocketHandle,
}
impl<'a> Net<'a> {
    pub fn new(
        store: &'static mut NetStorageStatic<'a>,
        ethdev: ethernet::EthernetDMA<'a>,
        ethernet_addr: EthernetAddress,
    ) -> Self {
        // Set IP address
        store.ip_addrs =
            [IpCidr::new(IpAddress::v4(192, 168, 1, 10).into(), 0)];

        let neighbor_cache =
            NeighborCache::new(&mut store.neighbor_cache_storage[..]);
        let routes = Routes::new(&mut store.routes_storage[..]);

        let iface = EthernetInterfaceBuilder::new(ethdev)
            .ethernet_addr(ethernet_addr)
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut store.ip_addrs[..])
            .routes(routes)
            .finalize();


        let udp_socket = {
            static mut UDP_SERVER_RX_DATA: [u8; 128] = [0; 128];
            static mut UDP_SERVER_TX_DATA: [u8; 128] = [0; 128];
            static mut UDP_SERVER_RX_METADATA: [PacketMetadata<IpEndpoint>; 1] = [UdpPacketMetadata::EMPTY; 1];
            static mut UDP_SERVER_TX_METADATA: [PacketMetadata<IpEndpoint>; 1] = [UdpPacketMetadata::EMPTY; 1];
            let udp_rx_buffer = UdpSocketBuffer::new(unsafe { &mut UDP_SERVER_RX_METADATA[..] },unsafe { &mut UDP_SERVER_RX_DATA[..] });
            let udp_tx_buffer = UdpSocketBuffer::new(unsafe { &mut UDP_SERVER_TX_METADATA[..] },unsafe { &mut UDP_SERVER_TX_DATA[..] });
            UdpSocket::new(udp_rx_buffer, udp_tx_buffer)
        };
        let tcp_socket = {
            static mut TCP_SERVER_RX_DATA: [u8; 128] = [0; 128];
            static mut TCP_SERVER_TX_DATA: [u8; 128] = [0; 128];
            let tcp_rx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_SERVER_RX_DATA[..] });
            let tcp_tx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_SERVER_TX_DATA[..] });
            TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer)
        };


        let mut sockets = SocketSet::new(&mut store.socket_set_entries[..]);
        let udp_handle = sockets.add(udp_socket);
        let tcp_handle = sockets.add(tcp_socket);

        return Net { iface, sockets, udp_handle, tcp_handle };
    }

    /// Polls on the ethernet interface. You should refer to the smoltcp
    /// documentation for poll() to understand how to call poll efficiently
    pub fn poll(&mut self, now: i64) {
        let timestamp = Instant::from_millis(now);

        self.iface
            .poll(&mut self.sockets, timestamp)
            .map(|_| ())
            .unwrap_or_else(|e| info!("Poll: {:?}", e));
    }
}

use rtt_target::{rprintln};
use smoltcp::storage::PacketMetadata;
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
        let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);
        let (eth_dma, eth_mac) = unsafe {
            ethernet::new_unchecked(
                ctx.device.ETHERNET_MAC,
                ctx.device.ETHERNET_MTL,
                ctx.device.ETHERNET_DMA,
                &mut DES_RING,
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
        let store = unsafe { &mut STORE };
        let mut net = Net::new(store, eth_dma, mac_addr);

        {
            let mut u_s = net.sockets.get::<UdpSocket>(net.udp_handle);
            if !u_s.is_open() {
                info!("UDP_Socket: {:?}", u_s.bind(6969));
            }
        }

        {
            let mut t_s = net.sockets.get::<TcpSocket>(net.tcp_handle);
            if !t_s.is_open() {
                info!("TCP_Socket: {:?}", t_s.listen(1234));
            }
        }

        // 1ms tick
        systick_init(ctx.core.SYST, ccdr.clocks);

        net.poll( 0);
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
        {
            let mut u_s: SocketRef<UdpSocket> = ctx.resources.net.sockets.get::<UdpSocket>(ctx.resources.net.udp_handle);
            if u_s.is_open(){
                let client = match u_s.recv() {
                    Ok((data, endpoint)) => {
                        info!("u_rx: {:?}, from: {:?}", data, endpoint);
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
        {
            let mut t_s: SocketRef<TcpSocket> = ctx.resources.net.sockets.get::<TcpSocket>(ctx.resources.net.tcp_handle);
            if !t_s.is_open() {
                t_s.listen(1234).unwrap();
            }
              if t_s.can_recv() {
                  match t_s.recv(|buffer| {
                      info!("t_rx: {:?}", buffer);
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



                  /*if let Some(endpoint) = client {
                      let data = b"hello from STM32\n";
                      u_s.send_slice(data, endpoint).unwrap();
                  }*/
               }
        }

        let time = TIME.load(Ordering::Relaxed);
        ctx.resources.net.poll(time as i64);
    }

    #[task(binds = SysTick, priority=15)]
    fn systick_tick(_: systick_tick::Context) {
        TIME.fetch_add(1, Ordering::Relaxed);
    }
};
