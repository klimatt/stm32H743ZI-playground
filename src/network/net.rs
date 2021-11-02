use smoltcp::iface::{
    EthernetInterface, EthernetInterfaceBuilder, Neighbor, NeighborCache,
    Route, Routes,
};
pub use smoltcp::socket::{SocketSet, SocketSetItem, UdpSocket, UdpSocketBuffer, UdpPacketMetadata, TcpSocketBuffer, TcpSocket, SocketHandle, SocketRef};
use smoltcp::time::{Instant, Duration};
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv6Cidr, IpEndpoint};
use stm32h7xx_hal::{ethernet, ethernet::PHY};
use smoltcp::storage::PacketMetadata;
use rtt_target::{rprintln};
use core::borrow::{Borrow, BorrowMut};

pub use crate::network::config;
use smoltcp::Error;

/// Ethernet descriptor rings are a global singleton
#[link_section = ".sram3.eth"]
pub(crate) static mut DES_RING: ethernet::DesRing = ethernet::DesRing::new();

/// Net storage with static initialisation - another global singleton
pub struct NetStorageStatic<'a> {
    ip_addrs: [IpCidr; 1],
    socket_set_entries: [Option<SocketSetItem<'a>>; 8],
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 8],
    routes_storage: [Option<(IpCidr, Route)>; 1],
}
pub(crate) static mut STORE: NetStorageStatic = NetStorageStatic {
    // Garbage
    ip_addrs: [IpCidr::Ipv6(Ipv6Cidr::SOLICITED_NODE_PREFIX)],
    socket_set_entries: [None, None, None, None, None, None, None, None],
    neighbor_cache_storage: [None; 8],
    routes_storage: [None; 1],
};

pub struct Net<'a> {
    iface: EthernetInterface<'a, ethernet::EthernetDMA<'a>>,
    pub(crate) sockets: SocketSet<'a>,
    pub(crate) udp_handle: SocketHandle,
    pub(crate) tcp_handle: SocketHandle,
}

impl<'a> Net<'a> {
    pub fn new(
        store: &'static mut NetStorageStatic<'a>,
        ethdev: ethernet::EthernetDMA<'a>,
        ethernet_addr: EthernetAddress,
    ) -> Self {
        // Set IP address
        store.ip_addrs =
            [IpCidr::new(IpAddress::v4(192,168,1,10).into(), 0)];

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
            static mut TCP_SERVER_RX_DATA: [u8; config::TCP_SERVER_RX_SIZE] = [0; config::TCP_SERVER_RX_SIZE];
            static mut TCP_SERVER_TX_DATA: [u8; config::TCP_SERVER_TX_SIZE] = [0; config::TCP_SERVER_TX_SIZE];
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
    pub fn poll(&mut self, time_now: i64) -> Result<(), Error>{
        let timestamp = Instant::from_millis(time_now);

        self.iface
            .poll(&mut self.sockets, timestamp)
            .map(|_| ())
            //.unwrap_or_else(|e| rprintln!("Poll: {:?}", e));
    }

    /*pub fn try_poll(&mut self, time_now: i64) -> Result<(), Duration>{
        let timestamp = Instant::from_millis(time_now );
        match self.iface.poll_delay(&mut self.sockets, timestamp){
            None => { Err(Duration::from_millis(100))}
            Some(d) => {
                if d == Duration::from_millis(0){
                    Ok(())
                }
                else {
                    Err(d)
                }
            }
        }
    }*/
}