use embedded_websocket::framer::{Stream, ReadResult};
use smoltcp::{Error, socket::TcpSocket};
use smoltcp::socket::SocketRef;

pub use embedded_websocket::WebSocketServer;
pub use embedded_websocket::WebSocketState;
pub use embedded_websocket::WebSocketOptions;
pub use embedded_websocket::framer::Framer;
pub use embedded_websocket::WebSocketContext;
use bbqueue::BBBuffer;
use embedded_websocket::{Server, EmptyRng, WebSocket, WebSocketReadResult, WebSocketReceiveMessageType, WebSocketSendMessageType};
use core::borrow::{BorrowMut, Borrow};
use rtt_target::rprintln;
#[derive(Debug)]
pub enum BMWSError {
    Err,
    WebSocket(embedded_websocket::Error),
    //TODO add errors
}

#[derive(PartialEq, Copy, Clone, Debug)]
pub enum BMWSState{
    Connected(embedded_websocket::WebSocketState),
    NotConnected
    //TODO add errors
}

#[derive(PartialEq, Copy, Clone, Debug)]
pub enum FrameResult{
    None,
    WebSocketType(WebSocketReceiveMessageType)
}


#[derive(Debug)]
pub enum HeaderError{
    Partial,
    WebSocket(embedded_websocket::Error),
    NoContext,
    //TODO add errors
}

pub struct BareMetalWebSocketServer{
    ws_server: Option<WebSocketServer>,
    ws_context: Option<WebSocketContext>,
    pub(crate) state: BMWSState,
    buf_rx: [u8; 1000],
    buf_tx: [u8; 1000]
}

impl BareMetalWebSocketServer{
    pub fn new() -> BareMetalWebSocketServer{
        BareMetalWebSocketServer{
            ws_server: None,
            ws_context: None,
            state: BMWSState::NotConnected,
            buf_rx: [0u8; 1000],
            buf_tx: [0u8; 1000]
        }
    }

    pub fn state(&self) -> BMWSState {
        self.state
    }

    pub fn is_connected(&self) -> bool {
        self.state != BMWSState::NotConnected
    }

    pub fn init(&mut self, ctx: WebSocketContext){
        self.ws_server = Some(WebSocketServer::new_server());
        self.ws_context = Some(ctx);
    }

    pub fn accept(&mut self) -> Result<&[u8], BMWSError> {
        let len = match self.ws_server.as_mut().unwrap().server_accept(
            self.ws_context.as_ref().unwrap().sec_websocket_key.borrow(),
            None,
            self.buf_rx.borrow_mut()
        ){
            Ok(len) => { len }
            Err(e) => { return Err(BMWSError::WebSocket(e))}
        };
        self.state = BMWSState::Connected(self.ws_server.as_ref().unwrap().state);
        Ok(&self.buf_rx[0..len])
    }

    pub fn get_ws_frame<'a>(&mut self, typ: FrameResult, data: &[u8]) -> Option<&[u8]> {
        match typ{
            FrameResult::None => {
                self.buf_tx[..data.len()].copy_from_slice(data);
                Some(&self.buf_tx[..data.len()])
            }
            FrameResult::WebSocketType(msg_type) => {
                let send_msg_type = match msg_type{
                    WebSocketReceiveMessageType::Text => { WebSocketSendMessageType::Text }
                    WebSocketReceiveMessageType::Binary => { WebSocketSendMessageType::Binary }
                    WebSocketReceiveMessageType::CloseMustReply => { WebSocketSendMessageType::CloseReply }
                    WebSocketReceiveMessageType::Ping => { WebSocketSendMessageType::Pong }
                    _ => {WebSocketSendMessageType::Binary }
                };


                let len = self.ws_server.as_mut().unwrap()
                    .write(send_msg_type, true, data, &mut self.buf_tx).unwrap();

                Some(&self.buf_tx[..len])
            }
        }
    }

    pub fn read(&mut self, read_buf: &[u8]) -> Result<(FrameResult, Option<&[u8]>),BMWSError>{

        if read_buf.is_empty() {
            return Ok((FrameResult::None, None))
        }

        let ws_result = match self.ws_server.as_mut().unwrap()
            .read(
                read_buf,
                &mut self.buf_rx[0..],
            ){
            Ok(res) => { res }
            Err(e) => { return Err(BMWSError::Err) }
        };
        match ws_result.message_type {
            WebSocketReceiveMessageType::CloseCompleted => {
                self.state = BMWSState::NotConnected;
                return Ok((FrameResult::WebSocketType(WebSocketReceiveMessageType::CloseCompleted), None))
            },
            WebSocketReceiveMessageType::Ping => {
                return Ok((FrameResult::WebSocketType(WebSocketReceiveMessageType::Ping), None))
            },
            WebSocketReceiveMessageType::Pong => {
                return Ok((FrameResult::WebSocketType(WebSocketReceiveMessageType::Pong), None))
            },
            WebSocketReceiveMessageType::CloseMustReply => {
                self.state = BMWSState::NotConnected;
                rprintln!("{:?}", self.ws_server.as_ref().unwrap().state);
                let len = self.ws_server.as_mut().unwrap()
                    .write(WebSocketSendMessageType::CloseReply, true, &self.buf_rx[..ws_result.len_to], &mut self.buf_tx).unwrap();
                self.ws_server.as_mut().unwrap().state = WebSocketState::None;
                return Ok((FrameResult::WebSocketType(WebSocketReceiveMessageType::CloseMustReply),
                           Some(&self.buf_tx[..len])))
            }
            _ => {
                let len = self.ws_server.as_mut().unwrap()
                    .write(WebSocketSendMessageType::Text, true, &self.buf_rx[..ws_result.len_to], &mut self.buf_tx).unwrap();
                return Ok((FrameResult::WebSocketType(WebSocketReceiveMessageType::Text),
                           Some(&self.buf_tx[..len])))
            },
        }
    }
}


pub fn https_request_parser(data: &[u8]) -> Option<WebSocketContext>{
    let mut headers = [httparse::EMPTY_HEADER; 16];
    let mut request = httparse::Request::new(&mut headers);
    let ws_option = match request
        .parse(data)
        .unwrap()
    {
        httparse::Status::Complete(len) => {
            let headers = request.headers.iter().map(|f| (f.name, f.value));
            match embedded_websocket::read_http_header(headers) {
                Ok(ws) => {
                    return ws;
                }
                Err(e) => {
                    return None
                }
            }
        }
        httparse::Status::Partial => {
            return None;
        }
    };
}











#[derive(Debug)]
pub enum WebSocketError{
    Err,
    //TODO add errors
}

impl Stream<WebSocketError> for SocketRef<'_, TcpSocket<'_>> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, WebSocketError> {
        if self.may_recv(){
            match self.recv(|buffer| {
                let len = buffer.len();
                (len, buffer)
            }){
                Ok(rx) => {
                    let len = rx.len();
                    buf[0..len].copy_from_slice(&rx[0..len]);
                    return Ok(len)
                }
                Err(e) => {
                    return Err(WebSocketError::Err);
                }
            }
        }
        Err(WebSocketError::Err)
    }

    fn write_all(&mut self, buf: &[u8]) -> Result<(), WebSocketError> {
        if self.can_send() {
            let len = buf.len();
            match self.send_slice(&buf[0..len]){
                Ok(size) => { return Ok(()); } // ????
                Err(e) => { return Err(WebSocketError::Err); }
            }
        }
        Err(WebSocketError::Err)
    }
}

pub const ROOT_HTML : &str = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\nContent-Length: 2590\r\nConnection: close\r\n\r\n<!doctype html>
<html>
<head>
    <meta content='text/html;charset=utf-8' http-equiv='Content-Type' />
    <meta content='utf-8' http-equiv='encoding' />
    <meta name='viewport' content='width=device-width, initial-scale=0.5, maximum-scale=0.5, user-scalable=0' />
    <meta name='apple-mobile-web-app-capable' content='yes' />
    <meta name='apple-mobile-web-app-status-bar-style' content='black' />
    <title>Web Socket Demo</title>
    <style type='text/css'>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font: 13px Helvetica, Arial; }
        form { background: #000; padding: 3px; position: fixed; bottom: 0; width: 100%; }
        form input { border: 0; padding: 10px; width: 90%; margin-right: .5%; }
        form button { width: 9%; background: rgb(130, 200, 255); border: none; padding: 10px; }
        #messages { list-style-type: none; margin: 0; padding: 0; }
        #messages li { padding: 5px 10px; }
        #messages li:nth-child(odd) { background: #eee; }
    </style>
</head>
<body>
    <ul id='messages'></ul>
    <form action=''>
    <input id='txtBox' autocomplete='off' /><button>Send</button>
    </form>
    <script type='text/javascript' src='http://code.jquery.com/jquery-1.11.1.js' ></script>
    <script type='text/javascript'>
        var CONNECTION;
        window.onload = function () {
            // open the connection to the Web Socket server
            CONNECTION = new WebSocket('ws://localhost:1337/chat');
			// CONNECTION = new WebSocket('ws://' + location.host + ':1337/chat');

            // When the connection is open
            CONNECTION.onopen = function () {
                $('#messages').append($('<li>').text('Connection opened'));
            };

            // when the connection is closed by the server
            CONNECTION.onclose = function () {
                $('#messages').append($('<li>').text('Connection closed'));
            };

            // Log errors
            CONNECTION.onerror = function (e) {
                console.log('An error occured');
            };

            // Log messages from the server
            CONNECTION.onmessage = function (e) {
                $('#messages').append($('<li>').text(e.data));
            };
        };

		$(window).on('beforeunload', function(){
			CONNECTION.close();
		});

        // when we press the Send button, send the text to the server
        $('form').submit(function(){
            CONNECTION.send($('#txtBox').val());
            $('#txtBox').val('');
            return false;
        });
    </script>
</body>
</html>";