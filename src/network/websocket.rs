use embedded_websocket::framer::Stream;
use smoltcp::{Error, socket::TcpSocket};
use smoltcp::socket::SocketRef;

pub use embedded_websocket::WebSocketServer;
pub use embedded_websocket::WebSocketState;
pub use embedded_websocket::WebSocketOptions;
pub use embedded_websocket::framer::Framer;
pub use embedded_websocket::WebSocketContext;

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

/*pub fn context_parser() -> Option<WebSocketContext>{

}*/

/*pub fn handle_non_websocket_http_request(stream: SocketRef<'_, TcpSocket<'_>>){
    stream.write_all(&ROOT_HTML.as_bytes());
}*/

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