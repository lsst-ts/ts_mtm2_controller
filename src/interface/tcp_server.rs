// This file is part of ts_mtm2_controller.
//
// Developed for the Vera Rubin Observatory Systems.
// This product includes software developed by the LSST Project
// (https://www.lsst.org).
// See the COPYRIGHT file at the top-level directory of this distribution
// for details of code ownership.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

use log::{debug, info};
use serde_json::Value;
use std::io::{BufReader, BufWriter, Read, Write};
use std::net::{Shutdown, TcpListener, TcpStream};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::thread::sleep;
use std::time::Duration;

pub struct TcpServer {
    _name: String,
    _listener: TcpListener,
    _reader: Option<BufReader<TcpStream>>,
    _writer: Option<BufWriter<TcpStream>>,
    // Timeout in milliseconds.
    pub timeout: u64,
    // Buffer to read the received message.
    _buffer: Vec<u8>,
    // Terminator of the message.
    _terminator: Vec<u8>,
    // Stop the server.
    _stop: Arc<AtomicBool>,
    // The server is connected or not.
    _is_connected: Arc<AtomicBool>,
}

impl TcpServer {
    /// Create a new TcpServer instance.
    ///
    /// # Arguments
    /// * `name` - Name of ther server.
    /// * `host` - A string slice that holds the hostname or IP address.
    /// * `port` - An integer that holds the port number. Put 0 to let the OS
    /// choose the port number.
    /// * `timeout` - Timeout in milliseconds.
    /// * `terminator` - A vector that holds the terminator.
    /// * `stop` - An Arc instance that holds the AtomicBool instance to stop
    /// the server.
    ///
    /// # Returns
    /// A TcpServer instance.
    pub fn new(
        name: &str,
        host: &str,
        port: i32,
        timeout: u64,
        terminator: &Vec<u8>,
        stop: &Arc<AtomicBool>,
    ) -> Self {
        let listener =
            TcpListener::bind(format!("{}:{}", host, port)).expect("Tcp listener should bind.");
        listener
            .set_nonblocking(true)
            .expect("Tcp listener should set non-blocking.");

        const DEFAULT_BUFFER_SIZE: usize = 100;

        Self {
            _name: String::from(name),

            _listener: listener,
            _reader: None,
            _writer: None,
            timeout: timeout,

            _buffer: Vec::with_capacity(DEFAULT_BUFFER_SIZE),
            _terminator: terminator.clone(),

            _stop: stop.clone(),
            _is_connected: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Get the port number.
    ///
    /// # Returns
    /// An integer that holds the port number. If the port number is not
    /// available, return -1.
    pub fn get_port(&self) -> i32 {
        match self._listener.local_addr() {
            Ok(addr) => addr.port() as i32,
            Err(_) => -1,
        }
    }

    /// Accept a connection.
    ///
    /// # Returns
    /// A boolean value. If the connection is accepted, return true. Otherwise,
    /// return false.
    fn accept(&mut self) -> bool {
        if let Ok((stream, _)) = self._listener.accept() {
            stream
                .set_nodelay(true)
                .expect("Tcp stream should set nodelay.");
            stream
                .set_nonblocking(true)
                .expect("Tcp stream should set non-blocking.");

            if let Ok(peer_addr) = stream.peer_addr() {
                info!("{} is connected from {}.", self._name, peer_addr);
            }

            self._is_connected.store(true, Ordering::Relaxed);

            // Wrap the stream to the BufReader and BufWriter.
            self._reader = Some(BufReader::new(
                stream.try_clone().expect("Tcp stream should clone."),
            ));
            self._writer = Some(BufWriter::new(stream));

            return true;
        };

        false
    }

    /// Check if the server is connected.
    ///
    /// # Returns
    /// A boolean value. If the server is connected, return true. Otherwise,
    /// return false.
    fn is_connected(&self) -> bool {
        self._reader.is_some() && self._writer.is_some()
    }

    /// Get the reference to the connection status. This is useful to check the
    /// connection status in a separate thread.
    ///
    /// # Returns
    /// An Arc instance to check the connection status.
    pub fn get_connection_status_reference(&self) -> Arc<AtomicBool> {
        self._is_connected.clone()
    }

    /// Read the JSON data.
    ///
    /// # Returns
    /// A Value instance that holds the JSON data.
    pub fn read_json(&mut self) -> Value {
        let data = self.read_string();
        match serde_json::from_str(&data) {
            Ok(value) => value,
            Err(_) => {
                if !data.is_empty() {
                    info!("{} receives non-JSON message: {}.", self._name, data);
                }
                Value::Null
            }
        }
    }

    /// Read the string data.
    ///
    /// # Returns
    /// A string slice that holds the data.
    fn read_string(&mut self) -> String {
        let terminator_length = self._terminator.len();
        // Always clear the buffer before reading the data.
        self._buffer.clear();
        if let Some(stream) = self._reader.as_mut() {
            let mut byte = [0; 1];
            loop {
                match stream.read(&mut byte) {
                    Ok(0) => {
                        // The client is disconnected.
                        debug!(
                            "{} is disconnected from the client when reading.",
                            self._name
                        );
                        self.close_stream();
                        break;
                    }

                    Ok(_) => {
                        self._buffer.push(byte[0]);
                        // Check the terminator.
                        if self._buffer.ends_with(&self._terminator) {
                            break;
                        }
                    }

                    Err(_) => {
                        break;
                    }
                }
            }
        }

        if self._buffer.len() < terminator_length {
            String::new()
        } else {
            match String::from_utf8(
                self._buffer[..(self._buffer.len() - terminator_length)].to_vec(),
            ) {
                Ok(message) => {
                    debug!("{} receives: {}.", self._name, message);

                    message
                }
                Err(_) => String::new(),
            }
        }
    }

    /// Write the JSON item.
    ///
    /// # Arguments
    /// * `item` - A Value instance that holds the JSON data.
    pub fn write_json(&mut self, item: &Value) {
        self.write_string(item.to_string());
        self.flush();
    }

    /// Write the JSON items. This is used for the telemetry. If there is still
    /// data in the buffer, we don't want to write the new data but to flush the
    /// existing data instead.
    ///
    /// # Arguments
    /// * `items` - A vector of Value instances that holds the JSON data.
    pub fn write_jsons(&mut self, items: &Vec<Value>) {
        if let Some(stream) = self._writer.as_mut() {
            if stream.buffer().len() != 0 {
                self.flush();
                return;
            }
        }

        for item in items {
            self.write_string(item.to_string());
        }
        self.flush();
    }

    /// Write the string data to buffer.
    ///
    /// # Arguments
    /// * `data` - A string that holds the data.
    fn write_string(&mut self, data: String) {
        if let Some(stream) = self._writer.as_mut() {
            // Add the terminator.
            let mut data_with_terminator = data;
            data_with_terminator.push_str(&String::from_utf8_lossy(&self._terminator));

            // Write the data to the buffer. Ignore the possible error.
            let _ = stream.write(data_with_terminator.as_bytes());
        }
    }

    /// Flush the stream.
    fn flush(&mut self) {
        if let Some(stream) = self._writer.as_mut() {
            if let Err(error) = stream.flush() {
                match error.kind() {
                    std::io::ErrorKind::WouldBlock => {
                        // The operation would block, we can try again later.
                    }
                    _ => {
                        // The client is disconnected.
                        debug!(
                            "{} fails to flush the stream: {}. Diconnecting...",
                            self._name, error
                        );
                        self.close_stream();
                    }
                }
            }
        }
    }

    /// Close the stream.
    fn close_stream(&mut self) {
        info!("{} is disconnected.", self._name);

        // Flush the stream in the writer first before closing the stream.
        if let Some(stream) = self._writer.as_mut() {
            // Ignore the possible error.
            let _ = stream.flush();
        }

        // Shutdown the stream in the reader and writer.
        if let Some(stream) = self._reader.as_ref() {
            // Ignore the possible error.
            let _ = stream.get_ref().shutdown(Shutdown::Both);
        }

        self._reader = None;
        self._writer = None;

        self._is_connected.store(false, Ordering::Relaxed);
    }

    /// Run the server.
    ///
    /// # Arguments
    /// * `callback_periodic` - A periodic callback function that is called
    /// once the server is connected. It takes two mutable references to the
    /// TcpServer and other data.
    /// * `callback_first_time` - An optional callback function that is called
    /// once the server is connected for the first time. It takes two mutable
    /// references to the TcpServer and other data.
    /// * `other` - A mutable reference to the other data that is used in
    /// callback function.
    pub fn run<F1, F2, T>(
        &mut self,
        mut callback_periodic: F1,
        mut callback_first_time: Option<F2>,
        other: &mut T,
    ) where
        F1: FnMut(&mut TcpServer, &mut T),
        F2: FnMut(&mut TcpServer, &mut T),
    {
        info!("{} is running.", self._name);
        while !self._stop.load(Ordering::Relaxed) {
            if self.is_connected() {
                callback_periodic(self, other);
            } else {
                if self.accept() {
                    if let Some(ref mut callback) = callback_first_time {
                        callback(self, other);
                    }
                } else {
                    sleep(Duration::from_millis(self.timeout));
                }
            }
        }

        info!("{} is stopped.", self._name);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use serde_json::json;
    use std::sync::Arc;
    use std::thread::spawn;

    use crate::constants::{LOCAL_HOST, TERMINATOR};
    use crate::utility::client_read_and_assert;

    const MAX_TIMEOUT: u64 = 200;

    fn create_tcp_server() -> (TcpServer, Arc<AtomicBool>) {
        let stop = Arc::new(AtomicBool::new(false));
        let server = TcpServer::new("server", LOCAL_HOST, 0, 100, &TERMINATOR.to_vec(), &stop);

        (server, stop)
    }

    fn create_tcp_client(port: i32) -> TcpStream {
        TcpStream::connect(format!("{}:{}", LOCAL_HOST, port)).expect("Tcp stream should connect.")
    }

    fn accept_connection(server: &mut TcpServer) -> bool {
        let max_timeout_counter = MAX_TIMEOUT / server.timeout;

        let mut timeout_counter = 0;
        while timeout_counter < max_timeout_counter {
            if server.accept() {
                return true;
            } else {
                timeout_counter += 1;

                sleep(Duration::from_millis(server.timeout));
            }
        }

        false
    }

    fn callback_periodic(server: &mut TcpServer, other: &mut i32) {
        if server.read_json().is_null() {
            *other += 1;
        }

        server.write_string(other.to_string());

        sleep(Duration::from_millis(200));
    }

    #[test]
    fn test_stop() {
        let (server, stop) = create_tcp_server();

        assert_eq!(Arc::strong_count(&stop), 2);

        stop.store(true, Ordering::Relaxed);

        assert!(server._stop.load(Ordering::Relaxed));
    }

    #[test]
    fn test_get_port() {
        let server = create_tcp_server().0;

        assert!(server.get_port() > 0);
    }

    #[test]
    fn test_accept_fail() {
        let mut server = create_tcp_server().0;

        assert!(!accept_connection(&mut server));
    }

    #[test]
    fn test_accept_scuccess() {
        let mut server = create_tcp_server().0;

        let _client = create_tcp_client(server.get_port());

        assert!(accept_connection(&mut server));
    }

    #[test]
    fn test_is_connected() {
        let mut server = create_tcp_server().0;
        let is_connected = server.get_connection_status_reference();

        // Not connected.
        assert!(!server.is_connected());
        assert!(!is_connected.load(Ordering::Relaxed));

        // Connected.
        let _client = create_tcp_client(server.get_port());
        accept_connection(&mut server);

        assert!(server.is_connected());
        assert!(is_connected.load(Ordering::Relaxed));
    }

    #[test]
    fn test_read_json() {
        let mut server = create_tcp_server().0;

        // Not connected. But it is safe to read the JSON data.
        assert!(server.read_json().is_null());

        // Connected.
        let mut client = create_tcp_client(server.get_port());
        accept_connection(&mut server);

        // Not a JSON data.
        client
            .write_all("abc\r\n".as_bytes())
            .expect("Tcp stream should write.");
        client.flush().expect("Tcp stream should flush.");
        sleep(Duration::from_millis(server.timeout));

        assert!(server.read_json().is_null());

        // Write the JSON data.
        client
            .write_all("{\"id\":\"name\",\"value\":1}\r\n".as_bytes())
            .expect("Tcp stream should write.");
        client.flush().expect("Tcp stream should flush.");
        sleep(Duration::from_millis(server.timeout));

        assert_eq!(server.read_json(), json!({"id":"name","value":1}));

        // No new data.
        assert!(server.read_json().is_null());
    }

    #[test]
    fn test_read_string() {
        let mut server = create_tcp_server().0;

        // Not connected. But it is safe to read the string data.
        assert_eq!(server.read_string(), "");

        // Connected.
        let mut client = create_tcp_client(server.get_port());
        accept_connection(&mut server);
        assert!(server.is_connected());

        // Safe to read the string data since the client is connected.
        assert_eq!(server.read_string(), "");

        // Write the data.
        client
            .write_all("data\r\n".as_bytes())
            .expect("Tcp stream should write.");
        client.flush().expect("Tcp stream should flush.");
        sleep(Duration::from_millis(server.timeout));

        assert_eq!(server.read_string(), "data");

        // Read the data again even though the data is empty.
        assert_eq!(server.read_string(), "");

        // Close the connection.
        client
            .shutdown(Shutdown::Both)
            .expect("Tcp stream of the client should shutdown.");
        sleep(Duration::from_millis(server.timeout));

        // After the reading, the server notices that the client is
        // disconnected.
        assert_eq!(server.read_string(), "");
        assert!(!server.is_connected());
    }

    #[test]
    fn test_write_json() {
        let mut server = create_tcp_server().0;

        // Not connected. But it is safe to write the JSON data.
        let item = json!({"id":"name","value":1});
        server.write_json(&item);

        // Connected.
        let mut client = create_tcp_client(server.get_port());
        accept_connection(&mut server);

        server.write_json(&item);

        client_read_and_assert(&mut client, "{\"id\":\"name\",\"value\":1}\r\n");
    }

    #[test]
    fn test_write_jsons() {
        let mut server = create_tcp_server().0;

        // Not connected. But it is safe to write the JSON data.
        let items = vec![
            json!({"id":"name","value":1}),
            json!({"id":"name","value":2}),
        ];
        server.write_jsons(&items);

        // Connected.
        let mut client = create_tcp_client(server.get_port());
        accept_connection(&mut server);

        server.write_jsons(&items);

        client_read_and_assert(
            &mut client,
            "{\"id\":\"name\",\"value\":1}\r\n{\"id\":\"name\",\"value\":2}\r\n",
        );
    }

    #[test]
    fn test_write_string() {
        let mut server = create_tcp_server().0;

        // Not connected. But it is safe to write the string data.
        server.write_string(String::from("data"));

        // Connected.
        let mut client = create_tcp_client(server.get_port());
        accept_connection(&mut server);
        assert!(server.is_connected());

        // Safe to write the string data since the client is connected.

        // Data 1.
        server.write_string(String::from("data1"));
        server.flush();

        client_read_and_assert(&mut client, "data1\r\n");

        // Data 2.
        server.write_string(String::from("data2"));
        server.flush();

        client_read_and_assert(&mut client, "data2\r\n");

        // Close the connection.
        client
            .shutdown(Shutdown::Both)
            .expect("Tcp stream of the client should shutdown.");

        // After the writing, the server notices that the client is
        // disconnected.
        for _ in 0..5 {
            server.write_string(String::from("data"));
            server.flush();
            sleep(Duration::from_millis(server.timeout));
        }

        assert!(!server.is_connected());
    }

    #[test]
    fn test_close_stream() {
        let mut server = create_tcp_server().0;
        let _client = create_tcp_client(server.get_port());
        accept_connection(&mut server);

        server.close_stream();

        assert!(server._reader.is_none());
        assert!(server._writer.is_none());
    }

    #[test]
    fn test_run() {
        let (mut server, stop) = create_tcp_server();
        let port = server.get_port();
        let is_connected = server.get_connection_status_reference();

        // Run the server in a separate thread
        let mut count = 0;
        let handle = spawn(move || {
            server.run(
                callback_periodic,
                None::<fn(&mut TcpServer, &mut i32)>,
                &mut count,
            );
        });

        // Create a client to connect to the server.
        let mut client = create_tcp_client(port);

        // Sleep for a while and check the connection.
        sleep(Duration::from_millis(MAX_TIMEOUT));
        assert!(client.peer_addr().is_ok());
        assert!(is_connected.load(Ordering::Relaxed));

        // Sleep for a while and close the server.
        sleep(Duration::from_millis(1000));
        stop.store(true, Ordering::Relaxed);

        for idx in 1..3 {
            client_read_and_assert(&mut client, format!("{}\r\n", idx).as_str());
        }

        assert!(handle.join().is_ok());
    }

    #[test]
    fn test_run_mulitple_connections() {
        let (mut server, stop) = create_tcp_server();
        let port = server.get_port();
        let is_connected = server.get_connection_status_reference();

        // Run the server in a separate thread
        let mut count = 0;
        let handle = spawn(move || {
            server.run(
                callback_periodic,
                None::<fn(&mut TcpServer, &mut i32)>,
                &mut count,
            );
        });

        // Create a client to connect to the server.
        for _ in 0..2 {
            let client = create_tcp_client(port);

            // Sleep for a while and check the connection.
            sleep(Duration::from_millis(MAX_TIMEOUT));
            assert!(is_connected.load(Ordering::Relaxed));

            // Close the client.
            let _ = client.shutdown(Shutdown::Both);

            // Sleep for a while and check the connection.
            sleep(Duration::from_millis(MAX_TIMEOUT));
            assert!(!is_connected.load(Ordering::Relaxed));
        }

        // Close the server.
        stop.store(true, Ordering::Relaxed);

        assert!(handle.join().is_ok());
    }
}
