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

use log::info;
use serde_json::Value;
use std::sync::mpsc::{sync_channel, Receiver, SyncSender};
use std::thread::sleep;
use std::time::Duration;

use crate::constants::BOUND_SYNC_CHANNEL;
use crate::interface::tcp_server::TcpServer;
use crate::utility::{get_message_name, is_telemetry};

pub struct TelemetryServer {
    // Sender of the messages to the TCP/IP.
    _sender_to_tcp: SyncSender<Vec<Value>>,
    // Receiver of the messages to the TCP/IP.
    pub receiver_to_tcp: Receiver<Vec<Value>>,
    // Sender of the messages from the TCP/IP.
    pub sender_from_tcp: SyncSender<Value>,
}

impl TelemetryServer {
    /// Create a new telemetry server instance.
    ///
    /// # Arguments
    /// * `sender` - Sender of the messages from the TCP/IP.
    ///
    /// # Returns
    /// Telemetry server.
    pub fn new(sender: &SyncSender<Value>) -> Self {
        let (sender_to_tcp, receiver_to_tcp) = sync_channel(BOUND_SYNC_CHANNEL);

        Self {
            _sender_to_tcp: sender_to_tcp,
            receiver_to_tcp: receiver_to_tcp,

            sender_from_tcp: sender.clone(),
        }
    }

    /// Get the sender to the TCP/IP.
    ///
    /// # Returns
    /// Sender.
    pub fn get_sender_to_tcp(&self) -> SyncSender<Vec<Value>> {
        self._sender_to_tcp.clone()
    }

    /// Process the telemetry.
    ///
    /// # Arguments
    /// * `tcp_server` - TCP server.
    /// * `telemetry_server` - Telemetry server.
    pub fn process_telemetry(tcp_server: &mut TcpServer, telemetry_server: &mut TelemetryServer) {
        let mut is_processed = false;

        // Check the telemetry from the TCP/IP and send to the internal use.
        let telemetry_received = tcp_server.read_json();
        if !telemetry_received.is_null() {
            let name = get_message_name(&telemetry_received);
            if is_telemetry(&name) {
                // Ignore the error if the receiver is disconnected.
                let _ = telemetry_server
                    .sender_from_tcp
                    .try_send(telemetry_received);

                is_processed = true;
            } else {
                info!("Invalid telemetry message: {telemetry_received}.");
            }
        }

        // Check the internal telemetry and send to the TCP/IP.
        if let Ok(telemetry_send) = telemetry_server.receiver_to_tcp.try_recv() {
            tcp_server.write_jsons(&telemetry_send);

            is_processed = true;
        }

        // Sleep for a while to avoid busy waiting if no telemetry is received or
        // sent.
        if !is_processed {
            sleep(Duration::from_millis(tcp_server.timeout));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use serde_json::json;
    use std::net::TcpStream;
    use std::sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    };
    use std::thread::spawn;

    use crate::constants::{LOCAL_HOST, TERMINATOR};
    use crate::utility::{client_read_and_assert, client_write_and_sleep};

    const SLEEP_TIME: u64 = 100;
    const MAX_TIMEOUT: u64 = 200;

    fn create_telemetry_server() -> (TelemetryServer, SyncSender<Value>, Receiver<Value>) {
        let (sender_from_tcp, receiver_from_tcp) = sync_channel(BOUND_SYNC_CHANNEL);

        (
            TelemetryServer::new(&sender_from_tcp),
            sender_from_tcp,
            receiver_from_tcp,
        )
    }

    fn create_tcp_server() -> (TcpServer, Arc<AtomicBool>) {
        let stop = Arc::new(AtomicBool::new(false));
        let server = TcpServer::new(
            "telemetry",
            LOCAL_HOST,
            0,
            SLEEP_TIME,
            &TERMINATOR.to_vec(),
            &stop,
        );

        (server, stop)
    }

    fn create_tcp_client(port: i32) -> TcpStream {
        TcpStream::connect(format!("{}:{}", LOCAL_HOST, port)).expect("Tcp stream should connect.")
    }

    #[test]
    fn test_process_telemetry() {
        let (mut telemetry_server, _, receiver_from_tcp) = create_telemetry_server();
        let sender = telemetry_server.get_sender_to_tcp();

        let (mut tcp_server, stop) = create_tcp_server();
        let port = tcp_server.get_port();

        let handle = spawn(move || {
            tcp_server.run(
                TelemetryServer::process_telemetry,
                None::<fn(&mut TcpServer, &mut TelemetryServer)>,
                &mut telemetry_server,
            );
        });

        // Create a client to connect to the server.
        let mut client = create_tcp_client(port);
        sleep(Duration::from_millis(MAX_TIMEOUT));

        // Read the message from the TCP/IP.
        for value in 0..3 {
            client_write_and_sleep(
                &mut client,
                &format!("{{\"id\":\"tel_tlm\",\"value\":{value}}}\r\n"),
                SLEEP_TIME,
            );

            assert_eq!(
                receiver_from_tcp.recv().unwrap(),
                json!({"id": "tel_tlm", "value": value})
            );
        }

        // Write the message to the TCP/IP.
        for value in 0..3 {
            sender
                .try_send(vec![
                    json!({"id": "tel_tlm1", "value": value}),
                    json!({"id": "tel_tlm2", "value": value}),
                ])
                .unwrap();

            sleep(Duration::from_millis(SLEEP_TIME));

            client_read_and_assert(
                &mut client,
                &format!("{{\"id\":\"tel_tlm1\",\"value\":{value}}}\r\n{{\"id\":\"tel_tlm2\",\"value\":{value}}}\r\n"),
            );
        }

        // close the server.
        stop.store(true, Ordering::Relaxed);

        assert!(handle.join().is_ok());
    }
}
