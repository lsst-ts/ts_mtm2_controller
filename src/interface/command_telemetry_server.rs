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
use std::collections::HashMap;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    mpsc::SyncSender,
    Arc,
};
use std::thread::{sleep, spawn, JoinHandle};
use std::time::Duration;

use crate::constants::TERMINATOR;
use crate::enums::Commander;
use crate::interface::command_server::CommandServer;
use crate::interface::tcp_server::TcpServer;
use crate::interface::telemetry_server::TelemetryServer;
use crate::telemetry::event::Event;

pub struct CommandTelemetryServer {
    // Name of the server.
    _name: Commander,
    // A string that holds the hostname or IP address.
    _host: String,
    // Port numbers
    _ports: HashMap<String, i32>,
    // Timeout in milliseconds.
    pub timeout: u64,
    // An Arc instance that holds the AtomicBool instance to stop the server.
    _stop: Arc<AtomicBool>,
    // Sender of the messages from the TCP/IP.
    _sender_from_tcp: SyncSender<Value>,
    // Senders of the messages to the TCP/IP servers.
    pub senders_to_tcp: HashMap<String, Option<SyncSender<Vec<Value>>>>,
    // Server's connection status.
    _connections: HashMap<String, Arc<AtomicBool>>,
    // Thread handles for the command and telemetry servers
    _handles: Vec<JoinHandle<()>>,
    // List of the registered commands.
    _commands: Vec<String>,
}

impl CommandTelemetryServer {
    /// Create a new command telemetry server instance.
    ///
    /// # Arguments
    /// * `name` - Name of the server.
    /// * `host` - A string slice that holds the hostname or IP address.
    /// * `port_command` - An integer that holds the port number of command
    /// server. Put 0 to let the OS choose a port.
    /// * `port_telemetry` - An integer that holds the port number of telemetry
    /// server. Put 0 to let the OS choose a port.
    /// * `timeout` - Timeout in milliseconds.
    /// * `stop` - An Arc instance that holds the AtomicBool instance to stop
    /// the server.
    /// * `sender` - Sender of the messages from the TCP/IP.
    ///
    /// # Returns
    /// A command telemetry server instance.
    pub fn new(
        name: Commander,
        host: &str,
        port_command: i32,
        port_telemetry: i32,
        timeout: u64,
        stop: &Arc<AtomicBool>,
        sender: SyncSender<Value>,
    ) -> Self {
        let mut ports = HashMap::new();
        ports.insert("command".to_string(), port_command);
        ports.insert("telemetry".to_string(), port_telemetry);

        let mut connections = HashMap::new();
        connections.insert("command".to_string(), Arc::new(AtomicBool::new(false)));
        connections.insert("telemetry".to_string(), Arc::new(AtomicBool::new(false)));

        let mut senders_to_tcp = HashMap::new();
        senders_to_tcp.insert("command".to_string(), None);
        senders_to_tcp.insert("telemetry".to_string(), None);

        Self {
            _name: name,

            _host: String::from(host),
            _ports: ports,
            timeout: timeout,

            _stop: stop.clone(),

            _sender_from_tcp: sender,
            senders_to_tcp: senders_to_tcp,

            _connections: connections,

            _handles: Vec::new(),

            _commands: Vec::new(),
        }
    }

    /// Register a command.
    ///
    /// # Arguments
    /// * `command_name` - Name of the command that begins with "cmd_".
    /// Otherwise, it is ignored.
    pub fn register_command(&mut self, command_name: &str) {
        if !command_name.starts_with("cmd_") {
            return;
        }

        let command = String::from(command_name);
        if !self._commands.contains(&command) {
            self._commands.push(command);
        }
    }

    /// Run the command and telemetry servers.
    ///
    /// # Returns
    /// A tuple of two integers. The first integer is the command port number.
    /// The second integer is the telemetry port number.
    pub fn run_servers(&mut self) -> (i32, i32) {
        // Create the command and telemetry servers
        let mut command_server = CommandServer::new(&self._sender_from_tcp);
        let mut telemetry_server = TelemetryServer::new(&self._sender_from_tcp);

        // Register the commands
        command_server.register_commands(&self._commands);

        // Get the sender to the TCP/IP
        self.senders_to_tcp.insert(
            "command".to_string(),
            Some(command_server.get_sender_to_tcp()),
        );
        self.senders_to_tcp.insert(
            "telemetry".to_string(),
            Some(telemetry_server.get_sender_to_tcp()),
        );

        // Create the TCP servers
        let name_command_server = self._name.as_ref().to_string() + " CommandServer";
        let mut tcp_server_command = TcpServer::new(
            &name_command_server,
            &self._host,
            self._ports["command"],
            self.timeout,
            &TERMINATOR.to_vec(),
            &self._stop,
        );

        let name_telemetry_server = self._name.as_ref().to_string() + " TelemetryServer";
        let mut tcp_server_telemetry = TcpServer::new(
            &name_telemetry_server,
            &self._host,
            self._ports["telemetry"],
            self.timeout,
            &TERMINATOR.to_vec(),
            &self._stop,
        );

        // Get the reference to check if the server is connected
        self._connections.insert(
            "command".to_string(),
            tcp_server_command.get_connection_status_reference(),
        );
        self._connections.insert(
            "telemetry".to_string(),
            tcp_server_telemetry.get_connection_status_reference(),
        );

        // Get the ports
        let port_command = tcp_server_command.get_port();
        let port_telemetry = tcp_server_telemetry.get_port();

        // Run the servers
        let handle_command = spawn(move || {
            tcp_server_command.run(
                CommandServer::process_command,
                Some(CommandServer::process_first_connection),
                &mut command_server,
            );
        });

        let handle_telemetry = spawn(move || {
            tcp_server_telemetry.run(
                TelemetryServer::process_telemetry,
                None::<fn(&mut TcpServer, &mut TelemetryServer)>,
                &mut telemetry_server,
            );
        });

        self._handles.push(handle_command);
        self._handles.push(handle_telemetry);

        (port_command, port_telemetry)
    }

    /// Run the monitoring of the connection status.
    pub fn run_monitor_loop(&mut self) {
        info!(
            "{} command-telemetry server starts to monitor the connection status.",
            self._name.as_ref()
        );

        let mut are_connected = false;
        while !self._stop.load(Ordering::Relaxed) {
            if self.are_connected() != are_connected {
                are_connected = !are_connected;

                if let Err(_) = self
                    ._sender_from_tcp
                    .try_send(Event::get_message_tcp_ip_connected(are_connected))
                {
                    info!(
                        "{} command-telemetry server failed to send the connection status.",
                        self._name.as_ref()
                    );
                }
            }

            sleep(Duration::from_millis(self.timeout));
        }

        self.stop();

        info!(
            "{} command-telemetry server stops the monitoring of connection status.",
            self._name.as_ref()
        );
    }

    /// Check if the servers are connected.
    ///
    /// # Returns
    /// A boolean value. If the servers are connected, return true. Otherwise,
    /// return false.
    fn are_connected(&self) -> bool {
        self._connections["command"].load(Ordering::Relaxed)
            && self._connections["telemetry"].load(Ordering::Relaxed)
    }

    /// Stop the command and telemetry servers.
    fn stop(&mut self) {
        self._stop.store(true, Ordering::Relaxed);

        for handle in self._handles.drain(..) {
            handle.join().expect("Server handle should join.");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use serde_json::json;
    use std::net::{Shutdown, TcpStream};
    use std::sync::mpsc::{sync_channel, Receiver, SyncSender};
    use std::thread::sleep;
    use std::time::Duration;

    use crate::constants::{BOUND_SYNC_CHANNEL, LOCAL_HOST};
    use crate::enums::CommandStatus;
    use crate::utility::{acknowledge_command, client_read_and_assert, client_write_and_sleep};

    const TIMEOUT: u64 = 100;
    const MAX_TIMEOUT: u64 = 200;

    fn create_command_telemetry_server() -> (
        CommandTelemetryServer,
        Arc<AtomicBool>,
        SyncSender<Value>,
        Receiver<Value>,
    ) {
        let (sender_from_tcp, receiver_from_tcp) = sync_channel(BOUND_SYNC_CHANNEL);

        let stop = Arc::new(AtomicBool::new(false));
        let server = CommandTelemetryServer::new(
            Commander::GUI,
            LOCAL_HOST,
            0,
            0,
            TIMEOUT,
            &stop,
            sender_from_tcp.clone(),
        );

        (server, stop, sender_from_tcp, receiver_from_tcp)
    }

    fn create_tcp_clients(port_command: i32, port_telemetry: i32) -> (TcpStream, TcpStream) {
        (
            TcpStream::connect(format!("{}:{}", LOCAL_HOST, port_command))
                .expect("Tcp command stream should connect."),
            TcpStream::connect(format!("{}:{}", LOCAL_HOST, port_telemetry))
                .expect("Tcp telemetry stream should connect."),
        )
    }

    #[test]
    fn test_register_command() {
        let mut server = create_command_telemetry_server().0;

        // Ignore the command that does not start with "cmd_".
        server.register_command("test");
        assert_eq!(server._commands, Vec::<String>::new());

        // First command.
        server.register_command("cmd_test");
        assert_eq!(server._commands, vec!["cmd_test"]);

        // Repeated command.
        server.register_command("cmd_test");
        assert_eq!(server._commands, vec!["cmd_test"]);

        // Second command.
        server.register_command("cmd_test2");
        assert_eq!(server._commands, vec!["cmd_test", "cmd_test2"]);
    }

    #[test]
    fn test_are_connected() {
        let server = create_command_telemetry_server().0;

        assert!(!server.are_connected());
    }

    #[test]
    fn test_stop() {
        let mut server = create_command_telemetry_server().0;

        server.stop();

        assert!(server._stop.load(Ordering::Relaxed));
    }

    #[test]
    fn test_run_servers() {
        let (mut server, _, _, receiver_from_tcp) = create_command_telemetry_server();
        server.register_command("cmd_move");

        // Run the server
        let (port_command, port_telemetry) = server.run_servers();

        assert!(!server.are_connected());

        // Create the clients to connect the servers
        let (mut client_command, mut client_telemetry) =
            create_tcp_clients(port_command, port_telemetry);

        sleep(Duration::from_millis(MAX_TIMEOUT));
        assert!(server.are_connected());

        // Command client writes the command
        client_write_and_sleep(
            &mut client_command,
            "{\"id\":\"cmd_move\",\"sequence_id\":0}\r\n",
            TIMEOUT,
        );

        client_read_and_assert(
            &mut client_command,
            "{\"id\":\"ack\",\"sequence_id\":0}\r\n",
        );

        assert_eq!(
            receiver_from_tcp.recv().unwrap(),
            json!({"id": "cmd_move", "sequence_id": 0})
        );

        server.senders_to_tcp["command"]
            .as_ref()
            .unwrap()
            .try_send(vec![acknowledge_command(CommandStatus::Success, 0)])
            .expect("Sender should send the message.");

        sleep(Duration::from_millis(TIMEOUT));

        client_read_and_assert(
            &mut client_command,
            "{\"id\":\"success\",\"sequence_id\":0}\r\n",
        );

        // Telemetry client writes the telemetry
        client_write_and_sleep(
            &mut client_telemetry,
            "{\"id\":\"tel_tlm\",\"value\":1}\r\n",
            TIMEOUT,
        );

        assert_eq!(
            receiver_from_tcp.recv().unwrap(),
            json!({"id": "tel_tlm", "value": 1})
        );

        server.stop();
    }

    #[test]
    fn test_run_monitor_loop() {
        let (mut server, stop, _, receiver_from_tcp) = create_command_telemetry_server();

        // Run the servers
        let (port_command, port_telemetry) = server.run_servers();

        // Run the monitor loops
        let handle = spawn(move || {
            server.run_monitor_loop();
        });

        // Create the clients to connect the servers
        let (client_command, client_telemetry) = create_tcp_clients(port_command, port_telemetry);

        assert_eq!(
            receiver_from_tcp.recv().unwrap(),
            json!({"id": "tcpIpConnected", "isConnected": true})
        );

        // Client disconnects the servers
        let _ = client_command.shutdown(Shutdown::Both);
        let _ = client_telemetry.shutdown(Shutdown::Both);

        assert_eq!(
            receiver_from_tcp.recv().unwrap(),
            json!({"id": "tcpIpConnected", "isConnected": false})
        );

        stop.store(true, Ordering::Relaxed);
        let _ = handle.join();
    }
}
