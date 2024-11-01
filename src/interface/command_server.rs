use log::info;
use serde_json::Value;
use std::sync::mpsc::{sync_channel, Receiver, SyncSender};
use std::thread::sleep;
use std::time::Duration;

use crate::constants::BOUND_SYNC_CHANNEL;
use crate::enums::CommandStatus;
use crate::interface::tcp_server::TcpServer;
use crate::utility::{
    acknowledge_command, get_message_name, get_message_sequence_id, is_command, is_event,
};

pub struct CommandServer {
    // Sender of the messages to the TCP/IP.
    _sender_to_tcp: SyncSender<Vec<Value>>,
    // Receiver of the messages to the TCP/IP.
    pub receiver_to_tcp: Receiver<Vec<Value>>,
    // Sender of the messages from the TCP/IP.
    pub sender_from_tcp: SyncSender<Value>,
    // List of the registered commands.
    _commands: Vec<String>,
    // Last sequence ID.
    _last_sequence_id: i64,
}

impl CommandServer {
    /// Create a new command server instance.
    ///
    /// # Arguments
    /// * `sender` - Sender of the messages from the TCP/IP.
    ///
    /// # Returns
    /// Command server.
    pub fn new(sender: &SyncSender<Value>) -> Self {
        let (sender_to_tcp, receiver_to_tcp) = sync_channel(BOUND_SYNC_CHANNEL);

        Self {
            _sender_to_tcp: sender_to_tcp,
            receiver_to_tcp: receiver_to_tcp,

            sender_from_tcp: sender.clone(),

            _commands: Vec::new(),

            _last_sequence_id: -1,
        }
    }

    /// Get the sender to the TCP/IP.
    ///
    /// # Returns
    /// Sender.
    pub fn get_sender_to_tcp(&self) -> SyncSender<Vec<Value>> {
        self._sender_to_tcp.clone()
    }

    /// Register the commands.
    ///
    /// # Arguments
    /// * `commands` - A vector of strings that holds the commands. The command
    /// name should begin with "cmd_"
    pub fn register_commands(&mut self, commands: &Vec<String>) {
        self._commands = commands.clone();
    }

    /// Check if the command is registered or not.
    ///
    /// # Arguments
    /// * `command` - Command that begins with "cmd_".
    ///
    /// # Returns
    /// True if the command is registered, false otherwise.
    pub fn is_registered_command(&self, command: &str) -> bool {
        self._commands.contains(&String::from(command))
    }

    /// Reset the sequence ID.
    pub fn reset_sequence_id(&mut self) {
        self._last_sequence_id = -1;
    }

    /// Check the sequence ID.
    ///
    /// # Arguments
    /// * `sequence_id` - Sequence ID, which should be >= 0.
    ///
    /// # Returns
    /// Lost sequence IDs.
    pub fn check_sequence_id(&mut self, sequence_id: i64) -> Vec<i64> {
        if self._last_sequence_id == -1 {
            self._last_sequence_id = sequence_id;
            return Vec::new();
        }

        let expected_sequence_id = self._last_sequence_id + 1;

        let mut lost_sequence_ids = Vec::new();
        if sequence_id > expected_sequence_id {
            lost_sequence_ids = (expected_sequence_id..sequence_id).collect();
        }

        self._last_sequence_id = sequence_id;

        return lost_sequence_ids;
    }

    /// Process the command.
    ///
    /// # Arguments
    /// * `tcp_server` - TCP server.
    /// * `command_server` - Command server.
    pub fn process_command(tcp_server: &mut TcpServer, command_server: &mut CommandServer) {
        let mut is_processed = false;

        // Check the command/event from the TCP/IP and send to the internal use.
        let message_received = tcp_server.read_json();
        if !message_received.is_null() {
            let mut is_ok_to_send = false;

            let name = get_message_name(&message_received);

            // Check if the message is a command.
            if is_command(&name) {
                let sequence_id = get_message_sequence_id(&message_received);

                // Check if the command is registered.
                if command_server.is_registered_command(&name) {
                    // Check the sequence ID. For the lost ones, send a NoAck.
                    let lost_sequence_ids = command_server.check_sequence_id(sequence_id);
                    for lost_sequence_id in lost_sequence_ids {
                        tcp_server.write_json(&acknowledge_command(
                            CommandStatus::NoAck,
                            lost_sequence_id,
                        ));
                    }

                    // Acknowledge the command.
                    tcp_server.write_json(&acknowledge_command(CommandStatus::Ack, sequence_id));

                    is_ok_to_send = true;
                } else {
                    tcp_server.write_json(&acknowledge_command(CommandStatus::NoAck, sequence_id));
                }
            } else if is_event(&name) {
                // Send the event to the internal use.
                is_ok_to_send = true;
            }

            if is_ok_to_send {
                // Ignore the error if the receiver is disconnected.
                let _ = command_server.sender_from_tcp.try_send(message_received);
            } else {
                info!("Invalid command/event message: {message_received}.");
            }

            is_processed = true;
        }

        // Check the internal command result or event, and send to the TCP/IP.
        if let Ok(message_send) = command_server.receiver_to_tcp.try_recv() {
            tcp_server.write_jsons(&message_send);

            is_processed = true;
        }

        // Sleep for a while to avoid busy waiting if no telemetry is received or
        // sent.
        if !is_processed {
            sleep(Duration::from_millis(tcp_server.timeout));
        }
    }

    /// Process the first connection.
    ///
    /// # Arguments
    /// * `tcp_server` - TCP server.
    /// * `command_server` - Command server.
    pub fn process_first_connection(
        _tcp_server: &mut TcpServer,
        command_server: &mut CommandServer,
    ) {
        command_server.reset_sequence_id();
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

    fn create_command_server() -> (CommandServer, SyncSender<Value>, Receiver<Value>) {
        let (sender_from_tcp, receiver_from_tcp) = sync_channel(BOUND_SYNC_CHANNEL);

        (
            CommandServer::new(&sender_from_tcp),
            sender_from_tcp,
            receiver_from_tcp,
        )
    }

    fn create_tcp_server() -> (TcpServer, Arc<AtomicBool>) {
        let stop = Arc::new(AtomicBool::new(false));
        let server = TcpServer::new(
            "command",
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
    fn test_register_commands() {
        let mut command_server = create_command_server().0;

        let commands = vec!["cmd_1".to_string(), "cmd_2".to_string()];
        command_server.register_commands(&commands);

        assert_eq!(command_server._commands, commands);
    }

    #[test]
    fn test_is_registered_command() {
        let mut command_server = create_command_server().0;

        command_server.register_commands(&vec!["cmd_test".to_string()]);

        assert_eq!(command_server.is_registered_command("cmd_test"), true);
        assert_eq!(command_server.is_registered_command("cmd_test2"), false);
    }

    #[test]
    fn test_reset_sequence_id() {
        let mut command_server = create_command_server().0;
        command_server._last_sequence_id = 5;

        command_server.reset_sequence_id();

        assert_eq!(command_server._last_sequence_id, -1);
    }

    #[test]
    fn test_check_sequence_id() {
        let mut command_server = create_command_server().0;

        // First connection
        assert_eq!(command_server.check_sequence_id(0), Vec::<i64>::new());
        assert_eq!(command_server._last_sequence_id, 0);

        assert_eq!(command_server.check_sequence_id(2), vec![1]);
        assert_eq!(command_server._last_sequence_id, 2);

        // Second connection
        command_server.reset_sequence_id();

        assert_eq!(command_server.check_sequence_id(5), Vec::<i64>::new());
        assert_eq!(command_server._last_sequence_id, 5);

        assert_eq!(command_server.check_sequence_id(6), Vec::<i64>::new());
        assert_eq!(command_server._last_sequence_id, 6);

        assert_eq!(command_server.check_sequence_id(9), vec![7, 8]);
        assert_eq!(command_server._last_sequence_id, 9);

        assert_eq!(command_server.check_sequence_id(10), Vec::<i64>::new());
        assert_eq!(command_server._last_sequence_id, 10);
    }

    #[test]
    fn test_process_command() {
        let (mut command_server, _, receiver_from_tcp) = create_command_server();

        // Change the last sequence ID to simulate the previous connection.
        command_server._last_sequence_id = 10;

        // Register the commands.
        command_server.register_commands(&vec!["cmd_move".to_string()]);

        let sender_to_tcp = command_server.get_sender_to_tcp();

        let (mut tcp_server, stop) = create_tcp_server();
        let port = tcp_server.get_port();

        let handle = spawn(move || {
            tcp_server.run(
                CommandServer::process_command,
                Some(CommandServer::process_first_connection),
                &mut command_server,
            );
        });

        // Create a client to connect to the server.
        let mut client = create_tcp_client(port);
        sleep(Duration::from_millis(MAX_TIMEOUT));

        // Read the command from the TCP/IP.

        // Not registered command.
        client_write_and_sleep(
            &mut client,
            "{\"id\":\"cmd_stop\",\"sequence_id\":0}\r\n",
            SLEEP_TIME,
        );

        client_read_and_assert(&mut client, "{\"id\":\"noack\",\"sequence_id\":0}\r\n");

        // Registered command.
        client_write_and_sleep(
            &mut client,
            "{\"id\":\"cmd_move\",\"sequence_id\":1}\r\n",
            SLEEP_TIME,
        );

        client_read_and_assert(&mut client, "{\"id\":\"ack\",\"sequence_id\":1}\r\n");

        assert_eq!(
            receiver_from_tcp.recv().unwrap(),
            json!({"id": "cmd_move", "sequence_id": 1})
        );

        // Get the NoAck for the lost sequence ID.
        client_write_and_sleep(
            &mut client,
            "{\"id\":\"cmd_move\",\"sequence_id\":4}\r\n",
            SLEEP_TIME,
        );

        for idx in 2..4 {
            client_read_and_assert(
                &mut client,
                &format!("{{\"id\":\"noack\",\"sequence_id\":{idx}}}\r\n"),
            );
        }

        client_read_and_assert(&mut client, "{\"id\":\"ack\",\"sequence_id\":4}\r\n");

        assert_eq!(
            receiver_from_tcp.recv().unwrap(),
            json!({"id": "cmd_move", "sequence_id": 4})
        );

        // Read the event from the TCP/IP.
        for value in 0..3 {
            client_write_and_sleep(
                &mut client,
                &format!(
                    "{{\"id\":\"evt_inPosition\",\"compName\": \"MTMount\",\"value\":{value}}}\r\n"
                ),
                SLEEP_TIME,
            );

            assert_eq!(
                receiver_from_tcp.recv().unwrap(),
                json!({"id": "evt_inPosition", "compName": "MTMount", "value": value})
            );
        }

        // Write the message to the TCP/IP.
        for value in 0..3 {
            sender_to_tcp
                .try_send(vec![
                    json!({"id": "inPosition", "value": value}),
                    json!({"id": "inVelocity", "value": value}),
                ])
                .unwrap();

            sleep(Duration::from_millis(SLEEP_TIME));

            client_read_and_assert(
                &mut client,
                &format!("{{\"id\":\"inPosition\",\"value\":{value}}}\r\n{{\"id\":\"inVelocity\",\"value\":{value}}}\r\n"),
            );
        }

        // close the server.
        stop.store(true, Ordering::Relaxed);

        assert!(handle.join().is_ok());
    }
}
