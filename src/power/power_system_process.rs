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
use std::sync::{
    atomic::{AtomicBool, Ordering},
    mpsc::{sync_channel, Receiver, SyncSender},
    Arc,
};
use std::thread::sleep;
use std::time::{Duration, Instant};

use crate::command::{
    command_power_system::{
        CommandPower, CommandResetBreakers, CommandSwitchDigitalOutput,
        CommandToggleBitClosedLoopControl,
    },
    command_schema::{Command, CommandSchema},
};
use crate::constants::BOUND_SYNC_CHANNEL;
use crate::enums::PowerType;
use crate::mock::mock_plant::MockPlant;
use crate::power::power_system::PowerSystem;
use crate::telemetry::telemetry::Telemetry;
use crate::utility::{get_message_name, get_message_sequence_id};

pub struct PowerSystemProcess {
    // Power system
    _power_system: PowerSystem,
    // Command schema
    _command_schema: CommandSchema,
    // Sender of the telemetry to the model.
    _sender_to_model: SyncSender<Telemetry>,
    // Sender of the message to the power system.
    _sender_to_power_system: SyncSender<Value>,
    // Receiver of the message to the power system.
    _receiver_to_power_system: Receiver<Value>,
    // Stop the loop.
    _stop: Arc<AtomicBool>,
}

impl PowerSystemProcess {
    /// Create a new instance of the power system process.
    ///
    /// # Arguments
    /// * `plant` - Plant model. Put None if the hardware mode is applied.
    /// * `sender_to_model` - The sender to the model.
    /// * `stop` - An Arc instance that holds the AtomicBool instance to stop
    /// the loop.
    ///
    /// # Returns
    /// New instance of the power system process.
    pub fn new(
        plant: Option<MockPlant>,
        sender_to_model: &SyncSender<Telemetry>,
        stop: &Arc<AtomicBool>,
    ) -> Self {
        // Sender and receiver to the power system
        let (sender_to_power_system, receiver_to_power_system) = sync_channel(BOUND_SYNC_CHANNEL);

        Self {
            _power_system: PowerSystem::new(plant),

            _command_schema: Self::create_command_schema(),

            _sender_to_model: sender_to_model.clone(),

            _sender_to_power_system: sender_to_power_system,
            _receiver_to_power_system: receiver_to_power_system,

            _stop: stop.clone(),
        }
    }

    /// Create the command schema.
    ///
    /// # Returns
    /// Command schema.
    fn create_command_schema() -> CommandSchema {
        let mut command_schema = CommandSchema::new();
        command_schema.add_command(Box::new(CommandPower));
        command_schema.add_command(Box::new(CommandResetBreakers));
        command_schema.add_command(Box::new(CommandToggleBitClosedLoopControl));
        command_schema.add_command(Box::new(CommandSwitchDigitalOutput));

        command_schema
    }

    /// Get the sender to the power system.
    ///
    /// # Returns
    /// The sender to the power system.
    pub fn get_sender_to_power_system(&self) -> SyncSender<Value> {
        self._sender_to_power_system.clone()
    }

    /// Run the power system.
    pub fn run(&mut self) {
        info!("Power system is running.");

        self._power_system.init_default_digital_output();

        let loop_time = self._power_system.config.loop_time;

        let maximum_counter_telemetry = self._power_system.config.maximum_counter_telemetry;
        let maximum_counter_toggle_bit = self
            ._power_system
            .config
            .maximum_counter_closed_loop_control_bit;
        let maximun_counter_common = self
            .calculate_least_common_multiple(maximum_counter_telemetry, maximum_counter_toggle_bit);

        let mut counter = 0;
        while !self._stop.load(Ordering::Relaxed) {
            // Time the power loop.
            let now = Instant::now();

            // Process the command and the telemetry.
            if (counter % maximum_counter_telemetry) == 0 {
                // Process the messages.
                let mut command_result = None;
                if let Ok(message) = self._receiver_to_power_system.try_recv() {
                    let command_name = get_message_name(&message);
                    let is_new_power_command = (command_name == CommandPower.name())
                        || (command_name == CommandResetBreakers.name());
                    command_result = Some(self._command_schema.execute(
                        &message,
                        Some(&mut self._power_system),
                        None,
                        None,
                    ));

                    // Skip the command result if we are just beginning to track
                    // the power command.
                    if is_new_power_command && command_result.is_some() {
                        command_result = None;
                    }
                }

                // Read the module and get the telemetry.
                let telemetry = self._power_system.get_telemetry_data();

                // Check the power supply error.
                let digital_output = telemetry.digital_output;
                let digital_input = telemetry.digital_input;
                self._power_system
                    .check_power_supply_error(digital_output, digital_input);

                // Transition the states based on the telemetry data.
                self._power_system.transition_state(
                    PowerType::Communication,
                    telemetry.power_raw["commVoltage"],
                    digital_output,
                    digital_input,
                );
                self._power_system.transition_state(
                    PowerType::Motor,
                    telemetry.power_raw["motorVoltage"],
                    digital_output,
                    digital_input,
                );

                // Send the telemetry and event data to the model and ignore the
                // error.
                let events = if self._power_system.event_queue.has_event() {
                    Some(self._power_system.event_queue.get_events_and_clear())
                } else {
                    None
                };

                if command_result.is_none() && self._power_system.has_command_result() {
                    // Only publish the command result if the command is
                    // external.
                    let result = self._power_system.get_command_result();
                    let mut is_external_command_result = false;
                    if let Some(message) = &result {
                        is_external_command_result = get_message_sequence_id(message) != -1;
                    }

                    if is_external_command_result {
                        command_result = result;
                    }
                }

                let _ = self._sender_to_model.try_send(Telemetry::new(
                    Some(telemetry),
                    None,
                    command_result,
                    events,
                ));
            }

            // Toggle the bit for the closed-loop control for the safety module
            // to use.
            if (counter % maximum_counter_toggle_bit) == 0 {
                self._power_system.toggle_bit_closed_loop_control();
            }

            // Update the counter.
            counter += 1;
            if (counter % maximun_counter_common) == 0 {
                counter = 0;
            }

            // Sleep with the remaining time.
            let cycle_time = now.elapsed().as_millis() as u64;
            if loop_time > cycle_time {
                sleep(Duration::from_millis(loop_time - cycle_time));
            }
        }

        info!("Power system is stopped.");
    }

    /// Calculate the least common multiple of two numbers.
    ///
    /// # Arguments
    /// * `a` - First number.
    /// * `b` - Second number.
    ///
    /// # Returns
    /// The least common multiple of the two numbers.
    fn calculate_least_common_multiple(&self, a: i32, b: i32) -> i32 {
        let mut lcm = a;
        while (lcm % b) != 0 {
            lcm += a;
        }

        lcm
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::path::Path;
    use std::thread::spawn;

    use crate::utility::read_file_stiffness;

    fn create_power_system_process() -> (PowerSystemProcess, Receiver<Telemetry>) {
        // Plant model
        let filepath = Path::new("config/stiff_matrix_m2.yaml");
        let stiffness = read_file_stiffness(filepath);
        let plant = MockPlant::new(&stiffness, 0.0);

        let stop = Arc::new(AtomicBool::new(false));

        let (sender_to_model, receiver_to_model) = sync_channel(BOUND_SYNC_CHANNEL);

        (
            PowerSystemProcess::new(Some(plant), &sender_to_model, &stop),
            receiver_to_model,
        )
    }

    fn wait_events(receiver_to_model: &Receiver<Telemetry>) -> Telemetry {
        let latest_telemetry;
        loop {
            match receiver_to_model.try_recv() {
                Ok(telemetry) => {
                    if telemetry.events.is_some() {
                        latest_telemetry = telemetry;
                        break;
                    }
                }

                Err(_) => {
                    sleep(Duration::from_millis(100));
                }
            }
        }

        latest_telemetry
    }

    #[test]
    fn test_new() {
        let power_system_process = create_power_system_process().0;

        assert_eq!(power_system_process._command_schema.number_of_commands(), 4);
    }

    #[test]
    fn test_run() {
        let (mut power_system_process, receiver_to_model) = create_power_system_process();
        let stop = power_system_process._stop.clone();

        let sender_to_power_system = power_system_process.get_sender_to_power_system();

        let handle = spawn(move || {
            power_system_process.run();
        });

        sleep(Duration::from_millis(500));

        // Turn on the communication power.
        let _ = sender_to_power_system.try_send(json!({
            "id": "cmd_power",
            "sequence_id": 1,
            "status": true,
            "powerType": 2,
        }));

        // Check the telemetry data.

        // Command result should be received.
        let latest_telemetry = wait_events(&receiver_to_model);

        assert_eq!(
            latest_telemetry.events.unwrap(),
            vec![json!({
                "id": "powerSystemState",
                "powerType": 2,
                "status": true,
                "state": 3,
            }),]
        );

        // Check the next events.
        let latest_telemetry = wait_events(&receiver_to_model);

        assert_eq!(
            latest_telemetry.events.unwrap(),
            vec![json!({
                "id": "powerSystemState",
                "powerType": 2,
                "status": true,
                "state": 4,
            }),]
        );

        let latest_telemetry = wait_events(&receiver_to_model);

        assert_eq!(
            latest_telemetry.events.unwrap(),
            vec![json!({
                "id": "powerSystemState",
                "powerType": 2,
                "status": true,
                "state": 5,
            }),]
        );

        assert_eq!(
            latest_telemetry.command_result.unwrap(),
            json!({"id": "success", "sequence_id": 1}),
        );

        // Reset the breakers
        let _ = sender_to_power_system.try_send(json!({
            "id": "cmd_resetBreakers",
            "sequence_id": 2,
            "powerType": 2,
        }));

        // Check the next events.
        let latest_telemetry = wait_events(&receiver_to_model);
        assert_eq!(
            latest_telemetry.events.unwrap(),
            vec![json!({
                "id": "powerSystemState",
                "powerType": 2,
                "status": true,
                "state": 4,
            }),]
        );

        let latest_telemetry = wait_events(&receiver_to_model);

        assert_eq!(
            latest_telemetry.events.unwrap(),
            vec![json!({
                "id": "powerSystemState",
                "powerType": 2,
                "status": true,
                "state": 5,
            }),]
        );

        assert_eq!(
            latest_telemetry.command_result.unwrap(),
            json!({"id": "success", "sequence_id": 2}),
        );

        // Turn off the communication power.
        let _ = sender_to_power_system.try_send(json!({
            "id": "cmd_power",
            "sequence_id": 3,
            "status": false,
            "powerType": 2,
        }));

        // Check the next events.
        let latest_telemetry = wait_events(&receiver_to_model);
        assert_eq!(
            latest_telemetry.events.unwrap(),
            vec![json!({
                "id": "powerSystemState",
                "powerType": 2,
                "status": false,
                "state": 6,
            }),]
        );

        let latest_telemetry = wait_events(&receiver_to_model);

        assert_eq!(
            latest_telemetry.events.unwrap(),
            vec![json!({
                "id": "powerSystemState",
                "powerType": 2,
                "status": false,
                "state": 2,
            }),]
        );

        assert_eq!(
            latest_telemetry.command_result.unwrap(),
            json!({"id": "success", "sequence_id": 3}),
        );

        // Close the server.
        stop.store(true, Ordering::Relaxed);

        assert!(handle.join().is_ok());
    }

    #[test]
    fn test_calculate_least_common_multiple() {
        let (power_system_process, _) = create_power_system_process();

        let lcm = power_system_process.calculate_least_common_multiple(3, 5);

        assert_eq!(lcm, 15);
    }
}
