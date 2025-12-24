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

use log::{error, info};
use serde_json::{json, Value};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    mpsc::{sync_channel, Receiver, SyncSender},
    Arc,
};
use std::thread::sleep;
use std::time::{Duration, Instant};

use crate::command::{
    command_data_acquisition::{CommandSetDataAcquisitionMode, CommandSwitchDigitalOutput},
    command_power_system::{CommandPower, CommandResetBreakers},
    command_schema::{Command, CommandSchema},
};
use crate::constants::BOUND_SYNC_CHANNEL;
use crate::enums::{BitEnum, DataAcquisitionMode, PowerType};
use crate::power::power_system::PowerSystem;
use crate::telemetry::{telemetry::Telemetry, telemetry_power::TelemetryPower};
use crate::utility::{get_message_name, get_message_sequence_id};

pub struct PowerSystemProcess {
    // Power system
    _power_system: PowerSystem,
    // Is the simulation mode or not
    _is_simulation_mode: bool,
    // Latest received and processed power telemetry from the data acquisition
    // process
    _latest_telemetry: Option<TelemetryPower>,
    // Command schema
    _command_schema: CommandSchema,
    // Sender to the data acquisition
    _sender_to_daq: SyncSender<Value>,
    // Sender of the telemetry to the model
    _sender_to_model: SyncSender<Telemetry>,
    // Sender of the message to the power system
    _sender_to_power_system: SyncSender<Value>,
    // Receiver of the message to the power system
    _receiver_to_power_system: Receiver<Value>,
    // Sender of the telemetry to the power system
    _sender_telemetry_to_power_system: SyncSender<TelemetryPower>,
    // Receiver of the telemetry to the power system
    _receiver_telemetry_to_power_system: Receiver<TelemetryPower>,
    // Stop the loop
    _stop: Arc<AtomicBool>,
}

impl PowerSystemProcess {
    /// Create a new instance of the power system process.
    ///
    /// # Arguments
    /// * `is_simulation_mode` - Is the simulation mode or not.
    /// * `sender_to_daq` - The sender to the data acquisition.
    /// * `sender_to_model` - The sender to the model.
    /// * `stop` - An Arc instance that holds the AtomicBool instance to stop
    ///   the loop.
    ///
    /// # Returns
    /// New instance of the power system process.
    pub fn new(
        is_simulation_mode: bool,
        sender_to_daq: &SyncSender<Value>,
        sender_to_model: &SyncSender<Telemetry>,
        stop: &Arc<AtomicBool>,
    ) -> Self {
        // Sender and receiver to the power system (commands).
        let (sender_to_power_system, receiver_to_power_system) = sync_channel(BOUND_SYNC_CHANNEL);

        // Sender and receiver of the telemetry to the power system.
        let (sender_telemetry_to_power_system, receiver_telemetry_to_power_system) =
            sync_channel(BOUND_SYNC_CHANNEL);

        Self {
            _power_system: PowerSystem::new(),

            _is_simulation_mode: is_simulation_mode,

            _latest_telemetry: None,

            _command_schema: Self::create_command_schema(),

            _sender_to_daq: sender_to_daq.clone(),
            _sender_to_model: sender_to_model.clone(),

            _sender_to_power_system: sender_to_power_system,
            _receiver_to_power_system: receiver_to_power_system,

            _sender_telemetry_to_power_system: sender_telemetry_to_power_system,
            _receiver_telemetry_to_power_system: receiver_telemetry_to_power_system,

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

        command_schema
    }

    /// Get the sender to the power system.
    ///
    /// # Returns
    /// The sender to the power system.
    pub fn get_sender_to_power_system(&self) -> SyncSender<Value> {
        self._sender_to_power_system.clone()
    }

    /// Get the sender of the telemetry to the power system.
    ///
    /// # Returns
    /// The sender of the telemetry to the power system.
    pub fn get_sender_telemetry_to_power_system(&self) -> SyncSender<TelemetryPower> {
        self._sender_telemetry_to_power_system.clone()
    }

    /// Run the power system.
    pub fn run(&mut self) {
        info!("Power system is running.");

        let loop_time = self._power_system.config.loop_time;
        let maximum_counter_telemetry = self._power_system.config.maximum_counter_telemetry;

        let mut counter = 0;
        while !self._stop.load(Ordering::Relaxed) {
            // Time the power loop.
            let now = Instant::now();

            // Try to receive the latest telemetry from the data acquisition
            // process and process it.
            if let Ok(mut telemetry) = self._receiver_telemetry_to_power_system.try_recv() {
                self._power_system.process_telemetry_data(&mut telemetry);

                // Check the power supply error.
                let digital_output = telemetry.digital_output;
                let digital_input = telemetry.digital_input;
                self._power_system
                    .check_power_supply_error(digital_output, digital_input);

                // Transition the states based on the received telemetry data.
                // In the simulation mode, we only do the state transition when
                // receiving the new telemetry data. This is because the plant
                // model in the DataAcquisition will simulate the change of
                // the current and voltage values.
                if self._is_simulation_mode {
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
                }

                self._latest_telemetry = Some(telemetry);
            }

            // Transition the states based on the received telemetry data from
            // hardware.
            if !self._is_simulation_mode {
                if let Some(telemetry) = self._latest_telemetry.as_mut() {
                    let digital_output = telemetry.digital_output;
                    let digital_input = telemetry.digital_input;

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
                };
            }

            // Process the command.
            if (counter % maximum_counter_telemetry) == 0 {
                // Process the messages.
                let mut command_result = None;
                if let Ok(message) = self._receiver_to_power_system.try_recv() {
                    if self
                        .update_data_acquisition_mode_when_power_off(&message)
                        .is_none()
                    {
                        error!(
                            "Failed to update the data acquisition mode when power is turned off."
                        );
                    }

                    let command_name = get_message_name(&message);
                    let is_new_power_command = (command_name == CommandPower.name())
                        || (command_name == CommandResetBreakers.name());
                    command_result = Some(self._command_schema.execute(
                        &message,
                        None,
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

                // Apply the actions to the data acquisition process.
                if self._power_system.has_actions() {
                    let actions = self._power_system.get_actions_and_clear();
                    for (digital_output, status) in actions {
                        let _ = self._sender_to_daq.try_send(json!({
                            "id": CommandSwitchDigitalOutput.name(),
                            "bit": digital_output.bit_value() as u64,
                            "status": status as u8,
                        }));
                    }
                }

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
                    let result = self._power_system.get_any_command_result();
                    let mut is_external_command_result = false;
                    if let Some(message) = &result {
                        is_external_command_result = get_message_sequence_id(message) != -1;
                    }

                    if is_external_command_result {
                        command_result = result;
                    }
                }

                let _ = self._sender_to_model.try_send(Telemetry::new(
                    self._latest_telemetry.clone(),
                    None,
                    command_result,
                    events,
                ));
            }

            // Update the counter.
            counter += 1;
            if counter >= maximum_counter_telemetry {
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

    /// Update the data acquisition mode when the power is turned off.
    ///
    /// # Arguments
    /// * `message` - The message to process.
    ///
    /// # Returns
    /// Option indicating success or failure.
    fn update_data_acquisition_mode_when_power_off(&self, message: &Value) -> Option<()> {
        let command_name = get_message_name(message);
        let is_new_power_off_command =
            (command_name == CommandPower.name()) && (message["status"] == false);
        if is_new_power_off_command {
            let discriminant = message["powerType"].as_u64()?;
            let power_type = PowerType::from_repr(discriminant as u8)?;
            let is_powered_on = self._power_system.is_powered_on(power_type);
            if is_powered_on {
                let mode = if (power_type == PowerType::Motor)
                    && (self._power_system.is_powered_on(PowerType::Communication))
                {
                    DataAcquisitionMode::Telemetry
                } else {
                    DataAcquisitionMode::Idle
                };

                self._sender_to_daq
                    .try_send(json!({
                        "id": CommandSetDataAcquisitionMode.name(),
                        "mode": mode as u8,
                    }))
                    .ok()?;
            }
        }

        Some(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use serde_json::json;
    use std::thread::spawn;

    use crate::daq::data_acquisition_process::DataAcquisitionProcess;
    use crate::telemetry::telemetry_control_loop::TelemetryControlLoop;

    fn create_power_system_process() -> (
        PowerSystemProcess,
        Receiver<Telemetry>,
        SyncSender<Value>,
        Receiver<Value>,
    ) {
        let stop = Arc::new(AtomicBool::new(false));

        let (sender_to_model, receiver_to_model) = sync_channel(BOUND_SYNC_CHANNEL);
        let (sender_to_daq, receiver_to_daq) =
            DataAcquisitionProcess::create_sender_and_receiver_to_data_acquisition();

        (
            PowerSystemProcess::new(true, &sender_to_daq, &sender_to_model, &stop),
            receiver_to_model,
            sender_to_daq,
            receiver_to_daq,
        )
    }

    fn create_data_acquisition_process(
        sender_telemetry_to_power: &SyncSender<TelemetryPower>,
        sender_to_model: &SyncSender<Telemetry>,
        sender_to_daq: SyncSender<Value>,
        receiver_to_daq: Receiver<Value>,
        stop: &Arc<AtomicBool>,
    ) -> (DataAcquisitionProcess, Receiver<TelemetryControlLoop>) {
        let (sender_telemetry_to_control_loop, receiver_telemetry_to_control_loop) =
            sync_channel(BOUND_SYNC_CHANNEL);

        (
            DataAcquisitionProcess::new(
                true,
                &sender_telemetry_to_control_loop,
                sender_telemetry_to_power,
                sender_to_model,
                sender_to_daq,
                receiver_to_daq,
                stop,
            ),
            receiver_telemetry_to_control_loop,
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

        assert_eq!(power_system_process._command_schema.number_of_commands(), 2);
    }

    #[test]
    fn test_run() {
        let (mut power_system_process, receiver_to_model, sender_to_daq, receiver_to_daq) =
            create_power_system_process();
        let stop = power_system_process._stop.clone();

        let (mut data_acquisition_process, _receiver_telemetry_to_control_loop) =
            create_data_acquisition_process(
                &power_system_process._sender_telemetry_to_power_system,
                &power_system_process._sender_to_model,
                sender_to_daq,
                receiver_to_daq,
                &stop,
            );

        let sender_to_power_system = power_system_process.get_sender_to_power_system();

        let handle_power_system = spawn(move || {
            power_system_process.run();
        });
        let handle_data_acquisition = spawn(move || {
            data_acquisition_process.run();
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

        assert!(handle_power_system.join().is_ok());
        assert!(handle_data_acquisition.join().is_ok());
    }
}
