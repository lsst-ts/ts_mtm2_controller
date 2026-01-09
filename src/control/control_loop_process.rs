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
use std::time::{Duration, Instant};

use crate::command::{
    command_control_loop::{
        CommandApplyForces, CommandMoveActuators, CommandPositionMirror, CommandResetActuatorSteps,
        CommandResetForceOffsets, CommandSetClosedLoopControlMode, CommandSetConfig,
        CommandSetExternalElevation,
    },
    command_data_acquisition::CommandMoveActuatorSteps,
    command_schema::{Command, CommandSchema},
};
use crate::config::Config;
use crate::constants::BOUND_SYNC_CHANNEL;
use crate::control::control_loop::ControlLoop;
use crate::telemetry::{telemetry::Telemetry, telemetry_control_loop::TelemetryControlLoop};
use crate::utility::{get_message_name, get_message_sequence_id};

pub struct ControlLoopProcess {
    // Control loop
    pub control_loop: ControlLoop,
    // Command schema
    _command_schema: CommandSchema,
    // Sender to the data acquisition
    _sender_to_daq: SyncSender<Value>,
    // Sender of the telemetry to the model
    _sender_to_model: SyncSender<Telemetry>,
    // Sender of the message to the control loop
    _sender_to_control_loop: SyncSender<Value>,
    // Receiver of the message to the control loop
    _receiver_to_control_loop: Receiver<Value>,
    // Sender of the telemetry to the control loop
    _sender_telemetry_to_control_loop: SyncSender<TelemetryControlLoop>,
    // Receiver of the telemetry to the control loop
    _receiver_telemetry_to_control_loop: Receiver<TelemetryControlLoop>,
    // Stop the loop
    _stop: Arc<AtomicBool>,
}

impl ControlLoopProcess {
    /// Create a new instance of the control loop process.
    ///
    /// # Arguments
    /// * `config` - The configuration.
    /// * `is_mirror` - Is the mirror or the surrogate.
    /// * `is_simulation_mode` - Is the simulation mode or not.
    /// * `sender_to_daq` - The sender to the data acquisition.
    /// * `sender_to_model` - The sender to the model.
    /// * `stop` - An Arc instance that holds the AtomicBool instance to stop
    ///   the loop.
    ///
    /// # Returns
    /// New instance of the control loop process.
    pub fn new(
        config: &Config,
        is_mirror: bool,
        is_simulation_mode: bool,
        sender_to_daq: &SyncSender<Value>,
        sender_to_model: &SyncSender<Telemetry>,
        stop: &Arc<AtomicBool>,
    ) -> Self {
        // Sender and receiver to the control loop (commands).
        let (sender_to_control_loop, receiver_to_control_loop) = sync_channel(BOUND_SYNC_CHANNEL);

        // Sender and receiver of the telemetry to the control loop.
        let (sender_telemetry_to_control_loop, receiver_telemetry_to_control_loop) =
            sync_channel(BOUND_SYNC_CHANNEL);

        Self {
            control_loop: ControlLoop::new(config, is_mirror, is_simulation_mode),

            _command_schema: Self::create_command_schema(),

            _sender_to_daq: sender_to_daq.clone(),
            _sender_to_model: sender_to_model.clone(),

            _sender_to_control_loop: sender_to_control_loop,
            _receiver_to_control_loop: receiver_to_control_loop,

            _sender_telemetry_to_control_loop: sender_telemetry_to_control_loop,
            _receiver_telemetry_to_control_loop: receiver_telemetry_to_control_loop,

            _stop: stop.clone(),
        }
    }

    /// Create the command schema.
    ///
    /// # Returns
    /// Command schema.
    fn create_command_schema() -> CommandSchema {
        let mut command_schema = CommandSchema::new();
        command_schema.add_command(Box::new(CommandSetClosedLoopControlMode));
        command_schema.add_command(Box::new(CommandApplyForces));
        command_schema.add_command(Box::new(CommandResetForceOffsets));
        command_schema.add_command(Box::new(CommandPositionMirror));
        command_schema.add_command(Box::new(CommandResetActuatorSteps));
        command_schema.add_command(Box::new(CommandMoveActuators));
        command_schema.add_command(Box::new(CommandSetConfig));
        command_schema.add_command(Box::new(CommandSetExternalElevation));

        command_schema
    }

    /// Get the sender to the control loop.
    ///
    /// # Returns
    /// The sender to the control loop.
    pub fn get_sender_to_control_loop(&self) -> SyncSender<Value> {
        self._sender_to_control_loop.clone()
    }

    /// Get the sender of the telemetry to the control loop.
    ///
    /// # Returns
    /// The sender of the telemetry to the control loop.
    pub fn get_sender_telemetry_to_control_loop(&self) -> SyncSender<TelemetryControlLoop> {
        self._sender_telemetry_to_control_loop.clone()
    }

    /// Run the control loop.
    pub fn run(&mut self) {
        info!("Control loop is running.");

        // Telemetry command name
        let telemetry_command_name = CommandSetExternalElevation.name();

        let period = (1000.0 / self.control_loop.config.control_frequency) as u64;
        let mut seq_id_move_actuator_steps = 0;
        let mut processed_telemetry: Option<TelemetryControlLoop> = None;
        while !self._stop.load(Ordering::Relaxed) {
            // Time the control loop
            let now = Instant::now();

            // Process the received telemetry from the data acquisition.
            if let Some(telemetry) = &mut processed_telemetry {
                // We need to check the sequence ID here because we need to
                // make sure the new telemetry is the expected one that the
                // inner-loop controlloer (ILC) has moved the actuators.
                if telemetry.seq_id_move_actuator_steps == seq_id_move_actuator_steps {
                    self.control_loop.process_telemetry_data(telemetry);
                    self.control_loop.step(telemetry);
                }
            }

            // Process the messages.
            let mut command_result = None;
            let mut had_processed_telemetry_command = false;
            let mut is_telemetry_command = false;
            let mut is_internal_command = false;
            while let Ok(message) = self._receiver_to_control_loop.try_recv() {
                command_result = Some(self._command_schema.execute(
                    &message,
                    None,
                    None,
                    Some(&mut self.control_loop),
                    None,
                ));

                is_internal_command = get_message_sequence_id(&message) == -1;

                // If we receive a telemetry command as the first time,
                // continue to process the second command.
                is_telemetry_command = get_message_name(&message) == telemetry_command_name;

                if (!had_processed_telemetry_command) && is_telemetry_command {
                    had_processed_telemetry_command = true;
                    continue;
                }

                // Break the loop if we processed a non-telemetry
                // command or two consecutive telemetry commands.
                if !is_telemetry_command || had_processed_telemetry_command {
                    break;
                }
            }

            // For the telemetry/internal command, no need to send the result.
            if is_telemetry_command || is_internal_command {
                command_result = None;
            }

            if let Some(steps) = self.control_loop.take_steps_to_move_actuators() {
                seq_id_move_actuator_steps =
                    self.get_next_seq_id_move_actuator_steps(seq_id_move_actuator_steps);
                if self
                    ._sender_to_daq
                    .try_send(json!({
                        "id": CommandMoveActuatorSteps.name(),
                        "seq_id_move_actuator_steps": seq_id_move_actuator_steps,
                        "steps": steps,
                    }))
                    .is_err()
                {
                    error!(
                        "Failed to send the move actuator steps command to the data acquisition with sequence ID {}.",
                        seq_id_move_actuator_steps
                    );
                };
            }

            // Send the telemetry and event data to the model and ignore the
            // error.
            let events = if self.control_loop.event_queue.has_event() {
                Some(self.control_loop.event_queue.get_events_and_clear())
            } else {
                None
            };

            let cycle_time = now.elapsed().as_millis() as u64;

            if let Some(telemetry) = &mut processed_telemetry {
                telemetry.cycle_time = (cycle_time as f64) / 1000.0;
            }

            let _ = self._sender_to_model.try_send(Telemetry::new(
                None,
                processed_telemetry.take(),
                command_result,
                events,
            ));

            // Calculate the remaining time to wait for the new telemetry.
            let wait_telemetry_time = period.saturating_sub(cycle_time);
            match self
                ._receiver_telemetry_to_control_loop
                .recv_timeout(Duration::from_millis(wait_telemetry_time))
            {
                Ok(telemetry) => {
                    processed_telemetry = Some(telemetry);
                }
                Err(_) => {
                    processed_telemetry = None;
                }
            }
        }

        info!("Control loop is stopped.");
    }

    /// Get the next sequence ID for the move actuator steps command.
    ///
    /// # Arguments
    /// * `current_id` - The current sequence ID.
    ///
    /// # Returns
    /// The next sequence ID.
    fn get_next_seq_id_move_actuator_steps(&self, current_id: i32) -> i32 {
        if current_id == i32::MAX {
            0
        } else {
            current_id + 1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use serde_json::json;
    use std::path::Path;
    use std::thread::{sleep, spawn};

    use crate::command::command_control_loop::{
        CommandMoveActuators, CommandSetClosedLoopControlMode,
    };
    use crate::daq::data_acquisition_process::DataAcquisitionProcess;
    use crate::enums::DataAcquisitionMode;
    use crate::mock::mock_constants::PLANT_STEP_TO_ENCODER;
    use crate::telemetry::telemetry_power::TelemetryPower;

    fn create_control_loop_process() -> (
        ControlLoopProcess,
        Receiver<Telemetry>,
        SyncSender<Value>,
        Receiver<Value>,
    ) {
        let config = Config::new(
            Path::new("config/parameters_control.yaml"),
            Path::new("config/lut/handling"),
        );

        let stop = Arc::new(AtomicBool::new(false));

        let (sender_to_model, receiver_to_model) = sync_channel(BOUND_SYNC_CHANNEL);
        let (sender_to_daq, receiver_to_daq) =
            DataAcquisitionProcess::create_sender_and_receiver_to_data_acquisition();

        (
            ControlLoopProcess::new(
                &config,
                false,
                true,
                &sender_to_daq,
                &sender_to_model,
                &stop,
            ),
            receiver_to_model,
            sender_to_daq,
            receiver_to_daq,
        )
    }

    fn create_data_acquisition_process(
        sender_telemetry_to_control_loop: &SyncSender<TelemetryControlLoop>,
        sender_to_model: &SyncSender<Telemetry>,
        sender_to_daq: SyncSender<Value>,
        receiver_to_daq: Receiver<Value>,
        stop: &Arc<AtomicBool>,
    ) -> (DataAcquisitionProcess, Receiver<TelemetryPower>) {
        let (sender_telemetry_to_power, receiver_telemetry_to_power) =
            sync_channel(BOUND_SYNC_CHANNEL);

        (
            DataAcquisitionProcess::new(
                true,
                sender_telemetry_to_control_loop,
                &sender_telemetry_to_power,
                sender_to_model,
                sender_to_daq,
                receiver_to_daq,
                stop,
            ),
            receiver_telemetry_to_power,
        )
    }

    #[test]
    fn test_new() {
        let control_loop_process = create_control_loop_process().0;

        assert_eq!(control_loop_process._command_schema.number_of_commands(), 8);
    }

    #[test]
    fn test_run() {
        let (mut control_loop_process, receiver_to_model, sender_to_daq, receiver_to_daq) =
            create_control_loop_process();
        let stop = control_loop_process._stop.clone();

        let (mut data_acquisition_process, _receiver_telemetry_power) =
            create_data_acquisition_process(
                &control_loop_process._sender_telemetry_to_control_loop,
                &control_loop_process._sender_to_model,
                sender_to_daq,
                receiver_to_daq,
                &stop,
            );
        data_acquisition_process.daq.mode = DataAcquisitionMode::Telemetry;
        if let Some(plant) = &mut data_acquisition_process.daq.plant {
            plant.power_system_communication.is_power_on = true;
        }

        let sender_to_control_loop = control_loop_process.get_sender_to_control_loop();

        let handle_control_loop = spawn(move || {
            control_loop_process.run();
        });
        let handle_data_acquisition = spawn(move || {
            data_acquisition_process.run();
        });

        sleep(Duration::from_millis(500));

        // Set the closed-loop control mode.
        let _ = sender_to_control_loop.try_send(json!({
            "id": CommandSetClosedLoopControlMode.name(),
            "sequence_id": 2,
            "mode": 3,
        }));

        // Check the telemetry data.
        let mut latest_telemetry = Telemetry::new(None, None, None, None);
        loop {
            match receiver_to_model.recv_timeout(Duration::from_millis(50)) {
                Ok(mut telemetry) => {
                    if telemetry.control_loop.is_some() {
                        latest_telemetry.control_loop = telemetry.control_loop.take();
                    }

                    if telemetry.events.is_some() {
                        latest_telemetry.events = telemetry.events.take();
                    }

                    if telemetry.command_result.is_some() {
                        latest_telemetry.command_result = telemetry.command_result.take();
                        break;
                    }
                }
                Err(_) => {
                    break;
                }
            }
        }

        assert_eq!(
            latest_telemetry.command_result.unwrap(),
            json!({
                "id": "success",
                "sequence_id": 2,
            })
        );
        assert_eq!(
            latest_telemetry.events.unwrap(),
            vec![
                json!({
                    "id": "closedLoopControlMode",
                    "mode": 3,
                }),
                json!({
                    "id": "forceBalanceSystemStatus",
                    "status": false,
                })
            ]
        );

        let mut telemetry_control_loop = latest_telemetry.control_loop.unwrap();

        assert_eq!(telemetry_control_loop.seq_id_move_actuator_steps, 0);
        assert_eq!(telemetry_control_loop.inclinometer["raw"], 90.0);

        // Move the actuator steps.
        let _ = sender_to_control_loop.try_send(json!({
            "id": CommandMoveActuators.name(),
            "sequence_id": 3,
            "actuatorCommand": 1,
            "actuators": [0, 1, 2],
            "displacement": 80,
            "unit": 2,
        }));

        let expected_encoder_value = (80.0 * PLANT_STEP_TO_ENCODER) as i32;
        loop {
            if let Ok(mut telemetry) = receiver_to_model.recv_timeout(Duration::from_millis(50)) {
                if let Some(control_loop) = &telemetry.control_loop {
                    if control_loop.ilc_encoders[0] == expected_encoder_value {
                        latest_telemetry.control_loop = telemetry.control_loop.take();
                        break;
                    }
                }
            }
        }

        telemetry_control_loop = latest_telemetry.control_loop.unwrap();

        assert!(telemetry_control_loop.seq_id_move_actuator_steps > 0);
        assert_eq!(
            telemetry_control_loop.ilc_encoders[0],
            expected_encoder_value,
        );

        // Close the server.
        stop.store(true, Ordering::Relaxed);

        assert!(handle_control_loop.join().is_ok());
        assert!(handle_data_acquisition.join().is_ok());
    }

    #[test]
    fn test_get_next_seq_id_move_actuator_steps() {
        let control_loop_process = create_control_loop_process().0;

        assert_eq!(
            control_loop_process.get_next_seq_id_move_actuator_steps(0),
            1
        );
        assert_eq!(
            control_loop_process.get_next_seq_id_move_actuator_steps(i32::MAX),
            0
        );
    }
}
