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
    command_data_acquisition::{
        CommandGetInnerLoopControlMode, CommandMoveActuatorSteps, CommandSetDataAcquisitionMode,
        CommandSetInnerLoopControlMode, CommandSwitchDigitalOutput,
    },
    command_schema::CommandSchema,
};
use crate::constants::BOUND_SYNC_CHANNEL;
use crate::daq::data_acquisition::DataAcquisition;
use crate::enums::DataAcquisitionMode;
use crate::telemetry::{
    telemetry::Telemetry, telemetry_control_loop::TelemetryControlLoop,
    telemetry_power::TelemetryPower,
};
use crate::utility::get_message_sequence_id;

pub struct DataAcquisitionProcess {
    // Data acquisition (DAQ) system
    pub daq: DataAcquisition,
    // Command schema
    _command_schema: CommandSchema,
    // Sender of the telemetry to the control loop (inner-loop controller, ILC)
    _sender_telemetry_to_control_loop: SyncSender<TelemetryControlLoop>,
    // Sender of the telemetry to the power system
    _sender_telemetry_to_power: SyncSender<TelemetryPower>,
    // Sender of the message to the model (commands result and events)
    _sender_to_model: SyncSender<Telemetry>,
    // Sender of the message to DAQ
    _sender_to_daq: SyncSender<Value>,
    // Receiver of the message to DAQ
    _receiver_to_daq: Receiver<Value>,
    // Stop the loop.
    _stop: Arc<AtomicBool>,
}

impl DataAcquisitionProcess {
    /// Create a new data acquisition (DAQ) process.
    ///
    /// # Arguments
    /// * `is_simulation_mode` - Whether the system is in simulation mode.
    /// * `sender_telemetry_to_control_loop` - The sender of the inner-loop
    ///   controller (ILC) telemetry to the control loop.
    /// * `sender_telemetry_to_power` - The sender of the telemetry to the
    ///   power system.
    /// * `sender_to_model` - The sender of the message to the model.
    /// * `sender_to_daq` - The sender of the message to DAQ.
    /// * `receiver_to_daq` - The receiver of the message to DAQ.
    /// * `stop` - The atomic boolean to stop the loop.
    ///
    /// # Returns
    /// A new data acquisition process.
    pub fn new(
        is_simulation_mode: bool,
        sender_telemetry_to_control_loop: &SyncSender<TelemetryControlLoop>,
        sender_telemetry_to_power: &SyncSender<TelemetryPower>,
        sender_to_model: &SyncSender<Telemetry>,
        sender_to_daq: SyncSender<Value>,
        receiver_to_daq: Receiver<Value>,
        stop: &Arc<AtomicBool>,
    ) -> Self {
        let daq = DataAcquisition::new(is_simulation_mode);

        // Check the frequencies
        let config = &daq.config;
        Self::check_divisor(config.frequency_loop, config.frequency_toggle_bit);

        Self {
            daq,

            _command_schema: Self::create_command_schema(),

            _sender_telemetry_to_control_loop: sender_telemetry_to_control_loop.clone(),
            _sender_telemetry_to_power: sender_telemetry_to_power.clone(),
            _sender_to_model: sender_to_model.clone(),

            _sender_to_daq: sender_to_daq,
            _receiver_to_daq: receiver_to_daq,

            _stop: stop.clone(),
        }
    }

    /// Check that the frequency is a multiple of the divisor.
    ///
    /// # Arguments
    /// * `frequency` - The frequency to check.
    /// * `divisor` - The divisor.
    ///
    /// # Panics
    /// Panics if the frequency is not a multiple of the divisor.
    fn check_divisor(frequency: f64, divisor: f64) {
        if (frequency % divisor) != 0.0 {
            panic!(
                "The frequency {} must be a multiple of {}.",
                frequency, divisor
            );
        }
    }

    /// Create the command schema.
    ///
    /// # Returns
    /// Command schema.
    fn create_command_schema() -> CommandSchema {
        let mut command_schema = CommandSchema::new();
        command_schema.add_command(Box::new(CommandGetInnerLoopControlMode));
        command_schema.add_command(Box::new(CommandSetInnerLoopControlMode));
        command_schema.add_command(Box::new(CommandSwitchDigitalOutput));
        command_schema.add_command(Box::new(CommandMoveActuatorSteps));
        command_schema.add_command(Box::new(CommandSetDataAcquisitionMode));

        command_schema
    }

    /// Create a sender and receiver to the data acquisition.
    ///
    /// # Notes
    /// This function is to avoid the circular dependency when working with
    /// multiple processes.
    ///
    /// # Returns
    /// A tuple of the sender and receiver to the data acquisition.
    pub fn create_sender_and_receiver_to_data_acquisition() -> (SyncSender<Value>, Receiver<Value>)
    {
        sync_channel(BOUND_SYNC_CHANNEL)
    }

    /// Get the sender to the data acquisition.
    ///
    /// # Returns
    /// The sender to the data acquisition.
    pub fn get_sender_to_data_acquisition(&self) -> SyncSender<Value> {
        self._sender_to_daq.clone()
    }

    /// Run the data acquisition loop.
    pub fn run(&mut self) {
        info!("Data acquisition loop is running.");

        self.daq.init_default_digital_output();

        // Max counter to toggle the closed-loop control bit.
        let config = &self.daq.config;
        let max_counter_toggle_bit = (config.frequency_loop / config.frequency_toggle_bit) as u64;

        let period_loop = (1000.0 / config.frequency_loop) as u64;
        let mut counter = 0;
        while !self._stop.load(Ordering::Relaxed) {
            // Time the data acquisition loop.
            let now = Instant::now();

            // Process the message.
            let mut command_result = None;
            if let Ok(message) = self._receiver_to_daq.try_recv() {
                command_result = Some(self._command_schema.execute(
                    &message,
                    Some(&mut self.daq),
                    None,
                    None,
                    None,
                ));

                // For the internal command, no need to send the result.
                let is_internal_command = get_message_sequence_id(&message) == -1;
                if is_internal_command {
                    command_result = None;
                }
            }

            // Send the command result or events to the model. Ignore the
            // error.
            let has_events = self.daq.event_queue.has_event();
            if command_result.is_some() || has_events {
                // Get the events.
                let events = if has_events {
                    Some(self.daq.event_queue.get_events_and_clear())
                } else {
                    None
                };

                let _ = self._sender_to_model.try_send(Telemetry::new(
                    None,
                    None,
                    command_result,
                    events,
                ));
            }

            // Send the telemetry and ignore the error.
            // Always send the power telemetry data.
            let _ = self
                ._sender_telemetry_to_power
                .try_send(self.daq.get_telemetry_power());

            // When the system is not in idle mode, there is the ILC
            // telemetry data to send.
            if self.daq.mode != DataAcquisitionMode::Idle {
                let _ = self
                    ._sender_telemetry_to_control_loop
                    .try_send(self.daq.get_telemetry_ilc());
            }

            // Toggle the bit of the closed-loop control for the safety module
            // to use.
            if (counter % max_counter_toggle_bit) == 0 {
                self.daq.toggle_bit_closed_loop_control();
            }

            // Update the counter.
            counter += 1;
            if counter >= max_counter_toggle_bit {
                counter = 0;
            }

            // Sleep with the remaining time.
            let cycle_time = now.elapsed().as_millis() as u64;
            if period_loop > cycle_time {
                sleep(Duration::from_millis(period_loop - cycle_time));
            }
        }

        self.daq.end_default_digital_output();

        info!("Data acquisition loop is stopped.");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use serde_json::json;
    use std::thread::spawn;

    use crate::mock::mock_constants::TEST_DIGITAL_OUTPUT_NO_POWER;

    fn create_data_acquisition_process() -> (
        DataAcquisitionProcess,
        Receiver<TelemetryControlLoop>,
        Receiver<TelemetryPower>,
        Receiver<Telemetry>,
    ) {
        let (sender_telemetry_to_control_loop, receiver_telemetry_to_control_loop) =
            sync_channel(BOUND_SYNC_CHANNEL);
        let (sender_telemetry_to_power, receiver_telemetry_to_power) =
            sync_channel(BOUND_SYNC_CHANNEL);
        let (sender_to_model, receiver_to_model) = sync_channel(BOUND_SYNC_CHANNEL);
        let (sender_to_daq, receiver_to_daq) =
            DataAcquisitionProcess::create_sender_and_receiver_to_data_acquisition();

        let stop = Arc::new(AtomicBool::new(false));

        (
            DataAcquisitionProcess::new(
                true,
                &sender_telemetry_to_control_loop,
                &sender_telemetry_to_power,
                &sender_to_model,
                sender_to_daq,
                receiver_to_daq,
                &stop,
            ),
            receiver_telemetry_to_control_loop,
            receiver_telemetry_to_power,
            receiver_to_model,
        )
    }

    #[test]
    fn test_new() {
        let control_loop_process = create_data_acquisition_process().0;

        assert_eq!(control_loop_process._command_schema.number_of_commands(), 5);
    }

    #[test]
    fn test_check_divisor_success() {
        DataAcquisitionProcess::check_divisor(40.0, 10.0);
        DataAcquisitionProcess::check_divisor(40.0, 1.0);
    }

    #[test]
    #[should_panic(expected = "The frequency 40 must be a multiple of 7.")]
    fn test_check_divisor_failure() {
        DataAcquisitionProcess::check_divisor(40.0, 7.0);
    }

    #[test]
    fn test_run() {
        let (
            mut data_acquisition,
            receiver_telemetry_to_control_loop,
            receiver_telemetry_to_power,
            receiver_to_model,
        ) = create_data_acquisition_process();
        let stop = data_acquisition._stop.clone();

        let sender_to_data_acquisition = data_acquisition.get_sender_to_data_acquisition();
        let handle = spawn(move || {
            data_acquisition.run();
        });

        // Check to get the power telemetry data.
        let received_telemetry_power;
        loop {
            if let Ok(telemetry_power) =
                receiver_telemetry_to_power.recv_timeout(Duration::from_millis(50))
            {
                received_telemetry_power = telemetry_power;
                break;
            }
        }

        assert_eq!(
            received_telemetry_power.digital_output,
            TEST_DIGITAL_OUTPUT_NO_POWER
        );

        // There should be no ILC telemetry data in idle mode.
        let mut has_telemetry_control_loop = false;
        loop {
            match receiver_telemetry_to_control_loop.try_recv() {
                Ok(_) => {
                    has_telemetry_control_loop = true;
                }
                Err(_) => {
                    break;
                }
            }
        }

        assert!(!has_telemetry_control_loop);

        // Set the data acquisition mode.
        let _ = sender_to_data_acquisition.try_send(json!({
            "id": "cmd_setDataAcquisitionMode",
            "sequence_id": 2,
            "mode": 2,
        }));

        // Check the telemetry data.
        let latest_telemetry;
        loop {
            if let Ok(telemetry) = receiver_to_model.recv_timeout(Duration::from_millis(50)) {
                if let Some(_) = &telemetry.command_result {
                    latest_telemetry = telemetry;
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

        // There should be ILC telemetry data when not in idle mode.
        loop {
            if let Ok(_) =
                receiver_telemetry_to_control_loop.recv_timeout(Duration::from_millis(50))
            {
                has_telemetry_control_loop = true;
                break;
            }
        }

        assert!(has_telemetry_control_loop);

        // Close the server.
        stop.store(true, Ordering::Relaxed);

        assert!(handle.join().is_ok());
    }
}
