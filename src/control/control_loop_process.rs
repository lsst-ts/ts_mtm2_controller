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
    command_control_loop::{
        CommandApplyForces, CommandGetInnerLoopControlMode, CommandMoveActuators,
        CommandPositionMirror, CommandResetActuatorSteps, CommandResetForceOffsets,
        CommandSetClosedLoopControlMode, CommandSetConfig, CommandSetExternalElevation,
        CommandSetInnerLoopControlMode,
    },
    command_schema::CommandSchema,
};
use crate::config::Config;
use crate::constants::BOUND_SYNC_CHANNEL;
use crate::control::control_loop::ControlLoop;
use crate::telemetry::telemetry::Telemetry;

pub struct ControlLoopProcess {
    // Control loop
    pub control_loop: ControlLoop,
    // Command schema
    _command_schema: CommandSchema,
    // Sender of the telemetry to the model.
    _sender_to_model: SyncSender<Telemetry>,
    // Sender of the message to the control loop.
    _sender_to_control_loop: SyncSender<Value>,
    // Receiver of the message to the control loop.
    _receiver_to_control_loop: Receiver<Value>,
    // Stop the loop.
    _stop: Arc<AtomicBool>,
}

impl ControlLoopProcess {
    /// Create a new instance of the control loop process.
    ///
    /// # Arguments
    /// * `config` - The configuration.
    /// * `is_mirror` - Is the mirror or the surrogate.
    /// * `is_simulation_mode` - Is the simulation mode or not.
    /// * `sender_to_model` - The sender to the model.
    /// * `stop` - An Arc instance that holds the AtomicBool instance to stop
    /// the loop.
    ///
    /// # Returns
    /// New instance of the control loop process.
    pub fn new(
        config: &Config,
        is_mirror: bool,
        is_simulation_mode: bool,
        sender_to_model: &SyncSender<Telemetry>,
        stop: &Arc<AtomicBool>,
    ) -> Self {
        // Sender and receiver to the control loop
        let (sender_to_control_loop, receiver_to_control_loop) = sync_channel(BOUND_SYNC_CHANNEL);

        Self {
            control_loop: ControlLoop::new(config, is_mirror, is_simulation_mode),

            _command_schema: Self::create_command_schema(),

            _sender_to_model: sender_to_model.clone(),

            _sender_to_control_loop: sender_to_control_loop,
            _receiver_to_control_loop: receiver_to_control_loop,

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
        command_schema.add_command(Box::new(CommandGetInnerLoopControlMode));
        command_schema.add_command(Box::new(CommandSetInnerLoopControlMode));

        command_schema
    }

    /// Get the sender to the control loop.
    ///
    /// # Returns
    /// The sender to the control loop.
    pub fn get_sender_to_control_loop(&self) -> SyncSender<Value> {
        self._sender_to_control_loop.clone()
    }

    /// Run the control loop.
    pub fn run(&mut self) {
        info!("Control loop is running.");

        let period = (1000.0 / self.control_loop.config.control_frequency) as u64;
        while !self._stop.load(Ordering::Relaxed) {
            // Time the control loop
            let now = Instant::now();

            // Process the messages.
            let mut command_result = None;
            if let Ok(message) = self._receiver_to_control_loop.try_recv() {
                command_result = Some(self._command_schema.execute(
                    &message,
                    None,
                    Some(&mut self.control_loop),
                    None,
                ));
            }

            // Run the control loop
            self.control_loop.step();

            // Send the telemetry and event data to the model and ignore the
            // error.
            let mut telemetry = self.control_loop.telemetry.clone();

            let events = if self.control_loop.event_queue.has_event() {
                Some(self.control_loop.event_queue.get_events_and_clear())
            } else {
                None
            };

            let cycle_time = now.elapsed().as_millis() as u64;
            telemetry.cycle_time = (cycle_time as f64) / 1000.0;

            let _ = self._sender_to_model.try_send(Telemetry::new(
                None,
                Some(telemetry),
                command_result,
                events,
            ));

            // Sleep with the remaining time
            if period > cycle_time {
                sleep(Duration::from_millis(period - cycle_time));
            }
        }

        info!("Control loop is stopped.");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::path::Path;
    use std::thread::spawn;

    fn create_control_loop_process() -> (ControlLoopProcess, Receiver<Telemetry>) {
        let config = Config::new(
            Path::new("config/parameters_control.yaml"),
            Path::new("config/lut/handling"),
        );

        let stop = Arc::new(AtomicBool::new(false));

        let (sender_to_model, receiver_to_model) = sync_channel(BOUND_SYNC_CHANNEL);

        (
            ControlLoopProcess::new(&config, false, true, &sender_to_model, &stop),
            receiver_to_model,
        )
    }

    #[test]
    fn test_new() {
        let control_loop_process = create_control_loop_process().0;

        assert_eq!(
            control_loop_process._command_schema.number_of_commands(),
            10
        );
    }

    #[test]
    fn test_run() {
        let (mut control_loop_process, receiver_to_model) = create_control_loop_process();
        let stop = control_loop_process._stop.clone();

        let sender_to_control_loop = control_loop_process.get_sender_to_control_loop();

        let handle = spawn(move || {
            control_loop_process.run();
        });

        sleep(Duration::from_millis(500));

        // Set the closed-loop control mode.
        let _ = sender_to_control_loop.try_send(json!({
            "id": "cmd_setClosedLoopControlMode",
            "sequence_id": 2,
            "mode": 2,
        }));

        // Check the telemetry data.
        sleep(Duration::from_millis(500));

        let mut latest_telemetry = Telemetry::new(None, None, None, None);
        loop {
            match receiver_to_model.try_recv() {
                Ok(telemetry) => {
                    if let Some(_result) = &telemetry.command_result {
                        latest_telemetry = telemetry;
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
                    "mode": 2,
                }),
                json!({
                    "id": "forceBalanceSystemStatus",
                    "status": false,
                })
            ]
        );

        // Close the server.
        stop.store(true, Ordering::Relaxed);

        assert!(handle.join().is_ok());
    }
}
