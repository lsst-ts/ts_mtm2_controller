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

use log::error;
use serde_json::{json, Value};
use std::collections::HashSet;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::sync::mpsc::SyncSender;

use crate::command::{
    command_control_loop::{
        CommandSetClosedLoopControlMode, CommandSetConfig, CommandSetExternalElevation,
    },
    command_power_system::{CommandSwitchDigitalOutput, CommandToggleBitClosedLoopControl},
    command_schema::Command,
};
use crate::config::Config;
use crate::constants::{NUM_ACTUATOR, NUM_AXIAL_ACTUATOR, NUM_HARDPOINTS, NUM_HARDPOINTS_AXIAL};
use crate::control::{lut::Lut, math_tool::check_hardpoints};
use crate::enums::{
    BitEnum, ClosedLoopControlMode, Commander, DigitalOutput, DigitalOutputStatus, ErrorCode,
    InnerLoopControlMode, PowerSystemState, PowerType,
};
use crate::error_handler::ErrorHandler;
use crate::event_queue::EventQueue;
use crate::status::Status;
use crate::telemetry::{event::Event, telemetry::Telemetry};
use crate::utility::get_message_name;

pub struct Controller {
    // System status
    pub status: Status,
    // The commander: CSC or GUI.
    pub commander: Commander,
    // Error handler.
    pub error_handler: ErrorHandler,
    // Events to publish
    pub event_queue: EventQueue,
    // Last effect telemetry.
    pub last_effective_telemetry: Telemetry,
    // Sender to the power system.
    pub sender_to_power_system: Option<SyncSender<Value>>,
    // Sender to the control loop.
    pub sender_to_control_loop: Option<SyncSender<Value>>,
}

impl Controller {
    /// Create a new controller.
    ///
    /// # Arguments
    /// * `lut_dir` - The path to the directory that contains the look-up table.
    ///
    /// # Returns
    /// A new controller.
    pub fn new(lut_dir: &Path) -> Self {
        Self {
            status: Status::new(),

            // By default, the commander is CSC.
            commander: Commander::CSC,

            error_handler: ErrorHandler::new(&Config::new(
                Path::new("config/parameters_control.yaml"),
                lut_dir,
            )),

            event_queue: EventQueue::new(),

            last_effective_telemetry: Telemetry::new(None, None, None, None),

            sender_to_power_system: None,
            sender_to_control_loop: None,
        }
    }

    /// Set the commander.
    ///
    /// # Arguments
    /// * `commander` - The commander: CSC or GUI.
    ///
    /// # Returns
    /// Some if the commander is set. Otherwise, None.
    pub fn set_commander(&mut self, commander: Commander) -> Option<()> {
        self.commander = commander;
        self.event_queue
            .add_event(Event::get_message_commandable_by_dds(
                commander == Commander::CSC,
            ));

        Some(())
    }

    /// Set the hardpoints.
    ///
    /// # Arguments
    /// * `hardpoints` - Six 0-based hardpoints to set (order is from low to
    /// high).
    ///
    /// # Returns
    /// Some if the hardpoints are set. Otherwise, None.
    pub fn set_hardpoints(&mut self, hardpoints: &Vec<usize>) -> Option<()> {
        // The first 3 should be the axial actuators. The last 3 should be the
        // tangent links.
        for idx in 0..NUM_HARDPOINTS {
            if idx < NUM_HARDPOINTS_AXIAL {
                if hardpoints[idx] >= NUM_AXIAL_ACTUATOR {
                    return None;
                }
            } else {
                if hardpoints[idx] < NUM_AXIAL_ACTUATOR {
                    return None;
                }
            }
        }

        // Check the geometry of the hardpoints.
        if let Err(_) = check_hardpoints(
            &self
                .error_handler
                .config_control_loop
                .cell_geometry
                .loc_act_axial,
            &hardpoints[..NUM_HARDPOINTS_AXIAL],
            &hardpoints[NUM_HARDPOINTS_AXIAL..],
        ) {
            return None;
        }

        // Update the configuration.
        let mut config = self.error_handler.config_control_loop.clone();
        config.hardpoints = hardpoints.clone();

        self.send_config_to_control_loop_and_update(config)
    }

    /// Send the configuration to the control loop and update the error handler.
    ///
    /// # Notes
    /// This function can only be called when the control loop is not in
    /// closed-loop mode. If the control loop is in closed-loop mode, it will
    /// return None and log an error message.
    ///
    /// # Arguments
    /// * `config` - The configuration to send.
    ///
    /// # Returns
    /// Some if the configuration is sent. Otherwise, None.
    pub fn send_config_to_control_loop_and_update(&mut self, config: Config) -> Option<()> {
        if self.status.mode == ClosedLoopControlMode::ClosedLoop {
            error!("Cannot send configuration to control loop when the mode is closed-loop mode.");

            return None;
        }

        if let Some(sender) = self.sender_to_control_loop.as_ref() {
            match sender.try_send(json!(
                {
                    "id": CommandSetConfig.name(),
                    "config": serde_json::to_string(&config).ok()?,
                }
            )) {
                Ok(_) => {
                    self.event_queue
                        .add_event(Event::get_message_hardpoint_list(&config.hardpoints));
                    self.event_queue
                        .add_event(Event::get_message_inclination_telemetry_source(
                            config.use_external_elevation_angle,
                        ));
                    self.event_queue
                        .add_event(Event::get_message_temperature_offset(
                            &config.ref_temperature,
                        ));
                    self.event_queue
                        .add_event(Event::get_message_open_loop_max_limit(
                            config.enable_open_loop_max_limit,
                        ));
                    self.event_queue
                        .add_event(Event::get_message_enabled_faults_mask(
                            config.enabled_faults_mask,
                        ));

                    self.error_handler.config_control_loop = config;
                    return Some(());
                }
                Err(_) => return None,
            }
        }

        None
    }

    /// Set the configuration.
    ///
    /// # Arguments
    /// * `file` - The file name of the configuration.
    ///
    /// # Returns
    /// Some if the configuration is set. Otherwise, None.
    pub fn set_config(&mut self, file: &str) -> Option<()> {
        let mut config = self.error_handler.config_control_loop.clone();

        let lut_dir_current = Path::new(&config.lut.dir_name);
        let parent = lut_dir_current.parent()?;
        let lut_dir_new = parent.join(file);
        if !lut_dir_new.exists() {
            error!(
                "Configuration directory does not exist: {}",
                lut_dir_new.display()
            );

            return None;
        }

        config.lut = Lut::new(lut_dir_new.as_path());

        let result = self.send_config_to_control_loop_and_update(config.clone());
        if result.is_some() {
            self.event_queue
                .add_event(Event::get_message_config(&config));
        }

        result
    }

    /// Set the enable open-loop maximum limit.
    ///
    /// # Arguments
    /// * `enable` - Enable or disable the open-loop maximum limit.
    ///
    /// # Returns
    /// Some if the open-loop maximum limit is set. Otherwise, None.
    pub fn set_enable_open_loop_max_limit(&mut self, enable: bool) -> Option<()> {
        // We can not enable the open-loop maximum limit when the control loop
        // is in closed-loop mode.
        if enable && (self.status.mode == ClosedLoopControlMode::ClosedLoop) {
            return None;
        }

        // Update the configuration.
        let mut config = self.error_handler.config_control_loop.clone();
        config.enable_open_loop_max_limit = enable;
        self.send_config_to_control_loop_and_update(config)
    }

    /// Set the enabled faults mask.
    ///
    /// # Arguments
    /// * `mask` - The mask to set the enabled faults.
    ///
    /// # Returns
    /// Some if the mask is set. Otherwise, None.
    pub fn set_enabled_faults_mask(&mut self, mask: u64) -> Option<()> {
        self.error_handler.update_enabled_faults_mask(mask);

        // Update the configuration.
        let config = self.error_handler.config_control_loop.clone();
        self.send_config_to_control_loop_and_update(config)
    }

    /// Update the external elevation.
    ///
    /// # Arguments
    /// * `message` - Message to update the external elevation.
    pub fn update_external_elevation(&self, message: &Value) {
        if get_message_name(message) == "tel_elevation" {
            let mut message_command = message.clone();
            message_command["id"] = CommandSetExternalElevation.name().into();

            if let Some(sender) = self.sender_to_control_loop.as_ref() {
                let _ = sender.try_send(message_command);
            }
        }
    }

    /// Switch the force balance system on or off.
    ///
    /// # Arguments
    /// * `balance_on` - True to switch on the force balance system, false to
    /// switch it off.
    ///
    /// # Returns
    /// Some if the force balance system is switched on or off. Otherwise, None.
    pub fn switch_force_balance_system(&mut self, balance_on: bool) -> Option<()> {
        let power_system = &self.status.power_system;
        if (!power_system[&PowerType::Communication].is_power_on())
            || (!power_system[&PowerType::Motor].is_power_on())
        {
            error!(
                "Cannot switch the force balance system when the power systems are not powered on."
            );

            return None;
        }

        // Switch on/off the force balance system.
        let mode = if balance_on {
            ClosedLoopControlMode::ClosedLoop
        } else {
            ClosedLoopControlMode::OpenLoop
        };

        self.update_closed_loop_control_mode(mode)
    }

    /// Update the closed-loop control mode.
    ///
    /// # Arguments
    /// * `mode` - Closed-loop control mode.
    ///
    /// # Returns
    /// Some if the mode is sent. Otherwise, None.
    pub fn update_closed_loop_control_mode(&mut self, mode: ClosedLoopControlMode) -> Option<()> {
        // Turn off the open-loop maximum limit if the mode is closed-loop.
        if self
            .error_handler
            .config_control_loop
            .enable_open_loop_max_limit
            && mode == ClosedLoopControlMode::ClosedLoop
        {
            if self.set_enable_open_loop_max_limit(false).is_none() {
                return None;
            }
        }

        if let Some(sender) = self.sender_to_control_loop.as_ref() {
            return sender
                .try_send(json!(
                    {
                        "id": CommandSetClosedLoopControlMode.name(),
                        "mode": mode as u8,
                    }
                ))
                .ok();
        }

        None
    }

    /// Update the internal status that the mirror is in position or not.
    ///
    /// # Arguments
    /// * `is_in_position` - Is in position or not.
    ///
    /// # Returns
    /// Some if the status is updated. Otherwise, None.
    pub fn update_internal_status_is_in_position(&mut self, is_in_position: bool) -> Option<()> {
        if self.status.is_in_position != is_in_position {
            self.status.is_in_position = is_in_position;

            return Some(());
        }

        None
    }

    /// Get the error events related to the error handler.
    ///
    /// # Returns
    /// A vector of events.
    pub fn get_error_handler_events(&mut self) -> Vec<Value> {
        let mut events = Vec::new();

        // Check the cell temperature high warning.
        let is_cell_temperature_high = self.error_handler.has_error(ErrorCode::WarnCellTemp);
        if let Some(()) = self.update_interal_status_cell_temperature_high(is_cell_temperature_high)
        {
            events.push(Event::get_message_cell_temperature_high_warning(
                is_cell_temperature_high,
            ));
        }

        // Check the limit switch.
        let limit_switch_retract = self.error_handler.ilc["limit_switch_retract"].to_owned();
        let limit_switch_extend = self.error_handler.ilc["limit_switch_extend"].to_owned();
        if let Some(()) =
            self.update_internal_status_limit_switch(limit_switch_retract, limit_switch_extend)
        {
            let limit_switch = &self.status.limit_switch;
            events.push(Event::get_message_limit_switch_status(
                &limit_switch["retract"],
                &limit_switch["extend"],
            ));
        }

        // Check the summary faults status.
        let summary_faults_status = self.error_handler.summary_faults_status;
        if let Some(()) = self.update_interal_status_summary_faults_status(summary_faults_status) {
            events.push(Event::get_message_summary_faults_status(
                summary_faults_status,
            ));
        }

        events
    }

    /// Update the internal status of cell temperature.
    ///
    /// # Arguments
    /// * `is_cell_temperature_high` - Is cell temperature high or not.
    ///
    /// # Returns
    /// Some if the status is updated. Otherwise, None.
    fn update_interal_status_cell_temperature_high(
        &mut self,
        is_cell_temperature_high: bool,
    ) -> Option<()> {
        if self.status.is_cell_temperature_high != is_cell_temperature_high {
            self.status.is_cell_temperature_high = is_cell_temperature_high;

            return Some(());
        }

        None
    }

    /// Update the internal status of limit switch.
    ///
    /// # Arguments
    /// * `limit_switch_retract` - Triggered retracted limit switch.
    /// * `limit_switch_extend` - Triggered extended limit switch.
    ///
    /// # Returns
    /// Some if the status is updated. Otherwise, None.
    fn update_internal_status_limit_switch(
        &mut self,
        limit_switch_retract: HashSet<i32>,
        limit_switch_extend: HashSet<i32>,
    ) -> Option<()> {
        if self.status.limit_switch["retract"] != limit_switch_retract
            || self.status.limit_switch["extend"] != limit_switch_extend
        {
            // Update the internal status
            self.status
                .limit_switch
                .insert(String::from("retract"), limit_switch_retract);
            self.status
                .limit_switch
                .insert(String::from("extend"), limit_switch_extend);

            return Some(());
        }

        None
    }

    /// Update the internal status of summary faults status.
    ///
    /// # Arguments
    /// * `summary_faults_status` - Summary faults status.
    ///
    /// # Returns
    /// Some if the status is updated. Otherwise, None.
    fn update_interal_status_summary_faults_status(
        &mut self,
        summary_faults_status: u64,
    ) -> Option<()> {
        if self.status.summary_faults_status != summary_faults_status {
            self.status.summary_faults_status = summary_faults_status;

            return Some(());
        }

        None
    }

    /// Update the internal status of interlock.
    ///
    /// # Arguments
    /// * `is_interlock_on` - Is interlock on or not.
    ///
    /// # Returns
    /// Some if the status is updated. Otherwise, None.
    pub fn update_internal_status_interlock(&mut self, is_interlock_on: bool) -> Option<()> {
        if self.status.is_interlock_on != is_interlock_on {
            self.status.is_interlock_on = is_interlock_on;

            return Some(());
        }

        None
    }

    /// Update the internal status of digital output.
    ///
    /// # Arguments
    /// * `digital_output` - Digital output.
    ///
    /// # Returns
    /// Some if the status is updated. Otherwise, None.
    pub fn update_internal_status_digital_output(&mut self, digital_output: u8) -> Option<()> {
        if self.status.digital_output != digital_output {
            self.status.digital_output = digital_output;

            return Some(());
        }

        None
    }

    /// Update the internal status of digital input.
    ///
    /// # Arguments
    /// * `digital_input` - Digital input.
    ///
    /// # Returns
    /// Some if the status is updated. Otherwise, None.
    pub fn update_internal_status_digital_input(&mut self, digital_input: u32) -> Option<()> {
        if self.status.digital_input != digital_input {
            self.status.digital_input = digital_input;

            return Some(());
        }

        None
    }

    /// Update the internal status of power system status.
    ///
    /// # Arguments
    /// * `event` - Event.
    ///
    /// # Returns
    /// Some if the status is updated. Otherwise, None.
    pub fn update_internal_status_power_system(&mut self, event: &Value) -> Option<()> {
        let power_type = PowerType::from_repr(event["powerType"].as_u64()? as u8)?;
        let status = event["status"].as_bool()?;
        let power_system_state = PowerSystemState::from_repr(event["state"].as_u64()? as u8)?;

        self.status
            .update_power_system(power_type, status, power_system_state)
    }

    /// Update the internal status of closed-loop control mode.
    ///
    /// # Arguments
    /// * `event` - Event.
    ///
    /// # Returns
    /// Some if the status is updated. Otherwise, None.
    pub fn update_internal_status_mode(&mut self, event: &Value) -> Option<()> {
        let new_mode: ClosedLoopControlMode =
            ClosedLoopControlMode::from_repr(event["mode"].as_u64()? as u8)?;

        // Turn on and toggle the closed-loop control bit in power system if
        // the system is in the closed-loop control.
        let mut do_toggle_bit = false;
        if (self.status.mode != ClosedLoopControlMode::ClosedLoop)
            && (new_mode == ClosedLoopControlMode::ClosedLoop)
        {
            do_toggle_bit = true;
        }

        if let Some(sender) = self.sender_to_power_system.as_ref() {
            if sender
                .try_send(json!(
                    {
                        "id": CommandToggleBitClosedLoopControl.name(),
                        "status": do_toggle_bit,
                    }
                ))
                .is_err()
            {
                error!("Failed to toggle the closed-loop control bit in power system.");
            }

            // If the new mode is not the closed-loop control mode, this can
            // make sure we turn off the bit of closed-loop control.
            if !do_toggle_bit {
                if sender
                    .try_send(json!(
                        {
                            "id": CommandSwitchDigitalOutput.name(),
                            "bit": DigitalOutput::ClosedLoopControl.bit_value() as u8,
                            "status": DigitalOutputStatus::BinaryLowLevel as u8,
                        }
                    ))
                    .is_err()
                {
                    error!("Failed to turn off the closed-loop control bit in power system.");
                }
            }
        }

        // Update the internal status
        self.status.mode = new_mode;

        Some(())
    }

    /// Update the internal status of inner loop control mode.
    ///
    /// # Arguments
    /// * `event` - Event.
    ///
    /// # Returns
    /// Some if the status is updated. Otherwise, None.
    pub fn update_internal_status_ilc_mode(&mut self, event: &Value) -> Option<()> {
        let address = event["address"].as_u64()? as usize;
        let mode = InnerLoopControlMode::from_repr(event["mode"].as_u64()? as u8)?;

        self.status.ilc_modes[address] = mode;

        Some(())
    }

    /// Save the mirror position to a file.
    ///
    /// # Arguments
    /// * `filepath` - An optional path to the file where the mirror position
    /// will be saved.
    /// * `file` - An optional file handle to write the mirror position. This is
    /// useful for the testing purpose.
    ///
    /// # Returns
    /// A result containing the content written to the file or an error if
    /// the operation fails.
    pub fn save_mirror_position(
        &self,
        filepath: Option<&Path>,
        file: Option<File>,
    ) -> std::io::Result<String> {
        match &self.last_effective_telemetry.control_loop {
            Some(telemetry) => {
                // Create the file
                let mut position_file;
                if filepath.is_some() {
                    position_file = File::create(filepath.unwrap())?;
                } else {
                    position_file = file.unwrap();
                }

                // Write the header and hardpoints
                let mut content = String::new();
                let header = r#"---
# Mirror position

# 0-based hardpoints from low to high. The first 3 hardpoints are the axial
# actuators, and the last 3 hardpoints are the tangent links.
"#;
                content.push_str(header);
                content.push_str(&format!(
                    "hardpoints: {:?}\n",
                    self.error_handler.config_control_loop.hardpoints
                ));

                let positions = &telemetry.actuator_positions;
                content.push_str("\n# 78 actuator positions in meter.\n");
                for idx in 0..NUM_ACTUATOR {
                    if idx == 0 {
                        content.push_str(&format!("positions: [{},\n", positions[idx]));
                    } else if idx == (NUM_ACTUATOR - 1) {
                        content.push_str(&format!("            {}]\n", positions[idx]));
                    } else {
                        content.push_str(&format!("            {},\n", positions[idx]));
                    }
                }

                position_file.write_all(content.as_bytes())?;
                position_file.flush()?;

                Ok(content)
            }

            None => {
                // Fail if there is no available telemetry
                Err(std::io::Error::other(
                    "No effective telemetry available to save the mirror position.",
                ))
            }
        }
    }

    /// Set the mirror home position.
    ///
    /// # Returns
    /// Some if the home position is set. Otherwise, None.
    pub fn set_mirror_home(&mut self) -> Option<()> {
        match &self.last_effective_telemetry.control_loop {
            Some(telemetry) => {
                // Update the home position
                let mm_to_m = 1e-3;
                self.error_handler
                    .config_control_loop
                    .hardpoints
                    .iter()
                    .enumerate()
                    .for_each(|(idx, hardpoint)| {
                        self.error_handler.config_control_loop.disp_hardpoint_home[idx] =
                            telemetry.actuator_positions[*hardpoint] * mm_to_m;
                    });

                // Update the configuration
                let config = self.error_handler.config_control_loop.clone();
                self.send_config_to_control_loop_and_update(config)
            }

            None => {
                error!("No effective telemetry available to set the mirror home.");
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use serde_json::json;
    use std::sync::mpsc::{sync_channel, Receiver};
    use tempfile::tempfile;

    use crate::constants::BOUND_SYNC_CHANNEL;
    use crate::telemetry::telemetry_control_loop::TelemetryControlLoop;

    fn create_controller() -> (Controller, Receiver<Value>, Receiver<Value>) {
        let (sender_to_power_system, receiver_to_power_system) = sync_channel(BOUND_SYNC_CHANNEL);
        let (sender_to_control_loop, receiver_to_control_loop) = sync_channel(BOUND_SYNC_CHANNEL);

        let mut controller = Controller::new(Path::new("config/lut/handling"));
        controller.sender_to_power_system = Some(sender_to_power_system);
        controller.sender_to_control_loop = Some(sender_to_control_loop);

        (
            controller,
            receiver_to_power_system,
            receiver_to_control_loop,
        )
    }

    #[test]
    fn test_set_commander() {
        let mut controller = create_controller().0;

        assert!(controller.set_commander(Commander::GUI).is_some());
        assert_eq!(controller.commander, Commander::GUI);
        assert_eq!(
            controller.event_queue.get_events_and_clear(),
            vec![Event::get_message_commandable_by_dds(false)],
        );
    }

    #[test]
    fn test_set_hardpoints() {
        let (mut controller, _, _receiver_to_control_loop) = create_controller();

        // Invalid hardpoints
        assert!(controller.set_hardpoints(&vec![0, 1, 2, 3, 4, 6]).is_none());

        // Valid hardpoints
        let hardpoints = vec![4, 14, 24, 72, 74, 76];

        assert!(controller.set_hardpoints(&hardpoints).is_some());
        assert_eq!(
            controller.error_handler.config_control_loop.hardpoints,
            hardpoints
        );
    }

    #[test]
    fn test_send_config_to_control_loop_and_update() {
        let (mut controller, _, _receiver_to_control_loop) = create_controller();

        // Should succeed when the mode is not closed-loop
        let mut config = controller.error_handler.config_control_loop.clone();
        let hardpoints = vec![4, 14, 24, 72, 74, 76];
        config.hardpoints = hardpoints.clone();

        assert!(controller
            .send_config_to_control_loop_and_update(config.clone())
            .is_some());
        assert_eq!(
            controller.event_queue.get_events_and_clear(),
            vec![
                Event::get_message_hardpoint_list(&hardpoints),
                Event::get_message_inclination_telemetry_source(
                    controller
                        .error_handler
                        .config_control_loop
                        .use_external_elevation_angle
                ),
                Event::get_message_temperature_offset(
                    &controller.error_handler.config_control_loop.ref_temperature
                ),
                Event::get_message_open_loop_max_limit(
                    controller
                        .error_handler
                        .config_control_loop
                        .enable_open_loop_max_limit
                ),
                Event::get_message_enabled_faults_mask(
                    controller
                        .error_handler
                        .config_control_loop
                        .enabled_faults_mask
                ),
            ],
        );

        // Should fail when the mode is closed-loop
        controller.status.mode = ClosedLoopControlMode::ClosedLoop;
        assert!(controller
            .send_config_to_control_loop_and_update(config)
            .is_none());
    }

    #[test]
    fn test_set_config() {
        let (mut controller, _, _receiver_to_control_loop) = create_controller();

        // Invalid config file
        assert!(controller.set_config("invalid_file").is_none());

        // Valid config file
        let file = "optical";
        assert!(controller.set_config(file).is_some());
        assert_eq!(
            controller.event_queue.get_events_and_clear(),
            vec![
                Event::get_message_hardpoint_list(
                    &controller.error_handler.config_control_loop.hardpoints
                ),
                Event::get_message_inclination_telemetry_source(
                    controller
                        .error_handler
                        .config_control_loop
                        .use_external_elevation_angle
                ),
                Event::get_message_temperature_offset(
                    &controller.error_handler.config_control_loop.ref_temperature
                ),
                Event::get_message_open_loop_max_limit(
                    controller
                        .error_handler
                        .config_control_loop
                        .enable_open_loop_max_limit
                ),
                Event::get_message_enabled_faults_mask(
                    controller
                        .error_handler
                        .config_control_loop
                        .enabled_faults_mask
                ),
                Event::get_message_config(&controller.error_handler.config_control_loop),
            ],
        );
    }

    #[test]
    fn test_set_enable_open_loop_max_limit() {
        let (mut controller, _, _receiver_to_control_loop) = create_controller();

        // Invalid open-loop maximum limit
        controller.status.mode = ClosedLoopControlMode::ClosedLoop;

        assert!(controller.set_enable_open_loop_max_limit(true).is_none());

        // Valid open-loop maximum limit
        controller.status.mode = ClosedLoopControlMode::OpenLoop;

        assert!(controller.set_enable_open_loop_max_limit(true).is_some());
        assert!(
            controller
                .error_handler
                .config_control_loop
                .enable_open_loop_max_limit
        );
    }

    #[test]
    fn test_set_enabled_faults_mask() {
        let (mut controller, _, _receiver_to_control_loop) = create_controller();

        assert!(controller.set_enabled_faults_mask(1).is_some());
        assert_eq!(
            controller
                .error_handler
                .config_control_loop
                .enabled_faults_mask,
            1
        );
    }

    #[test]
    fn test_update_external_elevation() {
        let (controller, _, receiver_to_control_loop) = create_controller();

        controller.update_external_elevation(&json!({
            "id": "tel_elevation",
            "compName": "mtmount",
            "actualPosition": 10.0,
        }));

        let message = receiver_to_control_loop.recv().unwrap();
        assert_eq!(
            message,
            json!({"id": "cmd_setExternalElevation", "compName": "mtmount", "actualPosition": 10.0})
        );
    }

    #[test]
    fn test_switch_force_balance_system() {
        let (mut controller, _, receiver_to_control_loop) = create_controller();

        // Should fail if the power systems are not powered on
        assert!(controller.switch_force_balance_system(true).is_none());

        // Set the power system states to powered on.
        controller.status.update_power_system(
            PowerType::Communication,
            true,
            PowerSystemState::PoweredOn,
        );
        controller
            .status
            .update_power_system(PowerType::Motor, true, PowerSystemState::PoweredOn);

        // Switch on the force balance system.
        assert!(controller.switch_force_balance_system(true).is_some());
        assert_eq!(
            receiver_to_control_loop.try_recv().ok(),
            Some(json!({
                "id": "cmd_setClosedLoopControlMode",
                "mode": 4,
            }))
        );

        // Switch off the force balance system.
        assert!(controller.switch_force_balance_system(false).is_some());
        assert_eq!(
            receiver_to_control_loop.try_recv().ok(),
            Some(json!({
                "id": "cmd_setClosedLoopControlMode",
                "mode": 3,
            }))
        );
    }

    #[test]
    fn test_update_closed_loop_control_mode() {
        let (mut controller, _, receiver_to_control_loop) = create_controller();
        controller
            .error_handler
            .config_control_loop
            .enable_open_loop_max_limit = true;

        controller.update_closed_loop_control_mode(ClosedLoopControlMode::ClosedLoop);

        assert!(
            !controller
                .error_handler
                .config_control_loop
                .enable_open_loop_max_limit
        );

        // Bypass some messages that are not used in the test
        let mut message;
        loop {
            message = receiver_to_control_loop.recv().unwrap();
            if message["id"] == "cmd_setClosedLoopControlMode" {
                break;
            }
        }

        assert_eq!(
            message,
            json!({"id": "cmd_setClosedLoopControlMode", "mode": 4})
        );
    }

    #[test]
    fn test_update_internal_status_is_in_position() {
        let mut controller = create_controller().0;

        assert!(controller
            .update_internal_status_is_in_position(true)
            .is_some());
        assert!(controller.status.is_in_position);

        assert!(controller
            .update_internal_status_is_in_position(true)
            .is_none());
    }

    #[test]
    fn test_get_error_handler_events() {
        let mut controller = create_controller().0;

        controller.error_handler.add_error(ErrorCode::WarnCellTemp);
        controller.error_handler.ilc.insert(
            String::from("limit_switch_retract"),
            HashSet::from([1, 2, 3]),
        );
        controller.status.summary_faults_status = 1;

        let events = controller.get_error_handler_events();

        assert_eq!(events.len(), 3);
    }

    #[test]
    fn test_update_internal_status_cell_temperature_high() {
        let mut controller = create_controller().0;

        assert!(controller
            .update_interal_status_cell_temperature_high(true)
            .is_some());
        assert!(controller.status.is_cell_temperature_high);

        assert!(controller
            .update_interal_status_cell_temperature_high(true)
            .is_none());
    }

    #[test]
    fn test_update_internal_status_limit_switch() {
        let mut controller = create_controller().0;

        let limit_switch_retract = HashSet::from([1, 2, 3]);
        let limit_switch_extend = HashSet::from([4, 5, 6]);

        assert!(controller
            .update_internal_status_limit_switch(
                limit_switch_retract.clone(),
                limit_switch_extend.clone()
            )
            .is_some());
        assert_eq!(
            controller.status.limit_switch["retract"],
            limit_switch_retract
        );
        assert_eq!(
            controller.status.limit_switch["extend"],
            limit_switch_extend
        );

        assert!(controller
            .update_internal_status_limit_switch(limit_switch_retract, limit_switch_extend)
            .is_none());
    }

    #[test]
    fn test_update_internal_status_summary_faults_status() {
        let mut controller = create_controller().0;

        assert!(controller
            .update_interal_status_summary_faults_status(1)
            .is_some());
        assert_eq!(controller.status.summary_faults_status, 1);

        assert!(controller
            .update_interal_status_summary_faults_status(1)
            .is_none());
    }

    #[test]
    fn test_update_internal_status_interlock() {
        let mut controller = create_controller().0;

        assert!(controller.update_internal_status_interlock(false).is_some());
        assert!(!controller.status.is_interlock_on);

        assert!(controller.update_internal_status_interlock(false).is_none());
    }

    #[test]
    fn test_update_internal_status_digital_output() {
        let mut controller = create_controller().0;

        assert!(controller
            .update_internal_status_digital_output(1)
            .is_some());
        assert_eq!(controller.status.digital_output, 1);

        assert!(controller
            .update_internal_status_digital_output(1)
            .is_none());
    }

    #[test]
    fn test_update_internal_status_digital_input() {
        let mut controller = create_controller().0;

        assert!(controller.update_internal_status_digital_input(1).is_some());
        assert_eq!(controller.status.digital_input, 1);

        assert!(controller.update_internal_status_digital_input(1).is_none());
    }

    #[test]
    fn test_update_internal_status_power_system() {
        let mut controller = create_controller().0;

        let event = json!({
            "powerType": 2,
            "status": true,
            "state": 2,
        });

        assert!(controller
            .update_internal_status_power_system(&event)
            .is_some());

        let power_system = &controller.status.power_system[&PowerType::Communication];
        assert!(power_system.is_power_on);
        assert_eq!(power_system.state, PowerSystemState::PoweredOff);
    }

    #[test]
    fn test_update_internal_status_mode() {
        let (mut controller, receiver_to_power_system, _) = create_controller();

        // Telemetry-only mode
        assert!(controller
            .update_internal_status_mode(&json!({
                "mode": 2,
            }))
            .is_some());
        assert_eq!(controller.status.mode, ClosedLoopControlMode::TelemetryOnly,);

        assert_eq!(
            receiver_to_power_system.recv().unwrap(),
            json!({"id": "cmd_toggleBitClosedLoopControl", "status": false})
        );
        assert_eq!(
            receiver_to_power_system.recv().unwrap(),
            json!({"id": "cmd_switchDigitalOutput", "bit": 32, "status": 1})
        );

        // Closed-loop mode
        assert!(controller
            .update_internal_status_mode(&json!({
                "mode": 4,
            }))
            .is_some());
        assert_eq!(controller.status.mode, ClosedLoopControlMode::ClosedLoop,);

        assert_eq!(
            receiver_to_power_system.recv().unwrap(),
            json!({"id": "cmd_toggleBitClosedLoopControl", "status": true})
        );
    }

    #[test]
    fn test_update_internal_status_ilc_mode() {
        let mut controller = create_controller().0;

        let event = json!({
            "address": 2,
            "mode": 2,
        });

        assert!(controller.update_internal_status_ilc_mode(&event).is_some());
        assert_eq!(
            controller.status.ilc_modes[2],
            InnerLoopControlMode::Disabled,
        );
    }

    #[test]
    fn test_save_mirror_position() {
        let mut controller = create_controller().0;

        // Should fail if there is no effective telemetry
        assert!(controller
            .save_mirror_position(None, Some(tempfile().unwrap()))
            .is_err());

        // Set the last effective telemetry and save the position
        let mut telemetry = TelemetryControlLoop::new();
        telemetry.actuator_positions = vec![0.15; NUM_ACTUATOR];

        controller.last_effective_telemetry.control_loop = Some(telemetry);

        let file = tempfile().unwrap();
        let result: Result<String, std::io::Error> =
            controller.save_mirror_position(None, Some(file));

        assert!(result.is_ok());

        let content = result.unwrap();

        assert_eq!(content.lines().count(), 86);
        assert!(content.contains("\nhardpoints: [5, 15, 25, 73, 75, 77]\n"));
        assert!(content.contains("\npositions: [0.15,\n"));
        assert!(content.contains("0.15,\n            0.15]\n"));
    }

    #[test]
    fn test_set_mirror_home() {
        let (mut controller, _, _receiver_to_control_loop) = create_controller();

        // Should fail if there is no effective telemetry
        assert!(controller.set_mirror_home().is_none());

        // Set the last effective telemetry and update the positions
        let mut telemetry = TelemetryControlLoop::new();

        let new_home_positions = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        controller
            .error_handler
            .config_control_loop
            .hardpoints
            .iter()
            .enumerate()
            .for_each(|(idx, hardpoint)| {
                telemetry.actuator_positions[*hardpoint] = new_home_positions[idx];
            });
        controller.last_effective_telemetry.control_loop = Some(telemetry);

        assert!(controller.set_mirror_home().is_some());
        assert_eq!(
            controller
                .error_handler
                .config_control_loop
                .disp_hardpoint_home,
            vec![0.001, 0.002, 0.003, 0.004, 0.005, 0.006]
        );
    }
}
