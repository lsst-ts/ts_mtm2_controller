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

use serde_json::{json, Value};
use std::collections::HashSet;
use std::path::Path;

use crate::config::Config;
use crate::constants::{NUM_TEMPERATURE_EXHAUST, NUM_TEMPERATURE_INTAKE};
use crate::enums::{
    ClosedLoopControlMode, InclinationTelemetrySource, InnerLoopControlMode, PowerSystemState,
    PowerType,
};
use crate::utility::get_parameter;

pub struct Event;
impl Event {
    /// Get the message of the M2 is in position or not.
    ///
    /// # Arguments
    /// * `in_position` - True if the M2 assembly is in position, false
    /// otherwise.
    ///
    /// # Returns
    /// The message of the M2 is in position or not.
    pub fn get_message_in_position(in_position: bool) -> Value {
        json!({
            "id": "m2AssemblyInPosition",
            "inPosition": in_position,
        })
    }

    /// Get the message that the cell temperature is high or not.
    ///
    /// # Arguments
    /// * `is_high` - True if the cell temperature is high, false otherwise.
    ///
    /// # Returns
    /// The message that the cell temperature is high or not.
    pub fn get_message_cell_temperature_high_warning(is_high: bool) -> Value {
        json!({
            "id": "cellTemperatureHiWarning",
            "hiWarning": is_high,
        })
    }

    /// Get the message that the M2 is commandable by DDS or not.
    ///
    /// # Arguments
    /// * `state` - Commandable by DDS or not.
    ///
    /// # Returns
    /// The message that the M2 is commandable by DDS or not.
    pub fn get_message_commandable_by_dds(state: bool) -> Value {
        json!({
            "id": "commandableByDDS",
            "state": state,
        })
    }

    /// Get the message that the interlock is on or not.
    ///
    /// # Arguments
    /// * `state` - Interlock is on or not.
    ///
    /// # Returns
    /// The message that the interlock is on or not.
    pub fn get_message_interlock(state: bool) -> Value {
        json!({
            "id": "interlock",
            "state": state,
        })
    }

    /// Get the message that the TCP/IP connection is on or not.
    ///
    /// # Arguments
    /// * `is_connected` - TCP/IP connection is on or not.
    ///
    /// # Returns
    /// The message that the TCP/IP connection is on or not.
    pub fn get_message_tcp_ip_connected(is_connected: bool) -> Value {
        json!({
            "id": "tcpIpConnected",
            "isConnected": is_connected,
        })
    }

    /// Get the message of the hardpoint list.
    ///
    /// # Arguments
    /// * `hardpoints` - 0-based hardpoint list.
    ///
    /// # Returns
    /// The message of the hardpoint list (1-based).
    pub fn get_message_hardpoint_list(hardpoints: &Vec<usize>) -> Value {
        let hardpoint_list: Vec<usize> = hardpoints.iter().map(|hardpoint| hardpoint + 1).collect();

        json!({
            "id": "hardpointList",
            "actuators": hardpoint_list,
        })
    }

    /// Get the message of the bypassed actuator inner-loop controllers (ILCs).
    ///
    /// # Arguments
    /// * `ilcs` - Bypassed ILCs.
    ///
    /// # Returns
    /// The message of the bypassed actuator ILCs.
    pub fn get_message_bypassed_actuator_ilcs(ilcs: &Vec<usize>) -> Value {
        json!({
            "id": "bypassedActuatorILCs",
            "ilcs": ilcs,
        })
    }

    /// Get the message that the force balance system is on or not.
    ///
    /// # Arguments
    /// * `status` - Force balance system is on or not.
    ///
    /// # Returns
    /// The message that the force balance system is on or not.
    pub fn get_message_force_balance_system_status(status: bool) -> Value {
        json!({
            "id": "forceBalanceSystemStatus",
            "status": status,
        })
    }

    /// Get the message of the inclination telemetry source.
    ///
    /// # Arguments
    /// * `is_external_source` - Is the external source or not.
    ///
    /// # Returns
    /// The message of the inclination telemetry source.
    pub fn get_message_inclination_telemetry_source(is_external_source: bool) -> Value {
        let source = if is_external_source {
            InclinationTelemetrySource::MtMount
        } else {
            InclinationTelemetrySource::OnBoard
        };

        json!({
            "id": "inclinationTelemetrySource",
            "source": source as u8,
        })
    }

    /// Get the message of the temperature offset in degree C.
    ///
    /// # Arguments
    /// * `ring` - Offset of ring temperatures.
    ///
    /// # Returns
    /// The message of the temperature offset in degree C.
    pub fn get_message_temperature_offset(ring: &Vec<f64>) -> Value {
        json!({
            "id": "temperatureOffset",
            "ring": ring,
            "intake": vec![0.0; NUM_TEMPERATURE_INTAKE],
            "exhaust": vec![0.0; NUM_TEMPERATURE_EXHAUST],
        })
    }

    /// Get the message of the digital output.
    ///
    /// # Arguments
    /// * `digital_output` - Digital output.
    ///
    /// # Returns
    /// The message of the digital output.
    pub fn get_message_digital_output(digital_output: u8) -> Value {
        json!({
            "id": "digitalOutput",
            "value": digital_output,
        })
    }

    /// Get the message of the digital input.
    ///
    /// # Arguments
    /// * `digital_input` - Digital input.
    ///
    /// # Returns
    /// The message of the digital input.
    pub fn get_message_digital_input(digital_input: u32) -> Value {
        json!({
            "id": "digitalInput",
            "value": digital_input,
        })
    }

    /// Get the message of the configuration.
    ///
    /// # Arguments
    /// * `config` - Configuration.
    ///
    /// # Returns
    /// The message of the configuration.
    pub fn get_message_config(config: &Config) -> Value {
        // TODO: Some of these values are hardcoded temporarily. They should be
        // read from the configuration file.
        let control_parameters_file = Path::new(&config.filename);

        let mut control_parameters = String::new();
        if let Some(file_name) = control_parameters_file.file_name() {
            if let Some(name) = file_name.to_str() {
                control_parameters = name.to_string();
            }
        }

        let mut lut_parameters = String::new();
        if let Some(file_name) = Path::new(&config.lut.dir_name).file_name() {
            if let Some(name) = file_name.to_str() {
                lut_parameters = name.to_string();
            }
        }

        json!({
            "id": "config",
            "configuration": Self::get_config_dir(control_parameters_file),
            "version": "v1.0",
            "controlParameters": control_parameters,
            "lutParameters": lut_parameters,
            "powerWarningMotor": 5.0,
            "powerFaultMotor": 10.0,
            "powerThresholdMotor": 20.0,
            "powerWarningComm": 5.0,
            "powerFaultComm": 10.0,
            "powerThresholdComm": 10.0,
            "inPositionAxial": get_parameter::<f64>(control_parameters_file, "in_position_threshold_axial"),
            "inPositionTangent": get_parameter::<f64>(control_parameters_file, "in_position_threshold_tangent"),
            "inPositionSample": get_parameter::<f64>(control_parameters_file, "in_position_window_size"),
            "timeoutSal": 15.0,
            "timeoutCrio": 1.0,
            "timeoutIlc": 3,
            "inclinometerDelta": config.max_angle_difference,
            "inclinometerDiffEnabled": config.enable_angle_comparison,
            "cellTemperatureDelta": config.max_cell_temperature_difference,
        })
    }

    /// Get the configuration directory.
    ///
    /// # Arguments
    /// * `config_path` - Path to the configuration file.
    ///
    /// # Returns
    /// The configuration directory.
    fn get_config_dir(config_path: &Path) -> String {
        let mut config_dir = String::new();
        if let Some(parent) = config_path.parent() {
            if let Some(parent_name) = parent.to_str() {
                config_dir = parent_name.to_string();
            }
        }

        config_dir
    }

    /// Get the message of the open-loop maximum limit.
    ///
    /// # Arguments
    /// * `status` - Open-loop maximum limit is enabled or not.
    ///
    /// # Returns
    /// The message of the open-loop maximum limit.
    pub fn get_message_open_loop_max_limit(status: bool) -> Value {
        json!({
            "id": "openLoopMaxLimit",
            "status": status,
        })
    }

    /// Get the message of the limit switch status.
    ///
    /// # Arguments
    /// * `limit_switch_retract` - Triggered retracted limit switch.
    /// * `limit_switch_extend` - Triggered extended limit switch.
    ///
    /// # Returns
    /// The message of the limit switch status.
    pub fn get_message_limit_switch_status(
        limit_switch_retract: &HashSet<i32>,
        limit_switch_extend: &HashSet<i32>,
    ) -> Value {
        json!({
            "id": "limitSwitchStatus",
            "retract": limit_switch_retract,
            "extend": limit_switch_extend,
        })
    }

    /// Get the message of the power system state.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    /// * `status` - Power status is on or not.
    /// * `power_system_state` - Power system state.
    ///
    /// # Returns
    /// The message of the power system state.
    pub fn get_message_power_system_state(
        power_type: PowerType,
        status: bool,
        power_system_state: PowerSystemState,
    ) -> Value {
        json!({
            "id": "powerSystemState",
            "powerType": power_type as u8,
            "status": status,
            "state": power_system_state as u8,
        })
    }

    /// Get the message of the closed-loop control mode.
    ///
    /// # Arguments
    /// * `mode` - Closed-loop control mode.
    ///
    /// # Returns
    /// The message of the closed-loop control mode.
    pub fn get_message_closed_loop_control_mode(mode: ClosedLoopControlMode) -> Value {
        json!({
            "id": "closedLoopControlMode",
            "mode": mode as u8,
        })
    }

    /// Get the message of the inner-loop control mode.
    ///
    /// # Arguments
    /// * `address` - 0-based address.
    /// * `mode` - Inner-loop control mode.
    ///
    /// # Returns
    /// The message of the inner-loop control mode.
    pub fn get_message_inner_loop_control_mode(
        address: usize,
        mode: InnerLoopControlMode,
    ) -> Value {
        json!({
            "id": "innerLoopControlMode",
            "address": address,
            "mode": mode as u8,
        })
    }

    /// Get the message of the summary faults status.
    ///
    /// # Arguments
    /// * `status` - Summary faults status.
    ///
    /// # Returns
    /// The message of the summary faults status.
    pub fn get_message_summary_faults_status(status: u64) -> Value {
        json!({
            "id": "summaryFaultsStatus",
            "status": status,
        })
    }

    /// Get the message of the enabled faults mask.
    ///
    /// # Arguments
    /// * `mask` - Enabled faults mask.
    ///
    /// # Returns
    /// The message of the enabled faults mask.
    pub fn get_message_enabled_faults_mask(mask: u64) -> Value {
        json!({
            "id": "enabledFaultsMask",
            "mask": mask,
        })
    }

    /// Get the message of the available look-up table (LUT) configuration
    /// files.
    ///
    /// # Arguments
    /// * `filename` - Configuration file name.
    ///
    /// # Returns
    /// The message of the available LUT configuration files.
    pub fn get_message_configuration_files(filename: &String) -> Value {
        let config_dir = Self::get_config_dir(Path::new(filename));

        let lut = Path::new(&config_dir).join("lut");
        let mut files = Vec::new();
        if let Ok(entries) = lut.read_dir() {
            for entry in entries {
                if let Ok(dir_entry) = entry {
                    if let Some(name) = dir_entry.file_name().to_str() {
                        files.push(name.to_string());
                    }
                }
            }
        }

        json!({
            "id": "configurationFiles",
            "files": files,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::constants::NUM_TEMPERATURE_RING;

    fn create_config() -> Config {
        Config::new(
            Path::new("config/parameters_control.yaml"),
            Path::new("config/lut/handling"),
        )
    }

    #[test]
    fn test_get_message_in_position() {
        assert_eq!(
            Event::get_message_in_position(true),
            json!({
                "id": "m2AssemblyInPosition",
                "inPosition": true,
            })
        );
    }

    #[test]
    fn test_get_message_cell_temperature_high_warning() {
        assert_eq!(
            Event::get_message_cell_temperature_high_warning(true),
            json!({
                "id": "cellTemperatureHiWarning",
                "hiWarning": true,
            })
        );
    }

    #[test]
    fn test_get_message_commandable_by_dds() {
        assert_eq!(
            Event::get_message_commandable_by_dds(true),
            json!({
                "id": "commandableByDDS",
                "state": true,
            })
        );
    }

    #[test]
    fn test_get_message_interlock() {
        assert_eq!(
            Event::get_message_interlock(true),
            json!({
                "id": "interlock",
                "state": true,
            })
        );
    }

    #[test]
    fn test_get_message_tcp_ip_connected() {
        assert_eq!(
            Event::get_message_tcp_ip_connected(true),
            json!({
                "id": "tcpIpConnected",
                "isConnected": true,
            })
        );
    }

    #[test]
    fn test_get_message_hardpoint_list() {
        assert_eq!(
            Event::get_message_hardpoint_list(&vec![0, 1, 2, 3, 4, 5]),
            json!({
                "id": "hardpointList",
                "actuators": vec![1, 2, 3, 4, 5, 6],
            })
        );
    }

    #[test]
    fn test_get_message_bypassed_actuator_ilcs() {
        assert_eq!(
            Event::get_message_bypassed_actuator_ilcs(&vec![1]),
            json!({
                "id": "bypassedActuatorILCs",
                "ilcs": vec![1],
            })
        );
    }

    #[test]
    fn test_get_message_force_balance_system_status() {
        assert_eq!(
            Event::get_message_force_balance_system_status(true),
            json!({
                "id": "forceBalanceSystemStatus",
                "status": true,
            })
        );
    }

    #[test]
    fn test_get_message_inclination_telemetry_source() {
        assert_eq!(
            Event::get_message_inclination_telemetry_source(true),
            json!({
                "id": "inclinationTelemetrySource",
                "source": InclinationTelemetrySource::MtMount as u8,
            })
        );
        assert_eq!(
            Event::get_message_inclination_telemetry_source(false),
            json!({
                "id": "inclinationTelemetrySource",
                "source": InclinationTelemetrySource::OnBoard as u8,
            })
        );
    }

    #[test]
    fn test_get_message_temperature_offset() {
        assert_eq!(
            Event::get_message_temperature_offset(&vec![1.0; NUM_TEMPERATURE_RING]),
            json!({
                "id": "temperatureOffset",
                "ring": vec![1.0; NUM_TEMPERATURE_RING],
                "intake": vec![0.0; NUM_TEMPERATURE_INTAKE],
                "exhaust": vec![0.0; NUM_TEMPERATURE_EXHAUST],
            })
        );
    }

    #[test]
    fn test_get_message_digital_output() {
        assert_eq!(
            Event::get_message_digital_output(1),
            json!({
                "id": "digitalOutput",
                "value": 1,
            })
        );
    }

    #[test]
    fn test_get_message_digital_input() {
        assert_eq!(
            Event::get_message_digital_input(1),
            json!({
                "id": "digitalInput",
                "value": 1,
            })
        );
    }

    #[test]
    fn test_get_message_config() {
        let config = create_config();
        assert_eq!(
            Event::get_message_config(&config),
            json!({
            "id": "config",
            "configuration": "config",
            "version": "v1.0",
            "controlParameters": "parameters_control.yaml",
            "lutParameters": "handling",
            "powerWarningMotor": 5.0,
            "powerFaultMotor": 10.0,
            "powerThresholdMotor": 20.0,
            "powerWarningComm": 5.0,
            "powerFaultComm": 10.0,
            "powerThresholdComm": 10.0,
            "inPositionAxial": 1.5,
            "inPositionTangent": 10.0,
            "inPositionSample": 1.0,
            "timeoutSal": 15.0,
            "timeoutCrio": 1.0,
            "timeoutIlc": 3,
            "inclinometerDelta": 2.0,
            "inclinometerDiffEnabled": false,
            "cellTemperatureDelta": 2.0,
            })
        );
    }

    #[test]
    fn test_get_message_open_loop_max_limit() {
        assert_eq!(
            Event::get_message_open_loop_max_limit(true),
            json!({
                "id": "openLoopMaxLimit",
                "status": true,
            })
        );
    }

    #[test]
    fn test_get_message_limit_switch_status() {
        let mut retract = HashSet::new();
        retract.insert(1);
        let mut extend = HashSet::new();
        extend.insert(2);
        assert_eq!(
            Event::get_message_limit_switch_status(&retract, &extend),
            json!({
                "id": "limitSwitchStatus",
                "retract": retract,
                "extend": extend,
            })
        );
    }

    #[test]
    fn test_get_message_power_system_state() {
        assert_eq!(
            Event::get_message_power_system_state(
                PowerType::Motor,
                true,
                PowerSystemState::PoweredOn
            ),
            json!({
                "id": "powerSystemState",
                "powerType": PowerType::Motor as u8,
                "status": true,
                "state": PowerSystemState::PoweredOn as u8,
            })
        );
    }

    #[test]
    fn test_get_message_closed_loop_control_mode() {
        assert_eq!(
            Event::get_message_closed_loop_control_mode(ClosedLoopControlMode::OpenLoop),
            json!({
                "id": "closedLoopControlMode",
                "mode": ClosedLoopControlMode::OpenLoop as u8,
            })
        );
    }

    #[test]
    fn test_get_message_inner_loop_control_mode() {
        assert_eq!(
            Event::get_message_inner_loop_control_mode(1, InnerLoopControlMode::Enabled),
            json!({
                "id": "innerLoopControlMode",
                "address": 1,
                "mode": InnerLoopControlMode::Enabled as u8,
            })
        );
    }

    #[test]
    fn test_get_message_summary_faults_status() {
        assert_eq!(
            Event::get_message_summary_faults_status(1),
            json!({
                "id": "summaryFaultsStatus",
                "status": 1,
            })
        );
    }

    #[test]
    fn test_get_message_enabled_faults_mask() {
        assert_eq!(
            Event::get_message_enabled_faults_mask(1),
            json!({
                "id": "enabledFaultsMask",
                "mask": 1,
            })
        );
    }

    #[test]
    fn test_get_message_configuration_files() {
        let config = create_config();
        assert_eq!(
            Event::get_message_configuration_files(&config.filename),
            json!({
                "id": "configurationFiles",
                "files": vec!["handling", "optical"],
            })
        );
    }
}
