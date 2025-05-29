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

use log::{error, warn};
use std::collections::{HashMap, HashSet};
use std::path::Path;
use strum::IntoEnumIterator;

use crate::config::Config;
use crate::constants::NUM_AXIAL_ACTUATOR;
use crate::control::actuator::Actuator;
use crate::enums::{BitEnum, ErrorCode};
use crate::power::config_power::ConfigPower;
use crate::telemetry::{
    telemetry_control_loop::TelemetryControlLoop, telemetry_power::TelemetryPower,
};

pub struct ErrorHandler {
    // Configuration of the control loop.
    pub config_control_loop: Config,
    // Configuration of the power system.
    pub config_power: ConfigPower,
    // Actuators
    pub actuators: Vec<Actuator>,
    // Summary of the faults status.
    pub summary_faults_status: u64,
    _faults_mask: u64,
    // Inner loop controller (ILC) status.
    // The hash set of ilc contains the 0-based actuator index that has the
    // fault or triggered limit switches.
    pub ilc: HashMap<String, HashSet<i32>>,
    // Count of the cycle time that is out of the maximum continuously.
    _count_out_max_cycle_time: i32,
}

impl ErrorHandler {
    /// Create a new error handler.
    ///
    /// # Arguments
    /// * `config_control_loop` - Configuration of the control loop.
    ///
    /// # Returns
    /// A new error handler.
    pub fn new(config_control_loop: &Config) -> Self {
        let mut ilc = HashMap::new();
        ["fault", "limit_switch_retract", "limit_switch_extend"]
            .iter()
            .for_each(|key| {
                ilc.insert(String::from(*key), HashSet::new());
            });

        Self {
            config_control_loop: config_control_loop.clone(),
            config_power: ConfigPower::new(),
            actuators: Actuator::from_cell_mapping_file(Path::new(
                "config/cell/cell_actuator_mapping.yaml",
            )),
            summary_faults_status: 0,
            _faults_mask: Self::get_faults_mask(),
            ilc: ilc,
            _count_out_max_cycle_time: 0,
        }
    }

    /// Get the faults mask that is the sum of all the error codes that begin
    /// with "Fault".
    ///
    /// # Returns
    /// The faults mask.
    fn get_faults_mask() -> u64 {
        ErrorCode::iter().fold(0, |acc, error_code| {
            if Self::is_fault_error_code(error_code) {
                acc + error_code.bit_value()
            } else {
                acc
            }
        })
    }

    /// Check if the error code is a fault error code. If not, it is a warning
    /// error code.
    ///
    /// # Arguments
    /// * `error_code` - Error code.
    ///
    /// # Returns
    /// True if the error code is a fault error code. Otherwise, False.
    fn is_fault_error_code(error_code: ErrorCode) -> bool {
        format!("{:?}", error_code).starts_with("Fault")
    }

    /// Clear the summary faults status.
    pub fn clear(&mut self) {
        self.summary_faults_status = 0;

        self.ilc.iter_mut().for_each(|(_, value)| {
            value.clear();
        });

        self._count_out_max_cycle_time = 0;
    }

    /// Update the enabled faults mask.
    ///
    /// # Arguments
    /// * `enabled_faults_mask` - Enabled faults mask.
    pub fn update_enabled_faults_mask(&mut self, enabled_faults_mask: u64) {
        self.config_control_loop.enabled_faults_mask = enabled_faults_mask;

        self.summary_faults_status &= self.config_control_loop.enabled_faults_mask;
    }

    /// Add a error to the summary faults status.
    ///
    /// # Notes
    /// Compare the error code with the enabled faults mask first. Only the
    /// enabled faults will be added to the summary faults status.
    ///
    /// # Arguments
    /// * `error_code` - Error code.
    pub fn add_error(&mut self, error_code: ErrorCode) {
        if self.has_error(error_code) {
            return;
        }

        let bit_value = error_code.bit_value();
        if self.config_control_loop.enabled_faults_mask & bit_value == 0 {
            return;
        }

        if Self::is_fault_error_code(error_code) {
            error!("Detected the fault: {:?}.", error_code);
        } else {
            warn!("Detected the warning: {:?}.", error_code);
        }

        self.summary_faults_status |= bit_value;
    }

    /// Check if there is a specific error.
    ///
    /// # Arguments
    /// * `error_code` - Error code.
    pub fn has_error(&self, error_code: ErrorCode) -> bool {
        (self.summary_faults_status & error_code.bit_value()) != 0
    }

    /// Clear a error from the summary faults status.
    ///
    /// # Arguments
    /// * `error_code` - Error code.
    pub fn clear_error(&mut self, error_code: ErrorCode) {
        if self.has_error(error_code) {
            self.summary_faults_status &= !error_code.bit_value();
        }
    }

    /// Check if there is any fault.
    ///
    /// # Returns
    /// True if there is any fault. Otherwise, False.
    pub fn has_fault(&self) -> bool {
        (self.summary_faults_status & self._faults_mask) != 0
    }

    /// Check the system condition of control loop.
    ///
    /// # Arguments
    /// * `telemetry` - Telemetry data.
    /// * `is_closed_loop` - Is under the closed-loop control or not.
    pub fn check_condition_control_loop(
        &mut self,
        telemetry: &TelemetryControlLoop,
        is_closed_loop: bool,
    ) {
        self.check_ilc_status(&telemetry.ilc_status);
        if self.ilc["fault"].len() > 0 {
            self.add_error(ErrorCode::FaultActuatorIlcRead);
        }

        if (self.ilc["limit_switch_retract"].len() > 0)
            || (self.ilc["limit_switch_extend"].len() > 0)
        {
            let error_code_limit_switch = if is_closed_loop {
                ErrorCode::FaultActuatorLimitCL
            } else {
                ErrorCode::WarnActuatorLimitOL
            };

            self.add_error(error_code_limit_switch);
        }

        if self.is_encoder_out_limit(&telemetry.ilc_encoders, true) {
            self.add_error(ErrorCode::FaultAxialActuatorEncoderRange);
        }
        if self.is_encoder_out_limit(&telemetry.ilc_encoders, false) {
            self.add_error(ErrorCode::FaultTangentActuatorEncoderRange);
        }

        if self.is_actuator_force_out_limit(&telemetry.forces["measured"], is_closed_loop) {
            self.add_error(ErrorCode::FaultExcessiveForce);
        }

        if self.is_tangent_force_error_out_limit(&telemetry.tangent_force_error) {
            self.add_error(ErrorCode::FaultTangentLoadCell);
        }

        if self.is_cell_temperature_high(
            &telemetry.temperature["intake"],
            &telemetry.temperature["exhaust"],
        ) {
            self.add_error(ErrorCode::WarnCellTemp);
        }

        if self.is_inclinometer_angle_diff_high(
            telemetry.inclinometer["processed"],
            telemetry.inclinometer["external"],
        ) {
            self.add_error(ErrorCode::FaultElevationAngleDiff);
        }

        self.check_cycle_time(telemetry.cycle_time);
    }

    /// Check the inner loop controller (ILC) status.
    ///
    /// # Arguments
    /// * `ilc_status` - The ILC status.
    fn check_ilc_status(&mut self, ilc_status: &Vec<u8>) {
        let mut fault: Vec<i32> = Vec::new();
        let mut limit_switch_retract: Vec<i32> = Vec::new();
        let mut limit_switch_extend: Vec<i32> = Vec::new();
        // The bit mask here is based on the LTS-346.
        for (idx, data) in ilc_status.iter().enumerate() {
            // Bit 0
            if *data & 0b0000_0001 != 0 {
                fault.push(idx as i32);
            }
            // Bit 2
            if *data & 0b0000_0100 != 0 {
                limit_switch_retract.push(idx as i32);
            }
            // Bit 3
            if *data & 0b0000_1000 != 0 {
                limit_switch_extend.push(idx as i32);
            }
        }

        if let Some(set_fault) = self.ilc.get_mut("fault") {
            set_fault.extend(fault);
        }
        if let Some(set_limit_switch_retract) = self.ilc.get_mut("limit_switch_retract") {
            set_limit_switch_retract.extend(limit_switch_retract);
        }
        if let Some(set_limit_switch_extend) = self.ilc.get_mut("limit_switch_extend") {
            set_limit_switch_extend.extend(limit_switch_extend);
        }
    }

    /// The encoder value is out of limit or not.
    ///
    /// # Arguments
    /// * `encoders`: The 78 encoder values to check.
    /// * `is_axial`: Is the encoder for axial actuator or not. If not, it
    /// is for tangent actuator.
    ///
    /// # Returns
    /// True if any of the encoders is out of limit. Otherwise, False.
    fn is_encoder_out_limit(&mut self, encoders: &Vec<i32>, is_axial: bool) -> bool {
        if is_axial {
            for (idx, encoder) in encoders[0..NUM_AXIAL_ACTUATOR].iter().enumerate() {
                if self.actuators[idx].is_encoder_out_limit(*encoder) {
                    return true;
                }
            }
        } else {
            for (idx, encoder) in encoders[NUM_AXIAL_ACTUATOR..].iter().enumerate() {
                if self.actuators[NUM_AXIAL_ACTUATOR + idx].is_encoder_out_limit(*encoder) {
                    return true;
                }
            }
        }

        false
    }

    /// The actuator force is out of limit or not.
    ///
    /// # Arguments:
    /// * `force`: 78 actuator force in Newton.
    /// * `is_closed_loop`: Is under the closed-loop control or not.
    ///
    /// # Returns
    /// True if the actuator force is out of limit. Otherwise, False.
    pub fn is_actuator_force_out_limit(&self, force: &Vec<f64>, is_closed_loop: bool) -> bool {
        // Decide the force limits
        let mut force_limit_axial: f64;
        let mut force_limit_tangent: f64;

        let force_limit = &self.config_control_loop.force_limit;
        if is_closed_loop {
            force_limit_axial = force_limit["limit_force_axial_closed_loop"];
            force_limit_tangent = force_limit["limit_force_tangent_closed_loop"];
        } else {
            force_limit_axial = force_limit["limit_force_axial_open_loop"];
            force_limit_tangent = force_limit["limit_force_tangent_open_loop"];
        }

        if self.config_control_loop.enable_open_loop_max_limit {
            force_limit_axial = force_limit["max_limit_force_axial_open_loop"];
            force_limit_tangent = force_limit["max_limit_force_tangent_open_loop"];
        }

        // Compare the force with the limit
        force[..NUM_AXIAL_ACTUATOR]
            .iter()
            .any(|x| x.abs() > force_limit_axial)
            || force[NUM_AXIAL_ACTUATOR..]
                .iter()
                .any(|x| x.abs() > force_limit_tangent)
    }

    /// The tangent link force error is out of limit or not.
    ///
    /// # Arguments
    /// * `tangent_force_error`: The force error of tangent link in Newton. The
    /// first 6 elements are the "force". The last 2 elements are the "weight"
    /// and "sum".
    ///
    /// # Returns
    /// True if the force error of tangent link is out of limit. Otherwise,
    /// False.
    fn is_tangent_force_error_out_limit(&self, tangent_force_error: &Vec<f64>) -> bool {
        let tangent_force_error_threshold = &self.config_control_loop.tangent_force_error_threshold;

        let is_out_limit_total_weight = tangent_force_error[6].abs()
            >= tangent_force_error_threshold["tangent_link_total_weight_error"];

        let is_out_limit_theta_z = tangent_force_error[7].abs()
            >= tangent_force_error_threshold["tangent_link_theta_z_moment"];

        // Tangent link: A1, A4
        let is_out_limit_non_load = [0, 3].iter().any(|idx| {
            tangent_force_error[*idx].abs()
                >= tangent_force_error_threshold["tangent_link_non_load_bearing_link"]
        });

        // Tangent link: A2, A3, A5, A6
        let is_out_limit_load = [1, 2, 4, 5].iter().any(|idx| {
            tangent_force_error[*idx].abs()
                >= tangent_force_error_threshold["tangent_link_load_bearing_link"]
        });

        is_out_limit_total_weight
            || is_out_limit_theta_z
            || is_out_limit_non_load
            || is_out_limit_load
    }

    /// The cell temperature is high or not.
    ///
    /// # Arguments
    /// * `intake`: The intake temperature of the cell in degree Celsius.
    /// * `exhaust`: The exhaust temperature of the cell in degree Celsius.
    ///
    /// # Returns
    /// True if the cell temperature is high. Otherwise, False.
    fn is_cell_temperature_high(&self, intake: &Vec<f64>, exhaust: &Vec<f64>) -> bool {
        let diff = (exhaust[0] - intake[0] + exhaust[1] - intake[1]) / 2.0;
        diff > self.config_control_loop.max_cell_temperature_difference
    }

    /// The inclinometer angle difference is high or not.
    ///
    /// # Arguments
    /// * `processed`: The processed inclinometer angle in degree.
    /// * `external`: The external inclinometer angle in degree.
    ///
    /// # Returns
    /// True if the inclinometer angle difference is high. Otherwise, False.
    fn is_inclinometer_angle_diff_high(&self, processed: f64, external: f64) -> bool {
        if self.config_control_loop.enable_angle_comparison {
            (processed - external).abs() > self.config_control_loop.max_angle_difference
        } else {
            false
        }
    }

    /// Check the cycle time.
    ///
    /// # Arguments
    /// * `cycle_time` - The cycle time in second.
    fn check_cycle_time(&mut self, cycle_time: f64) {
        if cycle_time > (1.0 / self.config_control_loop.control_frequency) {
            self._count_out_max_cycle_time += 1;
        } else {
            self._count_out_max_cycle_time = 0;
        }

        if self._count_out_max_cycle_time == 0 {
            self.clear_error(ErrorCode::WarnCrioTiming);
        } else if self._count_out_max_cycle_time >= self.config_control_loop.max_out_cycle_time {
            self.add_error(ErrorCode::FaultCrioTiming);
        } else {
            self.add_error(ErrorCode::WarnCrioTiming);
        }
    }

    /// Check the system condition of power system.
    ///
    /// # Arguments
    /// * `telemetry` - Telemetry data.
    /// * `is_power_on_communication` - Is the communication power on or not.
    /// * `is_power_on_motor` - Is the motor power on or not.
    pub fn check_condition_power_system(
        &mut self,
        telemetry: &TelemetryPower,
        is_power_on_communication: bool,
        is_power_on_motor: bool,
    ) {
        if is_power_on_communication {
            self.check_voltage(
                telemetry.power_processed["commVoltage"],
                ErrorCode::FaultCommVoltage,
                ErrorCode::WarnCommVoltage,
            );

            if telemetry.power_processed["commCurrent"]
                > self.config_power.excessive_current_communication
            {
                self.add_error(ErrorCode::FaultCommOverCurrent);
            }
        }

        if is_power_on_motor {
            self.check_voltage(
                telemetry.power_processed["motorVoltage"],
                ErrorCode::FaultMotorVoltage,
                ErrorCode::WarnMotorVoltage,
            );

            if telemetry.power_processed["motorCurrent"] > self.config_power.excessive_current_motor
            {
                self.add_error(ErrorCode::FaultMotorOverCurrent);
            }
        }
    }

    /// Check the voltage.
    ///
    /// # Arguments
    /// * `voltage` - The voltage in volt.
    /// * `error_code_fault` - The error code for the fault level.
    /// * `error_code_warning` - The error code for the warning level.
    fn check_voltage(
        &mut self,
        voltage: f64,
        error_code_fault: ErrorCode,
        error_code_warning: ErrorCode,
    ) {
        // Check the fault level.
        if self.is_out_range(
            voltage,
            self.config_power.fault_voltage_min,
            self.config_power.fault_voltage_max,
        ) {
            self.add_error(error_code_fault);
        }

        // Only check the warning level if there is no fault.
        if !self.has_error(error_code_fault) {
            if self.is_out_range(
                voltage,
                self.config_power.warning_voltage_min,
                self.config_power.warning_voltage_max,
            ) {
                self.add_error(error_code_warning);
            }
        }
    }

    /// Check if the value is out of the range.
    ///
    /// # Arguments
    /// * `value` - The value to check.
    /// * `min` - The minimum value.
    /// * `max` - The maximum value.
    ///
    /// # Returns
    /// True if the value is out of the range. Otherwise, False.
    fn is_out_range(&self, value: f64, min: f64, max: f64) -> bool {
        (value < min) || (value > max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    use crate::constants::NUM_ACTUATOR;

    fn create_error_handler() -> ErrorHandler {
        let config = Config::new(
            Path::new("config/parameters_control.yaml"),
            Path::new("config/lut/handling"),
        );
        ErrorHandler::new(&config)
    }

    #[test]
    fn test_get_faults_mask() {
        assert_eq!(ErrorHandler::get_faults_mask(), 0xAB8000030CD7EAD8);
    }

    #[test]
    fn test_is_fault_error_code() {
        assert!(ErrorHandler::is_fault_error_code(
            ErrorCode::FaultActuatorIlcRead
        ));
        assert!(!ErrorHandler::is_fault_error_code(
            ErrorCode::IgnoreWarnStaleData
        ));
    }

    #[test]
    fn test_clear() {
        let mut error_handler = create_error_handler();
        error_handler.summary_faults_status = 0b1010;
        error_handler.ilc.get_mut("fault").unwrap().insert(1);

        error_handler._count_out_max_cycle_time = 1;

        error_handler.clear();

        assert_eq!(error_handler.summary_faults_status, 0);
        assert_eq!(error_handler.ilc["fault"].len(), 0);
        assert_eq!(error_handler._count_out_max_cycle_time, 0);
    }

    #[test]
    fn test_update_enabled_faults_mask() {
        let mut error_handler = create_error_handler();
        error_handler.summary_faults_status = 0b1010;

        error_handler.update_enabled_faults_mask(0b1100);

        assert_eq!(
            error_handler.config_control_loop.enabled_faults_mask,
            0b1100
        );
        assert_eq!(error_handler.summary_faults_status, 0b1000);
    }

    #[test]
    fn test_add_error() {
        let mut error_handler = create_error_handler();
        error_handler.update_enabled_faults_mask(0b110);

        error_handler.add_error(ErrorCode::IgnoreWarnStaleData);
        assert_eq!(error_handler.summary_faults_status, 0);

        error_handler.add_error(ErrorCode::IgnoreFaultStaleData);
        assert_eq!(error_handler.summary_faults_status, 0b10);

        error_handler.add_error(ErrorCode::IgnoreWarnBroadcast);
        assert_eq!(error_handler.summary_faults_status, 0b110);
    }

    #[test]
    fn test_clear_error() {
        let mut error_handler = create_error_handler();
        error_handler.summary_faults_status = 0b10;

        error_handler.clear_error(ErrorCode::IgnoreWarnStaleData);
        assert_eq!(error_handler.summary_faults_status, 0b10);

        error_handler.clear_error(ErrorCode::IgnoreFaultStaleData);
        assert_eq!(error_handler.summary_faults_status, 0b0);
    }

    #[test]
    fn test_has_error() {
        let mut error_handler = create_error_handler();
        error_handler.summary_faults_status = 0b10;

        assert!(!error_handler.has_error(ErrorCode::IgnoreWarnStaleData));
        assert!(error_handler.has_error(ErrorCode::IgnoreFaultStaleData));
    }

    #[test]
    fn test_has_fault() {
        let mut error_handler = create_error_handler();
        error_handler.update_enabled_faults_mask(0b1110);

        error_handler.add_error(ErrorCode::WarnActuatorLimitOL);
        assert!(!error_handler.has_fault());

        error_handler.add_error(ErrorCode::FaultActuatorIlcRead);
        assert!(error_handler.has_fault());
    }

    #[test]
    fn test_check_condition_control_loop() {
        let mut error_handler = create_error_handler();

        let mut telemetry = TelemetryControlLoop::new();
        if let Some(value) = telemetry.forces.get_mut("measured") {
            value[1] = 1000.0;
        }
        telemetry.ilc_status[0..3].copy_from_slice(&[0b0000_0100, 0b0000_1001, 0b0000_1101]);

        telemetry.ilc_encoders[0] = 100000000;
        telemetry.ilc_encoders[NUM_AXIAL_ACTUATOR] = 100000000;

        error_handler.check_condition_control_loop(&telemetry, true);

        let expected_faults_status = [
            ErrorCode::FaultActuatorIlcRead,
            ErrorCode::FaultActuatorLimitCL,
            ErrorCode::FaultExcessiveForce,
            ErrorCode::FaultAxialActuatorEncoderRange,
            ErrorCode::FaultTangentActuatorEncoderRange,
        ]
        .iter()
        .fold(0, |acc, error_code| acc + error_code.bit_value());

        assert_eq!(error_handler.summary_faults_status, expected_faults_status);
    }

    #[test]
    fn test_check_ilc_status() {
        let mut error_handler = create_error_handler();
        let ilc_status = vec![0b0000_0100, 0b0000_1001, 0b0000_1101];

        error_handler.check_ilc_status(&ilc_status);

        // Make sure the elements in set should be unique.
        error_handler.check_ilc_status(&ilc_status);

        assert_eq!(error_handler.ilc["fault"], HashSet::from([1, 2]));
        assert_eq!(
            error_handler.ilc["limit_switch_retract"],
            HashSet::from([0, 2])
        );
        assert_eq!(
            error_handler.ilc["limit_switch_extend"],
            HashSet::from([1, 2])
        );
    }

    #[test]
    fn test_is_encoder_out_limit() {
        let mut error_handler = create_error_handler();

        // In limit
        let mut encoders = vec![0; NUM_ACTUATOR];

        assert!(!error_handler.is_encoder_out_limit(&encoders, true));
        assert!(!error_handler.is_encoder_out_limit(&encoders, false));

        // Out of limit
        // Test the axial actuator only
        encoders[0] = 100000000;

        assert!(error_handler.is_encoder_out_limit(&encoders, true));
        assert!(!error_handler.is_encoder_out_limit(&encoders, false));

        // Test the axial and tangent actuators
        encoders[NUM_AXIAL_ACTUATOR] = 100000000;

        assert!(error_handler.is_encoder_out_limit(&encoders, true));
        assert!(error_handler.is_encoder_out_limit(&encoders, false));

        // Test the tangent actuator only
        encoders[0] = 0;

        assert!(!error_handler.is_encoder_out_limit(&encoders, true));
        assert!(error_handler.is_encoder_out_limit(&encoders, false));
    }

    #[test]
    fn test_is_actuator_force_out_limit_axial() {
        let mut error_handler = create_error_handler();
        let mut force = vec![0.0; NUM_ACTUATOR];

        // Test the closed-loop limit
        force[2] = 460.0;

        assert!(error_handler.is_actuator_force_out_limit(&force, true));
        assert!(!error_handler.is_actuator_force_out_limit(&force, false));

        // Test the open-loop limit
        force[2] = 500.0;

        assert!(error_handler.is_actuator_force_out_limit(&force, false));

        error_handler.config_control_loop.enable_open_loop_max_limit = true;
        assert!(!error_handler.is_actuator_force_out_limit(&force, false));

        // Test the open-loop maximum limit
        force[2] = 650.0;

        assert!(error_handler.is_actuator_force_out_limit(&force, false));
    }

    #[test]
    fn test_is_actuator_force_out_limit_tangent() {
        let mut error_handler = create_error_handler();
        let mut force = vec![0.0; NUM_ACTUATOR];

        // Test the closed-loop limit
        force[NUM_AXIAL_ACTUATOR + 2] = 5900.0;

        assert!(error_handler.is_actuator_force_out_limit(&force, true));
        assert!(!error_handler.is_actuator_force_out_limit(&force, false));

        // Test the open-loop limit
        force[NUM_AXIAL_ACTUATOR + 2] = 6100.0;

        assert!(error_handler.is_actuator_force_out_limit(&force, false));

        error_handler.config_control_loop.enable_open_loop_max_limit = true;
        assert!(!error_handler.is_actuator_force_out_limit(&force, false));

        // Test the open-loop maximum limit
        force[NUM_AXIAL_ACTUATOR + 2] = 6300.0;

        assert!(error_handler.is_actuator_force_out_limit(&force, false));
    }

    #[test]
    fn test_is_tangent_force_error_out_limit() {
        let error_handler = create_error_handler();
        let mut tangent_force_error = vec![0.0; 8];

        // Test the total weight error
        tangent_force_error[6] = -2000.0;

        assert!(error_handler.is_tangent_force_error_out_limit(&tangent_force_error));

        // Test the theta_z moment error
        tangent_force_error[6] = 0.0;
        tangent_force_error[7] = -1000.0;

        assert!(error_handler.is_tangent_force_error_out_limit(&tangent_force_error));

        // Test the non-load bearing link error
        tangent_force_error[7] = 0.0;
        tangent_force_error[0] = -2000.0;

        assert!(error_handler.is_tangent_force_error_out_limit(&tangent_force_error));

        // Test the load bearing link error
        tangent_force_error[0] = 0.0;
        tangent_force_error[1] = -2000.0;

        assert!(error_handler.is_tangent_force_error_out_limit(&tangent_force_error));
    }

    #[test]
    fn test_is_cell_temperature_high() {
        let error_handler = create_error_handler();

        assert!(!error_handler.is_cell_temperature_high(&vec![0.0, 0.0], &vec![0.0, 0.0]));
        assert!(error_handler.is_cell_temperature_high(&vec![0.0, 0.0], &vec![10.0, 10.0]));
    }

    #[test]
    fn test_is_inclinometer_angle_diff_high() {
        let mut error_handler = create_error_handler();
        error_handler.config_control_loop.enable_angle_comparison = false;

        assert!(!error_handler.is_inclinometer_angle_diff_high(0.0, 1.0));

        error_handler.config_control_loop.enable_angle_comparison = true;

        let max_angle_difference = error_handler.config_control_loop.max_angle_difference;
        assert!(!error_handler.is_inclinometer_angle_diff_high(0.0, max_angle_difference - 1.0));
        assert!(error_handler.is_inclinometer_angle_diff_high(0.0, max_angle_difference + 1.0));
    }

    #[test]
    fn test_check_cycle_time() {
        let mut error_handler = create_error_handler();

        // Normal cycle time
        error_handler.check_cycle_time(0.001);
        assert_eq!(error_handler._count_out_max_cycle_time, 0);

        // Has the warning error
        let max_out_cycle_time = error_handler.config_control_loop.max_out_cycle_time;

        for _ in 0..(max_out_cycle_time - 1) {
            error_handler.check_cycle_time(1.0);
        }

        assert!(error_handler.has_error(ErrorCode::WarnCrioTiming));
        assert!(!error_handler.has_error(ErrorCode::FaultCrioTiming));

        // Has the fault error
        error_handler.check_cycle_time(1.0);

        assert!(error_handler.has_error(ErrorCode::WarnCrioTiming));
        assert!(error_handler.has_error(ErrorCode::FaultCrioTiming));

        // Clear the warning error but the fault error still exists
        error_handler.check_cycle_time(0.001);

        assert!(!error_handler.has_error(ErrorCode::WarnCrioTiming));
        assert!(error_handler.has_error(ErrorCode::FaultCrioTiming));

        assert_eq!(error_handler._count_out_max_cycle_time, 0);
    }

    #[test]
    fn test_check_condition_power_system() {
        let mut error_handler = create_error_handler();

        // Normal voltage and current
        let mut telemetry = TelemetryPower::new();
        telemetry
            .power_processed
            .insert(String::from("commVoltage"), 24.1);
        telemetry
            .power_processed
            .insert(String::from("motorVoltage"), 23.9);
        telemetry
            .power_processed
            .insert(String::from("commCurrent"), 6.0);
        telemetry
            .power_processed
            .insert(String::from("motorCurrent"), 10.0);

        error_handler.check_condition_power_system(&telemetry, true, true);

        assert!(!error_handler.has_fault());

        // Abnormal voltage and current
        telemetry
            .power_processed
            .insert(String::from("commVoltage"), 20.0);
        telemetry
            .power_processed
            .insert(String::from("motorVoltage"), 26.0);
        telemetry
            .power_processed
            .insert(String::from("commCurrent"), 11.0);
        telemetry
            .power_processed
            .insert(String::from("motorCurrent"), 21.0);

        error_handler.check_condition_power_system(&telemetry, true, true);

        assert!(error_handler.has_error(ErrorCode::FaultCommVoltage));
        assert!(error_handler.has_error(ErrorCode::WarnMotorVoltage));
        assert!(error_handler.has_error(ErrorCode::FaultCommOverCurrent));
        assert!(error_handler.has_error(ErrorCode::FaultMotorOverCurrent));

        // No fault if the power is off
        error_handler.clear();

        error_handler.check_condition_power_system(&telemetry, false, false);

        assert!(!error_handler.has_fault());
    }

    #[test]
    fn test_check_voltage() {
        let mut error_handler = create_error_handler();

        // No error
        error_handler.check_voltage(
            24.0,
            ErrorCode::FaultCommVoltage,
            ErrorCode::WarnCommVoltage,
        );

        assert!(!error_handler.has_fault());

        // Test the fault level
        error_handler.check_voltage(
            20.0,
            ErrorCode::FaultCommVoltage,
            ErrorCode::WarnCommVoltage,
        );

        assert!(error_handler.has_error(ErrorCode::FaultCommVoltage));
        assert!(!error_handler.has_error(ErrorCode::WarnCommVoltage));

        // Test the warning level
        error_handler.clear();

        error_handler.check_voltage(
            26.0,
            ErrorCode::FaultCommVoltage,
            ErrorCode::WarnCommVoltage,
        );

        assert!(!error_handler.has_error(ErrorCode::FaultCommVoltage));
        assert!(error_handler.has_error(ErrorCode::WarnCommVoltage));
    }

    #[test]
    fn test_is_out_range() {
        let error_handler = create_error_handler();

        assert!(error_handler.is_out_range(0.5, 1.0, 2.0));
        assert!(error_handler.is_out_range(2.5, 1.0, 2.0));

        assert!(!error_handler.is_out_range(1.5, 1.0, 2.0));
    }
}
