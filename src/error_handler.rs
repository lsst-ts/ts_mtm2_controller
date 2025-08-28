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

use log::{error, info, warn};
use std::collections::{HashMap, HashSet};
use std::path::Path;
use strum::IntoEnumIterator;

use crate::config::Config;
use crate::constants::NUM_AXIAL_ACTUATOR;
use crate::control::actuator::Actuator;
use crate::enums::{BitEnum, DigitalInput, DigitalOutput, ErrorCode, PowerType};
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
    // Counts to check the power voltages.
    _count_voltage_communication: i32,
    _count_voltage_motor: i32,
    _max_count_voltage_communication: i32,
    _max_count_voltage_motor: i32,
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

        // Calculate the maximum counts to check the voltages.
        let config_power = ConfigPower::new();
        let max_count_voltage_communication = config_power
            .get_time_power_on(PowerType::Communication)
            / (config_power.loop_time as i32);
        let max_count_voltage_motor =
            config_power.get_time_power_on(PowerType::Motor) / (config_power.loop_time as i32);

        Self {
            config_control_loop: config_control_loop.clone(),
            config_power: config_power,
            actuators: Actuator::from_cell_mapping_file(Path::new(
                "config/cell/cell_actuator_mapping.yaml",
            )),
            summary_faults_status: 0,
            _faults_mask: Self::get_faults_mask(),
            ilc: ilc,
            _count_out_max_cycle_time: 0,

            _count_voltage_communication: 0,
            _count_voltage_motor: 0,

            _max_count_voltage_communication: max_count_voltage_communication,

            _max_count_voltage_motor: max_count_voltage_motor,
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

        info!(
            "Update the enabled faults mask to {:#X}.",
            enabled_faults_mask
        );

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
        // Check the digital input and output
        let digital_input = telemetry.digital_input;
        let digital_output = telemetry.digital_output;

        let (has_fault_power_supply_load_share, has_fault_power_health, has_fault_interlock) =
            Self::check_power_supply_health(
                self.config_power.is_boost_current_fault_enabled,
                digital_input,
                digital_output,
            );
        if has_fault_power_supply_load_share {
            self.add_error(ErrorCode::FaultPowerSupplyLoadShare);
        }
        if has_fault_power_health {
            self.add_error(ErrorCode::FaultPowerHealth);
        }

        // Check the communication voltage.
        if is_power_on_communication {
            if self._count_voltage_communication >= self._max_count_voltage_communication {
                let voltage = telemetry.power_processed["commVoltage"];
                self.check_breaker_operation_voltage(PowerType::Communication, voltage);
                self.check_voltage_in_range(
                    voltage,
                    ErrorCode::FaultCommVoltage,
                    ErrorCode::WarnCommVoltage,
                );

                if telemetry.power_processed["commCurrent"]
                    > self.config_power.excessive_current_communication
                {
                    self.add_error(ErrorCode::FaultCommOverCurrent);
                }

                self.check_breakers(PowerType::Communication, digital_input);
            } else {
                self._count_voltage_communication += 1;
            }
        } else {
            self._count_voltage_communication = 0;
        }

        // Check the motor voltage.
        if is_power_on_motor {
            if self._count_voltage_motor >= self._max_count_voltage_motor {
                let voltage = telemetry.power_processed["motorVoltage"];
                self.check_breaker_operation_voltage(PowerType::Motor, voltage);
                self.check_voltage_in_range(
                    voltage,
                    ErrorCode::FaultMotorVoltage,
                    ErrorCode::WarnMotorVoltage,
                );

                if telemetry.power_processed["motorCurrent"]
                    > self.config_power.excessive_current_motor
                {
                    self.add_error(ErrorCode::FaultMotorOverCurrent);
                }

                self.check_breakers(PowerType::Motor, digital_input);

                if has_fault_interlock {
                    self.add_error(ErrorCode::IgnoreFaultInterlock);
                }
            } else {
                self._count_voltage_motor += 1;
            }
        } else {
            self._count_voltage_motor = 0;
        }
    }

    /// Check the power supply health.
    ///
    /// # Arguments
    /// * `is_boost_current_fault_enabled` - Is the boost current fault enabled
    /// or not.
    /// * `digital_input` - The digital input value.
    /// * `digital_output` - The digital output value.
    ///
    /// # Returns
    /// A tuple containing three boolean values:
    /// * The first value indicates if there is a fault in power supply load
    /// sharing.
    /// * The second value indicates if there is a fault in power health.
    /// * The third value indicates if there is a fault in interlock.
    pub fn check_power_supply_health(
        is_boost_current_fault_enabled: bool,
        digital_input: u32,
        digital_output: u8,
    ) -> (bool, bool, bool) {
        let mut has_fault_power_supply_load_share = false;
        let mut has_fault_power_health = false;
        let mut has_fault_interlock = false;

        // Active high inputs
        let is_redundancy_ok = (digital_input & DigitalInput::RedundancyOK.bit_value()) != 0;
        let is_load_distribution_ok =
            (digital_input & DigitalInput::LoadDistributionOK.bit_value()) != 0;
        let is_dc_ok_1 = (digital_input & DigitalInput::PowerSupplyDC1OK.bit_value()) != 0;
        let is_dc_ok_2 = (digital_input & DigitalInput::PowerSupplyDC2OK.bit_value()) != 0;

        let is_motor_relay_output_on =
            (digital_output & DigitalOutput::MotorPower.bit_value()) != 0;
        let is_interlock_enable =
            (digital_output & DigitalOutput::InterlockEnable.bit_value()) != 0;

        // Active low inputs
        let is_boost_current_on_1 =
            (digital_input & DigitalInput::PowerSupplyCurrent1OK.bit_value()) == 0;
        let is_boost_current_on_2 =
            (digital_input & DigitalInput::PowerSupplyCurrent2OK.bit_value()) == 0;

        let is_interlock_power_relay_on =
            (digital_input & DigitalInput::InterlockPowerRelay.bit_value()) == 0;

        if !(is_redundancy_ok && is_load_distribution_ok) {
            has_fault_power_supply_load_share = true;
        }

        if (!(is_dc_ok_1 && is_dc_ok_2))
            || (is_boost_current_fault_enabled && (is_boost_current_on_1 || is_boost_current_on_2))
        {
            has_fault_power_health = true;
        }

        if is_motor_relay_output_on && is_interlock_enable && (!is_interlock_power_relay_on) {
            has_fault_interlock = true;
        }

        (
            has_fault_power_supply_load_share,
            has_fault_power_health,
            has_fault_interlock,
        )
    }

    /// Check the voltage is in the range or not.
    ///
    /// # Arguments
    /// * `voltage` - The voltage in volt.
    /// * `error_code_fault` - The error code for the fault level.
    /// * `error_code_warning` - The error code for the warning level.
    fn check_voltage_in_range(
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

    /// Check the breaker operation voltage.
    ///
    /// # Arguments
    /// * `power_type` - The type of power system.
    /// * `voltage` - The voltage to check.
    pub fn check_breaker_operation_voltage(&mut self, power_type: PowerType, voltage: f64) {
        if voltage < self.config_power.breaker_operating_voltage {
            self.add_error(ErrorCode::IgnoreFaultHardware);

            match power_type {
                PowerType::Motor => {
                    self.add_error(ErrorCode::FaultMotorVoltage);
                }
                PowerType::Communication => {
                    self.add_error(ErrorCode::FaultCommVoltage);
                }
            }
        }
    }

    /// Check the breakers.
    ///
    /// # Arguments
    /// * `power_type` - Type of the power system.
    /// * `digital_input` - Digital input value to check the breaker status.
    fn check_breakers(&mut self, power_type: PowerType, digital_input: u32) {
        let (has_warning_1, has_fault_1, has_warning_2, has_fault_2, has_warning_3, has_fault_3);

        match power_type {
            PowerType::Motor => {
                (has_warning_1, has_fault_1) = self.check_breaker_line(
                    digital_input & DigitalInput::J1W9N1MotorPowerBreaker.bit_value() == 0,
                    digital_input & DigitalInput::J1W9N2MotorPowerBreaker.bit_value() == 0,
                    digital_input & DigitalInput::J1W9N3MotorPowerBreaker.bit_value() == 0,
                );
                (has_warning_2, has_fault_2) = self.check_breaker_line(
                    digital_input & DigitalInput::J2W10N1MotorPowerBreaker.bit_value() == 0,
                    digital_input & DigitalInput::J2W10N2MotorPowerBreaker.bit_value() == 0,
                    digital_input & DigitalInput::J2W10N3MotorPowerBreaker.bit_value() == 0,
                );
                (has_warning_3, has_fault_3) = self.check_breaker_line(
                    digital_input & DigitalInput::J3W11N1MotorPowerBreaker.bit_value() == 0,
                    digital_input & DigitalInput::J3W11N2MotorPowerBreaker.bit_value() == 0,
                    digital_input & DigitalInput::J3W11N3MotorPowerBreaker.bit_value() == 0,
                );
            }
            PowerType::Communication => {
                (has_warning_1, has_fault_1) = self.check_breaker_line(
                    digital_input & DigitalInput::J1W12N1CommunicationPowerBreaker.bit_value() == 0,
                    digital_input & DigitalInput::J1W12N2CommunicationPowerBreaker.bit_value() == 0,
                    true,
                );
                (has_warning_2, has_fault_2) = self.check_breaker_line(
                    digital_input & DigitalInput::J2W13N1CommunicationPowerBreaker.bit_value() == 0,
                    digital_input & DigitalInput::J2W13N2CommunicationPowerBreaker.bit_value() == 0,
                    true,
                );
                (has_warning_3, has_fault_3) = self.check_breaker_line(
                    digital_input & DigitalInput::J3W14N1CommunicationPowerBreaker.bit_value() == 0,
                    digital_input & DigitalInput::J3W14N2CommunicationPowerBreaker.bit_value() == 0,
                    true,
                );
            }
        }

        // For the warnings and faults, their status is based on any one of them
        // having a fault or a warning. It is possible that we could have
        // warnings and faults at the same time.
        if has_warning_1 || has_warning_2 || has_warning_3 {
            self.add_error(ErrorCode::WarnSingleBreakerTrip);
        }

        if has_fault_1 || has_fault_2 || has_fault_3 {
            match power_type {
                PowerType::Motor => self.add_error(ErrorCode::FaultMotorMultiBreaker),
                PowerType::Communication => self.add_error(ErrorCode::FaultCommMultiBreaker),
            }
        }
    }

    /// Check the breaker line.
    ///
    /// # Arguments
    /// * `is_closed_1` - The first breaker is closed or not.
    /// * `is_closed_2` - The second breaker is closed or not.
    /// * `is_closed_3` - The third breaker is closed or not.
    ///
    /// # Returns
    /// A tuple containing the status of the three breakers. The first element
    /// is true if there is one open breaker (Warning), and the second element
    /// is true if there is more than one open breaker (Fault).
    fn check_breaker_line(
        &self,
        is_closed_1: bool,
        is_closed_2: bool,
        is_closed_3: bool,
    ) -> (bool, bool) {
        let status = (is_closed_1 as i32) + (is_closed_2 as i32) + (is_closed_3 as i32);

        match status {
            3 => (false, false),
            2 => (true, false),
            _ => (false, true),
        }
    }

    /// Check the power relay open fault when shut down.
    ///
    /// # Arguments
    /// * `power_type` - Type of the power system.
    /// * `voltage` - The voltage to check.
    pub fn check_power_relay_open_fault_when_shut_down(
        &mut self,
        power_type: PowerType,
        voltage: f64,
    ) {
        if voltage >= self.config_power.output_voltage_off_level {
            self.add_error(ErrorCode::FaultPowerRelayOpen);

            match power_type {
                PowerType::Motor => {
                    self.add_error(ErrorCode::IgnoreWarnMotorRelay);
                }
                PowerType::Communication => {
                    self.add_error(ErrorCode::IgnoreWarnCommRelay);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    use crate::constants::NUM_ACTUATOR;
    use crate::mock::mock_constants::{
        TEST_DIGITAL_INPUT_NO_POWER, TEST_DIGITAL_INPUT_POWER_COMM_MOTOR,
        TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
    };

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
        telemetry.digital_input = TEST_DIGITAL_INPUT_POWER_COMM_MOTOR;

        // Need to run some cycles to pass the maximum counts
        for _ in 0..(error_handler._max_count_voltage_communication
            + error_handler._max_count_voltage_motor)
        {
            error_handler.check_condition_power_system(&telemetry, true, true);
        }

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

        // Abnormal digital input
        telemetry.digital_input = TEST_DIGITAL_INPUT_NO_POWER;

        error_handler.check_condition_power_system(&telemetry, true, true);

        assert!(error_handler.has_error(ErrorCode::FaultCommMultiBreaker));
        assert!(error_handler.has_error(ErrorCode::FaultMotorMultiBreaker));

        // No fault if the power is off
        error_handler.clear();

        error_handler.check_condition_power_system(&telemetry, false, false);

        assert!(!error_handler.has_fault());

        // Existed fault even the power is off
        telemetry.digital_input =
            TEST_DIGITAL_INPUT_NO_POWER - DigitalInput::RedundancyOK.bit_value();

        error_handler.check_condition_power_system(&telemetry, true, true);

        assert!(error_handler.has_error(ErrorCode::FaultPowerSupplyLoadShare));
    }

    #[test]
    fn test_check_power_supply_health() {
        // No fault
        assert_eq!(
            ErrorHandler::check_power_supply_health(
                false,
                TEST_DIGITAL_INPUT_POWER_COMM_MOTOR,
                TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
            ),
            (false, false, false),
        );

        // Has fault
        assert_eq!(
            ErrorHandler::check_power_supply_health(
                false,
                TEST_DIGITAL_INPUT_NO_POWER
                    - DigitalInput::RedundancyOK.bit_value()
                    - DigitalInput::PowerSupplyDC1OK.bit_value(),
                TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
            ),
            (true, true, true),
        );
    }

    #[test]
    fn test_check_voltage_in_range() {
        let mut error_handler = create_error_handler();

        // No error
        error_handler.check_voltage_in_range(
            24.0,
            ErrorCode::FaultCommVoltage,
            ErrorCode::WarnCommVoltage,
        );

        assert!(!error_handler.has_fault());

        // Test the fault level
        error_handler.check_voltage_in_range(
            20.0,
            ErrorCode::FaultCommVoltage,
            ErrorCode::WarnCommVoltage,
        );

        assert!(error_handler.has_error(ErrorCode::FaultCommVoltage));
        assert!(!error_handler.has_error(ErrorCode::WarnCommVoltage));

        // Test the warning level
        error_handler.clear();

        error_handler.check_voltage_in_range(
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

    #[test]
    fn test_check_breaker_operation_voltage() {
        let mut error_handler = create_error_handler();

        // No fault
        let operating_voltage = error_handler.config_power.breaker_operating_voltage;

        error_handler.check_breaker_operation_voltage(PowerType::Motor, operating_voltage);
        assert!(!error_handler.has_fault());

        error_handler.check_breaker_operation_voltage(PowerType::Communication, operating_voltage);
        assert!(!error_handler.has_fault());

        // Has fault
        error_handler.check_breaker_operation_voltage(PowerType::Motor, operating_voltage - 1.0);
        assert!(error_handler.has_error(ErrorCode::IgnoreFaultHardware));
        assert!(error_handler.has_error(ErrorCode::FaultMotorVoltage));

        error_handler.clear();

        error_handler
            .check_breaker_operation_voltage(PowerType::Communication, operating_voltage - 1.0);
        assert!(error_handler.has_error(ErrorCode::IgnoreFaultHardware));
        assert!(error_handler.has_error(ErrorCode::FaultCommVoltage));
    }

    #[test]
    fn test_check_breakers() {
        let mut error_handler = create_error_handler();

        // No warning or fault
        error_handler.check_breakers(PowerType::Motor, TEST_DIGITAL_INPUT_POWER_COMM_MOTOR);
        assert!(!error_handler.has_fault());

        error_handler.check_breakers(
            PowerType::Communication,
            TEST_DIGITAL_INPUT_POWER_COMM_MOTOR,
        );
        assert!(!error_handler.has_fault());

        // Has warning
        error_handler.check_breakers(
            PowerType::Motor,
            TEST_DIGITAL_INPUT_POWER_COMM_MOTOR
                + DigitalInput::J3W11N3MotorPowerBreaker.bit_value(),
        );
        assert!(error_handler.has_error(ErrorCode::WarnSingleBreakerTrip));

        error_handler.clear();

        error_handler.check_breakers(
            PowerType::Communication,
            TEST_DIGITAL_INPUT_POWER_COMM_MOTOR
                + DigitalInput::J2W13N1CommunicationPowerBreaker.bit_value(),
        );
        assert!(error_handler.has_error(ErrorCode::WarnSingleBreakerTrip));

        // Has fault
        error_handler.check_breakers(PowerType::Motor, TEST_DIGITAL_INPUT_NO_POWER);
        assert!(error_handler.has_error(ErrorCode::FaultMotorMultiBreaker));

        error_handler.check_breakers(PowerType::Communication, TEST_DIGITAL_INPUT_NO_POWER);
        assert!(error_handler.has_error(ErrorCode::FaultCommMultiBreaker));
    }

    #[test]
    fn test_check_breaker_line() {
        let error_handler = create_error_handler();

        assert_eq!(
            (false, false),
            error_handler.check_breaker_line(true, true, true)
        );

        assert_eq!(
            (true, false),
            error_handler.check_breaker_line(true, true, false)
        );

        assert_eq!(
            (false, true),
            error_handler.check_breaker_line(true, false, false)
        );

        assert_eq!(
            (false, true),
            error_handler.check_breaker_line(false, false, false)
        );
    }

    #[test]
    fn test_check_power_relay_open_fault_when_shut_down() {
        let mut error_handler = create_error_handler();

        // No fault
        error_handler.check_power_relay_open_fault_when_shut_down(
            PowerType::Motor,
            error_handler.config_power.output_voltage_off_level - 1.0,
        );
        assert!(!error_handler.has_fault());

        error_handler.check_power_relay_open_fault_when_shut_down(
            PowerType::Communication,
            error_handler.config_power.output_voltage_off_level - 1.0,
        );
        assert!(!error_handler.has_fault());

        // Has fault
        error_handler.check_power_relay_open_fault_when_shut_down(
            PowerType::Motor,
            error_handler.config_power.output_voltage_off_level,
        );
        assert!(error_handler.has_error(ErrorCode::FaultPowerRelayOpen));
        assert!(error_handler.has_error(ErrorCode::IgnoreWarnMotorRelay));

        error_handler.clear();

        error_handler.check_power_relay_open_fault_when_shut_down(
            PowerType::Communication,
            error_handler.config_power.output_voltage_off_level,
        );
        assert!(error_handler.has_error(ErrorCode::FaultPowerRelayOpen));
        assert!(error_handler.has_error(ErrorCode::IgnoreWarnCommRelay));
    }
}
