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

use std::path::Path;

use crate::enums::PowerType;
use crate::utility::get_parameter;

pub struct ConfigPower {
    // Loop time in milliseconds used in the PowerSystemProcess.
    pub loop_time: u64,
    // Maximum of the counter of telemetry used in the PowerSystemProcess.
    pub maximum_counter_telemetry: i32,
    // Maximum of the counter of toggling the closed-loop control bit used in
    // the PowerSystemProcess.
    pub maximum_counter_closed_loop_control_bit: i32,
    // Output voltage off level in volt.
    pub output_voltage_off_level: f64,
    // Minimum of the warning voltage level.
    pub warning_voltage_min: f64,
    // Maximum of the warning voltage level.
    pub warning_voltage_max: f64,
    // Minimum of the fault voltage level.
    pub fault_voltage_min: f64,
    // Maximum of the fault voltage level.
    pub fault_voltage_max: f64,
    // Communication excessive current in amperes.
    pub excessive_current_communication: f64,
    // Motor excessive current in amperes.
    pub excessive_current_motor: f64,
    // Gain and offset for the motor current.
    pub current_gain_motor: f64,
    pub current_offset_motor: f64,
    // Boost current fault is enabled or not.
    pub is_boost_current_fault_enabled: bool,
    // Time to wait for the telemetry to stabilize in millisecond.
    pub telemetry_stable_time: i32,
    // Breaker operating voltage rise time in millisecond for the communication
    // and motor.
    pub breaker_voltage_rise_time_communication: i32,
    pub breaker_voltage_rise_time_motor: i32,
    // Output voltage settling time in millisecond for the communication and
    // motor.
    pub output_voltage_settling_time_communication: i32,
    pub output_voltage_settling_time_motor: i32,
    // Output voltage fall time in millisecond for the communication and motor.
    pub output_voltage_fall_time_communication: i32,
    pub output_voltage_fall_time_motor: i32,
    // The physical amounts of time in millisecond that they take for the power
    // relay to open and closed its contacts. These values are common to both
    // motor and communication power buses since they use the same relay type.
    pub relay_open_delay: i32,
    pub relay_close_delay: i32,
    // The minimum amount of time in millisecond that it takes to reset a
    // breaker.
    pub reset_breaker_pulse_width: i32,
    // The amount of time in millisecond that it takes for the breaker to power
    // up and stabilize its operating status once its operating voltage is
    // achieved.
    pub breaker_on_time: i32,
    // Specifies the minimum voltage level, plus some hysteresis, required to
    // operate the electronic breakers.
    pub breaker_operating_voltage: f64,
    // The maximum amount of time in millisecond from when the cRIO's motor
    // power on digital output is asserted on until the interlock device turns
    // on/off the motor power relay. This value is dependent on the interlock's
    // device programming.
    pub interlock_output_delay: i32,
}

impl ConfigPower {
    /// Create a new ConfigPower object.
    ///
    /// # Returns
    /// A new ConfigPower object.
    pub fn new() -> Self {
        let filepath = Path::new("config/parameters_power.yaml");

        let loop_time = get_parameter::<i32>(filepath, "loop_time");

        let norminal_voltage = get_parameter(filepath, "norminal_voltage");
        let warning_voltage_level = get_parameter(filepath, "warning_voltage_level");
        let fault_voltage_level = get_parameter(filepath, "fault_voltage_level");

        let (warning_voltage_min, warning_voltage_max) =
            Self::calculate_range(norminal_voltage, warning_voltage_level);
        let (fault_voltage_min, fault_voltage_max) =
            Self::calculate_range(norminal_voltage, fault_voltage_level);

        Self {
            loop_time: loop_time as u64,
            maximum_counter_telemetry: get_parameter::<i32>(filepath, "period_telemetry")
                / loop_time,
            maximum_counter_closed_loop_control_bit: get_parameter::<i32>(
                filepath,
                "period_toggle_closed_loop_control_bit",
            ) / loop_time,
            output_voltage_off_level: get_parameter(filepath, "output_voltage_off_level"),

            warning_voltage_min: warning_voltage_min,
            warning_voltage_max: warning_voltage_max,

            fault_voltage_min: fault_voltage_min,
            fault_voltage_max: fault_voltage_max,

            excessive_current_communication: get_parameter(
                filepath,
                "excessive_current_communication",
            ),
            excessive_current_motor: get_parameter(filepath, "excessive_current_motor"),

            current_gain_motor: get_parameter(filepath, "current_gain_motor"),
            current_offset_motor: get_parameter(filepath, "current_offset_motor"),

            is_boost_current_fault_enabled: get_parameter(
                filepath,
                "is_boost_current_fault_enabled",
            ),

            telemetry_stable_time: get_parameter(filepath, "telemetry_stable_time"),

            breaker_voltage_rise_time_communication: get_parameter(
                filepath,
                "breaker_voltage_rise_time_communication",
            ),
            breaker_voltage_rise_time_motor: get_parameter(
                filepath,
                "breaker_voltage_rise_time_motor",
            ),

            output_voltage_settling_time_communication: get_parameter(
                filepath,
                "output_voltage_settling_time_communication",
            ),
            output_voltage_settling_time_motor: get_parameter(
                filepath,
                "output_voltage_settling_time_motor",
            ),

            output_voltage_fall_time_communication: get_parameter(
                filepath,
                "output_voltage_fall_time_communication",
            ),
            output_voltage_fall_time_motor: get_parameter(
                filepath,
                "output_voltage_fall_time_motor",
            ),

            relay_open_delay: get_parameter(filepath, "relay_open_delay"),
            relay_close_delay: get_parameter(filepath, "relay_close_delay"),

            reset_breaker_pulse_width: get_parameter(filepath, "reset_breaker_pulse_width"),

            breaker_on_time: get_parameter(filepath, "breaker_on_time"),

            breaker_operating_voltage: get_parameter(filepath, "breaker_operating_voltage"),

            interlock_output_delay: get_parameter(filepath, "interlock_output_delay"),
        }
    }

    /// Calculate the range of values based on a given value and a percentage.
    ///
    /// # Arguments
    /// * `value` - The value to calculate the range for.
    /// * `percentage` - The percentage to calculate the range (0 - 100 %).
    ///
    /// # Returns
    /// A tuple containing the minimum and maximum values of the range.
    fn calculate_range(value: f64, percentage: f64) -> (f64, f64) {
        let min = value - value * percentage / 100.0;
        let max = value + value * percentage / 100.0;
        (min, max)
    }

    /// Get the time required to power on the system.
    ///
    /// # Arguments
    /// * `power_type` - The type of power.
    ///
    /// # Returns
    /// The time in milliseconds required to power on the system.
    pub fn get_time_power_on(&self, power_type: PowerType) -> i32 {
        if power_type == PowerType::Motor {
            return self.telemetry_stable_time
                + self.breaker_voltage_rise_time_motor
                + self.output_voltage_settling_time_motor;
        } else {
            return self.telemetry_stable_time
                + self.breaker_voltage_rise_time_communication
                + self.output_voltage_settling_time_communication;
        }
    }

    /// Get the time required to power off the system.
    ///
    /// # Arguments
    /// * `power_type` - The type of power.
    ///
    /// # Returns
    /// The time in milliseconds required to power off the system.
    pub fn get_time_power_off(&self, power_type: PowerType) -> i32 {
        if power_type == PowerType::Motor {
            return self.output_voltage_fall_time_motor;
        } else {
            return self.output_voltage_fall_time_communication;
        }
    }

    /// Get the time required to power on the breaker.
    ///
    /// # Arguments
    /// * `power_type` - The type of power.
    ///
    /// # Returns
    /// The time in milliseconds required to power on the breaker.
    pub fn get_time_breaker_on(&self, power_type: PowerType) -> i32 {
        if power_type == PowerType::Motor {
            return self.reset_breaker_pulse_width
                + self.breaker_on_time
                + self.relay_close_delay
                + self.interlock_output_delay;
        } else {
            return self.reset_breaker_pulse_width + self.breaker_on_time + self.relay_close_delay;
        }
    }

    /// Get the time required to power off the breaker.
    ///
    /// # Arguments
    /// * `power_type` - The type of power.
    ///
    /// # Returns
    /// The time in milliseconds required to power off the breaker.
    pub fn get_time_breaker_off(&self, power_type: PowerType) -> i32 {
        if power_type == PowerType::Motor {
            return self.relay_open_delay + self.interlock_output_delay;
        } else {
            return self.relay_open_delay;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_range() {
        let (min, max) = ConfigPower::calculate_range(50.0, 10.0);

        assert_eq!(min, 45.0);
        assert_eq!(max, 55.0);
    }

    #[test]
    fn test_get_time_power_on() {
        let config = ConfigPower::new();

        assert_eq!(config.get_time_power_on(PowerType::Motor), 210);
        assert_eq!(config.get_time_power_on(PowerType::Communication), 140);
    }

    #[test]
    fn test_get_time_power_off() {
        let config = ConfigPower::new();

        assert_eq!(config.get_time_power_off(PowerType::Motor), 300);
        assert_eq!(config.get_time_power_off(PowerType::Communication), 50);
    }

    #[test]
    fn test_get_time_breaker_on() {
        let config = ConfigPower::new();

        assert_eq!(config.get_time_breaker_on(PowerType::Motor), 1000);
        assert_eq!(config.get_time_breaker_on(PowerType::Communication), 950);
    }

    #[test]
    fn test_get_time_breaker_off() {
        let config = ConfigPower::new();

        assert_eq!(config.get_time_breaker_off(PowerType::Motor), 80);
        assert_eq!(config.get_time_breaker_off(PowerType::Communication), 30);
    }
}
