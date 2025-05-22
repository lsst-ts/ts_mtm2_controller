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

use crate::utility::get_parameter;

pub struct ConfigPower {
    // Loop time in milliseconds used in the PowerSystemProcess.
    pub loop_time: u64,
    // Maximum of the counter of telemetry used in the PowerSystemProcess.
    pub maximum_counter_telemetry: i32,
    // Maximum of the counter of toggling the closed-loop control bit used in
    // the PowerSystemProcess.
    pub maximum_counter_closed_loop_control_bit: i32,
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
}
