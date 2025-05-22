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
use std::collections::HashMap;

use crate::telemetry::telemetry_default::TelemetryDefault;

#[derive(Clone)]
pub struct TelemetryPower {
    // Motor and communication power (raw) in volt and ampere.
    pub power_raw: HashMap<String, f64>,
    // Motor and communication power (processed) in volt and ampere.
    pub power_processed: HashMap<String, f64>,
    // Digital output.
    pub digital_output: u8,
    // Digital input.
    pub digital_input: u32,
}

impl TelemetryDefault for TelemetryPower {
    fn get_messages(&self, digit: i32) -> Vec<Value> {
        let mut messages = Vec::new();
        messages.push(self.get_message_power_status_raw(digit));
        messages.push(self.get_message_power_status(digit));

        messages
    }
}

impl TelemetryPower {
    /// Create a new power telemetry object.
    pub fn new() -> Self {
        Self {
            power_raw: Self::initialize_dict_value(
                &["motorVoltage", "motorCurrent", "commVoltage", "commCurrent"],
                0.0,
            ),
            power_processed: Self::initialize_dict_value(
                &["motorVoltage", "motorCurrent", "commVoltage", "commCurrent"],
                0.0,
            ),
            digital_output: 0,
            digital_input: 0,
        }
    }

    /// Get the message of the power status.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the power status.
    fn get_message_power_status(&self, digit: i32) -> Value {
        json!({
            "id": "powerStatus",
            "motorVoltage": self.round(self.power_processed["motorVoltage"], digit),
            "motorCurrent": self.round(self.power_processed["motorCurrent"], digit),
            "commVoltage": self.round(self.power_processed["commVoltage"], digit),
            "commCurrent": self.round(self.power_processed["commCurrent"], digit),
        })
    }

    /// Get the message of the raw power status.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the raw power status.
    fn get_message_power_status_raw(&self, digit: i32) -> Value {
        json!({
            "id": "powerStatusRaw",
            "motorVoltage": self.round(self.power_raw["motorVoltage"], digit),
            "motorCurrent": self.round(self.power_raw["motorCurrent"], digit),
            "commVoltage": self.round(self.power_raw["commVoltage"], digit),
            "commCurrent": self.round(self.power_raw["commCurrent"], digit),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_messages() {
        let telemetry = TelemetryPower::new();

        let messages_off = telemetry.get_messages(2);

        assert_eq!(messages_off.len(), 2);
    }

    #[test]
    fn test_get_message_power_status() {
        let telemetry = TelemetryPower::new();

        let message = telemetry.get_message_power_status(2);

        assert_eq!(message["id"], "powerStatus");
        for key in ["motorVoltage", "motorCurrent", "commVoltage", "commCurrent"].iter() {
            assert_eq!(
                message[*key],
                telemetry.round(telemetry.power_processed[*key], 2)
            );
        }
    }

    #[test]
    fn test_get_message_power_status_raw() {
        let telemetry = TelemetryPower::new();

        let message = telemetry.get_message_power_status_raw(2);

        assert_eq!(message["id"], "powerStatusRaw");
        for key in ["motorVoltage", "motorCurrent", "commVoltage", "commCurrent"].iter() {
            assert_eq!(message[*key], telemetry.round(telemetry.power_raw[*key], 2));
        }
    }
}
