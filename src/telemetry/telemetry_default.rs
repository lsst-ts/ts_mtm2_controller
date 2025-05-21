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

use serde_json::Value;
use std::collections::HashMap;

pub trait TelemetryDefault {
    /// Initialize a dictionary with the same value for all keys.
    ///
    /// # Arguments
    /// * `keys` - A list of keys.
    /// * `value` - The value to be assigned to all keys.
    ///
    /// # Returns
    /// A dictionary with the same value for all keys.
    fn initialize_dict_value<T: Clone>(keys: &[&str], value: T) -> HashMap<String, T> {
        let mut dict = HashMap::new();
        for key in keys {
            dict.insert(String::from(*key), value.clone());
        }
        dict
    }

    /// Initialize a dictionary with a vector of the same value for all keys.
    ///
    /// # Arguments
    /// * `keys` - A list of keys.
    /// * `value` - The value to be assigned to all keys.
    /// * `number` - The number of elements in the vector.
    ///
    /// # Returns
    /// A dictionary with a vector of the same value for all keys.
    fn initialize_dict_vector<T: Clone>(
        keys: &[&str],
        value: T,
        number: usize,
    ) -> HashMap<String, Vec<T>> {
        let mut dict = HashMap::new();
        for key in keys {
            dict.insert(String::from(*key), vec![value.clone(); number]);
        }
        dict
    }

    /// Round a vector to a specific digit.
    ///
    /// # Arguments
    /// * `vector` - The vector to be rounded.
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The rounded vector.
    fn round_vector(&self, vector: &[f64], digit: i32) -> Vec<f64> {
        vector
            .iter()
            .map(|value| self.round(*value, digit))
            .collect()
    }

    /// Round a value to a specific digit.
    ///
    /// # Arguments
    /// * `value` - The value to be rounded.
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The rounded value.
    fn round(&self, value: f64, digit: i32) -> f64 {
        let normalized = 10.0_f64.powi(digit);
        (value * normalized).round() / normalized
    }

    /// Get the telemetry messages.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The messages.
    fn get_messages(&self, digit: i32) -> Vec<Value>;
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TelemetryTest;
    impl TelemetryDefault for TelemetryTest {
        fn get_messages(&self, _digit: i32) -> Vec<Value> {
            Vec::new()
        }
    }

    #[test]
    fn test_initialize_dict_value() {
        let keys = ["key1", "key2", "key3"];
        let value = 1.0;

        let dict: HashMap<String, f64> = TelemetryTest::initialize_dict_value(&keys, value);

        assert_eq!(dict.len(), keys.len());

        keys.iter().for_each(|key| {
            assert_eq!(dict[&(*key.to_string())], value);
        });
    }

    #[test]
    fn test_initialize_dict_vector() {
        let keys = ["key1", "key2", "key3"];
        let value = 1;
        let number = 3;

        let dict = TelemetryTest::initialize_dict_vector(&keys, value, number);

        assert_eq!(dict.len(), keys.len());

        keys.iter().for_each(|key| {
            assert_eq!(dict[&(*key.to_string())], vec![value; number]);
        });
    }

    #[test]
    fn test_round_vector() {
        let telemetry = TelemetryTest;

        let vector = vec![1.23456, 2.34567, 3.45678];

        assert_eq!(telemetry.round_vector(&vector, 1), vec![1.2, 2.3, 3.5]);
        assert_eq!(telemetry.round_vector(&vector, 2), vec![1.23, 2.35, 3.46]);

        assert_eq!(telemetry.round_vector(&vector[0..1], 2), vec![1.23]);
        assert_eq!(telemetry.round_vector(&vector[0..2], 3), vec![1.235, 2.346]);
    }

    #[test]
    fn test_round() {
        let telemetry = TelemetryTest;

        assert_eq!(telemetry.round(1.23456, 0), 1.0);
        assert_eq!(telemetry.round(1.23456, 1), 1.2);
        assert_eq!(telemetry.round(1.23456, 2), 1.23);
        assert_eq!(telemetry.round(1.23456, 3), 1.235);
    }
}
