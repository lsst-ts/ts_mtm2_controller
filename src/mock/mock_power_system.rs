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

#[derive(Clone)]
pub struct MockPowerSystem {
    // Is powered on or not.
    pub is_power_on: bool,
    // Is breaker on or not.
    pub is_breaker_on: bool,
    // Current voltage in volt.
    _voltage: f64,
    // Current current in ampere.
    _current: f64,
    // Maximum voltage in volt.
    _max_voltage: f64,
    // Maximum current in ampere.
    _max_current: f64,
    // Specifies the minimum voltage level, plus some hysteresis, required to
    // operate the electronic breakers.
    _breaker_operating_voltage: f64,
    // Current counts used to simulate the time for the increase and decrease of
    // the output voltage and current.
    _count_voltage: i32,
    _count_current: i32,
    // The maximum counts for the power and breaker operations.
    _max_count_power_on: i32,
    _max_count_power_off: i32,
    _max_count_breaker_on: i32,
    _max_count_breaker_off: i32,
}

impl MockPowerSystem {
    /// Mock power system to simulate the power system.
    ///
    /// # Arguments
    /// * `max_voltage` - Maximum voltage in volt.
    /// * `max_current` - Maximum current in ampere.
    /// * `breaker_operating_voltage` - Specifies the minimum voltage level,
    /// plus some hysteresis, required to operate the electronic breakers.
    /// * `time_unit` - Unit time in milliseconds used to convert the time
    /// values to counts.
    /// * `time_power_on` - Time in milliseconds to turn on the power.
    /// * `time_power_off` - Time in milliseconds to turn off the power.
    /// * `time_breaker_on` - Time in milliseconds to turn on the breaker.
    /// * `time_breaker_off` - Time in milliseconds to turn off the breaker.
    ///
    /// # Returns
    /// A new mock power system.
    pub fn new(
        max_voltage: f64,
        max_current: f64,
        breaker_operating_voltage: f64,
        time_unit: i32,
        time_power_on: i32,
        time_power_off: i32,
        time_breaker_on: i32,
        time_breaker_off: i32,
    ) -> Self {
        Self {
            is_power_on: false,
            is_breaker_on: false,

            _voltage: 0.0,
            _current: 0.0,

            _max_voltage: max_voltage,
            _max_current: max_current,

            _breaker_operating_voltage: breaker_operating_voltage,

            _count_voltage: 0,
            _count_current: 0,

            _max_count_power_on: time_power_on / time_unit,
            _max_count_power_off: time_power_off / time_unit,

            _max_count_breaker_on: time_breaker_on / time_unit,
            _max_count_breaker_off: time_breaker_off / time_unit,
        }
    }

    /// Get the voltage and current values.
    ///
    /// # Returns
    /// A tuple containing the voltage in volt and current in ampere.
    pub fn get_voltage_and_current(&mut self) -> (f64, f64) {
        (self.calculate_voltage(), self.calculate_current())
    }

    /// Calculate the voltage.
    ///
    /// # Returns
    /// The voltage in volt.
    fn calculate_voltage(&mut self) -> f64 {
        // Cache the voltage before updating it.
        let voltage = self._voltage;

        if self.is_power_on {
            Self::saturate_count(&mut self._count_voltage, self._max_count_power_on);

            if self._count_voltage < self._max_count_power_on {
                self._count_voltage += 1;

                self._voltage = self._max_voltage
                    * (self._count_voltage as f64 / self._max_count_power_on as f64);
            }
        } else {
            Self::saturate_count(&mut self._count_voltage, self._max_count_power_off);

            if self._count_voltage > 0 {
                self._count_voltage -= 1;

                self._voltage = self._max_voltage
                    * (self._count_voltage as f64 / self._max_count_power_off as f64);
            }
        }

        voltage
    }

    /// Saturate the count to the maximum count.
    ///
    /// # Arguments
    /// * `count` - A mutable reference to the count to be saturated.
    /// * `max_count` - The maximum count to saturate the counter to.
    fn saturate_count(count: &mut i32, max_count: i32) {
        if *count > max_count {
            *count = max_count;
        }
    }

    /// Calculate the current.
    ///
    /// # Returns
    /// The current in ampere.
    fn calculate_current(&mut self) -> f64 {
        // Cache the current before updating it.
        let current = self._current;

        if self.is_breaker_on {
            Self::saturate_count(&mut self._count_current, self._max_count_breaker_on);

            if self.is_breaker_enabled() && (self._count_current < self._max_count_breaker_on) {
                self._count_current += 1;

                self._current = self._max_current
                    * (self._count_current as f64 / self._max_count_breaker_on as f64);
            }
        } else {
            Self::saturate_count(&mut self._count_current, self._max_count_breaker_off);

            if self._count_current > 0 {
                self._count_current -= 1;

                self._current = self._max_current
                    * (self._count_current as f64 / self._max_count_breaker_off as f64);
            }
        }

        current
    }

    /// Check if the breaker is enabled.
    ///
    /// # Returns
    /// `true` if the breaker is enabled, `false` otherwise.
    pub fn is_breaker_enabled(&self) -> bool {
        self.is_breaker_on && self.is_power_on && (self._voltage >= self._breaker_operating_voltage)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use approx::assert_relative_eq;
    use std::f64::EPSILON;

    use crate::mock::mock_constants::{PLANT_CURRENT_COMMUNICATION, PLANT_VOLTAGE};

    fn create_mock_power_system() -> MockPowerSystem {
        MockPowerSystem::new(
            PLANT_VOLTAGE,
            PLANT_CURRENT_COMMUNICATION,
            19.0,
            10,
            30,
            20,
            50,
            30,
        )
    }

    #[test]
    fn test_new() {
        let power_system = create_mock_power_system();

        assert_eq!(power_system._max_count_power_on, 3);
        assert_eq!(power_system._max_count_power_off, 2);
        assert_eq!(power_system._max_count_breaker_on, 5);
        assert_eq!(power_system._max_count_breaker_off, 3);
    }

    #[test]
    fn test_get_voltage_and_current() {
        let mut power_system = create_mock_power_system();

        // Turn on the power and breaker.
        power_system.is_power_on = true;
        power_system.is_breaker_on = true;

        // Initial voltage and current.
        let (mut voltage, mut current) = power_system.get_voltage_and_current();

        assert_eq!(voltage, 0.0);
        assert_eq!(current, 0.0);
        assert_eq!(power_system._count_voltage, 1);
        assert_eq!(power_system._count_current, 0);

        // Saturate the voltage and current.
        for _ in 0..(power_system._max_count_power_on + power_system._max_count_breaker_on) {
            (voltage, current) = power_system.get_voltage_and_current();
        }

        assert_eq!(voltage, power_system._max_voltage);
        assert_eq!(current, power_system._max_current);
        assert_eq!(
            power_system._count_voltage,
            power_system._max_count_power_on
        );
        assert_eq!(
            power_system._count_current,
            power_system._max_count_breaker_on
        );

        // Turn off the power and breaker.
        power_system.is_power_on = false;
        power_system.is_breaker_on = false;

        // Voltage and current should be 0.0 in the final.
        for _ in 0..(power_system._max_count_power_off + power_system._max_count_breaker_off) {
            (voltage, current) = power_system.get_voltage_and_current();
        }

        assert_eq!(voltage, 0.0);
        assert_eq!(current, 0.0);
        assert_eq!(power_system._count_voltage, 0);
        assert_eq!(power_system._count_current, 0);
    }

    #[test]
    fn test_calculate_voltage() {
        let mut power_system = create_mock_power_system();

        // Initial voltage should be 0.0 and the count should be 0.
        assert_eq!(power_system.calculate_voltage(), 0.0);
        assert_eq!(power_system._count_voltage, 0);

        // Turn on the power.
        power_system.is_power_on = true;

        assert_eq!(power_system.calculate_voltage(), 0.0);
        assert_eq!(power_system._count_voltage, 1);
        assert_relative_eq!(
            power_system.calculate_voltage(),
            power_system._max_voltage / (power_system._max_count_power_on as f64),
            epsilon = EPSILON
        );
        assert_eq!(power_system._count_voltage, 2);

        for _ in 0..power_system._max_count_power_on {
            power_system.calculate_voltage();
        }

        assert_eq!(power_system.calculate_voltage(), power_system._max_voltage);
        assert_eq!(
            power_system._count_voltage,
            power_system._max_count_power_on
        );

        // Turn off the power.
        power_system.is_power_on = false;

        assert_eq!(power_system.calculate_voltage(), power_system._max_voltage);
        assert_eq!(
            power_system._count_voltage,
            power_system._max_count_power_off - 1
        );

        assert_relative_eq!(
            power_system.calculate_voltage(),
            power_system._max_voltage
                - (power_system._max_voltage / (power_system._max_count_power_off as f64)),
            epsilon = EPSILON
        );
        assert_eq!(
            power_system._count_voltage,
            power_system._max_count_power_off - 2
        );

        for _ in 0..power_system._max_count_power_off {
            power_system.calculate_voltage();
        }

        assert_eq!(power_system.calculate_voltage(), 0.0);
        assert_eq!(power_system._count_voltage, 0);
    }

    #[test]
    fn test_saturate_count() {
        // Should be saturated to the maximum count.
        let mut count = 5;
        MockPowerSystem::saturate_count(&mut count, 3);
        assert_eq!(count, 3);

        // Should not change if the count is less than or equal to the maximum
        // count.
        count = 2;
        MockPowerSystem::saturate_count(&mut count, 3);
        assert_eq!(count, 2);

        count = 3;
        MockPowerSystem::saturate_count(&mut count, 3);
        assert_eq!(count, 3);
    }

    #[test]
    fn test_calculate_current() {
        let mut power_system = create_mock_power_system();

        // Initial current should be 0.0 and the count should be 0.
        assert_eq!(power_system.calculate_current(), 0.0);
        assert_eq!(power_system._count_current, 0);

        // Turn on the power and breaker.
        power_system.is_power_on = true;
        power_system._voltage = power_system._breaker_operating_voltage;

        power_system.is_breaker_on = true;

        assert_eq!(power_system.calculate_current(), 0.0);
        assert_eq!(power_system._count_current, 1);
        assert_relative_eq!(
            power_system.calculate_current(),
            power_system._max_current / (power_system._max_count_breaker_on as f64),
            epsilon = EPSILON
        );
        assert_eq!(power_system._count_current, 2);

        for _ in 0..power_system._max_count_breaker_on {
            power_system.calculate_current();
        }

        assert_eq!(power_system.calculate_current(), power_system._max_current);
        assert_eq!(
            power_system._count_current,
            power_system._max_count_breaker_on
        );

        // Turn off the breaker.
        power_system.is_breaker_on = false;

        assert_eq!(power_system.calculate_current(), power_system._max_current);
        assert_eq!(
            power_system._count_current,
            power_system._max_count_breaker_off - 1
        );

        assert_relative_eq!(
            power_system.calculate_current(),
            power_system._max_current
                - (power_system._max_current / (power_system._max_count_breaker_off as f64)),
            epsilon = EPSILON
        );
        assert_eq!(
            power_system._count_current,
            power_system._max_count_breaker_off - 2
        );

        for _ in 0..power_system._max_count_breaker_off {
            power_system.calculate_current();
        }

        assert_eq!(power_system.calculate_current(), 0.0);
        assert_eq!(power_system._count_current, 0);
    }

    #[test]
    fn test_is_breaker_enabled() {
        let mut power_system = create_mock_power_system();

        assert!(!power_system.is_breaker_enabled());

        // Turn on the power and breaker.
        power_system.is_power_on = true;
        power_system.is_breaker_on = true;

        assert!(!power_system.is_breaker_enabled());

        // Set the voltage to the operating voltage.
        power_system._voltage = power_system._breaker_operating_voltage;

        assert!(power_system.is_breaker_enabled());
    }
}
