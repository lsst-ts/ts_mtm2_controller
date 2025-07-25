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

use crate::enums::{BitEnum, DigitalOutput, DigitalOutputStatus, PowerSystemState, PowerType};

#[derive(Copy, Clone)]
pub struct SubPowerSystem {
    // Power type of the sub power system.
    pub power_type: PowerType,
    // Power is on or not
    pub is_power_on: bool,
    // System state
    pub state: PowerSystemState,
    // Output voltage off level in volt.
    _output_voltage_off_level: f64,
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

impl SubPowerSystem {
    /// Create a new instance of the sub power system.
    ///
    /// # Arguments
    /// * `power_type` - Type of the power system.
    /// * `output_voltage_off_level` - Output voltage off level in volt.
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
    /// New instance of the sub power system.
    pub fn new(
        power_type: PowerType,
        output_voltage_off_level: f64,
        breaker_operating_voltage: f64,
        time_unit: i32,
        time_power_on: i32,
        time_power_off: i32,
        time_breaker_on: i32,
        time_breaker_off: i32,
    ) -> Self {
        Self {
            power_type,

            is_power_on: false,
            state: PowerSystemState::Init,

            _output_voltage_off_level: output_voltage_off_level,
            _breaker_operating_voltage: breaker_operating_voltage,

            _count_voltage: 0,
            _count_current: 0,

            _max_count_power_on: time_power_on / time_unit,
            _max_count_power_off: time_power_off / time_unit,

            _max_count_breaker_on: time_breaker_on / time_unit,
            _max_count_breaker_off: time_breaker_off / time_unit,
        }
    }

    /// Is the power on or not.
    ///
    /// # Returns
    /// True if the power is on. Otherwise, false.
    pub fn is_power_on(&self) -> bool {
        self.is_power_on
            && (self.state == PowerSystemState::PoweringOn
                || self.state == PowerSystemState::PoweredOn
                || self.state == PowerSystemState::ResettingBreakers)
    }

    /// Power on the system.
    ///
    /// # Returns
    /// Vector of tuples containing the digital output and its status to be
    /// applied.
    pub fn power_on(&mut self) -> Vec<(DigitalOutput, DigitalOutputStatus)> {
        self.is_power_on = true;
        self.state = PowerSystemState::PoweringOn;

        self._count_voltage = self._max_count_power_on;

        if self.power_type == PowerType::Motor {
            vec![
                (
                    DigitalOutput::MotorPower,
                    DigitalOutputStatus::BinaryHighLevel,
                ),
                (
                    DigitalOutput::ResetMotorBreakers,
                    DigitalOutputStatus::BinaryHighLevel,
                ),
            ]
        } else {
            vec![
                (
                    DigitalOutput::CommunicationPower,
                    DigitalOutputStatus::BinaryHighLevel,
                ),
                (
                    DigitalOutput::ResetCommunicationBreakers,
                    DigitalOutputStatus::BinaryHighLevel,
                ),
            ]
        }
    }

    /// Power off the system.
    ///
    /// # Returns
    /// Vector of tuples containing the digital output and its status to be
    /// applied.
    pub fn power_off(&mut self) -> Vec<(DigitalOutput, DigitalOutputStatus)> {
        self.is_power_on = false;
        self.state = PowerSystemState::PoweringOff;

        self._count_voltage = self._max_count_power_off;
        self._count_current = self._max_count_breaker_off;

        if self.power_type == PowerType::Motor {
            vec![(
                DigitalOutput::MotorPower,
                DigitalOutputStatus::BinaryLowLevel,
            )]
        } else {
            vec![(
                DigitalOutput::CommunicationPower,
                DigitalOutputStatus::BinaryLowLevel,
            )]
        }
    }

    /// Reset the breakers.
    ///
    /// # Notes
    /// To reset the breakers, put the reset breaker control output to a logic
    /// low (providing the falling edge the breakers see as the reset signal).
    ///
    /// # Arguments
    /// * `status` - The status to set the reset breakers output.
    ///
    /// # Returns
    /// Vector of tuples containing the digital output and its status to be
    /// applied.
    pub fn reset_breakers(
        &mut self,
        status: DigitalOutputStatus,
    ) -> Vec<(DigitalOutput, DigitalOutputStatus)> {
        self.state = PowerSystemState::ResettingBreakers;

        self._count_current = self._max_count_breaker_on;

        vec![(
            if self.power_type == PowerType::Motor {
                DigitalOutput::ResetMotorBreakers
            } else {
                DigitalOutput::ResetCommunicationBreakers
            },
            status,
        )]
    }

    /// Transition the state of the power system.
    ///
    /// # Arguments
    /// * `voltage` - Voltage in volt.
    /// * `current` - Current in ampere.
    /// * `digital_output` - Digital output value.
    ///
    /// # Returns
    /// Tuple containing three values:
    /// 1. True if the state is changed. Otherwise, false.
    /// 2. True if there is the error and need to power off the system.
    /// Otherwise, false.
    /// 3. Vector of tuples containing the digital output and its status to be
    /// applied.
    pub fn transition_state(
        &mut self,
        voltage: f64,
        current: f64,
        digital_output: u8,
    ) -> (bool, bool, Vec<(DigitalOutput, DigitalOutputStatus)>) {
        match self.state {
            PowerSystemState::PoweringOn => {
                // Update the counters for the voltage and current.
                if self._count_voltage > 0 {
                    self._count_voltage -= 1;
                }

                // If the voltage is below the operating voltage, power off the
                // system.
                if self._count_voltage == 0 {
                    if voltage >= self._breaker_operating_voltage {
                        // Check we can transition to the powered on state or
                        // not. Or we might need to reset the breakers instead.

                        // TODO: Instead of using the minimum current value, we
                        // should check the breaker status instead. Change this
                        // in a later time (OSW-745).
                        let minimum_current = 1.0;
                        if current > minimum_current {
                            self.state = PowerSystemState::PoweredOn;
                            return (true, false, Vec::new());
                        } else {
                            return (
                                true,
                                false,
                                self.reset_breakers(DigitalOutputStatus::BinaryLowLevel),
                            );
                        }
                    } else {
                        error!(
                            "Powering on failed: voltage < operating voltage for power system ({:?}) in power system state: {:?}.",
                            self.power_type,
                            PowerSystemState::PoweringOn
                        );
                        return (true, true, self.power_off());
                    }
                }
            }

            PowerSystemState::PoweringOff => {
                // Update the counters.
                if self._count_current > 0 {
                    self._count_current -= 1;
                }

                if self._count_current == 0 {
                    if self._count_voltage > 0 {
                        self._count_voltage -= 1;
                    }
                }

                // Check if the voltage is below the output voltage off level.
                if (self._count_voltage == 0) && (self._count_current == 0) {
                    let has_error = voltage >= self._output_voltage_off_level;
                    if has_error {
                        error!(
                            "Powering off failed: has the relay open fault for power system ({:?}) in power system state: {:?}.",
                            self.power_type,
                            PowerSystemState::PoweringOff
                        );
                    }

                    self.state = PowerSystemState::PoweredOff;

                    return (true, has_error, Vec::new());
                }
            }

            PowerSystemState::ResettingBreakers => {
                // Power off the system if the voltage is below the operating
                // voltage.
                if voltage < self._breaker_operating_voltage {
                    error!(
                        "Resetting breakers failed: voltage < operating voltage for power system ({:?}) in power system state: {:?}.",
                        self.power_type,
                        PowerSystemState::ResettingBreakers
                    );
                    return (true, true, self.power_off());
                }

                // Update the counter.
                if self._count_current > 0 {
                    self._count_current -= 1;
                }

                if self._count_current == 0 {
                    // Check if the reset breakers output is set to a logic
                    // low level. If yes, set it to a logic high level to reset
                    // the breakers.
                    if self.power_type == PowerType::Motor {
                        if (digital_output & DigitalOutput::ResetMotorBreakers.bit_value()) == 0 {
                            return (
                                false,
                                false,
                                self.reset_breakers(DigitalOutputStatus::BinaryHighLevel),
                            );
                        }
                    } else {
                        if (digital_output & DigitalOutput::ResetCommunicationBreakers.bit_value())
                            == 0
                        {
                            return (
                                false,
                                false,
                                self.reset_breakers(DigitalOutputStatus::BinaryHighLevel),
                            );
                        }
                    }

                    // TODO: Instead of using the minimum current value, we
                    // should check the breaker status instead. Change this
                    // in a later time (OSW-745).
                    let minimum_current = 1.0;
                    if current > minimum_current {
                        self.state = PowerSystemState::PoweredOn;

                        return (true, false, Vec::new());
                    } else {
                        error!(
                            "Resetting breakers failed for power system ({:?}) in power system state: {:?}.",
                            self.power_type,
                            PowerSystemState::ResettingBreakers
                        );
                        return (true, true, self.power_off());
                    }
                }
            }

            _ => {
                // No state transition for other states.
            }
        }

        (false, false, Vec::new())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::path::Path;

    use crate::mock::mock_constants::PLANT_CURRENT_COMMUNICATION;
    use crate::mock::mock_plant::MockPlant;
    use crate::power::config_power::ConfigPower;
    use crate::utility::read_file_stiffness;

    fn create_sub_power_system_and_plant() -> (SubPowerSystem, MockPlant) {
        let config_power = ConfigPower::new();

        let filepath = Path::new("config/stiff_matrix_m2.yaml");
        let stiffness = read_file_stiffness(filepath);

        (
            SubPowerSystem::new(
                PowerType::Communication,
                config_power.output_voltage_off_level,
                config_power.breaker_operating_voltage,
                config_power.loop_time as i32,
                config_power.get_time_power_on(PowerType::Communication),
                config_power.get_time_power_off(PowerType::Communication),
                config_power.get_time_breaker_on(PowerType::Communication),
                config_power.get_time_breaker_off(PowerType::Communication),
            ),
            MockPlant::new(&stiffness, 0.0),
        )
    }

    fn transition_state_until_change(
        sub_power_system: &mut SubPowerSystem,
        plant: &mut MockPlant,
    ) -> (f64, f64, bool) {
        let mut voltage;
        let mut current;

        let has_error_in_transition;

        loop {
            (voltage, current) = plant.power_system_communication.get_voltage_and_current();

            let (is_state_changed, has_error, actions) =
                sub_power_system.transition_state(voltage, current, plant.digital_output);
            actions.iter().for_each(|(digital_output, status)| {
                plant.switch_digital_output(*digital_output, *status);
            });

            if is_state_changed {
                has_error_in_transition = has_error;
                break;
            }
        }

        (voltage, current, has_error_in_transition)
    }

    #[test]
    fn test_is_power_on() {
        let mut sub_power_system = create_sub_power_system_and_plant().0;

        assert!(!sub_power_system.is_power_on());

        sub_power_system.is_power_on = true;
        sub_power_system.state = PowerSystemState::PoweringOn;
        assert!(sub_power_system.is_power_on());

        sub_power_system.state = PowerSystemState::PoweredOn;
        assert!(sub_power_system.is_power_on());

        sub_power_system.state = PowerSystemState::ResettingBreakers;
        assert!(sub_power_system.is_power_on());
    }

    #[test]
    fn test_power_on() {
        let (mut sub_power_system, mut plant) = create_sub_power_system_and_plant();

        let actions = sub_power_system.power_on();
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        assert!(plant.power_system_communication.is_power_on);
        assert!(plant.power_system_communication.is_breaker_on);

        assert_eq!(sub_power_system.state, PowerSystemState::PoweringOn);
        assert_eq!(
            sub_power_system._count_voltage,
            sub_power_system._max_count_power_on
        );
    }

    #[test]
    fn test_power_off() {
        let (mut sub_power_system, mut plant) = create_sub_power_system_and_plant();
        let actions = sub_power_system.power_on();
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        let actions = sub_power_system.power_off();
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        assert!(!plant.power_system_communication.is_power_on);
        assert!(!plant.power_system_communication.is_breaker_on);

        assert!(!sub_power_system.is_power_on());
        assert_eq!(sub_power_system.state, PowerSystemState::PoweringOff);
        assert_eq!(
            sub_power_system._count_voltage,
            sub_power_system._max_count_power_off
        );
        assert_eq!(
            sub_power_system._count_current,
            sub_power_system._max_count_breaker_off
        );
    }

    #[test]
    fn test_reset_breakers() {
        let (mut sub_power_system, mut plant) = create_sub_power_system_and_plant();
        plant.switch_digital_output(
            DigitalOutput::ResetCommunicationBreakers,
            DigitalOutputStatus::BinaryHighLevel,
        );

        assert_ne!(plant.digital_output, 0);

        let actions = sub_power_system.reset_breakers(DigitalOutputStatus::BinaryLowLevel);
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        assert_eq!(plant.digital_output, 0);

        assert_eq!(sub_power_system.state, PowerSystemState::ResettingBreakers);
        assert_eq!(
            sub_power_system._count_current,
            sub_power_system._max_count_breaker_on
        );
    }

    #[test]
    fn test_transition_state_power_on() {
        let (mut sub_power_system, mut plant) = create_sub_power_system_and_plant();
        plant.switch_digital_output(
            DigitalOutput::ResetCommunicationBreakers,
            DigitalOutputStatus::BinaryHighLevel,
        );

        let actions = sub_power_system.power_on();
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        let (voltage, current, has_error) =
            transition_state_until_change(&mut sub_power_system, &mut plant);

        assert_eq!(sub_power_system.state, PowerSystemState::ResettingBreakers);
        assert!(voltage >= sub_power_system._breaker_operating_voltage);
        assert_eq!(current, 0.0);
        assert!(!has_error);

        let (voltage, current, has_error) =
            transition_state_until_change(&mut sub_power_system, &mut plant);

        assert_eq!(sub_power_system.state, PowerSystemState::PoweredOn);
        assert!(voltage >= sub_power_system._breaker_operating_voltage);
        assert!(current >= PLANT_CURRENT_COMMUNICATION / 2.0);
        assert!(!has_error);
    }

    #[test]
    fn test_transition_state_power_off() {
        let (mut sub_power_system, mut plant) = create_sub_power_system_and_plant();
        plant.switch_digital_output(
            DigitalOutput::ResetCommunicationBreakers,
            DigitalOutputStatus::BinaryHighLevel,
        );

        // Power on the system first.
        let actions = sub_power_system.power_on();
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });
        transition_state_until_change(&mut sub_power_system, &mut plant);
        transition_state_until_change(&mut sub_power_system, &mut plant);

        assert_eq!(sub_power_system.state, PowerSystemState::PoweredOn);

        // Now power off the system without the error.
        let actions = sub_power_system.power_off();
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        let (voltage, current, has_error) =
            transition_state_until_change(&mut sub_power_system, &mut plant);

        assert_eq!(sub_power_system.state, PowerSystemState::PoweredOff);
        assert!(voltage < sub_power_system._output_voltage_off_level);
        assert_eq!(current, 0.0);
        assert!(!has_error);

        // There will be the error if the voltage is above the output voltage off level.
        let actions = sub_power_system.power_off();
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        sub_power_system._count_voltage = 0;
        sub_power_system._count_current = 0;

        let (state_is_changed, has_error, actions) = sub_power_system.transition_state(
            sub_power_system._output_voltage_off_level + 1.0,
            current,
            plant.digital_output,
        );
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        assert_eq!(sub_power_system.state, PowerSystemState::PoweredOff);
        assert!(state_is_changed);
        assert!(has_error);
    }

    #[test]
    fn test_test_transition_state_resetting_breakers_fail() {
        let (mut sub_power_system, mut plant) = create_sub_power_system_and_plant();

        let actions = sub_power_system.reset_breakers(DigitalOutputStatus::BinaryLowLevel);
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        // Should fail because the voltage is below the operating voltage.
        let (state_is_changed, has_error, _) = sub_power_system.transition_state(
            sub_power_system._breaker_operating_voltage - 1.0,
            0.0,
            plant.digital_output,
        );

        assert!(state_is_changed);
        assert!(has_error);
        assert_eq!(sub_power_system.state, PowerSystemState::PoweringOff);

        transition_state_until_change(&mut sub_power_system, &mut plant);

        assert_eq!(sub_power_system.state, PowerSystemState::PoweredOff);
    }
}
