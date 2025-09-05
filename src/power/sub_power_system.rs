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

use crate::enums::{
    BitEnum, DigitalInput, DigitalOutput, DigitalOutputStatus, PowerSystemState, PowerType,
};

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
    // Bit mask for the breakers.
    _breaker_mask: u32,
    // Current counts used to simulate the time for the increase and decrease of
    // the output voltage and breaker status.
    _count_voltage: i32,
    _count_breaker: i32,
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
            _breaker_mask: Self::get_breaker_mask(power_type),

            _count_voltage: 0,
            _count_breaker: 0,

            _max_count_power_on: time_power_on / time_unit,
            _max_count_power_off: time_power_off / time_unit,

            _max_count_breaker_on: time_breaker_on / time_unit,
            _max_count_breaker_off: time_breaker_off / time_unit,
        }
    }

    /// Get the breaker mask for the specified power type.
    ///
    /// # Arguments
    /// * `power_type` - Type of the power system.
    ///
    /// # Returns
    /// Bit mask for the breakers.
    fn get_breaker_mask(power_type: PowerType) -> u32 {
        let bits;
        if power_type == PowerType::Motor {
            bits = vec![
                DigitalInput::J1W9N1MotorPowerBreaker,
                DigitalInput::J1W9N2MotorPowerBreaker,
                DigitalInput::J1W9N3MotorPowerBreaker,
                DigitalInput::J2W10N1MotorPowerBreaker,
                DigitalInput::J2W10N2MotorPowerBreaker,
                DigitalInput::J2W10N3MotorPowerBreaker,
                DigitalInput::J3W11N1MotorPowerBreaker,
                DigitalInput::J3W11N2MotorPowerBreaker,
                DigitalInput::J3W11N3MotorPowerBreaker,
            ];
        } else {
            bits = vec![
                DigitalInput::J1W12N1CommunicationPowerBreaker,
                DigitalInput::J1W12N2CommunicationPowerBreaker,
                DigitalInput::J2W13N1CommunicationPowerBreaker,
                DigitalInput::J2W13N2CommunicationPowerBreaker,
                DigitalInput::J3W14N1CommunicationPowerBreaker,
                DigitalInput::J3W14N2CommunicationPowerBreaker,
            ];
        }

        bits.iter().fold(0, |acc, x| acc | x.bit_value())
    }

    /// Is the power on or not.
    ///
    /// # Returns
    /// True if the power is on. Otherwise, false.
    pub fn is_power_on(&self) -> bool {
        self.is_power_on && (self.state == PowerSystemState::PoweredOn)
    }

    /// Power on the system.
    ///
    /// # Returns
    /// Vector of tuples containing the digital output and its status to be
    /// applied.
    pub fn power_on(&mut self) -> Vec<(DigitalOutput, DigitalOutputStatus)> {
        self.is_power_on = true;
        self.update_and_log_state(PowerSystemState::PoweringOn);

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

    /// Update and log the state.
    ///
    /// # Arguments
    /// * `state` - The new state to set.
    fn update_and_log_state(&mut self, state: PowerSystemState) {
        self.state = state;

        info!(
            "The new power state is {:?} in the {:?} power system.",
            self.state, self.power_type
        );
    }

    /// Power off the system.
    ///
    /// # Returns
    /// Vector of tuples containing the digital output and its status to be
    /// applied.
    pub fn power_off(&mut self) -> Vec<(DigitalOutput, DigitalOutputStatus)> {
        self.is_power_on = false;
        self.update_and_log_state(PowerSystemState::PoweringOff);

        self._count_voltage = self._max_count_power_off;
        self._count_breaker = self._max_count_breaker_off;

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
        self.update_and_log_state(PowerSystemState::ResettingBreakers);

        self._count_breaker = self._max_count_breaker_on;

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
    /// # Notes
    /// For the failure conditions, make sure to be consistent with
    /// `Controller.update_internal_status_power_system_and_check_error()`.
    ///
    /// # Arguments
    /// * `voltage` - Voltage in volt.
    /// * `digital_output` - Digital output value.
    /// * `digital_input` - Digital input value.
    /// * `is_interlock_active` - True if the interlock is active. Otherwise,
    /// false.
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
        digital_output: u8,
        digital_input: u32,
        is_interlock_active: bool,
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
                        // Check the interlock status.
                        if (self.power_type == PowerType::Motor) && is_interlock_active {
                            error!(
                                "Powering on failed: interlock is active for power system ({:?}) in power system state: {:?}.",
                                self.power_type,
                                PowerSystemState::PoweringOn
                            );

                            return (true, true, self.power_off());
                        }

                        // Check we can transition to the powered on state or
                        // not. Or we might need to reset the breakers instead.
                        if self.are_breakers_enabled(digital_input) {
                            self.update_and_log_state(PowerSystemState::PoweredOn);

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
                if self._count_breaker > 0 {
                    self._count_breaker -= 1;
                }

                if self._count_breaker == 0 {
                    if self._count_voltage > 0 {
                        self._count_voltage -= 1;
                    }
                }

                // Check if the voltage is below the output voltage off level.
                if (self._count_voltage == 0) && (self._count_breaker == 0) {
                    let has_error = voltage >= self._output_voltage_off_level;
                    if has_error {
                        error!(
                            "Powering off failed: has the relay open fault for power system ({:?}) in power system state: {:?}.",
                            self.power_type,
                            PowerSystemState::PoweringOff
                        );
                    }

                    self.update_and_log_state(PowerSystemState::PoweredOff);

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
                if self._count_breaker > 0 {
                    self._count_breaker -= 1;
                }

                if self._count_breaker == 0 {
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

                    // Check the interlock status.
                    if (self.power_type == PowerType::Motor) && is_interlock_active {
                        error!(
                            "Resetting breakers: interlock is active for power system ({:?}) in power system state: {:?}.",
                            self.power_type,
                            PowerSystemState::ResettingBreakers
                        );

                        return (true, true, self.power_off());
                    }

                    // Transition the power state anyway. The ErrorHandler will
                    // check the breaker status if not all the breakers are
                    // enabled.
                    self.update_and_log_state(PowerSystemState::PoweredOn);

                    if !self.are_breakers_enabled(digital_input) {
                        warn!(
                            "Resetting breakers failed for power system ({:?}) in power system state: {:?}.",
                            self.power_type,
                            PowerSystemState::ResettingBreakers
                        );
                    }

                    return (true, false, Vec::new());
                }
            }

            _ => {
                // No state transition for other states.
            }
        }

        (false, false, Vec::new())
    }

    /// Check if all the breakers are enabled or not.
    ///
    /// # Notes
    /// If the inputs are active low, meaning that a low logic state on an input
    /// indicates that the breaker is closed. A logic high on an input means
    /// that the breaker is open or its input voltage is below its operating
    /// threshold.
    ///
    /// # Arguments
    /// * `digital_input` - Digital input value to check the breaker status.
    ///
    /// # Returns
    /// True if all the breakers are enabled. Otherwise, false.
    pub fn are_breakers_enabled(&self, digital_input: u32) -> bool {
        digital_input & self._breaker_mask == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::path::Path;

    use crate::mock::mock_constants::{
        PLANT_CURRENT_COMMUNICATION, TEST_DIGITAL_INPUT_NO_POWER, TEST_DIGITAL_INPUT_POWER_COMM,
    };
    use crate::mock::mock_plant::MockPlant;
    use crate::power::config_power::ConfigPower;
    use crate::utility::read_file_stiffness;

    fn create_sub_power_system_and_plant(power_type: PowerType) -> (SubPowerSystem, MockPlant) {
        let config_power = ConfigPower::new();

        let filepath = Path::new("config/stiff_matrix_m2.yaml");
        let stiffness = read_file_stiffness(filepath);

        (
            SubPowerSystem::new(
                power_type,
                config_power.output_voltage_off_level,
                config_power.breaker_operating_voltage,
                config_power.loop_time as i32,
                config_power.get_time_power_on(power_type),
                config_power.get_time_power_off(power_type),
                config_power.get_time_breaker_on(power_type),
                config_power.get_time_breaker_off(power_type),
            ),
            MockPlant::new(&stiffness, 0.0),
        )
    }

    fn transition_state_until_change(
        sub_power_system: &mut SubPowerSystem,
        plant: &mut MockPlant,
        is_interlock_active: bool,
    ) -> (f64, f64, bool) {
        let mut voltage;
        let mut current;

        let has_error_in_transition;

        loop {
            if sub_power_system.power_type == PowerType::Motor {
                (voltage, current) = plant.power_system_motor.get_voltage_and_current();
            } else {
                (voltage, current) = plant.power_system_communication.get_voltage_and_current();
            }

            let (is_state_changed, has_error, actions) = sub_power_system.transition_state(
                voltage,
                plant.digital_output,
                plant.get_digital_input(),
                is_interlock_active,
            );
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
    fn test_get_breaker_mask() {
        assert_eq!(
            SubPowerSystem::get_breaker_mask(PowerType::Motor),
            0b111111111000000
        );
        assert_eq!(
            SubPowerSystem::get_breaker_mask(PowerType::Communication),
            0b11111000000001000000000000000
        );
    }

    #[test]
    fn test_is_power_on() {
        let mut sub_power_system = create_sub_power_system_and_plant(PowerType::Communication).0;

        assert!(!sub_power_system.is_power_on());

        sub_power_system.is_power_on = true;
        sub_power_system.state = PowerSystemState::PoweringOn;
        assert!(!sub_power_system.is_power_on());

        sub_power_system.state = PowerSystemState::PoweredOn;
        assert!(sub_power_system.is_power_on());

        sub_power_system.state = PowerSystemState::ResettingBreakers;
        assert!(!sub_power_system.is_power_on());
    }

    #[test]
    fn test_power_on() {
        let (mut sub_power_system, mut plant) =
            create_sub_power_system_and_plant(PowerType::Communication);

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
        let (mut sub_power_system, mut plant) =
            create_sub_power_system_and_plant(PowerType::Communication);
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
            sub_power_system._count_breaker,
            sub_power_system._max_count_breaker_off
        );
    }

    #[test]
    fn test_reset_breakers() {
        let (mut sub_power_system, mut plant) =
            create_sub_power_system_and_plant(PowerType::Communication);
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
            sub_power_system._count_breaker,
            sub_power_system._max_count_breaker_on
        );
    }

    #[test]
    fn test_transition_state_power_on_success() {
        let (mut sub_power_system, mut plant) =
            create_sub_power_system_and_plant(PowerType::Communication);
        plant.switch_digital_output(
            DigitalOutput::ResetCommunicationBreakers,
            DigitalOutputStatus::BinaryHighLevel,
        );

        let actions = sub_power_system.power_on();
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        let (voltage, current, has_error) =
            transition_state_until_change(&mut sub_power_system, &mut plant, false);

        assert_eq!(sub_power_system.state, PowerSystemState::ResettingBreakers);
        assert!(voltage >= sub_power_system._breaker_operating_voltage);
        assert_eq!(current, 0.0);
        assert!(!has_error);

        let (voltage, current, has_error) =
            transition_state_until_change(&mut sub_power_system, &mut plant, false);

        assert_eq!(sub_power_system.state, PowerSystemState::PoweredOn);
        assert!(voltage >= sub_power_system._breaker_operating_voltage);
        assert!(current >= PLANT_CURRENT_COMMUNICATION / 2.0);
        assert!(!has_error);
    }

    #[test]
    fn test_transition_state_power_on_fail() {
        let (mut sub_power_system, mut plant) = create_sub_power_system_and_plant(PowerType::Motor);
        plant.switch_digital_output(
            DigitalOutput::ResetMotorBreakers,
            DigitalOutputStatus::BinaryHighLevel,
        );

        let actions = sub_power_system.power_on();
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        let (voltage, current, has_error) =
            transition_state_until_change(&mut sub_power_system, &mut plant, true);

        assert_eq!(sub_power_system.state, PowerSystemState::PoweringOff);
        assert!(voltage >= sub_power_system._breaker_operating_voltage);
        assert_eq!(current, 0.0);
        assert!(has_error);

        let (voltage, current, has_error) =
            transition_state_until_change(&mut sub_power_system, &mut plant, true);

        assert_eq!(sub_power_system.state, PowerSystemState::PoweredOff);
        assert_eq!(voltage, 0.0);
        assert_eq!(current, 0.0);
        assert!(!has_error);
    }

    #[test]
    fn test_transition_state_power_off() {
        let (mut sub_power_system, mut plant) =
            create_sub_power_system_and_plant(PowerType::Communication);
        plant.switch_digital_output(
            DigitalOutput::ResetCommunicationBreakers,
            DigitalOutputStatus::BinaryHighLevel,
        );

        // Power on the system first.
        let actions = sub_power_system.power_on();
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });
        transition_state_until_change(&mut sub_power_system, &mut plant, false);
        transition_state_until_change(&mut sub_power_system, &mut plant, false);

        assert_eq!(sub_power_system.state, PowerSystemState::PoweredOn);

        // Now power off the system without the error.
        let actions = sub_power_system.power_off();
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        let (voltage, current, has_error) =
            transition_state_until_change(&mut sub_power_system, &mut plant, false);

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
        sub_power_system._count_breaker = 0;

        let (state_is_changed, has_error, actions) = sub_power_system.transition_state(
            sub_power_system._output_voltage_off_level + 1.0,
            plant.digital_output,
            plant.get_digital_input(),
            false,
        );
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        assert_eq!(sub_power_system.state, PowerSystemState::PoweredOff);
        assert!(state_is_changed);
        assert!(has_error);
    }

    #[test]
    fn test_transition_state_resetting_breakers_fail_voltage() {
        let (mut sub_power_system, mut plant) =
            create_sub_power_system_and_plant(PowerType::Communication);

        let actions = sub_power_system.reset_breakers(DigitalOutputStatus::BinaryLowLevel);
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        // Should fail because the voltage is below the operating voltage.
        let (state_is_changed, has_error, _) = sub_power_system.transition_state(
            sub_power_system._breaker_operating_voltage - 1.0,
            plant.digital_output,
            plant.get_digital_input(),
            false,
        );

        assert!(state_is_changed);
        assert!(has_error);
        assert_eq!(sub_power_system.state, PowerSystemState::PoweringOff);

        transition_state_until_change(&mut sub_power_system, &mut plant, false);

        assert_eq!(sub_power_system.state, PowerSystemState::PoweredOff);
    }

    #[test]
    fn test_transition_state_resetting_breakers_fail_interlock() {
        let (mut sub_power_system, mut plant) = create_sub_power_system_and_plant(PowerType::Motor);

        let actions = sub_power_system.reset_breakers(DigitalOutputStatus::BinaryLowLevel);
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });

        // Should fail because the interlock is active.
        let has_error = transition_state_until_change(&mut sub_power_system, &mut plant, true).2;

        assert!(has_error);
        assert_eq!(sub_power_system.state, PowerSystemState::PoweringOff);

        transition_state_until_change(&mut sub_power_system, &mut plant, false);

        assert_eq!(sub_power_system.state, PowerSystemState::PoweredOff);
    }

    #[test]
    fn test_are_breakers_enabled() {
        let sub_power_system = create_sub_power_system_and_plant(PowerType::Communication).0;

        assert!(!sub_power_system.are_breakers_enabled(TEST_DIGITAL_INPUT_NO_POWER));
        assert!(sub_power_system.are_breakers_enabled(TEST_DIGITAL_INPUT_POWER_COMM));
    }
}
