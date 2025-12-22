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

use std::collections::HashMap;

use log::error;
use serde_json::Value;

use crate::enums::{
    CommandStatus, DigitalOutput, DigitalOutputStatus, PowerSystemState, PowerType,
};
use crate::error_handler::ErrorHandler;
use crate::event_queue::EventQueue;
use crate::power::{config_power::ConfigPower, sub_power_system::SubPowerSystem};
use crate::telemetry::event::Event;
use crate::telemetry::telemetry_power::TelemetryPower;
use crate::utility::acknowledge_command;

pub struct PowerCommandStatus {
    // Sequence ID of the command, put -1 for the internal command.
    pub sequence_id: i64,
    // The final power status is expected to be on or off
    pub is_power_on: bool,
}

pub struct PowerSystem {
    // Configuration of the power system
    pub config: ConfigPower,
    // Sub-power system
    pub subsystem: HashMap<PowerType, SubPowerSystem>,
    // Actions to apply to the hardware
    _actions: Vec<(DigitalOutput, DigitalOutputStatus)>,
    // Events to publish
    pub event_queue: EventQueue,
    // Has the power supply fault or not.
    _has_fault_power_supply: bool,
    // Has the interlock fault or not.
    _has_fault_interlock: bool,
    // Power command status
    _command_status: HashMap<PowerType, Option<PowerCommandStatus>>,
    // Power command result
    _command_result: HashMap<PowerType, Option<Value>>,
}

impl Default for PowerSystem {
    fn default() -> Self {
        Self::new()
    }
}

impl PowerSystem {
    /// Create a new instance of the power system.
    ///
    /// # Returns
    /// New instance of the power system.
    pub fn new() -> Self {
        let config_power = ConfigPower::new();

        let subsystem_motor = SubPowerSystem::new(
            PowerType::Motor,
            config_power.output_voltage_off_level,
            config_power.breaker_operating_voltage,
            config_power.loop_time as i32,
            config_power.get_time_power_on(PowerType::Motor),
            config_power.get_time_power_off(PowerType::Motor),
            config_power.get_time_breaker_on(PowerType::Motor),
            config_power.get_time_breaker_off(PowerType::Motor),
        );
        let subsystem_communication = SubPowerSystem::new(
            PowerType::Communication,
            config_power.output_voltage_off_level,
            config_power.breaker_operating_voltage,
            config_power.loop_time as i32,
            config_power.get_time_power_on(PowerType::Communication),
            config_power.get_time_power_off(PowerType::Communication),
            config_power.get_time_breaker_on(PowerType::Communication),
            config_power.get_time_breaker_off(PowerType::Communication),
        );
        let subsystem = HashMap::from([
            (PowerType::Motor, subsystem_motor),
            (PowerType::Communication, subsystem_communication),
        ]);

        Self {
            subsystem,

            config: config_power,

            _actions: Vec::new(),

            event_queue: EventQueue::new(),

            _has_fault_power_supply: false,
            _has_fault_interlock: false,

            _command_status: HashMap::from([
                (PowerType::Motor, None),
                (PowerType::Communication, None),
            ]),
            _command_result: HashMap::from([
                (PowerType::Motor, None),
                (PowerType::Communication, None),
            ]),
        }
    }

    /// Check if there are any actions to apply to the hardware.
    ///
    /// # Returns
    /// True if there are actions to apply, false otherwise.
    pub fn has_actions(&self) -> bool {
        !self._actions.is_empty()
    }

    /// Get the actions to apply to the hardware and clear the internal actions.
    ///
    /// # Returns
    /// Actions.
    pub fn get_actions_and_clear(&mut self) -> Vec<(DigitalOutput, DigitalOutputStatus)> {
        let actions = self._actions.clone();
        self._actions.clear();

        actions
    }

    /// Power on the system.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    /// * `sequence_id` - Sequence ID.
    ///
    /// # Returns
    /// Power system state.
    pub fn power_on(
        &mut self,
        power_type: PowerType,
        sequence_id: i64,
    ) -> Option<PowerSystemState> {
        // Fail the command if there is the power supply fault.
        if self._has_fault_power_supply {
            error!(
                "Cannot power on the {:?} power system if there is the power supply error.",
                power_type
            );

            return None;
        }

        // If there is the ongoing power command, reject the new power
        // command.
        if self._command_status[&power_type].is_some() {
            error!(
                "Cannot power on the {:?} power system if there is the ongoing power command.",
                power_type
            );

            return None;
        }

        // Return the current state if the power is already on
        if self.is_powered_on(power_type) {
            return Some(PowerSystemState::PoweredOn);
        }

        // Track the power command status
        self._command_status.insert(
            power_type,
            Some(PowerCommandStatus {
                sequence_id,
                is_power_on: true,
            }),
        );

        // Power on the system and track the status
        let mut actions;
        let state;
        if let Some(sub_power_system) = self.subsystem.get_mut(&power_type) {
            actions = sub_power_system.power_on();
            state = sub_power_system.state;
        } else {
            error!("Invalid power type: {:?}.", power_type);

            return None;
        }

        self._actions.append(&mut actions);

        // Add an event
        self.event_queue
            .add_event(Event::get_message_power_system_state(
                power_type, true, state,
            ));

        Some(state)
    }

    /// Check if the power system is powered on.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    ///
    /// # Returns
    /// True if the power system is powered on, false otherwise.
    pub fn is_powered_on(&self, power_type: PowerType) -> bool {
        if let Some(sub_power_system) = self.subsystem.get(&power_type) {
            if sub_power_system.is_power_on
                && (sub_power_system.state == PowerSystemState::PoweredOn)
            {
                return true;
            }
        }

        false
    }

    /// Power off the system.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    /// * `sequence_id` - Sequence ID. Put -1 for the internal command.
    ///
    /// # Returns
    /// Power system state.
    pub fn power_off(
        &mut self,
        power_type: PowerType,
        sequence_id: i64,
    ) -> Option<PowerSystemState> {
        // If there is the ongoing power-off command, reject the new power-off
        // command.
        if let Some(command_status) = &self._command_status[&power_type] {
            if !command_status.is_power_on {
                error!(
                    "Cannot power off the {:?} power system if there is the ongoing power-off command.",
                    power_type
                );

                return None;
            }
        }

        // Return the current state if the power is already off
        if self.is_powered_off(power_type) {
            return Some(PowerSystemState::PoweredOff);
        }

        // Track the power command status. Note we can power off the system even
        // there is an ongoing power-on command for the same power type.
        self._command_status.insert(
            power_type,
            Some(PowerCommandStatus {
                sequence_id,
                is_power_on: false,
            }),
        );

        // Power off the system
        let mut actions;
        let state;
        if let Some(sub_power_system) = self.subsystem.get_mut(&power_type) {
            actions = sub_power_system.power_off();
            state = sub_power_system.state;
        } else {
            error!("Invalid power type: {:?}.", power_type);

            return None;
        }

        self._actions.append(&mut actions);

        // Add an event
        self.event_queue
            .add_event(Event::get_message_power_system_state(
                power_type, false, state,
            ));

        Some(state)
    }

    /// Check if the power system is powered off.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    ///
    /// # Returns
    /// True if the power system is powered off, false otherwise.
    pub fn is_powered_off(&self, power_type: PowerType) -> bool {
        if let Some(sub_power_system) = self.subsystem.get(&power_type) {
            if (!sub_power_system.is_power_on)
                && (sub_power_system.state == PowerSystemState::PoweredOff)
            {
                return true;
            }
        }

        false
    }

    /// Reset breakers.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    /// * `sequence_id` - Sequence ID.
    ///
    /// # Returns
    /// Power system state if the breakers were reset. Otherwise, None.
    pub fn reset_breakers(
        &mut self,
        power_type: PowerType,
        sequence_id: i64,
    ) -> Option<PowerSystemState> {
        // If there is the ongoing power command, reject the new power
        // command.
        if self._command_status[&power_type].is_some() {
            error!(
                "Cannot reset breakers for {:?} power system if there is the ongoing power command.",
                power_type
            );

            return None;
        }

        // Only reset the breakers when the power is on
        if !self.is_powered_on(power_type) {
            error!(
                "Cannot reset breakers for {:?} power system if the state is not powered on.",
                power_type
            );

            return None;
        }

        // Track the power command status
        self._command_status.insert(
            power_type,
            Some(PowerCommandStatus {
                sequence_id,
                is_power_on: true,
            }),
        );

        // Reset the breakers
        let mut actions;
        let state;
        if let Some(sub_power_system) = self.subsystem.get_mut(&power_type) {
            actions = sub_power_system.reset_breakers(DigitalOutputStatus::BinaryLowLevel);
            state = sub_power_system.state;
        } else {
            error!("Invalid power type: {:?}.", power_type);

            return None;
        }

        self._actions.append(&mut actions);

        // Add an event
        self.event_queue
            .add_event(Event::get_message_power_system_state(
                power_type, true, state,
            ));

        Some(state)
    }

    /// Check the power supply error.
    ///
    /// # Arguments
    /// * `digital_output` - Digital output value.
    /// * `digital_input` - Digital input value.
    pub fn check_power_supply_error(&mut self, digital_output: u8, digital_input: u32) {
        let (has_fault_power_supply_load_share, has_fault_power_health, has_fault_interlock) =
            ErrorHandler::check_power_supply_health(
                self.config.is_boost_current_fault_enabled,
                digital_input,
                digital_output,
            );

        self._has_fault_power_supply = has_fault_power_supply_load_share || has_fault_power_health;
        self._has_fault_interlock = has_fault_interlock;
    }

    /// Transition the state of the power system. This function should be
    /// called at each loop.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    /// * `voltage` - Voltage in volt.
    /// * `digital_output` - Digital output value.
    /// * `digital_input` - Digital input value.
    ///
    /// # Returns
    /// Tuple containing two values:
    /// 1. True if the state is changed. Otherwise, false.
    /// 2. True if there is the error and need to power off the system.
    /// Otherwise, false.
    pub fn transition_state(
        &mut self,
        power_type: PowerType,
        voltage: f64,
        digital_output: u8,
        digital_input: u32,
    ) -> (bool, bool) {
        // Transition the state of the power system
        let mut is_state_changed = false;
        let mut has_error = false;
        let mut actions = Vec::new();

        let mut is_power_on = false;
        let mut state = PowerSystemState::Init;
        if let Some(sub_power_system) = self.subsystem.get_mut(&power_type) {
            (is_state_changed, has_error, actions) = sub_power_system.transition_state(
                voltage,
                digital_output,
                digital_input,
                self._has_fault_interlock,
            );

            is_power_on = sub_power_system.is_power_on;
            state = sub_power_system.state;
        }

        // Apply the actions to the hardware
        self._actions.append(&mut actions);

        // Add an event
        if is_state_changed {
            self.event_queue
                .add_event(Event::get_message_power_system_state(
                    power_type,
                    is_power_on,
                    state,
                ));

            // If we are tracking the power command status and we know the
            // final result, update the related command result.
            let mut command_result = Value::Null;
            if (state == PowerSystemState::PoweredOn) || (state == PowerSystemState::PoweredOff) {
                if let Some(current_command_status) = &self._command_status[&power_type] {
                    // We only need to check the power command status with
                    // is_power_on=true here. If this field is false, it
                    // will always work.
                    let mut command_status = CommandStatus::Success;
                    if current_command_status.is_power_on {
                        command_status = if is_power_on {
                            CommandStatus::Success
                        } else {
                            CommandStatus::Fail
                        };
                    }

                    command_result =
                        acknowledge_command(command_status, current_command_status.sequence_id);
                }
            }

            if !command_result.is_null() {
                self._command_result
                    .insert(power_type, Some(command_result));
                self._command_status.insert(power_type, None);
            }
        }

        (is_state_changed, has_error)
    }

    /// Get any command result.
    ///
    /// # Returns
    /// Any command result.
    pub fn get_any_command_result(&mut self) -> Option<Value> {
        if self._command_result[&PowerType::Motor].is_some() {
            return self.get_command_result(PowerType::Motor);
        } else if self._command_result[&PowerType::Communication].is_some() {
            return self.get_command_result(PowerType::Communication);
        }

        None
    }

    /// Get the command result.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    ///
    /// # Returns
    /// The command result.
    fn get_command_result(&mut self, power_type: PowerType) -> Option<Value> {
        self._command_result[&power_type].as_ref()?;

        let command_result = self._command_result[&power_type].clone();
        self._command_result.insert(power_type, None);

        command_result
    }

    /// Check if there is a command result.
    ///
    /// # Returns
    /// True if there is a command result, false otherwise.
    pub fn has_command_result(&self) -> bool {
        self._command_result.values().any(|result| result.is_some())
    }

    /// Process the telemetry data.
    ///
    /// # Arguments
    /// * `telemetry` - Telemetry data.
    pub fn process_telemetry_data(&self, telemetry: &mut TelemetryPower) {
        let mut power_processed = telemetry.power_raw.clone();

        // Only need to calibrate the motor current.
        let motor_current_calibrated = self.config.current_gain_motor
            * power_processed["motorCurrent"]
            + self.config.current_offset_motor;

        power_processed.insert(String::from("motorCurrent"), motor_current_calibrated);

        telemetry.power_processed = power_processed;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::path::Path;

    use crate::enums::{BitEnum, DigitalInput};
    use crate::mock::mock_constants::{
        TEST_DIGITAL_INPUT_NO_POWER, TEST_DIGITAL_INPUT_POWER_COMM,
        TEST_DIGITAL_INPUT_POWER_COMM_MOTOR, TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
    };
    use crate::mock::mock_plant::MockPlant;
    use crate::mock::mock_power_system::MockPowerSystem;
    use crate::telemetry::telemetry_power::TelemetryPower;
    use crate::utility::read_file_stiffness;

    fn create_power_system() -> (PowerSystem, MockPlant) {
        // Plant model
        let filepath = Path::new("config/stiff_matrix_m2.yaml");
        let stiffness = read_file_stiffness(filepath);
        let plant = MockPlant::new(&stiffness, 0.0);

        (PowerSystem::new(), plant)
    }

    fn run_until_breaker_enabled(power_system: &mut MockPowerSystem) {
        loop {
            if power_system.is_breaker_enabled() {
                break;
            }

            power_system.get_voltage_and_current();
        }
    }

    fn run_until_done(
        power_type: PowerType,
        power_system: &mut PowerSystem,
        plant: &mut MockPlant,
    ) -> Option<Value> {
        loop {
            let voltage = get_power(power_type, plant).0;
            power_system
                .transition_state(
                    power_type,
                    voltage,
                    plant.digital_output,
                    plant.get_digital_input(),
                )
                .0;

            if power_system.has_actions() {
                let actions = power_system.get_actions_and_clear();
                apply_actions_to_plant(&actions, plant);
            }

            if power_system.has_command_result() {
                break;
            }
        }

        power_system.get_command_result(power_type)
    }

    fn get_power(power_type: PowerType, plant: &mut MockPlant) -> (f64, f64) {
        if power_type == PowerType::Motor {
            return plant.power_system_motor.get_voltage_and_current();
        } else {
            return plant.power_system_communication.get_voltage_and_current();
        }
    }

    fn apply_actions_to_plant(
        actions: &[(DigitalOutput, DigitalOutputStatus)],
        plant: &mut MockPlant,
    ) {
        actions.iter().for_each(|(digital_output, status)| {
            plant.switch_digital_output(*digital_output, *status);
        });
    }

    fn init_default_digital_output(plant: &mut MockPlant) {
        [
            DigitalOutput::InterlockEnable,
            DigitalOutput::ResetMotorBreakers,
            DigitalOutput::ResetCommunicationBreakers,
        ]
        .iter()
        .for_each(|digital_output| {
            plant.switch_digital_output(*digital_output, DigitalOutputStatus::BinaryHighLevel);
        });
    }

    #[test]
    fn test_has_actions() {
        let mut power_system = create_power_system().0;
        assert!(!power_system.has_actions());

        power_system._actions.push((
            DigitalOutput::MotorPower,
            DigitalOutputStatus::BinaryHighLevel,
        ));

        assert!(power_system.has_actions());
    }

    #[test]
    fn test_get_actions_and_clear() {
        let mut power_system = create_power_system().0;

        power_system._actions.push((
            DigitalOutput::MotorPower,
            DigitalOutputStatus::BinaryHighLevel,
        ));
        let actions = power_system.get_actions_and_clear();

        assert_eq!(
            actions,
            vec![(
                DigitalOutput::MotorPower,
                DigitalOutputStatus::BinaryHighLevel
            )]
        );
        assert!(!power_system.has_actions());
    }

    #[test]
    fn test_power_on_success() {
        let (mut power_system, mut plant) = create_power_system();
        assert!(!power_system.subsystem[&PowerType::Motor].is_power_on);

        assert_eq!(
            power_system.power_on(PowerType::Motor, 1),
            Some(PowerSystemState::PoweringOn)
        );
        assert!(power_system.subsystem[&PowerType::Motor].is_power_on);
        assert!(power_system._command_status[&PowerType::Motor].is_some());
        assert!(
            power_system._command_status[&PowerType::Motor]
                .as_ref()
                .unwrap()
                .is_power_on
        );

        assert_eq!(
            run_until_done(PowerType::Motor, &mut power_system, &mut plant),
            Some(acknowledge_command(CommandStatus::Success, 1))
        );

        assert_eq!(
            power_system.event_queue.get_events_and_clear(),
            vec![
                Event::get_message_power_system_state(
                    PowerType::Motor,
                    true,
                    PowerSystemState::PoweringOn
                ),
                Event::get_message_power_system_state(
                    PowerType::Motor,
                    true,
                    PowerSystemState::ResettingBreakers
                ),
                Event::get_message_power_system_state(
                    PowerType::Motor,
                    true,
                    PowerSystemState::PoweredOn
                ),
            ]
        );
    }

    #[test]
    fn test_power_on_fail() {
        let mut power_system = create_power_system().0;

        // Has the power supply error
        power_system._has_fault_power_supply = true;

        assert!(power_system.power_on(PowerType::Motor, 1).is_none());

        // Has the ongoing power command
        power_system._has_fault_power_supply = false;

        assert_eq!(
            power_system.power_on(PowerType::Motor, 1),
            Some(PowerSystemState::PoweringOn)
        );

        assert!(power_system.power_on(PowerType::Motor, 1).is_none());
    }

    #[test]
    fn test_get_any_command_result() {
        let mut power_system = create_power_system().0;

        // No command result
        assert_eq!(power_system.get_any_command_result(), None);

        // Has the command results
        power_system._command_result.insert(
            PowerType::Motor,
            Some(acknowledge_command(CommandStatus::Success, 1)),
        );
        power_system._command_result.insert(
            PowerType::Communication,
            Some(acknowledge_command(CommandStatus::Success, 2)),
        );

        assert_eq!(
            power_system.get_any_command_result(),
            Some(acknowledge_command(CommandStatus::Success, 1))
        );
        assert_eq!(
            power_system.get_any_command_result(),
            Some(acknowledge_command(CommandStatus::Success, 2))
        );

        assert_eq!(power_system.get_any_command_result(), None);
    }

    #[test]
    fn test_get_command_result() {
        let mut power_system = create_power_system().0;

        // No command result
        assert_eq!(power_system.get_any_command_result(), None);

        // Has the command result
        power_system._command_result.insert(
            PowerType::Communication,
            Some(acknowledge_command(CommandStatus::Success, 1)),
        );

        assert_eq!(
            power_system.get_command_result(PowerType::Communication),
            Some(acknowledge_command(CommandStatus::Success, 1))
        );

        assert!(power_system._command_result[&PowerType::Communication].is_none());
    }

    #[test]
    fn test_has_command_result() {
        let mut power_system = create_power_system().0;

        // No command result
        assert!(!power_system.has_command_result());

        // Has the command result
        power_system._command_result.insert(
            PowerType::Communication,
            Some(acknowledge_command(CommandStatus::Success, 1)),
        );

        assert!(power_system.has_command_result());
    }

    #[test]
    fn test_is_powered_on() {
        let mut power_system = create_power_system().0;

        // Motor
        assert!(!power_system.is_powered_on(PowerType::Motor));

        let sub_power_system_motor = power_system.subsystem.get_mut(&PowerType::Motor).unwrap();
        sub_power_system_motor.is_power_on = true;
        sub_power_system_motor.state = PowerSystemState::PoweredOn;

        assert!(power_system.is_powered_on(PowerType::Motor));

        // Communication
        assert!(!power_system.is_powered_on(PowerType::Communication));

        let sub_power_system_communication = power_system
            .subsystem
            .get_mut(&PowerType::Communication)
            .unwrap();
        sub_power_system_communication.is_power_on = true;
        sub_power_system_communication.state = PowerSystemState::PoweredOn;

        assert!(power_system.is_powered_on(PowerType::Communication));
    }

    #[test]
    fn test_power_off() {
        let (mut power_system, mut plant) = create_power_system();

        power_system.power_on(PowerType::Motor, 1);
        assert!(power_system._command_status[&PowerType::Motor].is_some());

        assert_eq!(
            power_system.power_off(PowerType::Motor, 2),
            Some(PowerSystemState::PoweringOff)
        );
        assert!(!power_system.subsystem[&PowerType::Motor].is_power_on);
        assert!(power_system._command_status[&PowerType::Motor].is_some());
        assert!(
            !power_system._command_status[&PowerType::Motor]
                .as_ref()
                .unwrap()
                .is_power_on
        );

        assert_eq!(
            run_until_done(PowerType::Motor, &mut power_system, &mut plant),
            Some(acknowledge_command(CommandStatus::Success, 2))
        );

        assert_eq!(
            power_system.event_queue.get_events_and_clear(),
            vec![
                Event::get_message_power_system_state(
                    PowerType::Motor,
                    true,
                    PowerSystemState::PoweringOn
                ),
                Event::get_message_power_system_state(
                    PowerType::Motor,
                    false,
                    PowerSystemState::PoweringOff
                ),
                Event::get_message_power_system_state(
                    PowerType::Motor,
                    false,
                    PowerSystemState::PoweredOff
                )
            ]
        );
    }

    #[test]
    fn test_is_powered_off() {
        let mut power_system = create_power_system().0;

        // Motor
        assert!(!power_system.is_powered_off(PowerType::Motor));

        let sub_power_system_motor = power_system.subsystem.get_mut(&PowerType::Motor).unwrap();
        sub_power_system_motor.is_power_on = false;
        sub_power_system_motor.state = PowerSystemState::PoweredOff;

        assert!(power_system.is_powered_off(PowerType::Motor));

        // Communication
        assert!(!power_system.is_powered_off(PowerType::Communication));

        let sub_power_system_communication = power_system
            .subsystem
            .get_mut(&PowerType::Communication)
            .unwrap();
        sub_power_system_communication.is_power_on = false;
        sub_power_system_communication.state = PowerSystemState::PoweredOff;

        assert!(power_system.is_powered_off(PowerType::Communication));
    }

    #[test]
    fn test_reset_breakers() {
        let (mut power_system, mut plant) = create_power_system();
        init_default_digital_output(&mut plant);

        // Not powered on
        assert!(power_system
            .reset_breakers(PowerType::Communication, 1)
            .is_none());

        // Power on
        power_system.power_on(PowerType::Communication, 1);

        assert_eq!(
            run_until_done(PowerType::Communication, &mut power_system, &mut plant),
            Some(acknowledge_command(CommandStatus::Success, 1))
        );

        assert!(plant.digital_output & DigitalOutput::ResetCommunicationBreakers.bit_value() != 0);

        // Reset the breakers
        assert_eq!(
            power_system.reset_breakers(PowerType::Communication, 2),
            Some(PowerSystemState::ResettingBreakers)
        );
        apply_actions_to_plant(&power_system.get_actions_and_clear(), &mut plant);

        assert!(plant.digital_output & DigitalOutput::ResetCommunicationBreakers.bit_value() == 0);
        assert!(!plant.power_system_communication.is_breaker_on);
        assert_eq!(plant.get_digital_input(), TEST_DIGITAL_INPUT_NO_POWER);

        assert_eq!(
            run_until_done(PowerType::Communication, &mut power_system, &mut plant),
            Some(acknowledge_command(CommandStatus::Success, 2))
        );

        assert!(plant.digital_output & DigitalOutput::ResetCommunicationBreakers.bit_value() != 0);
        assert!(plant.power_system_communication.is_breaker_on);

        // Enable the breaker
        let mock_power_system = &mut plant.power_system_communication;
        run_until_breaker_enabled(mock_power_system);

        assert_eq!(plant.get_digital_input(), TEST_DIGITAL_INPUT_POWER_COMM);

        assert_eq!(
            power_system.event_queue.get_events_and_clear(),
            vec![
                Event::get_message_power_system_state(
                    PowerType::Communication,
                    true,
                    PowerSystemState::PoweringOn
                ),
                Event::get_message_power_system_state(
                    PowerType::Communication,
                    true,
                    PowerSystemState::ResettingBreakers
                ),
                Event::get_message_power_system_state(
                    PowerType::Communication,
                    true,
                    PowerSystemState::PoweredOn
                ),
                Event::get_message_power_system_state(
                    PowerType::Communication,
                    true,
                    PowerSystemState::ResettingBreakers
                ),
                Event::get_message_power_system_state(
                    PowerType::Communication,
                    true,
                    PowerSystemState::PoweredOn
                )
            ]
        );
    }

    #[test]
    fn test_check_power_supply_error() {
        let mut power_system = create_power_system().0;

        // No error
        power_system.check_power_supply_error(
            TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
            TEST_DIGITAL_INPUT_POWER_COMM_MOTOR,
        );

        assert!(!power_system._has_fault_power_supply);
        assert!(!power_system._has_fault_interlock);

        // Has the interlock error
        power_system.check_power_supply_error(
            TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
            TEST_DIGITAL_INPUT_NO_POWER
                - DigitalInput::RedundancyOK.bit_value()
                - DigitalInput::PowerSupplyDC1OK.bit_value(),
        );

        assert!(power_system._has_fault_power_supply);
        assert!(power_system._has_fault_interlock);
    }

    #[test]
    fn test_process_telemetry_data() {
        let power_system = create_power_system().0;

        let mut telemetry = TelemetryPower::new();
        telemetry.power_raw.insert(String::from("commCurrent"), 1.0);
        telemetry
            .power_raw
            .insert(String::from("motorCurrent"), 1.0);

        power_system.process_telemetry_data(&mut telemetry);

        assert_eq!(telemetry.power_processed["commCurrent"], 1.0);
        assert_eq!(telemetry.power_processed["motorCurrent"], 5.0);
    }
}
