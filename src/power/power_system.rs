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
use serde_json::Value;

use crate::enums::{
    BitEnum, CommandStatus, DigitalOutput, DigitalOutputStatus, PowerSystemState, PowerType,
};
use crate::event_queue::EventQueue;
use crate::mock::mock_plant::MockPlant;
use crate::power::{config_power::ConfigPower, sub_power_system::SubPowerSystem};
use crate::telemetry::event::Event;
use crate::telemetry::telemetry_power::TelemetryPower;
use crate::utility::acknowledge_command;

pub struct PowerCommandStatus {
    // Power type
    pub power_type: PowerType,
    // Sequence ID of the command, put -1 for the internal command.
    pub sequence_id: i64,
    // The final power status is expected to be on or off
    pub is_power_on: bool,
}

pub struct PowerSystem {
    // Configuration of the power system
    pub config: ConfigPower,
    // Motor power system
    pub system_motor: SubPowerSystem,
    // Communication power system
    pub system_communication: SubPowerSystem,
    // System is under the closed-loop control or not
    pub is_closed_loop_control: bool,
    // Events to publish
    pub event_queue: EventQueue,
    // Power command status
    _command_status: Option<PowerCommandStatus>,
    // Power command result
    _command_result: Option<Value>,
    // Plant model
    _plant: Option<MockPlant>,
}

impl PowerSystem {
    /// Create a new instance of the power system.
    ///
    /// # Arguments
    /// * `plant` - Plant model. Put None if the hardware mode is applied.
    ///
    /// # Returns
    /// New instance of the power system.
    pub fn new(plant: Option<MockPlant>) -> Self {
        let config_power = ConfigPower::new();

        Self {
            system_motor: SubPowerSystem::new(
                PowerType::Motor,
                config_power.output_voltage_off_level,
                config_power.breaker_operating_voltage,
                config_power.loop_time as i32,
                config_power.get_time_power_on(PowerType::Motor),
                config_power.get_time_power_off(PowerType::Motor),
                config_power.get_time_breaker_on(PowerType::Motor),
                config_power.get_time_breaker_off(PowerType::Motor),
            ),
            system_communication: SubPowerSystem::new(
                PowerType::Communication,
                config_power.output_voltage_off_level,
                config_power.breaker_operating_voltage,
                config_power.loop_time as i32,
                config_power.get_time_power_on(PowerType::Communication),
                config_power.get_time_power_off(PowerType::Communication),
                config_power.get_time_breaker_on(PowerType::Communication),
                config_power.get_time_breaker_off(PowerType::Communication),
            ),

            config: config_power,

            is_closed_loop_control: false,

            event_queue: EventQueue::new(),

            _command_status: None,
            _command_result: None,

            _plant: plant,
        }
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
        // If there is the ongoing power command, reject the new power
        // command.
        if self._command_status.is_some() {
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
        self._command_status = Some(PowerCommandStatus {
            power_type: power_type,
            sequence_id: sequence_id,
            is_power_on: true,
        });

        // Power on the system and track the status
        let actions;
        let state;
        if power_type == PowerType::Motor {
            actions = self.system_motor.power_on();
            state = self.system_motor.state;
        } else {
            actions = self.system_communication.power_on();
            state = self.system_communication.state;
        }

        self.apply_actions_to_hardware(&actions);

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
    fn is_powered_on(&mut self, power_type: PowerType) -> bool {
        let power_system = self.get_system_mut(power_type);
        if power_system.is_power_on && (power_system.state == PowerSystemState::PoweredOn) {
            return true;
        }

        false
    }

    /// Get the mutable power system.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    ///
    /// # Returns
    /// Mutable power system.
    fn get_system_mut(&mut self, power_type: PowerType) -> &mut SubPowerSystem {
        if power_type == PowerType::Motor {
            &mut self.system_motor
        } else {
            &mut self.system_communication
        }
    }

    /// Apply the actions to the hardware.
    ///
    /// # Arguments
    /// * `actions` - Actions to apply.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn apply_actions_to_hardware(&mut self, actions: &[(DigitalOutput, DigitalOutputStatus)]) {
        // Update the plant model
        if let Some(plant) = &mut self._plant {
            actions.iter().for_each(|(digital_output, status)| {
                plant.switch_digital_output(*digital_output, *status);
            });
        } else {
            // Update the hardware
            panic!("Not implemented yet.");
        }
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
        // command. The same thing if we are powering on/off the another power
        // system.
        if let Some(command_status) = &self._command_status {
            if (!command_status.is_power_on) || (command_status.power_type != power_type) {
                error!(
                    "Cannot power off the {:?} power system if there is the ongoing power command.",
                    power_type
                );

                return None;
            }
        }

        // Reset the closed-loop control bit
        self.is_closed_loop_control = false;

        // Return the current state if the power is already off
        if self.is_powered_off(power_type) {
            return Some(PowerSystemState::PoweredOff);
        }

        // Track the power command status. Note we can power off the system even
        // there is an ongoing power-on command for the same power type.
        self._command_status = Some(PowerCommandStatus {
            power_type: power_type,
            sequence_id: sequence_id,
            is_power_on: false,
        });

        // Power off the system
        let actions;
        let state;
        if power_type == PowerType::Motor {
            actions = self.system_motor.power_off();
            state = self.system_motor.state;
        } else {
            actions = self.system_communication.power_off();
            state = self.system_communication.state;
        }

        self.apply_actions_to_hardware(&actions);

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
    fn is_powered_off(&mut self, power_type: PowerType) -> bool {
        let power_system = self.get_system_mut(power_type);
        if (!power_system.is_power_on) && (power_system.state == PowerSystemState::PoweredOff) {
            return true;
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
        if self._command_status.is_some() {
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
        self._command_status = Some(PowerCommandStatus {
            power_type: power_type,
            sequence_id: sequence_id,
            is_power_on: true,
        });

        // Reset the breakers
        let actions;
        let state;
        if power_type == PowerType::Motor {
            actions = self
                .system_motor
                .reset_breakers(DigitalOutputStatus::BinaryLowLevel);
            state = self.system_motor.state;
        } else {
            actions = self
                .system_communication
                .reset_breakers(DigitalOutputStatus::BinaryLowLevel);
            state = self.system_communication.state;
        }

        self.apply_actions_to_hardware(&actions);

        // Add an event
        self.event_queue
            .add_event(Event::get_message_power_system_state(
                power_type, true, state,
            ));

        Some(state)
    }

    /// Transition the state of the power system.
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
        let is_state_changed;
        let has_error;
        let actions;

        let is_power_on;
        let state;
        if power_type == PowerType::Motor {
            (is_state_changed, has_error, actions) =
                self.system_motor
                    .transition_state(voltage, digital_output, digital_input);

            is_power_on = self.system_motor.is_power_on;
            state = self.system_motor.state;
        } else {
            (is_state_changed, has_error, actions) =
                self.system_communication
                    .transition_state(voltage, digital_output, digital_input);

            is_power_on = self.system_communication.is_power_on;
            state = self.system_communication.state;
        }

        // Apply the actions to the hardware
        self.apply_actions_to_hardware(&actions);

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
                if let Some(command_final_status) = &mut self._command_status {
                    if command_final_status.power_type == power_type {
                        // We only need to check the power command status with
                        // is_power_on=true here. If this field is false, it
                        // will always work.
                        let mut command_status = CommandStatus::Success;
                        if command_final_status.is_power_on {
                            command_status = if is_power_on {
                                CommandStatus::Success
                            } else {
                                CommandStatus::Fail
                            };
                        }

                        command_result =
                            acknowledge_command(command_status, command_final_status.sequence_id);
                    }
                }
            }

            if !command_result.is_null() {
                self._command_result = Some(command_result);
                self._command_status = None;
            }
        }

        (is_state_changed, has_error)
    }

    /// Get the command result.
    ///
    /// # Returns
    /// The command result.
    pub fn get_command_result(&mut self) -> Option<Value> {
        if self._command_result.is_none() {
            return None;
        }

        let command_result = self._command_result.clone();
        self._command_result = None;

        command_result
    }

    /// Check if there is a command result.
    ///
    /// # Returns
    /// True if there is a command result, false otherwise.
    pub fn has_command_result(&self) -> bool {
        self._command_result.is_some()
    }

    /// Get the telemetry data.
    ///
    /// # Returns
    /// Telemetry data.
    pub fn get_telemetry_data(&mut self) -> TelemetryPower {
        let mut telemetry = TelemetryPower::new();

        // Raw power data
        let (comm_voltage, comm_current) = self.get_power(PowerType::Communication);
        telemetry
            .power_raw
            .insert(String::from("commVoltage"), comm_voltage);
        telemetry
            .power_raw
            .insert(String::from("commCurrent"), comm_current);

        let (motor_voltage, motor_current) = self.get_power(PowerType::Motor);
        telemetry
            .power_raw
            .insert(String::from("motorVoltage"), motor_voltage);
        telemetry
            .power_raw
            .insert(String::from("motorCurrent"), motor_current);

        // Process the raw power data
        self.process_power_data(&mut telemetry);

        // Digital output and input
        telemetry.digital_output = self.get_digital_output();
        telemetry.digital_input = self.get_digital_input();

        telemetry
    }

    /// Get the power.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    ///
    /// # Returns
    /// A tuple of the power. The first element is the voltage in volt and the
    /// second element is the current in ampere.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn get_power(&mut self, power_type: PowerType) -> (f64, f64) {
        if let Some(plant) = &mut self._plant {
            if power_type == PowerType::Motor {
                return plant.power_system_motor.get_voltage_and_current();
            } else {
                return plant.power_system_communication.get_voltage_and_current();
            }
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Process the power data.
    ///
    /// # Arguments
    /// * `telemetry` - Telemetry data.
    fn process_power_data(&self, telemetry: &mut TelemetryPower) {
        let mut power_processed = telemetry.power_raw.clone();

        // Only need to calibrate the motor current.
        let motor_current_calibrated = self.config.current_gain_motor
            * power_processed["motorCurrent"]
            + self.config.current_offset_motor;

        power_processed.insert(String::from("motorCurrent"), motor_current_calibrated);

        telemetry.power_processed = power_processed;
    }

    /// Get the digital output.
    ///
    /// # Returns
    /// The digital output.
    ///
    /// # Panics
    /// If not in simulation mode.
    pub fn get_digital_output(&self) -> u8 {
        if let Some(plant) = &self._plant {
            return plant.digital_output;
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Get the digital input.
    ///
    /// # Returns
    /// The digital input.
    ///
    /// # Panics
    /// If not in simulation mode.
    pub fn get_digital_input(&self) -> u32 {
        if let Some(plant) = &self._plant {
            return plant.get_digital_input();
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Toggle the bit for the closed-loop control. This signal is used by the
    /// safety module to communicate with the global interlock system (GIS).
    ///
    /// # Panics
    /// If not in simulation mode.
    pub fn toggle_bit_closed_loop_control(&mut self) {
        if let Some(plant) = &mut self._plant {
            if self.is_closed_loop_control {
                plant.switch_digital_output(
                    DigitalOutput::ClosedLoopControl,
                    DigitalOutputStatus::ToggleBit,
                );
            }
        } else {
            panic!("Not implemented yet.");
        }
    }

    /// Initialize the default digital output.
    pub fn init_default_digital_output(&mut self) {
        [
            DigitalOutput::InterlockEnable,
            DigitalOutput::ResetMotorBreakers,
            DigitalOutput::ResetCommunicationBreakers,
        ]
        .iter()
        .for_each(|digital_output| {
            self.switch_digital_output(*digital_output, DigitalOutputStatus::BinaryHighLevel);
        });
    }

    /// Switch the digital output.
    ///
    /// # Arguments
    /// * `digital_output` - Digital output.
    /// * `status` - Digital output status.
    ///
    /// # Panics
    /// If not in simulation mode.
    pub fn switch_digital_output(
        &mut self,
        digital_output: DigitalOutput,
        status: DigitalOutputStatus,
    ) {
        if let Some(plant) = &mut self._plant {
            plant.switch_digital_output(digital_output, status);
        } else {
            panic!("Not implemented yet.");
        }

        // Turn off the power based on the bit value.
        let plant_digital_output_value = self.get_digital_output();
        if digital_output == DigitalOutput::MotorPower {
            if plant_digital_output_value & DigitalOutput::MotorPower.bit_value() == 0 {
                self.power_off(PowerType::Motor, -1);
            }
        } else if digital_output == DigitalOutput::CommunicationPower {
            if plant_digital_output_value & DigitalOutput::CommunicationPower.bit_value() == 0 {
                self.power_off(PowerType::Communication, -1);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::path::Path;

    use crate::enums::BitEnum;
    use crate::mock::mock_constants::{
        TEST_DIGITAL_INPUT_NO_POWER, TEST_DIGITAL_INPUT_POWER_COMM, TEST_DIGITAL_OUTPUT_NO_POWER,
        TEST_DIGITAL_OUTPUT_POWER_COMM,
    };
    use crate::mock::mock_power_system::MockPowerSystem;
    use crate::utility::read_file_stiffness;

    fn create_power_system() -> PowerSystem {
        // Plant model
        let filepath = Path::new("config/stiff_matrix_m2.yaml");
        let stiffness = read_file_stiffness(filepath);
        let plant = MockPlant::new(&stiffness, 0.0);

        PowerSystem::new(Some(plant))
    }

    fn run_until_breaker_enabled(power_system: &mut MockPowerSystem) {
        loop {
            if power_system.is_breaker_enabled() {
                break;
            }

            power_system.get_voltage_and_current();
        }
    }

    fn run_until_done(power_type: PowerType, power_system: &mut PowerSystem) -> Option<Value> {
        loop {
            let voltage = power_system.get_power(power_type).0;
            power_system
                .transition_state(
                    power_type,
                    voltage,
                    power_system.get_digital_output(),
                    power_system.get_digital_input(),
                )
                .0;

            if power_system.has_command_result() {
                break;
            }
        }

        power_system.get_command_result()
    }

    #[test]
    fn test_power_on() {
        let mut power_system = create_power_system();
        assert!(!power_system.system_motor.is_power_on);

        assert_eq!(
            power_system.power_on(PowerType::Motor, 1),
            Some(PowerSystemState::PoweringOn)
        );
        assert!(power_system.system_motor.is_power_on);
        assert!(power_system._command_status.is_some());
        assert!(power_system._command_status.as_ref().unwrap().is_power_on);

        assert_eq!(
            run_until_done(PowerType::Motor, &mut power_system),
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
    fn test_get_command_result() {
        let mut power_system = create_power_system();

        // No command result
        assert_eq!(power_system.get_command_result(), None);

        // Has the command result
        power_system._command_result = Some(acknowledge_command(CommandStatus::Success, 1));

        assert_eq!(
            power_system.get_command_result(),
            Some(acknowledge_command(CommandStatus::Success, 1))
        );

        assert!(power_system._command_result.is_none());
    }

    #[test]
    fn test_has_command_result() {
        let mut power_system = create_power_system();

        // No command result
        assert!(!power_system.has_command_result());

        // Has the command result
        power_system._command_result = Some(acknowledge_command(CommandStatus::Success, 1));

        assert!(power_system.has_command_result());
    }

    #[test]
    fn test_is_powered_on() {
        let mut power_system = create_power_system();

        // Motor
        assert!(!power_system.is_powered_on(PowerType::Motor));

        power_system.system_motor.is_power_on = true;
        power_system.system_motor.state = PowerSystemState::PoweredOn;

        assert!(power_system.is_powered_on(PowerType::Motor));

        // Communication
        assert!(!power_system.is_powered_on(PowerType::Communication));

        power_system.system_communication.is_power_on = true;
        power_system.system_communication.state = PowerSystemState::PoweredOn;

        assert!(power_system.is_powered_on(PowerType::Communication));
    }

    #[test]
    fn test_power_off() {
        let mut power_system = create_power_system();
        power_system.is_closed_loop_control = true;

        power_system.power_on(PowerType::Motor, 1);
        assert!(power_system._command_status.is_some());

        assert!(power_system
            .power_off(PowerType::Communication, -1)
            .is_none());

        assert_eq!(
            power_system.power_off(PowerType::Motor, 2),
            Some(PowerSystemState::PoweringOff)
        );
        assert!(!power_system.system_motor.is_power_on);
        assert!(power_system._command_status.is_some());
        assert!(!power_system._command_status.as_ref().unwrap().is_power_on);
        assert!(!power_system.is_closed_loop_control);

        assert_eq!(
            run_until_done(PowerType::Motor, &mut power_system),
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
        let mut power_system = create_power_system();

        // Motor
        assert!(!power_system.is_powered_off(PowerType::Motor));

        power_system.system_motor.is_power_on = false;
        power_system.system_motor.state = PowerSystemState::PoweredOff;

        assert!(power_system.is_powered_off(PowerType::Motor));

        // Communication
        assert!(!power_system.is_powered_off(PowerType::Communication));

        power_system.system_communication.is_power_on = false;
        power_system.system_communication.state = PowerSystemState::PoweredOff;

        assert!(power_system.is_powered_off(PowerType::Communication));
    }

    #[test]
    fn test_reset_breakers() {
        let mut power_system = create_power_system();
        power_system.init_default_digital_output();

        // Not powered on
        assert!(power_system
            .reset_breakers(PowerType::Communication, 1)
            .is_none());

        // Power on
        power_system.power_on(PowerType::Communication, 1);

        assert_eq!(
            run_until_done(PowerType::Communication, &mut power_system),
            Some(acknowledge_command(CommandStatus::Success, 1))
        );

        assert!(
            power_system.get_digital_output()
                & DigitalOutput::ResetCommunicationBreakers.bit_value()
                != 0
        );

        // Reset the breakers
        assert_eq!(
            power_system.reset_breakers(PowerType::Communication, 2),
            Some(PowerSystemState::ResettingBreakers)
        );

        assert!(
            power_system.get_digital_output()
                & DigitalOutput::ResetCommunicationBreakers.bit_value()
                == 0
        );
        assert!(
            !power_system
                ._plant
                .as_ref()
                .unwrap()
                .power_system_communication
                .is_breaker_on
        );
        assert_eq!(
            power_system.get_digital_input(),
            TEST_DIGITAL_INPUT_NO_POWER
        );

        assert_eq!(
            run_until_done(PowerType::Communication, &mut power_system),
            Some(acknowledge_command(CommandStatus::Success, 2))
        );

        assert!(
            power_system.get_digital_output()
                & DigitalOutput::ResetCommunicationBreakers.bit_value()
                != 0
        );
        assert!(
            power_system
                ._plant
                .as_ref()
                .unwrap()
                .power_system_communication
                .is_breaker_on
        );

        // Enable the breaker
        let mock_power_system = &mut power_system
            ._plant
            .as_mut()
            .unwrap()
            .power_system_communication;
        run_until_breaker_enabled(mock_power_system);

        assert_eq!(
            power_system.get_digital_input(),
            TEST_DIGITAL_INPUT_POWER_COMM
        );

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
    fn test_get_power() {
        let mut power_system = create_power_system();

        // Power is off
        let (mut voltage, mut current) = power_system.get_power(PowerType::Motor);

        assert_eq!(voltage, 0.0);
        assert_eq!(current, 0.0);

        // Power is on
        power_system.power_on(PowerType::Motor, 1);
        loop {
            (voltage, current) = power_system.get_power(PowerType::Motor);

            if (voltage > 0.0) && (current > 0.0) {
                break;
            }
        }

        assert_ne!(voltage, 0.0);
        assert_ne!(current, 0.0);
    }

    #[test]
    fn test_get_telemetry_data() {
        let mut power_system = create_power_system();
        power_system.init_default_digital_output();

        // Power on the communication and enable the breaker
        power_system.power_on(PowerType::Communication, 1);

        assert_eq!(
            run_until_done(PowerType::Communication, &mut power_system),
            Some(acknowledge_command(CommandStatus::Success, 1))
        );

        let mock_power_system = &mut power_system
            ._plant
            .as_mut()
            .unwrap()
            .power_system_communication;
        run_until_breaker_enabled(mock_power_system);

        // Get the telemetry data
        let mut telemetry = power_system.get_telemetry_data();

        assert_eq!(telemetry.digital_output, TEST_DIGITAL_OUTPUT_POWER_COMM);
        assert_eq!(telemetry.digital_input, TEST_DIGITAL_INPUT_POWER_COMM);

        assert_eq!(
            telemetry.power_processed["commCurrent"],
            power_system.get_power(PowerType::Communication).1
        );

        // // Power on the motor
        power_system.power_on(PowerType::Motor, 2);

        assert_eq!(
            run_until_done(PowerType::Motor, &mut power_system),
            Some(acknowledge_command(CommandStatus::Success, 2))
        );

        loop {
            telemetry = power_system.get_telemetry_data();

            if power_system.get_power(PowerType::Motor).1 > 0.0 {
                break;
            }
        }

        assert_ne!(
            telemetry.power_processed["motorCurrent"],
            power_system.get_power(PowerType::Motor).1
        );
    }

    #[test]
    fn test_toggle_bit_closed_loop_control() {
        let mut power_system = create_power_system();

        assert!(
            power_system.get_digital_output() & DigitalOutput::ClosedLoopControl.bit_value() == 0
        );

        // Not in the closed-loop control mode
        power_system.toggle_bit_closed_loop_control();

        assert!(
            power_system.get_digital_output() & DigitalOutput::ClosedLoopControl.bit_value() == 0
        );

        // In the closed-loop control mode
        power_system.is_closed_loop_control = true;

        power_system.toggle_bit_closed_loop_control();

        assert!(
            power_system.get_digital_output() & DigitalOutput::ClosedLoopControl.bit_value() != 0
        );

        power_system.toggle_bit_closed_loop_control();

        assert!(
            power_system.get_digital_output() & DigitalOutput::ClosedLoopControl.bit_value() == 0
        );
    }

    #[test]
    fn test_init_default_digital_output() {
        let mut power_system = create_power_system();
        assert_eq!(power_system.get_digital_output(), 0);

        power_system.init_default_digital_output();

        assert_eq!(
            power_system.get_digital_output(),
            TEST_DIGITAL_OUTPUT_NO_POWER
        );
    }

    #[test]
    fn test_switch_digital_output() {
        let mut power_system = create_power_system();

        // No motor power
        assert!(power_system.get_digital_output() & DigitalOutput::MotorPower.bit_value() == 0);

        power_system.switch_digital_output(
            DigitalOutput::MotorPower,
            DigitalOutputStatus::BinaryHighLevel,
        );

        assert!(power_system.get_digital_output() & DigitalOutput::MotorPower.bit_value() != 0);

        // Communication power is on
        power_system.power_on(PowerType::Communication, 1);

        power_system.switch_digital_output(
            DigitalOutput::CommunicationPower,
            DigitalOutputStatus::BinaryLowLevel,
        );

        assert!(!power_system.system_communication.is_power_on);
        assert!(
            !power_system
                ._plant
                .as_ref()
                .unwrap()
                .power_system_communication
                .is_power_on
        );
    }
}
