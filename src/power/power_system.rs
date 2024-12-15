use crate::enums::{DigitalOutput, DigitalOutputStatus, PowerSystemState, PowerType};
use crate::event_queue::EventQueue;
use crate::mock::mock_plant::MockPlant;
use crate::power::{config_power::ConfigPower, sub_power_system::SubPowerSystem};
use crate::telemetry::event::Event;
use crate::telemetry::telemetry_power::TelemetryPower;

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
        Self {
            config: ConfigPower::new(),

            system_motor: SubPowerSystem::new(),
            system_communication: SubPowerSystem::new(),

            is_closed_loop_control: false,

            event_queue: EventQueue::new(),

            _plant: plant,
        }
    }

    /// Power on the system.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    ///
    /// # Returns
    /// Power system state.
    pub fn power_on(&mut self, power_type: PowerType) -> PowerSystemState {
        // Update the internal state
        let power_system = self.get_system_mut(power_type);

        // Return the current state if the power is already on
        if power_system.is_power_on && (power_system.state == PowerSystemState::PoweredOn) {
            return power_system.state;
        }

        power_system.is_power_on = true;
        power_system.state = PowerSystemState::PoweringOn;

        let state = power_system.state;

        // Add an event
        self.event_queue
            .add_event(Event::get_message_power_system_state(
                power_type, true, state,
            ));

        // Update the plant model
        if let Some(plant) = &mut self._plant {
            if power_type == PowerType::Motor {
                plant.is_power_on_motor = true;
                plant.switch_digital_output(
                    DigitalOutput::MotorPower,
                    DigitalOutputStatus::BinaryHighLevel,
                );
            } else {
                plant.is_power_on_communication = true;
                plant.switch_digital_output(
                    DigitalOutput::CommunicationPower,
                    DigitalOutputStatus::BinaryHighLevel,
                );
            }
        }

        state
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

    /// Power off the system.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    ///
    /// # Returns
    /// Power system state.
    pub fn power_off(&mut self, power_type: PowerType) -> PowerSystemState {
        // Update the internal state
        let power_system = self.get_system_mut(power_type);

        // Return the current state if the power is already off
        if (!power_system.is_power_on) && (power_system.state == PowerSystemState::PoweredOff) {
            return power_system.state;
        }

        power_system.is_power_on = false;
        power_system.state = PowerSystemState::PoweringOff;

        let state = power_system.state;

        // Add an event
        self.event_queue
            .add_event(Event::get_message_power_system_state(
                power_type, false, state,
            ));

        // Update the plant model
        if let Some(plant) = &mut self._plant {
            if power_type == PowerType::Motor {
                plant.is_power_on_motor = false;
                plant.switch_digital_output(
                    DigitalOutput::MotorPower,
                    DigitalOutputStatus::BinaryLowLevel,
                );
            } else {
                plant.is_power_on_communication = false;
                plant.switch_digital_output(
                    DigitalOutput::CommunicationPower,
                    DigitalOutputStatus::BinaryLowLevel,
                );
            }
        }

        state
    }

    /// Reset breakers.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    ///
    /// # Returns
    /// Power system state.
    pub fn reset_breakers(&mut self, power_type: PowerType) -> PowerSystemState {
        // Update the internal state
        let power_system = self.get_system_mut(power_type);
        let is_power_on = power_system.is_power_on;
        if is_power_on {
            power_system.state = PowerSystemState::ResettingBreakers;
        }

        let state = power_system.state;

        // Add an event
        self.event_queue
            .add_event(Event::get_message_power_system_state(
                power_type,
                is_power_on,
                state,
            ));

        // Update the plant model
        if let Some(plant) = &mut self._plant {
            if power_type == PowerType::Motor {
                plant.is_power_on_motor = false;
                plant.switch_digital_output(
                    DigitalOutput::ResetMotorBreakers,
                    DigitalOutputStatus::BinaryLowLevel,
                );
            } else {
                plant.is_power_on_communication = false;
                plant.switch_digital_output(
                    DigitalOutput::ResetCommunicationBreakers,
                    DigitalOutputStatus::BinaryLowLevel,
                );
            }
        }

        state
    }

    /// Transition the state of the power system.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    ///
    /// # Returns
    /// Power system state.
    pub fn transition_state(&mut self, power_type: PowerType) -> PowerSystemState {
        let mut is_resetting_breakers = false;

        // Update the internal state
        let power_system = self.get_system_mut(power_type);

        let mut is_state_changed = false;
        if power_system.state == PowerSystemState::PoweringOn {
            power_system.state = PowerSystemState::PoweredOn;

            is_state_changed = true;
        } else if power_system.state == PowerSystemState::PoweringOff {
            power_system.state = PowerSystemState::PoweredOff;

            is_state_changed = true;
        } else if power_system.state == PowerSystemState::ResettingBreakers {
            power_system.state = PowerSystemState::PoweredOn;

            is_resetting_breakers = true;
            is_state_changed = true;
        }

        // Add an event
        let is_power_on = power_system.is_power_on;
        let state = power_system.state;
        if is_state_changed {
            self.event_queue
                .add_event(Event::get_message_power_system_state(
                    power_type,
                    is_power_on,
                    state,
                ));
        }

        // Update the plant model
        if is_resetting_breakers {
            if let Some(plant) = &mut self._plant {
                if power_type == PowerType::Motor {
                    plant.is_power_on_motor = true;
                    plant.switch_digital_output(
                        DigitalOutput::ResetMotorBreakers,
                        DigitalOutputStatus::BinaryHighLevel,
                    );
                } else {
                    plant.is_power_on_communication = true;
                    plant.switch_digital_output(
                        DigitalOutput::ResetCommunicationBreakers,
                        DigitalOutputStatus::BinaryHighLevel,
                    );
                }
            }
        }

        state
    }

    /// Get the telemetry data.
    ///
    /// # Returns
    /// Telemetry data.
    pub fn get_telemetry_data(&self) -> TelemetryPower {
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
    fn get_power(&self, power_type: PowerType) -> (f64, f64) {
        if let Some(plant) = &self._plant {
            if power_type == PowerType::Motor {
                return plant.get_power_motor();
            } else {
                return plant.get_power_communication();
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
        if status == DigitalOutputStatus::BinaryLowLevel {
            if digital_output == DigitalOutput::MotorPower {
                self.power_off(PowerType::Motor);
            } else if digital_output == DigitalOutput::CommunicationPower {
                self.power_off(PowerType::Communication);
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
        TEST_DIGITAL_INPUT_NO_POWER, TEST_DIGITAL_INPUT_POWER_COMM, TEST_DIGITAL_OUTPUT_POWER_COMM,
    };
    use crate::utility::read_file_stiffness;

    fn create_power_system() -> PowerSystem {
        // Plant model
        let filepath = Path::new("config/stiff_matrix_m2.yaml");
        let stiffness = read_file_stiffness(filepath);
        let plant = MockPlant::new(&stiffness, 0.0);

        PowerSystem::new(Some(plant))
    }

    #[test]
    fn test_power_on() {
        let mut power_system = create_power_system();
        assert!(!power_system.system_motor.is_power_on);

        assert_eq!(
            power_system.power_on(PowerType::Motor),
            PowerSystemState::PoweringOn
        );
        assert!(power_system.system_motor.is_power_on);

        assert_eq!(
            power_system.transition_state(PowerType::Motor),
            PowerSystemState::PoweredOn
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
                    PowerSystemState::PoweredOn
                )
            ]
        );

        assert!(power_system._plant.as_ref().unwrap().is_power_on_motor);
    }

    #[test]
    fn test_power_off() {
        let mut power_system = create_power_system();

        power_system.power_on(PowerType::Motor);

        assert_eq!(
            power_system.power_off(PowerType::Motor),
            PowerSystemState::PoweringOff
        );
        assert!(!power_system.system_motor.is_power_on);

        assert_eq!(
            power_system.transition_state(PowerType::Motor),
            PowerSystemState::PoweredOff
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

        assert!(!power_system._plant.as_ref().unwrap().is_power_on_motor);
    }

    #[test]
    fn test_reset_breakers() {
        let mut power_system = create_power_system();

        power_system.power_on(PowerType::Communication);

        assert!(
            power_system.get_digital_output()
                & DigitalOutput::ResetCommunicationBreakers.bit_value()
                != 0
        );

        // Reset the breakers
        assert_eq!(
            power_system.reset_breakers(PowerType::Communication),
            PowerSystemState::ResettingBreakers
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
                .is_power_on_communication
        );
        assert_eq!(
            power_system.get_digital_input(),
            TEST_DIGITAL_INPUT_NO_POWER
        );

        // Transition the state
        assert_eq!(
            power_system.transition_state(PowerType::Communication),
            PowerSystemState::PoweredOn
        );

        assert!(
            power_system.get_digital_output() & DigitalOutput::ResetMotorBreakers.bit_value() != 0
        );
        assert!(
            power_system
                ._plant
                .as_ref()
                .unwrap()
                .is_power_on_communication
        );
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
                )
            ]
        );
    }

    #[test]
    fn test_get_power() {
        let mut power_system = create_power_system();

        // Power is off
        let (voltage, current) = power_system.get_power(PowerType::Motor);

        assert_eq!(voltage, 0.0);
        assert_eq!(current, 0.0);

        // Power is on
        power_system.power_on(PowerType::Motor);
        let (voltage, current) = power_system.get_power(PowerType::Motor);

        assert_ne!(voltage, 0.0);
        assert_ne!(current, 0.0);
    }

    #[test]
    fn test_get_telemetry_data() {
        let mut power_system = create_power_system();

        // Power on the communication
        power_system.power_on(PowerType::Communication);

        let telemetry = power_system.get_telemetry_data();

        assert_eq!(telemetry.digital_output, TEST_DIGITAL_OUTPUT_POWER_COMM);
        assert_eq!(telemetry.digital_input, TEST_DIGITAL_INPUT_POWER_COMM);

        assert_eq!(
            telemetry.power_processed["motorCurrent"],
            power_system.get_power(PowerType::Motor).1
        );

        // Power on the motor
        power_system.power_on(PowerType::Motor);

        let telemetry = power_system.get_telemetry_data();

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
        power_system.power_on(PowerType::Communication);

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
                .is_power_on_communication
        );
    }
}
