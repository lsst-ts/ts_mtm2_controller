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

use crate::command::command_schema::Command;
use crate::control::control_loop::ControlLoop;
use crate::controller::Controller;
use crate::enums::{DigitalOutput, DigitalOutputStatus, PowerType};
use crate::power::power_system::PowerSystem;
use crate::utility::get_message_sequence_id;

/// Command to power on/off the system.
pub struct CommandPower;
impl Command for CommandPower {
    fn name(&self) -> &str {
        "cmd_power"
    }

    fn execute(
        &self,
        message: &Value,
        power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let discriminant = message["powerType"].as_u64()?;
        let power_type = PowerType::from_repr(discriminant as u8)?;

        let system = power_system?;
        let sequence_id = get_message_sequence_id(message);
        if message["status"].as_bool()? {
            system.power_on(power_type, sequence_id)?;
        } else {
            system.power_off(power_type, sequence_id)?;
        }

        Some(())
    }
}

/// Command to reset the breakers.
pub struct CommandResetBreakers;
impl Command for CommandResetBreakers {
    fn name(&self) -> &str {
        "cmd_resetBreakers"
    }

    fn execute(
        &self,
        message: &Value,
        power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let discriminant = message["powerType"].as_u64()?;
        let power_type = PowerType::from_repr(discriminant as u8)?;

        let system = power_system?;
        system.reset_breakers(power_type, get_message_sequence_id(message))?;

        Some(())
    }
}

/// Command to toggle the bit of closed loop control.
pub struct CommandToggleBitClosedLoopControl;
impl Command for CommandToggleBitClosedLoopControl {
    fn name(&self) -> &str {
        "cmd_toggleBitClosedLoopControl"
    }

    fn execute(
        &self,
        message: &Value,
        power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let system = power_system?;
        system.is_closed_loop_control = message["status"].as_bool()?;

        Some(())
    }
}

/// Command to switch the digital output.
pub struct CommandSwitchDigitalOutput;
impl Command for CommandSwitchDigitalOutput {
    fn name(&self) -> &str {
        "cmd_switchDigitalOutput"
    }

    fn execute(
        &self,
        message: &Value,
        power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let bit = message["bit"].as_u64()?;
        let digital_output = DigitalOutput::from_repr(bit as u8)?;

        let status = message["status"].as_u64()?;
        let digital_output_status = DigitalOutputStatus::from_repr(status as u8)?;

        let system = power_system?;
        system.switch_digital_output(digital_output, digital_output_status);

        Some(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::path::Path;

    use crate::enums::{BitEnum, PowerSystemState};
    use crate::mock::mock_constants::{PLANT_VOLTAGE, TEST_DIGITAL_INPUT_POWER_COMM};
    use crate::mock::mock_plant::MockPlant;
    use crate::utility::read_file_stiffness;

    fn create_power_system() -> PowerSystem {
        // Plant model
        let filepath = Path::new("config/stiff_matrix_m2.yaml");
        let stiffness = read_file_stiffness(filepath);
        let plant = MockPlant::new(&stiffness, 0.0);

        PowerSystem::new(Some(plant))
    }

    fn run_until_done(
        power_type: PowerType,
        power_system: &mut PowerSystem,
        voltage: f64,
        digital_input: u32,
    ) {
        loop {
            power_system
                .transition_state(
                    power_type,
                    voltage,
                    power_system.get_digital_output(),
                    digital_input,
                )
                .0;

            if power_system.has_command_result() {
                break;
            }
        }
    }

    #[test]
    fn test_command_power() {
        let mut power_system = create_power_system();

        let command = CommandPower;

        assert_eq!(command.name(), "cmd_power");

        // Power on
        assert!(command
            .execute(
                &json!({"powerType": 1, "status": true, "sequence_id": 1}),
                Some(&mut power_system),
                None,
                None
            )
            .is_some());

        assert!(power_system.subsystem[&PowerType::Motor].is_power_on);

        // Second time to power on should fail
        assert!(command
            .execute(
                &json!({"powerType": 1, "status": true, "sequence_id": 2}),
                Some(&mut power_system),
                None,
                None
            )
            .is_none());

        // Power off
        power_system.is_closed_loop_control = true;

        assert!(command
            .execute(
                &json!({"powerType": 1, "status": false}),
                Some(&mut power_system),
                None,
                None
            )
            .is_some());

        assert!(!power_system.subsystem[&PowerType::Motor].is_power_on);
        assert!(!power_system.is_closed_loop_control);
    }

    #[test]
    fn test_command_reset_breakers() {
        let mut power_system = create_power_system();
        power_system.power_on(PowerType::Communication, 1);

        // Note we put the final voltage and digital input values here to get
        // the command done as soon as possible.
        run_until_done(
            PowerType::Communication,
            &mut power_system,
            PLANT_VOLTAGE,
            TEST_DIGITAL_INPUT_POWER_COMM,
        );

        let command = CommandResetBreakers;

        assert_eq!(command.name(), "cmd_resetBreakers");
        assert!(command
            .execute(
                &json!({"powerType": 2, "sequence_id": 1}),
                Some(&mut power_system),
                None,
                None
            )
            .is_some());

        assert_eq!(
            power_system.subsystem[&PowerType::Communication].state,
            PowerSystemState::ResettingBreakers,
        );

        // Second time should fail
        assert!(command
            .execute(
                &json!({"powerType": 2, "sequence_id": 2}),
                Some(&mut power_system),
                None,
                None
            )
            .is_none());
    }

    #[test]
    fn test_command_toggle_bit_closed_loop_control() {
        let mut power_system = create_power_system();

        let command = CommandToggleBitClosedLoopControl;

        assert_eq!(command.name(), "cmd_toggleBitClosedLoopControl");
        assert!(command
            .execute(
                &json!({"status": true}),
                Some(&mut power_system),
                None,
                None
            )
            .is_some());

        assert!(power_system.is_closed_loop_control);
    }

    #[test]
    fn test_command_switch_digital_output() {
        let mut power_system = create_power_system();

        let command = CommandSwitchDigitalOutput;

        assert_eq!(command.name(), "cmd_switchDigitalOutput");
        assert!(command
            .execute(
                &json!({"bit": 1, "status": 2}),
                Some(&mut power_system),
                None,
                None
            )
            .is_some());

        assert!(power_system.get_digital_output() & DigitalOutput::MotorPower.bit_value() != 0);
    }
}
