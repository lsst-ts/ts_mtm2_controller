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
use crate::daq::data_acquisition::DataAcquisition;
use crate::enums::{DataAcquisitionMode, DigitalOutput, DigitalOutputStatus, InnerLoopControlMode};
use crate::power::power_system::PowerSystem;

/// Command to switch the digital output.
pub struct CommandSwitchDigitalOutput;
impl Command for CommandSwitchDigitalOutput {
    fn name(&self) -> &str {
        "cmd_switchDigitalOutput"
    }

    fn execute(
        &self,
        message: &Value,
        data_acquisition: Option<&mut DataAcquisition>,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let bit = message["bit"].as_u64()?;
        let digital_output = DigitalOutput::from_repr(bit as u8)?;

        let status = message["status"].as_u64()?;
        let digital_output_status = DigitalOutputStatus::from_repr(status as u8)?;

        let system = data_acquisition?;
        system.switch_digital_output(digital_output, digital_output_status)?;

        Some(())
    }
}

/// Command to set the mode of inner-loop controller.
pub struct CommandSetInnerLoopControlMode;
impl Command for CommandSetInnerLoopControlMode {
    fn name(&self) -> &str {
        "cmd_setInnerLoopControlMode"
    }

    fn execute(
        &self,
        message: &Value,
        data_acquisition: Option<&mut DataAcquisition>,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let system = data_acquisition?;

        let addresses = message["addresses"].as_array()?;
        for address in addresses {
            let address = address.as_u64()? as usize;
            let mode = InnerLoopControlMode::from_repr(message["mode"].as_u64()? as u8)?;
            system.set_ilc_mode(address, mode)?;
        }

        Some(())
    }
}

/// Command to get the mode of inner-loop controller.
pub struct CommandGetInnerLoopControlMode;
impl Command for CommandGetInnerLoopControlMode {
    fn name(&self) -> &str {
        "cmd_getInnerLoopControlMode"
    }

    fn execute(
        &self,
        message: &Value,
        data_acquisition: Option<&mut DataAcquisition>,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let system = data_acquisition?;

        let addresses = message["addresses"].as_array()?;
        for address in addresses {
            let address = address.as_u64()? as usize;
            system.get_ilc_mode(address)?;
        }

        Some(())
    }
}

/// Command to move the actuator steps.
pub struct CommandMoveActuatorSteps;
impl Command for CommandMoveActuatorSteps {
    fn name(&self) -> &str {
        "cmd_moveActuatorSteps"
    }

    fn execute(
        &self,
        message: &Value,
        data_acquisition: Option<&mut DataAcquisition>,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let mut steps = Vec::new();
        let seq_id = message["seq_id_move_actuator_steps"].as_i64()?;
        let step_list = message["steps"].as_array()?;
        for step_item in step_list {
            let step = step_item.as_i64()?;
            steps.push(step as i32);
        }

        let system = data_acquisition?;
        system.move_actuator_steps(seq_id as i32, &steps)?;

        Some(())
    }
}

/// Command to set the data acquisition mode.
pub struct CommandSetDataAcquisitionMode;
impl Command for CommandSetDataAcquisitionMode {
    fn name(&self) -> &str {
        "cmd_setDataAcquisitionMode"
    }

    fn execute(
        &self,
        message: &Value,
        data_acquisition: Option<&mut DataAcquisition>,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let discriminant = message["mode"].as_u64()?;
        let mode = DataAcquisitionMode::from_repr(discriminant as u8)?;

        let system = data_acquisition?;
        system.set_mode(mode)?;

        Some(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    use crate::constants::NUM_ACTUATOR;
    use crate::enums::BitEnum;

    fn create_data_acquisition() -> DataAcquisition {
        DataAcquisition::new(true)
    }

    #[test]
    fn test_command_switch_digital_output() {
        let mut data_acquisition = create_data_acquisition();

        let command = CommandSwitchDigitalOutput;

        assert_eq!(command.name(), "cmd_switchDigitalOutput");
        assert!(command
            .execute(
                &json!({"bit": 1, "status": 2}),
                Some(&mut data_acquisition),
                None,
                None,
                None
            )
            .is_some());

        assert!(
            data_acquisition.plant.unwrap().digital_output & DigitalOutput::MotorPower.bit_value()
                != 0
        );
    }

    #[test]
    fn test_command_set_inner_loop_control_mode() {
        let mut data_acquisition = create_data_acquisition();

        let command = CommandSetInnerLoopControlMode;

        assert_eq!(command.name(), "cmd_setInnerLoopControlMode");

        assert!(command
            .execute(
                &json!({"addresses": [0, 1], "mode": 2}),
                Some(&mut data_acquisition),
                None,
                None,
                None
            )
            .is_some());

        assert_eq!(
            data_acquisition.plant.as_ref().unwrap().get_ilc_mode(0),
            InnerLoopControlMode::Disabled
        );
        assert_eq!(
            data_acquisition.plant.as_ref().unwrap().get_ilc_mode(1),
            InnerLoopControlMode::Disabled
        );
    }

    #[test]
    fn test_command_get_inner_loop_control_mode() {
        let mut data_acquisition = create_data_acquisition();

        let command = CommandGetInnerLoopControlMode;

        assert_eq!(command.name(), "cmd_getInnerLoopControlMode");

        assert!(command
            .execute(
                &json!({"addresses": [0, 1]}),
                Some(&mut data_acquisition),
                None,
                None,
                None
            )
            .is_some());
    }

    #[test]
    fn test_command_move_actuator_steps() {
        let mut data_acquisition = create_data_acquisition();

        let command = CommandMoveActuatorSteps;

        assert_eq!(command.name(), "cmd_moveActuatorSteps");

        // Invalid number of steps
        assert!(command
            .execute(
                &json!({"steps": [10, 20, 30]}),
                Some(&mut data_acquisition),
                None,
                None,
                None
            )
            .is_none());

        // Valid number of steps and mode
        let steps = vec![-10; NUM_ACTUATOR];

        data_acquisition.set_mode(DataAcquisitionMode::Telemetry);
        assert!(command
            .execute(
                &json!({"seq_id_move_actuator_steps": 1, "steps": steps}),
                Some(&mut data_acquisition),
                None,
                None,
                None
            )
            .is_some());
        assert_eq!(data_acquisition.plant.unwrap().actuator_steps, steps);
    }

    #[test]
    fn test_command_set_data_acquisition_mode() {
        let mut data_acquisition = create_data_acquisition();

        let command = CommandSetDataAcquisitionMode;

        assert_eq!(command.name(), "cmd_setDataAcquisitionMode");
        assert!(command
            .execute(
                &json!({"mode": 2}),
                Some(&mut data_acquisition),
                None,
                None,
                None
            )
            .is_some());

        assert!(data_acquisition.mode == DataAcquisitionMode::Telemetry);
    }
}
