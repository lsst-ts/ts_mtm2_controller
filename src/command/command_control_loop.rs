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

use crate::command::command_schema::Command;
use crate::config::Config;
use crate::constants::{NUM_ACTUATOR, NUM_AXIAL_ACTUATOR, NUM_TANGENT_LINK};
use crate::control::control_loop::ControlLoop;
use crate::controller::Controller;
use crate::enums::{
    ActuatorDisplacementUnit, ClosedLoopControlMode, CommandActuator, InnerLoopControlMode,
};
use crate::power::power_system::PowerSystem;

/// Command to set the closed-loop control mode.
pub struct CommandSetClosedLoopControlMode;
impl Command for CommandSetClosedLoopControlMode {
    fn name(&self) -> &str {
        "cmd_setClosedLoopControlMode"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let discriminant = message["mode"].as_u64()?;

        let control = control_loop?;
        let mode = ClosedLoopControlMode::from_repr(discriminant as u8)?;
        control.update_control_mode(mode);

        Some(())
    }
}

/// Command to apply forces to the mirror.
pub struct CommandApplyForces;
impl CommandApplyForces {
    /// Get the force from the message.
    ///
    /// # Arguments
    /// * `message` - The message.
    ///
    /// # Returns
    /// The vector of force contains the axial force followed by the tangent
    /// force.
    pub fn get_force(message: &Value) -> Option<Vec<f64>> {
        let mut force = vec![0.0; NUM_ACTUATOR];

        let axial_forces = message["axial"].as_array()?;
        let tangent_forces = message["tangent"].as_array()?;

        // Check the length of the axial and tangent forces.
        if (axial_forces.len() != NUM_AXIAL_ACTUATOR) || (tangent_forces.len() != NUM_TANGENT_LINK)
        {
            return None;
        }

        // Get the axial and tangent forces.
        for idx in 0..NUM_AXIAL_ACTUATOR {
            force[idx] = axial_forces[idx].as_f64()?;
        }

        // Get the tangent forces.
        for idx in 0..NUM_TANGENT_LINK {
            force[idx + NUM_AXIAL_ACTUATOR] = tangent_forces[idx].as_f64()?;
        }

        Some(force)
    }
}

impl Command for CommandApplyForces {
    fn name(&self) -> &str {
        "cmd_applyForces"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let force = CommandApplyForces::get_force(message)?;

        let control = control_loop?;
        match control.apply_force(&force) {
            Ok(_) => return Some(()),
            Err(err) => {
                error!("Failed to apply force: {err}");

                return None;
            }
        }
    }
}

/// Command to reset the force offsets.
pub struct CommandResetForceOffsets;
impl Command for CommandResetForceOffsets {
    fn name(&self) -> &str {
        "cmd_resetForceOffsets"
    }

    fn execute(
        &self,
        _message: &Value,
        _power_system: Option<&mut PowerSystem>,
        control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let control = control_loop?;
        control.reset_force();

        Some(())
    }
}

/// Command to move the mirror's position.
pub struct CommandPositionMirror;
impl CommandPositionMirror {
    /// Get the position from the message.
    ///
    /// # Arguments
    /// * `message` - The message.
    ///
    /// # Returns
    /// The vector of position contains the x, y, z, x rotation, y rotation, and
    /// z rotation. The units are um and arcsec individually.
    pub fn get_position(message: &Value) -> Option<Vec<f64>> {
        let axes = ["x", "y", "z", "xRot", "yRot", "zRot"];
        let mut positions = vec![0.0; axes.len()];
        for (idx, axis) in axes.iter().enumerate() {
            positions[idx] = message[axis].as_f64()?;
        }

        Some(positions)
    }
}

impl Command for CommandPositionMirror {
    fn name(&self) -> &str {
        "cmd_positionMirror"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let positions = CommandPositionMirror::get_position(message)?;

        let control = control_loop?;
        control.handle_position_mirror(
            positions[0],
            positions[1],
            positions[2],
            positions[3],
            positions[4],
            positions[5],
        );

        Some(())
    }
}

/// Command to reset the actuator steps.
pub struct CommandResetActuatorSteps;
impl Command for CommandResetActuatorSteps {
    fn name(&self) -> &str {
        "cmd_resetActuatorSteps"
    }

    fn execute(
        &self,
        _message: &Value,
        _power_system: Option<&mut PowerSystem>,
        control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let control = control_loop?;
        control.reset_steps();

        Some(())
    }
}

/// Command to move the actuators.
pub struct CommandMoveActuators;
impl CommandMoveActuators {
    /// Get the command data from the message.
    ///
    /// # Arguments
    /// * `message` - The message.
    ///
    /// # Returns
    /// A tuple of a actuator command, a vector of actuator IDs, a displacement,
    /// and a unit. The actuator command is the command to be executed. The
    /// vector of actuator IDs contains the IDs of the actuators to be moved.
    /// The displacement is the displacement to be applied. The unit is the unit
    /// of the displacement.
    pub fn get_command_data(
        message: &Value,
    ) -> Option<(CommandActuator, Vec<usize>, f64, ActuatorDisplacementUnit)> {
        let discriminant = message["actuatorCommand"].as_u64()?;
        let command_actuator = CommandActuator::from_repr(discriminant as u8)?;

        let mut actuators = Vec::new();
        let mut displacement = 0.0;
        let mut unit = ActuatorDisplacementUnit::None;
        if command_actuator == CommandActuator::Start {
            let actuator_list = message["actuators"].as_array()?;
            for actuator in actuator_list {
                let actuator_id = actuator.as_u64()?;
                actuators.push(actuator_id as usize);
            }

            displacement = message["displacement"].as_f64()?;

            let discriminant = message["unit"].as_u64()?;
            unit = ActuatorDisplacementUnit::from_repr(discriminant as u8)?;
        }

        Some((command_actuator, actuators, displacement, unit))
    }
}

impl Command for CommandMoveActuators {
    fn name(&self) -> &str {
        "cmd_moveActuators"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let (command_actuator, actuators, displacement, unit) =
            CommandMoveActuators::get_command_data(message)?;

        let control = control_loop?;
        match control.move_actuators(command_actuator, &actuators, displacement, unit) {
            Ok(_) => return Some(()),
            Err(err) => {
                error!("Failed to move actuators: {err}");

                return None;
            }
        }
    }
}

/// Command to set the configuration.
pub struct CommandSetConfig;
impl Command for CommandSetConfig {
    fn name(&self) -> &str {
        "cmd_setConfig"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let new_config: Config = serde_json::from_str(message["config"].as_str()?).ok()?;

        let control = control_loop?;
        let hardpoints_are_changed = control.config.hardpoints != new_config.hardpoints;

        control.config = new_config;
        if hardpoints_are_changed {
            control.update_matrices_hardpoints();
        }

        Some(())
    }
}

/// Command to set the external elevation angle in degree.
pub struct CommandSetExternalElevation;
impl Command for CommandSetExternalElevation {
    fn name(&self) -> &str {
        "cmd_setExternalElevation"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        if message["compName"].as_str()? != "mtmount" {
            return None;
        }

        let control = control_loop?;
        control.telemetry.inclinometer.insert(
            String::from("external"),
            message["actualPosition"].as_f64()?,
        );

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
        _power_system: Option<&mut PowerSystem>,
        control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let control = control_loop?;

        let addresses = message["addresses"].as_array()?;
        for address in addresses {
            let address = address.as_u64()? as usize;
            let mode = InnerLoopControlMode::from_repr(message["mode"].as_u64()? as u8)?;
            control.set_ilc_mode(address, mode);
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
        _power_system: Option<&mut PowerSystem>,
        control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        let control = control_loop?;

        let addresses = message["addresses"].as_array()?;
        for address in addresses {
            let address = address.as_u64()? as usize;
            control.get_ilc_mode(address);
        }

        Some(())
    }
}

#[cfg(test)]
mod tests {
    use std::vec;

    use super::*;
    use serde_json::json;
    use std::path::Path;

    use crate::config::Config;
    use crate::constants::NUM_TEMPERATURE_RING;
    use crate::utility::assert_relative_eq_vector;

    const EPSILON: f64 = 1e-7;

    fn create_control_loop() -> ControlLoop {
        let config = Config::new(
            Path::new("config/parameters_control.yaml"),
            Path::new("config/lut/handling"),
        );

        ControlLoop::new(&config, false, true)
    }

    #[test]
    fn test_command_set_closed_loop_control_mode() {
        let mut control_loop = create_control_loop();

        let command = CommandSetClosedLoopControlMode;

        assert_eq!(command.name(), "cmd_setClosedLoopControlMode");

        assert!(command
            .execute(&json!({"mode": 3}), None, Some(&mut control_loop), None)
            .is_some());

        assert_eq!(
            control_loop.event_queue.get_events_and_clear(),
            vec![
                json!({
                    "id": "closedLoopControlMode",
                    "mode": 3,
                }),
                json!({
                    "id": "forceBalanceSystemStatus",
                    "status": false,
                })
            ]
        );
    }

    #[test]
    fn test_command_apply_forces() {
        let command = CommandApplyForces;

        assert_eq!(command.name(), "cmd_applyForces");

        // Incorrect message.
        assert!(CommandApplyForces::get_force(
            &json!({"axial": [1.0, 0.0], "tangent": [2.0, 0.0]})
        )
        .is_none());

        // Correct message.
        let mut force = vec![0.0; NUM_ACTUATOR];
        force[0] = 1.0;
        force[NUM_AXIAL_ACTUATOR] = 2.0;

        assert_eq!(
            CommandApplyForces::get_force(
                &json!({"axial": force[0..NUM_AXIAL_ACTUATOR], "tangent": force[NUM_AXIAL_ACTUATOR..NUM_ACTUATOR]})
            ).unwrap(),
            force,
        )
    }

    #[test]
    fn test_command_reset_force_offsets() {
        let mut control_loop = create_control_loop();

        let command = CommandResetForceOffsets;

        assert_eq!(command.name(), "cmd_resetForceOffsets");

        assert!(command
            .execute(&json!({}), None, Some(&mut control_loop), None)
            .is_some());
    }

    #[test]
    fn test_command_position_mirror() {
        let command = CommandPositionMirror;

        assert_eq!(command.name(), "cmd_positionMirror");

        // Incorrect message.
        assert!(CommandPositionMirror::get_position(&json!({
            "x": 1.0,
            "y": 2.0,
            "z": 3.0,
            "xRot": 4.0,
            "yRot": 5.0,
        }))
        .is_none());

        // Correct message.
        assert_eq!(
            CommandPositionMirror::get_position(&json!({
                "x": 1.0,
                "y": 2.0,
                "z": 3.0,
                "xRot": 4.0,
                "yRot": 5.0,
                "zRot": 6.0,
            }))
            .unwrap(),
            vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn test_command_reset_actuator_steps() {
        let mut control_loop = create_control_loop();

        let command = CommandResetActuatorSteps;

        assert_eq!(command.name(), "cmd_resetActuatorSteps");

        assert!(command
            .execute(&json!({}), None, Some(&mut control_loop), None)
            .is_some());
    }

    #[test]
    fn test_command_move_actuators() {
        let command = CommandMoveActuators;

        assert_eq!(command.name(), "cmd_moveActuators");

        // Incorrect message.
        assert!(CommandMoveActuators::get_command_data(&json!({
            "actuatorCommand": 5,
        }))
        .is_none());

        // Correct message.
        assert_eq!(
            CommandMoveActuators::get_command_data(&json!({
                "actuatorCommand": 1,
                "actuators": [0, 1, 2],
                "displacement": 1.3,
                "unit": 1,
            }))
            .unwrap(),
            (
                CommandActuator::Start,
                vec![0, 1, 2],
                1.3,
                ActuatorDisplacementUnit::Millimeter,
            )
        );
    }

    #[test]
    fn test_command_set_config() {
        let mut control_loop = create_control_loop();

        let command = CommandSetConfig;

        assert_eq!(command.name(), "cmd_setConfig");

        let mut config = Config::new(
            Path::new("config/parameters_control.yaml"),
            Path::new("config/lut/optical"),
        );
        config.hardpoints = vec![4, 14, 24, 72, 74, 76];

        assert!(command
            .execute(
                &json!({"config": serde_json::to_string(&config).unwrap()}),
                None,
                Some(&mut control_loop),
                None
            )
            .is_some());

        assert_eq!(control_loop.config.hardpoints, config.hardpoints);
        assert_eq!(control_loop.config.lut.dir_name, config.lut.dir_name);

        let lut_angle = 12.0;
        let ring_temperature = vec![1.0; NUM_TEMPERATURE_RING];
        let ref_temperature = vec![21.0; NUM_TEMPERATURE_RING];
        let enable_lut_temperature = true;

        assert_relative_eq_vector(
            &control_loop
                .config
                .lut
                .get_lut_forces(
                    lut_angle,
                    &ring_temperature,
                    &ref_temperature,
                    enable_lut_temperature,
                )
                .0,
            &config
                .lut
                .get_lut_forces(
                    lut_angle,
                    &ring_temperature,
                    &ref_temperature,
                    enable_lut_temperature,
                )
                .0,
            EPSILON,
        );
    }

    #[test]
    fn test_command_set_external_elevation() {
        let mut control_loop = create_control_loop();

        let command = CommandSetExternalElevation;

        assert_eq!(command.name(), "cmd_setExternalElevation");

        assert!(command
            .execute(
                &json!({"compName": "mtmount", "actualPosition": 1.0}),
                None,
                Some(&mut control_loop),
                None
            )
            .is_some());

        assert_eq!(
            control_loop.telemetry.inclinometer.get("external"),
            Some(&1.0)
        );
    }

    #[test]
    fn test_command_set_inner_loop_control_mode() {
        let mut control_loop = create_control_loop();

        let command = CommandSetInnerLoopControlMode;

        assert_eq!(command.name(), "cmd_setInnerLoopControlMode");

        assert!(command
            .execute(
                &json!({"addresses": [0, 1], "mode": 2}),
                None,
                Some(&mut control_loop),
                None
            )
            .is_some());

        assert_eq!(control_loop.get_ilc_mode(0), InnerLoopControlMode::Disabled);
        assert_eq!(control_loop.get_ilc_mode(1), InnerLoopControlMode::Disabled);
    }

    #[test]
    fn test_command_get_inner_loop_control_mode() {
        let mut control_loop = create_control_loop();

        let command = CommandGetInnerLoopControlMode;

        assert_eq!(command.name(), "cmd_getInnerLoopControlMode");

        control_loop.set_ilc_mode(0, InnerLoopControlMode::Enabled);
        control_loop.set_ilc_mode(1, InnerLoopControlMode::Enabled);

        assert!(command
            .execute(
                &json!({"addresses": [0, 1]}),
                None,
                Some(&mut control_loop),
                None
            )
            .is_some());
    }
}
