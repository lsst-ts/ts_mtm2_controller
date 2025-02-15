use serde_json::Value;

use crate::command::command_schema::Command;
use crate::constants::{NUM_HARDPOINTS, NUM_TEMPERATURE_RING};
use crate::control::control_loop::ControlLoop;
use crate::controller::Controller;
use crate::enums::{ClosedLoopControlMode, Commander, PowerType};
use crate::power::power_system::PowerSystem;

/// Command to clear the errors.
pub struct CommandClearErrors;
impl Command for CommandClearErrors {
    fn name(&self) -> &str {
        "cmd_clearErrors"
    }

    fn execute(
        &self,
        _message: &Value,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        controller: Option<&mut Controller>,
    ) -> Option<()> {
        let system_controller = controller?;
        system_controller.error_handler.clear();

        Some(())
    }
}

/// Command to switch on/off the force balance system.
pub struct CommandSwitchForceBalanceSystem;
impl Command for CommandSwitchForceBalanceSystem {
    fn name(&self) -> &str {
        "cmd_switchForceBalanceSystem"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        controller: Option<&mut Controller>,
    ) -> Option<()> {
        // Check the current state of the power system.
        let system_controller = controller?;
        let power_system = &system_controller.status.power_system;
        if (!power_system[&PowerType::Communication].is_power_on())
            || (!power_system[&PowerType::Motor].is_power_on())
        {
            return None;
        }

        // Switch on/off the force balance system.
        let mode = if message["status"].as_bool()? {
            ClosedLoopControlMode::ClosedLoop
        } else {
            ClosedLoopControlMode::OpenLoop
        };

        system_controller.update_closed_loop_control_mode(mode)
    }
}

/// Command to set the temperature offset.
pub struct CommandSetTemperatureOffset;
impl Command for CommandSetTemperatureOffset {
    fn name(&self) -> &str {
        "cmd_setTemperatureOffset"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        controller: Option<&mut Controller>,
    ) -> Option<()> {
        // Get the reference temperature.
        let ref_temperature = message["ring"].as_array()?;
        if ref_temperature.len() != NUM_TEMPERATURE_RING {
            return None;
        }

        // Update the configuration.
        let system_controller = controller?;

        let mut config = system_controller.error_handler.config_control_loop.clone();
        config.ref_temperature = ref_temperature
            .iter()
            .map(|x| x.as_f64())
            .collect::<Option<Vec<f64>>>()?;

        system_controller.send_config_to_control_loop_and_update(config)
    }
}

/// Command to switch the command source: GUI or CSC.
pub struct CommandSwitchCommandSource;
impl Command for CommandSwitchCommandSource {
    fn name(&self) -> &str {
        "cmd_switchCommandSource"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        controller: Option<&mut Controller>,
    ) -> Option<()> {
        let commander = if message["isRemote"].as_bool()? {
            Commander::CSC
        } else {
            Commander::GUI
        };

        let system_controller = controller?;
        system_controller.set_commander(commander)
    }
}

/// Command to enable the open-loop maximum limit.
pub struct CommandEnableOpenLoopMaxLimit;
impl Command for CommandEnableOpenLoopMaxLimit {
    fn name(&self) -> &str {
        "cmd_enableOpenLoopMaxLimit"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        controller: Option<&mut Controller>,
    ) -> Option<()> {
        let enable = message["status"].as_bool()?;

        let system_controller = controller?;
        system_controller.set_enable_open_loop_max_limit(enable)
    }
}

/// Command to save the current mirror position.
pub struct CommandSaveMirrorPosition;
impl Command for CommandSaveMirrorPosition {
    fn name(&self) -> &str {
        "cmd_saveMirrorPosition"
    }

    fn execute(
        &self,
        _message: &Value,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        // TODO: Implement this command.
        Some(())
    }
}

/// Command to load the configuration.
pub struct CommandLoadConfiguration;
impl Command for CommandLoadConfiguration {
    fn name(&self) -> &str {
        "cmd_loadConfiguration"
    }

    fn execute(
        &self,
        _message: &Value,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        // TODO: Implement this command.
        Some(())
    }
}

/// Command to set the control parameters.
pub struct CommandSetControlParameters;
impl Command for CommandSetControlParameters {
    fn name(&self) -> &str {
        "cmd_setControlParameters"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        controller: Option<&mut Controller>,
    ) -> Option<()> {
        let system_controller = controller?;

        let mut config = system_controller.error_handler.config_control_loop.clone();
        config.use_external_elevation_angle = message["useExternalElevationAngle"].as_bool()?;
        config.enable_angle_comparison = message["enableAngleComparison"].as_bool()?;
        config.max_angle_difference = message["maxAngleDifference"].as_f64()?;
        config.enable_lut_temperature = message["enableLutTemperature"].as_bool()?;
        system_controller.send_config_to_control_loop_and_update(config)
    }
}

/// Command to set the enabled-faults mask.
pub struct CommandSetEnabledFaultsMask;
impl Command for CommandSetEnabledFaultsMask {
    fn name(&self) -> &str {
        "cmd_setEnabledFaultsMask"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        controller: Option<&mut Controller>,
    ) -> Option<()> {
        let mask = message["mask"].as_u64()?;

        let system_controller = controller?;
        system_controller.set_enabled_faults_mask(mask)
    }
}

/// Command to set the look-up table (LUT) configuration file.
pub struct CommandSetConfigurationFile;
impl Command for CommandSetConfigurationFile {
    fn name(&self) -> &str {
        "cmd_setConfigurationFile"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        controller: Option<&mut Controller>,
    ) -> Option<()> {
        let system_controller = controller?;
        let file = message["file"].as_str()?;

        system_controller.set_config(file)
    }
}

/// Command to set the hardpoint list.
pub struct CommandSetHardpointList;
impl Command for CommandSetHardpointList {
    fn name(&self) -> &str {
        "cmd_setHardpointList"
    }

    fn execute(
        &self,
        message: &Value,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        controller: Option<&mut Controller>,
    ) -> Option<()> {
        // Get the hardpoints.
        let hardpoints = message["actuators"].as_array()?;
        if hardpoints.len() != NUM_HARDPOINTS {
            return None;
        }

        let mut hardpoints_sorted = hardpoints
            .iter()
            .map(|x| x.as_u64())
            .collect::<Option<Vec<u64>>>()?;
        hardpoints_sorted.sort();

        let new_hardpoints: Vec<usize> = hardpoints_sorted.iter().map(|x| *x as usize).collect();

        let system_controller = controller?;
        system_controller.set_hardpoints(&new_hardpoints)
    }
}

/// Command to run the script.
pub struct CommandRunScript;
impl Command for CommandRunScript {
    fn name(&self) -> &str {
        "cmd_runScript"
    }

    fn execute(
        &self,
        _message: &Value,
        _power_system: Option<&mut PowerSystem>,
        _control_loop: Option<&mut ControlLoop>,
        _controller: Option<&mut Controller>,
    ) -> Option<()> {
        // TODO: Implement this command.
        Some(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use serde_json::json;
    use std::path::Path;
    use std::sync::mpsc::{sync_channel, Receiver};

    use crate::constants::BOUND_SYNC_CHANNEL;
    use crate::enums::{ErrorCode, PowerSystemState};

    fn create_controller() -> (Controller, Receiver<Value>) {
        let (sender_to_control_loop, receiver_to_control_loop) = sync_channel(BOUND_SYNC_CHANNEL);

        let mut controller = Controller::new(Path::new("config/lut/handling"));
        controller.sender_to_control_loop = Some(sender_to_control_loop);

        (controller, receiver_to_control_loop)
    }

    #[test]
    fn test_command_clear_errors() {
        let mut controller = create_controller().0;
        controller
            .error_handler
            .add_error(ErrorCode::FaultCommVoltage);

        let command = CommandClearErrors;

        assert_eq!(command.name(), "cmd_clearErrors");

        assert!(command
            .execute(&json!({}), None, None, Some(&mut controller))
            .is_some());

        assert!(!controller.error_handler.has_fault());
    }

    #[test]
    fn test_command_switch_force_balance_system() {
        let (mut controller, receiver_to_control_loop) = create_controller();

        let command = CommandSwitchForceBalanceSystem;

        assert_eq!(command.name(), "cmd_switchForceBalanceSystem");

        // Should fail because the power system states are not powered on.
        assert!(command
            .execute(&json!({"status": true}), None, None, Some(&mut controller))
            .is_none());

        // Set the power system states to powered on.
        controller.status.update_power_system(
            PowerType::Communication,
            true,
            PowerSystemState::PoweredOn,
        );
        controller
            .status
            .update_power_system(PowerType::Motor, true, PowerSystemState::PoweredOn);

        // Switch on the force balance system.
        assert!(command
            .execute(&json!({"status": true}), None, None, Some(&mut controller))
            .is_some());
        assert_eq!(
            receiver_to_control_loop.try_recv().ok(),
            Some(json!({
                "id": "cmd_setClosedLoopControlMode",
                "mode": 4,
            }))
        );

        // Switch off the force balance system.
        assert!(command
            .execute(&json!({"status": false}), None, None, Some(&mut controller))
            .is_some());
        assert_eq!(
            receiver_to_control_loop.try_recv().ok(),
            Some(json!({
                "id": "cmd_setClosedLoopControlMode",
                "mode": 3,
            }))
        );
    }

    #[test]
    fn test_command_set_temperature_offset() {
        let (mut controller, _receiver_to_control_loop) = create_controller();

        let command = CommandSetTemperatureOffset;

        assert_eq!(command.name(), "cmd_setTemperatureOffset");

        // Should fail because the number of temperature rings is not correct.
        assert!(command
            .execute(&json!({"ring": [1.0]}), None, None, Some(&mut controller))
            .is_none());

        // Set the correct number of temperature rings.
        let ref_temperature = vec![2.0; NUM_TEMPERATURE_RING];
        assert!(command
            .execute(
                &json!({"ring": ref_temperature}),
                None,
                None,
                Some(&mut controller)
            )
            .is_some());
        assert_eq!(
            controller.error_handler.config_control_loop.ref_temperature,
            ref_temperature
        );
    }

    #[test]
    fn test_command_switch_command_source() {
        let mut controller = create_controller().0;

        let command = CommandSwitchCommandSource;

        assert_eq!(command.name(), "cmd_switchCommandSource");

        // Switch to GUI.
        assert!(command
            .execute(
                &json!({"isRemote": false}),
                None,
                None,
                Some(&mut controller)
            )
            .is_some());
        assert_eq!(controller.commander, Commander::GUI);

        // Switch to CSC.
        assert!(command
            .execute(
                &json!({"isRemote": true}),
                None,
                None,
                Some(&mut controller)
            )
            .is_some());
        assert_eq!(controller.commander, Commander::CSC);
    }

    #[test]
    fn test_command_enable_open_loop_max_limit() {
        let (mut controller, _receiver_to_control_loop) = create_controller();
        controller.status.mode = ClosedLoopControlMode::ClosedLoop;

        let command = CommandEnableOpenLoopMaxLimit;

        assert_eq!(command.name(), "cmd_enableOpenLoopMaxLimit");

        // Should fail because the control loop is in closed-loop mode.
        assert!(command
            .execute(&json!({"status": true}), None, None, Some(&mut controller))
            .is_none());

        // Should succeed.
        controller.status.mode = ClosedLoopControlMode::OpenLoop;

        assert!(command
            .execute(&json!({"status": true}), None, None, Some(&mut controller))
            .is_some());
        assert!(
            controller
                .error_handler
                .config_control_loop
                .enable_open_loop_max_limit
        );
    }

    #[test]
    fn test_command_set_control_parameters() {
        let (mut controller, _receiver_to_control_loop) = create_controller();

        let command = CommandSetControlParameters;

        assert_eq!(command.name(), "cmd_setControlParameters");

        // Should fail because the message is not correct.
        assert!(command
            .execute(&json!({}), None, None, Some(&mut controller))
            .is_none());

        // Set the correct message.
        assert!(command
            .execute(
                &json!({
                    "useExternalElevationAngle": true,
                    "enableAngleComparison": true,
                    "maxAngleDifference": 1.0,
                    "enableLutTemperature": true,
                }),
                None,
                None,
                Some(&mut controller)
            )
            .is_some());
        assert!(
            controller
                .error_handler
                .config_control_loop
                .use_external_elevation_angle
        );
        assert!(
            controller
                .error_handler
                .config_control_loop
                .enable_angle_comparison
        );
        assert_eq!(
            controller
                .error_handler
                .config_control_loop
                .max_angle_difference,
            1.0
        );
        assert!(
            controller
                .error_handler
                .config_control_loop
                .enable_lut_temperature
        );
    }

    #[test]
    fn test_command_set_enabled_faults_mask() {
        let (mut controller, _receiver_to_control_loop) = create_controller();

        let command = CommandSetEnabledFaultsMask;

        assert_eq!(command.name(), "cmd_setEnabledFaultsMask");

        // Set the enabled-faults mask.
        assert!(command
            .execute(&json!({"mask": 20}), None, None, Some(&mut controller))
            .is_some());
        assert_eq!(
            controller
                .error_handler
                .config_control_loop
                .enabled_faults_mask,
            20
        );
    }

    #[test]
    fn test_command_set_configuration_file() {
        let (mut controller, _receiver_to_control_loop) = create_controller();

        let command = CommandSetConfigurationFile;

        assert_eq!(command.name(), "cmd_setConfigurationFile");

        // Should fail because the file does not exist.
        assert!(command
            .execute(&json!({"file": "wrong"}), None, None, Some(&mut controller))
            .is_none());

        // Set the correct file.
        assert!(command
            .execute(
                &json!({"file": "optical"}),
                None,
                None,
                Some(&mut controller)
            )
            .is_some());
        assert_eq!(
            controller.error_handler.config_control_loop.lut.dir_name,
            "config/lut/optical"
        );
    }

    #[test]
    fn test_command_set_hardpoint_list() {
        let (mut controller, _receiver_to_control_loop) = create_controller();

        let command = CommandSetHardpointList;

        assert_eq!(command.name(), "cmd_setHardpointList");

        // Should fail because the number of hardpoints is not correct.
        assert!(command
            .execute(
                &json!({"actuators": [1]}),
                None,
                None,
                Some(&mut controller)
            )
            .is_none());

        // Set the correct number of hardpoints but wrong selection.
        assert!(command
            .execute(
                &json!({"actuators": vec![6, 5, 4, 72, 71, 74]}),
                None,
                None,
                Some(&mut controller)
            )
            .is_none());

        // Set the correct hardpoints
        assert!(command
            .execute(
                &json!({"actuators": vec![24, 14, 4, 74, 72, 76]}),
                None,
                None,
                Some(&mut controller)
            )
            .is_some());

        assert_eq!(
            controller.error_handler.config_control_loop.hardpoints,
            vec![4, 14, 24, 72, 74, 76]
        );
    }

    #[test]
    fn test_command_run_script() {
        let (mut controller, _receiver_to_control_loop) = create_controller();

        let command = CommandRunScript;

        assert_eq!(command.name(), "cmd_runScript");

        assert!(command
            .execute(&json!({}), None, None, Some(&mut controller))
            .is_some());
    }
}
