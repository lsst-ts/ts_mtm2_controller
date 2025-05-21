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

use crate::control::control_loop::ControlLoop;
use crate::controller::Controller;
use crate::enums::CommandStatus;
use crate::power::power_system::PowerSystem;
use crate::utility::get_message_name;
use crate::utility::{acknowledge_command, get_message_sequence_id};

pub trait Command {
    /// Get the name of the command.
    ///
    /// # Returns
    /// Command name.
    fn name(&self) -> &str;

    /// Execute the command.
    ///
    /// # Arguments
    /// * `message` - Command message to execute.
    /// * `power_system` - Power system to execute the command.
    /// * `control_loop` - Control loop to execute the command.
    ///
    /// # Returns
    /// Command execution result.
    fn execute(
        &self,
        message: &Value,
        power_system: Option<&mut PowerSystem>,
        control_loop: Option<&mut ControlLoop>,
        controller: Option<&mut Controller>,
    ) -> Option<()>;
}

pub struct CommandSchema {
    // List of commands.
    pub commands: Vec<Box<dyn Command + Send>>,
}

impl CommandSchema {
    /// Create a new command schema.
    pub fn new() -> Self {
        Self {
            commands: Vec::new(),
        }
    }

    /// Add a command to the schema.
    pub fn add_command(&mut self, command: Box<dyn Command + Send>) {
        self.commands.push(command);
    }

    /// Execute a command.
    ///
    /// # Arguments
    /// * `message` - Command message to execute.
    /// * `power_system` - Power system to execute the command.
    /// * `control_loop` - Control loop to execute the command.
    /// * `controller` - Controller to execute the command.
    ///
    /// # Returns
    /// Command execution result.
    pub fn execute(
        &self,
        message: &Value,
        power_system: Option<&mut PowerSystem>,
        control_loop: Option<&mut ControlLoop>,
        controller: Option<&mut Controller>,
    ) -> Value {
        let name = get_message_name(message);
        let sequence_id = get_message_sequence_id(message);

        let command_status: CommandStatus;
        for cmd in &self.commands {
            if cmd.name() == name {
                match cmd.execute(message, power_system, control_loop, controller) {
                    Some(_) => {
                        command_status = CommandStatus::Success;
                    }
                    None => {
                        error!("Command execution failed: {message}");

                        command_status = CommandStatus::Fail;
                    }
                }

                return acknowledge_command(command_status, sequence_id);
            }
        }

        error!("Unknown command: {message}");

        acknowledge_command(CommandStatus::Fail, sequence_id)
    }

    /// Get the number of commands.
    ///
    /// # Returns
    /// Number of commands.
    pub fn number_of_commands(&self) -> usize {
        self.commands.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    struct CommandTest;
    impl Command for CommandTest {
        fn name(&self) -> &str {
            "cmd_test"
        }

        fn execute(
            &self,
            message: &Value,
            _power_system: Option<&mut PowerSystem>,
            _control_loop: Option<&mut ControlLoop>,
            _controller: Option<&mut Controller>,
        ) -> Option<()> {
            if message["status"].as_bool()? {
                return Some(());
            }

            None
        }
    }

    fn create_command_schema() -> CommandSchema {
        let mut schema = CommandSchema::new();
        schema.add_command(Box::new(CommandTest));

        schema
    }

    #[test]
    fn test_new() {
        let schema = create_command_schema();

        assert_eq!(schema.commands.len(), 1);
    }

    #[test]
    fn test_execute_fail() {
        let schema = create_command_schema();

        // Not a valid command.
        let result = schema.execute(
            &json!({"id": "cmd_fail", "sequence_id": 1}),
            None,
            None,
            None,
        );

        assert_eq!(result.to_string(), r#"{"id":"fail","sequence_id":1}"#);

        // No status field.
        let result = schema.execute(
            &json!({"id": "cmd_test", "sequence_id": 2}),
            None,
            None,
            None,
        );

        assert_eq!(result.to_string(), r#"{"id":"fail","sequence_id":2}"#);

        // Status is false.
        let result = schema.execute(
            &json!({"id": "cmd_test", "sequence_id": 3, "status": false}),
            None,
            None,
            None,
        );

        assert_eq!(result.to_string(), r#"{"id":"fail","sequence_id":3}"#);
    }

    #[test]
    fn test_execute_success() {
        let schema = create_command_schema();

        // Has the sequence_id field.
        let result = schema.execute(
            &json!({"id": "cmd_test", "sequence_id": 1, "status": true}),
            None,
            None,
            None,
        );

        assert_eq!(result.to_string(), r#"{"id":"success","sequence_id":1}"#);

        // No sequence_id field.
        let result = schema.execute(&json!({"id": "cmd_test", "status": true}), None, None, None);

        assert_eq!(result.to_string(), r#"{"id":"success","sequence_id":-1}"#);
    }

    #[test]
    fn test_number_of_commands() {
        let schema = create_command_schema();

        assert_eq!(schema.number_of_commands(), 1);
    }
}
