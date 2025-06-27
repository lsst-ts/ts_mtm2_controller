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

use log::{debug, info};
use serde_json::Value;
use std::{
    collections::HashMap,
    path::Path,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{sync_channel, Receiver, SyncSender},
        Arc,
    },
    thread::{spawn, JoinHandle},
};

use crate::command::{
    command_control_loop::{
        CommandApplyForces, CommandGetInnerLoopControlMode, CommandMoveActuators,
        CommandPositionMirror, CommandResetActuatorSteps, CommandResetForceOffsets,
        CommandSetClosedLoopControlMode, CommandSetInnerLoopControlMode,
    },
    command_controller::{
        CommandClearErrors, CommandEnableOpenLoopMaxLimit, CommandLoadConfiguration,
        CommandRunScript, CommandSaveMirrorPosition, CommandSetConfigurationFile,
        CommandSetControlParameters, CommandSetEnabledFaultsMask, CommandSetHardpointList,
        CommandSetMirrorHome, CommandSetTemperatureOffset, CommandSwitchCommandSource,
        CommandSwitchForceBalanceSystem,
    },
    command_power_system::{CommandPower, CommandResetBreakers, CommandSwitchDigitalOutput},
    command_schema::{Command, CommandSchema},
};
use crate::constants::BOUND_SYNC_CHANNEL;
use crate::control::control_loop_process::ControlLoopProcess;
use crate::controller::Controller;
use crate::enums::{
    BitEnum, ClosedLoopControlMode, CommandStatus, Commander, DigitalInput, DigitalOutput,
    PowerType,
};
use crate::interface::command_telemetry_server::CommandTelemetryServer;
use crate::mock::mock_plant::MockPlant;
use crate::power::power_system_process::PowerSystemProcess;
use crate::telemetry::{
    event::Event, telemetry::Telemetry, telemetry_control_loop::TelemetryControlLoop,
    telemetry_default::TelemetryDefault, telemetry_power::TelemetryPower,
};
use crate::utility::{
    acknowledge_command, get_message_name, get_message_sequence_id, get_parameter, is_command,
    is_event, is_telemetry,
};

pub struct Model {
    // Controller
    _controller: Controller,
    // Controller's command schema
    _controller_command_schema: CommandSchema,
    // Commands
    _commands: HashMap<String, Vec<String>>,
    // Is the simulation mode or not.
    _is_simulation_mode: bool,
    // The digit of the telemetry.
    _telemetry_digit: i32,
    // Servers.
    _host: String,
    _ports: HashMap<String, i32>,
    // Receivers from the TCP/IP servers.
    _receivers_from_tcp: HashMap<Commander, Option<Receiver<Value>>>,
    // Receiver of the telemetry to the model.
    _receiver_to_model: Receiver<Telemetry>,
    // Senders of the messages to the TCP/IP command telemetry servers.
    // The value is a tuple of the command (0) and telemetry (1) senders.
    _senders_to_tcp: HashMap<
        Commander,
        (
            Option<SyncSender<Vec<Value>>>,
            Option<SyncSender<Vec<Value>>>,
        ),
    >,
    // Sender of the telemetry to the model.
    _sender_to_model: Option<SyncSender<Telemetry>>,
    // An Arc instance that holds the AtomicBool instance to stop the threads.
    pub stop: Arc<AtomicBool>,
    // Handles of the threads.
    _handles: Vec<JoinHandle<()>>,
}

impl Model {
    /// Create a new model.
    ///
    /// # Arguments
    /// * `is_simulation_mode` - Is the simulation mode or not.
    /// * `host` - Host.
    /// * `port_command_gui` - Command port for the GUI.
    /// * `port_telemetry_gui` - Telemetry port for the GUI.
    /// * `port_command_csc` - Command port for the CSC.
    /// * `port_telemetry_csc` - Telemetry port for the CSC.
    ///
    /// # Returns
    /// A new model.
    pub fn new(
        is_simulation_mode: bool,
        host: &str,
        port_command_gui: i32,
        port_telemetry_gui: i32,
        port_command_csc: i32,
        port_telemetry_csc: i32,
    ) -> Self {
        let config_file = Path::new("config/parameters_app.yaml");
        let stop = Arc::new(AtomicBool::new(false));

        let lut: String = get_parameter(config_file, "lut");
        info!("The look-up table of the {lut} is loaded.");

        let commands = Self::create_commands();

        let mut ports = HashMap::new();
        ports.insert("gui_command".to_string(), port_command_gui);
        ports.insert("gui_telemetry".to_string(), port_telemetry_gui);
        ports.insert("csc_command".to_string(), port_command_csc);
        ports.insert("csc_telemetry".to_string(), port_telemetry_csc);

        let mut receivers_from_tcp = HashMap::new();
        receivers_from_tcp.insert(Commander::GUI, None);
        receivers_from_tcp.insert(Commander::CSC, None);

        let mut senders_to_tcp = HashMap::new();
        senders_to_tcp.insert(Commander::GUI, (None, None));
        senders_to_tcp.insert(Commander::CSC, (None, None));

        let (sender_to_model, receiver_to_model) = sync_channel(BOUND_SYNC_CHANNEL);

        Self {
            _controller: Controller::new(Path::new(&format!("config/lut/{lut}"))),
            _controller_command_schema: Self::create_controller_command_schema(),
            _commands: commands,

            _is_simulation_mode: is_simulation_mode,
            _telemetry_digit: get_parameter(config_file, "telemetry_digit"),

            _host: host.to_string(),
            _ports: ports,

            _receivers_from_tcp: receivers_from_tcp,
            _receiver_to_model: receiver_to_model,

            _senders_to_tcp: senders_to_tcp,

            _sender_to_model: Some(sender_to_model),

            stop: stop,

            _handles: Vec::new(),
        }
    }

    /// Create commands.
    ///
    /// # Returns
    /// Commands.
    fn create_commands() -> HashMap<String, Vec<String>> {
        // The commands here need to be in the
        // PowerSystemProcess.create_command_schema()
        let commands_power = vec![
            CommandPower.name().to_string(),
            CommandResetBreakers.name().to_string(),
            CommandSwitchDigitalOutput.name().to_string(),
        ];

        // The commands here need to be in the
        // ControlLoopProcess.create_command_schema()
        let commands_control_loop = vec![
            CommandSetClosedLoopControlMode.name().to_string(),
            CommandApplyForces.name().to_string(),
            CommandResetForceOffsets.name().to_string(),
            CommandPositionMirror.name().to_string(),
            CommandResetActuatorSteps.name().to_string(),
            CommandMoveActuators.name().to_string(),
            CommandGetInnerLoopControlMode.name().to_string(),
            CommandSetInnerLoopControlMode.name().to_string(),
        ];

        let controller_command_schema = Self::create_controller_command_schema();
        let commands_controller = controller_command_schema
            .commands
            .iter()
            .map(|command| command.name().to_string())
            .collect();

        let mut commands = HashMap::new();
        commands.insert("power".to_string(), commands_power);
        commands.insert("control_loop".to_string(), commands_control_loop);
        commands.insert("controller".to_string(), commands_controller);

        commands
    }

    /// Create the controller's command schema.
    ///
    /// # Returns
    /// Command schema.
    fn create_controller_command_schema() -> CommandSchema {
        let mut command_schema = CommandSchema::new();
        command_schema.add_command(Box::new(CommandClearErrors));
        command_schema.add_command(Box::new(CommandSwitchForceBalanceSystem));
        command_schema.add_command(Box::new(CommandSetTemperatureOffset));
        command_schema.add_command(Box::new(CommandSwitchCommandSource));
        command_schema.add_command(Box::new(CommandEnableOpenLoopMaxLimit));
        command_schema.add_command(Box::new(CommandSaveMirrorPosition));
        command_schema.add_command(Box::new(CommandSetMirrorHome));
        command_schema.add_command(Box::new(CommandLoadConfiguration));
        command_schema.add_command(Box::new(CommandSetControlParameters));
        command_schema.add_command(Box::new(CommandSetEnabledFaultsMask));
        command_schema.add_command(Box::new(CommandSetConfigurationFile));
        command_schema.add_command(Box::new(CommandSetHardpointList));
        command_schema.add_command(Box::new(CommandRunScript));

        command_schema
    }

    /// Stop the threads.
    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);

        for handle in self._handles.drain(..) {
            handle.join().expect("Process handle should join.");
        }
    }

    /// Run the processes.
    pub fn run_processes(&mut self) {
        self.run_command_telemetry_server();

        let plant = self.run_control_loop();

        self.run_power_system(plant);

        // Drop the internal sender to the model. This is to let the self.step()
        // wakes up when all the senders are dropped once we stop the
        // application.
        self._sender_to_model = None;
    }

    /// Run the command and telemetry servers.
    fn run_command_telemetry_server(&mut self) {
        // Run the servers
        let (mut server_gui, mut server_csc, receiver_from_tcp_gui, receiver_from_tcp_csc) =
            Model::create_servers(
                Path::new("config/parameters_app.yaml"),
                &self._commands,
                &self._host,
                self._ports["gui_command"],
                self._ports["gui_telemetry"],
                self._ports["csc_command"],
                self._ports["csc_telemetry"],
                &self.stop,
            );

        self._receivers_from_tcp
            .insert(Commander::GUI, Some(receiver_from_tcp_gui));
        self._receivers_from_tcp
            .insert(Commander::CSC, Some(receiver_from_tcp_csc));

        let (tcp_port_command_gui, tcp_port_telemetry_gui) = server_gui.run_servers();
        let (tcp_port_command_csc, tcp_port_telemetry_csc) = server_csc.run_servers();

        self._senders_to_tcp.insert(
            Commander::GUI,
            (
                server_gui.senders_to_tcp["command"].clone(),
                server_gui.senders_to_tcp["telemetry"].clone(),
            ),
        );
        self._senders_to_tcp.insert(
            Commander::CSC,
            (
                server_csc.senders_to_tcp["command"].clone(),
                server_csc.senders_to_tcp["telemetry"].clone(),
            ),
        );

        // Update the ports
        self._ports
            .insert("gui_command".to_string(), tcp_port_command_gui);
        self._ports
            .insert("gui_telemetry".to_string(), tcp_port_telemetry_gui);
        self._ports
            .insert("csc_command".to_string(), tcp_port_command_csc);
        self._ports
            .insert("csc_telemetry".to_string(), tcp_port_telemetry_csc);

        info!(
            "GUI command port: {tcp_port_command_gui}, GUI telemetry port: {tcp_port_telemetry_gui}."
        );
        info!(
            "CSC command port: {tcp_port_command_csc}, CSC telemetry port: {tcp_port_telemetry_csc}."
        );

        // Run the monitor loops
        self.run_monitor_loop(server_gui);
        self.run_monitor_loop(server_csc);
    }

    /// Create servers.
    ///
    /// # Arguments
    /// * `config_file` - Configuration file.
    /// * `commands` - Commands.
    /// * `host` - Host.
    /// * `port_command_gui` - Command port for the GUI.
    /// * `port_telemetry_gui` - Telemetry port for the GUI.
    /// * `port_command_csc` - Command port for the CSC.
    /// * `port_telemetry_csc` - Telemetry port for the CSC.
    /// * `stop` - Stop flag.
    ///
    /// # Returns
    /// * `CommandTelemetryServer` - GUI server.
    /// * `CommandTelemetryServer` - CSC server.
    /// * `Receiver<Value>` - Receiver from the GUI.
    /// * `Receiver<Value>` - Receiver from the CSC.
    fn create_servers(
        config_file: &Path,
        commands: &HashMap<String, Vec<String>>,
        host: &str,
        port_command_gui: i32,
        port_telemetry_gui: i32,
        port_command_csc: i32,
        port_telemetry_csc: i32,
        stop: &Arc<AtomicBool>,
    ) -> (
        CommandTelemetryServer,
        CommandTelemetryServer,
        Receiver<Value>,
        Receiver<Value>,
    ) {
        let timeout: i32 = get_parameter(config_file, "timeout");

        let (sender_from_tcp_gui, receiver_from_tcp_gui) = sync_channel(BOUND_SYNC_CHANNEL);
        let (sender_from_tcp_csc, receiver_from_tcp_csc) = sync_channel(BOUND_SYNC_CHANNEL);

        let mut server_gui = CommandTelemetryServer::new(
            Commander::GUI,
            host,
            port_command_gui,
            port_telemetry_gui,
            timeout as u64,
            &stop,
            sender_from_tcp_gui,
        );
        let mut server_csc = CommandTelemetryServer::new(
            Commander::CSC,
            host,
            port_command_csc,
            port_telemetry_csc,
            timeout as u64,
            stop,
            sender_from_tcp_csc,
        );

        let all_commands: Vec<String> = commands
            .values()
            .flat_map(|commands| commands.iter())
            .cloned()
            .collect();
        for command in all_commands.iter() {
            server_gui.register_command(command);
            server_csc.register_command(command);
        }

        (
            server_gui,
            server_csc,
            receiver_from_tcp_gui,
            receiver_from_tcp_csc,
        )
    }

    /// Run the monitor loop.
    ///
    /// # Arguments
    /// * `server` - Command telemetry server.
    fn run_monitor_loop(&mut self, mut server: CommandTelemetryServer) {
        let handle = spawn(move || {
            server.run_monitor_loop();
        });

        self._handles.push(handle);
    }

    /// Run the control loop.
    ///
    /// # Returns
    /// Plant model.
    fn run_control_loop(&mut self) -> Option<MockPlant> {
        // Read the configuration file
        let config_file = Path::new("config/parameters_app.yaml");
        let is_mirror = get_parameter(config_file, "is_mirror");

        let mode = if is_mirror { "mirror" } else { "surrogate" };
        info!("The control parameters of the {mode} is applied.");

        // Run the control loop
        let mut control_loop_process = ControlLoopProcess::new(
            &self._controller.error_handler.config_control_loop,
            is_mirror,
            self._is_simulation_mode,
            &self
                ._sender_to_model
                .as_ref()
                .expect("Should have a telemetry sender to the model."),
            &self.stop,
        );

        let plant = control_loop_process.control_loop.plant.clone();

        self._controller.sender_to_control_loop =
            Some(control_loop_process.get_sender_to_control_loop());

        let handle = spawn(move || {
            control_loop_process.run();
        });

        self._handles.push(handle);

        plant
    }

    /// Run the power system.
    ///
    /// # Arguments
    /// * `plant` - Plant model. Put None if the hardware mode is applied.
    fn run_power_system(&mut self, mut plant: Option<MockPlant>) {
        // Make sure the mock plant (if any) has no power
        if let Some(mock_plant) = plant.as_mut() {
            mock_plant.power_system_communication.is_power_on = false;
            mock_plant.power_system_communication.is_breaker_on = false;

            mock_plant.power_system_motor.is_power_on = false;
            mock_plant.power_system_motor.is_breaker_on = false;
        }

        let mut power_system_process = PowerSystemProcess::new(
            plant,
            &self
                ._sender_to_model
                .as_ref()
                .expect("Should have a telemetry sender to the model."),
            &self.stop,
        );

        self._controller.sender_to_power_system =
            Some(power_system_process.get_sender_to_power_system());

        let handle = spawn(move || {
            power_system_process.run();
        });

        self._handles.push(handle);
    }

    /// Step the model. This function has a blocking call to wait for the
    /// new telemetry.
    pub fn step(&mut self) {
        // Check the message from the GUI
        let mut gui_has_valid_command = false;
        if let Some(receiver) = &self._receivers_from_tcp[&Commander::GUI] {
            if let Ok(message) = receiver.try_recv() {
                gui_has_valid_command = Self::process_message(
                    &mut self._controller,
                    &self._commands,
                    &self._controller_command_schema,
                    &message,
                    Commander::GUI,
                    self._senders_to_tcp[&Commander::GUI]
                        .0
                        .as_ref()
                        .expect("Should have a command/event sender to the GUI."),
                );
            }
        }

        // Change the commander if the GUI has a valid command
        if gui_has_valid_command && (self._controller.commander != Commander::GUI) {
            self._controller.commander = Commander::GUI;
        }

        // Check the message from the CSC
        if let Some(receiver) = &self._receivers_from_tcp[&Commander::CSC] {
            if let Ok(message) = receiver.try_recv() {
                Self::process_message(
                    &mut self._controller,
                    &self._commands,
                    &self._controller_command_schema,
                    &message,
                    Commander::CSC,
                    self._senders_to_tcp[&Commander::CSC]
                        .0
                        .as_ref()
                        .expect("Should have a command/event sender to the CSC."),
                );
            }
        }

        // Check the telemetry message. Note the blocking call of the recv() is
        // used here. This is because the model should always get the telemetry
        // message regualarly if the processes of control loop and power system
        // are running.
        match self._receiver_to_model.recv() {
            Ok(telemetry) => {
                if let Some(command_result) = telemetry.command_result {
                    // Check the sequencd id is -1 or not. If it is, this is
                    // the internal command.
                    if get_message_sequence_id(&command_result) != -1 {
                        if let Some(sender) = &self._senders_to_tcp[&self._controller.commander].0 {
                            let _ = sender.try_send(vec![command_result.clone()]);
                        }

                        // Update the last effective telemetry.
                        self._controller.last_effective_telemetry.command_result =
                            Some(command_result);
                    }
                }

                let mut all_events: Vec<Value> = Vec::new();
                if let Some(mut events) = telemetry.events {
                    self.process_subsystem_event(&events);
                    all_events.append(&mut events);
                }

                if let Some(telemetry_power) = telemetry.power {
                    // Check the power condition.
                    self._controller.error_handler.check_condition_power_system(
                        &telemetry_power,
                        self._controller.status.power_system[&PowerType::Communication]
                            .is_power_on(),
                        self._controller.status.power_system[&PowerType::Motor].is_power_on(),
                    );

                    // Update the internal status.
                    let mut events_power = self.update_status_telemetry_power(&telemetry_power);
                    all_events.append(&mut events_power);

                    // Publish the telemetry.
                    self.publish_telemetry(telemetry_power.get_messages(self._telemetry_digit));

                    // Update the last effective telemetry.
                    self._controller.last_effective_telemetry.power = Some(telemetry_power);
                }

                if let Some(telemetry_control_loop) = telemetry.control_loop {
                    // The telemetry here is meaningful only when there is the
                    // communication power.
                    if self._controller.status.power_system[&PowerType::Communication].is_power_on()
                    {
                        // Check the system condition.
                        self.check_condition_control_loop(&telemetry_control_loop);

                        // Update the internal status.
                        let mut events_control_loop =
                            self.update_status_telemetry_control_loop(&telemetry_control_loop);
                        all_events.append(&mut events_control_loop);

                        // Publish the telemetry.
                        self.publish_telemetry(
                            telemetry_control_loop.get_messages(self._telemetry_digit),
                        );

                        // Update the last effective telemetry.
                        self._controller.last_effective_telemetry.control_loop =
                            Some(telemetry_control_loop);
                    }
                }

                // Publish the error events.
                all_events.append(&mut self._controller.get_error_handler_events());

                // Publish the controller events.
                if self._controller.event_queue.has_event() {
                    all_events.append(&mut self._controller.event_queue.get_events_and_clear());
                }

                self.publish_events(all_events);
            }

            Err(_) => {
                info!("All senders to the model are dropped.")
            }
        }
    }

    /// Process the received message.
    ///
    /// # Arguments
    /// * `controller` - Controller.
    /// * `commands` - Commands.
    /// * `controller_command_schema` - Controller's command schema.
    /// * `message` - Message.
    /// * `source` - Commander.
    /// * `sender_to_tcp` - Sender to the TCP/IP.
    ///
    /// # Returns
    /// Is the valid command or not.
    fn process_message(
        controller: &mut Controller,
        all_commands: &HashMap<String, Vec<String>>,
        controller_command_schema: &CommandSchema,
        message: &Value,
        source: Commander,
        sender_to_tcp: &SyncSender<Vec<Value>>,
    ) -> bool {
        debug!("Process the message: {message}.");

        let name = get_message_name(&message);
        if is_command(&name) {
            // Fail the command if the source is CSC but the commander is GUI
            if (controller.commander == Commander::GUI) && (source == Commander::CSC) {
                let _ = sender_to_tcp.try_send(vec![acknowledge_command(
                    CommandStatus::Fail,
                    get_message_sequence_id(message),
                )]);

                return false;
            }

            // Process the command

            // Pass the power command to the power system
            if all_commands["power"].contains(&name) {
                if let Some(sender) = controller.sender_to_power_system.as_ref() {
                    let _ = sender.try_send(message.clone());
                }

                return true;
            }

            // Pass the control-loop command to the control loop
            if all_commands["control_loop"].contains(&name) {
                if let Some(sender) = controller.sender_to_control_loop.as_ref() {
                    let _ = sender.try_send(message.clone());
                }

                return true;
            }

            // Command for the controller
            if all_commands["controller"].contains(&name) {
                let result =
                    controller_command_schema.execute(message, None, None, Some(controller));
                let _ = sender_to_tcp.try_send(vec![result]);

                return true;
            }

            // Should not arrive here
            return false;
        } else if is_telemetry(&name) {
            controller.update_external_elevation(message);
        } else if is_event(&name) {
            // At the moment, the control system does not require the event
            // from other telescope components.
        } else {
            // Connection status from command-telemetry server
            if name == "tcpIpConnected" {
                if let Some(is_connected) = message["isConnected"].as_bool() {
                    controller
                        .status
                        .update_connection_status(source, is_connected);
                    if is_connected {
                        let _ = sender_to_tcp.try_send(Self::get_welcome_messages(controller));
                    }
                }
            }
        }

        return false;
    }

    /// Update the status according to the new telemetry of power system.
    ///
    /// # Arguments
    /// * `telemetry` - Telemetry of the power system.
    ///
    /// # Returns
    /// The event messages.
    fn update_status_telemetry_power(&mut self, telemetry: &TelemetryPower) -> Vec<Value> {
        let mut events = Vec::new();

        // Update the digital output
        let digital_output = telemetry.digital_output;
        if let Some(()) = self
            ._controller
            .update_internal_status_digital_output(digital_output)
        {
            events.push(Event::get_message_digital_output(digital_output));
        }

        // Update the digital input
        let digital_input = telemetry.digital_input;
        if let Some(()) = self
            ._controller
            .update_internal_status_digital_input(digital_input)
        {
            events.push(Event::get_message_digital_input(digital_input));
        }

        // Update the interlock
        let is_interlock_on = (digital_output & DigitalOutput::InterlockEnable.bit_value() == 0)
            | (digital_input & DigitalInput::InterlockPowerRelay.bit_value() != 0);
        if let Some(()) = self
            ._controller
            .update_internal_status_interlock(is_interlock_on)
        {
            events.push(Event::get_message_interlock(is_interlock_on));
        }

        events
    }

    /// Check the condition of the control loop.
    ///
    /// # Arguments
    /// * `telemetry` - Telemetry of the control loop.
    fn check_condition_control_loop(&mut self, telemetry: &TelemetryControlLoop) {
        // Check the system condition.
        let is_closed_loop = self._controller.status.mode == ClosedLoopControlMode::ClosedLoop;
        self._controller
            .error_handler
            .check_condition_control_loop(telemetry, is_closed_loop);

        // Put the control loop in the telemetry-only mode if there
        // is a fault.
        let mode = self._controller.status.mode;
        if self._controller.error_handler.has_fault()
            && ((mode == ClosedLoopControlMode::OpenLoop)
                || (mode == ClosedLoopControlMode::ClosedLoop))
        {
            if self
                ._controller
                .update_closed_loop_control_mode(ClosedLoopControlMode::TelemetryOnly)
                .is_none()
            {
                debug!("Failed to change the control mode to be telemetry only when there is the fault. Stopping the control system...");

                self.stop();
            };
        }
    }

    /// Update the status according to the new telemetry of control loop.
    ///
    /// # Arguments
    /// * `telemetry` - Telemetry of the control loop.
    ///
    /// # Returns
    /// The event messages.
    fn update_status_telemetry_control_loop(
        &mut self,
        telemetry: &TelemetryControlLoop,
    ) -> Vec<Value> {
        let mut events = Vec::new();

        // Update the mirror is in position or not
        let is_in_position = telemetry.is_in_position;
        if let Some(()) = self
            ._controller
            .update_internal_status_is_in_position(is_in_position)
        {
            events.push(Event::get_message_in_position(is_in_position));
        }

        events
    }

    /// Publish the telemetry.
    ///
    /// # Arguments
    /// * `messages` - Messages.
    ///
    /// # Returns
    /// Did the publishment or not.
    fn publish_telemetry(&self, messages: Vec<Value>) -> bool {
        // No connection, return immediately
        if !self.is_connected() {
            return false;
        }

        // Publish the messages
        let (is_connected_gui, is_connected_csc) = self.get_connection_status();
        if is_connected_gui {
            let _ = self._senders_to_tcp[&Commander::GUI]
                .1
                .as_ref()
                .expect("Should have a telemetry sender to the GUI.")
                .try_send(messages.clone());
        }
        if is_connected_csc {
            let _ = self._senders_to_tcp[&Commander::CSC]
                .1
                .as_ref()
                .expect("Should have a telemetry sender to the CSC.")
                .try_send(messages);
        }

        true
    }

    /// Check there is a connection or not.
    ///
    /// # Returns
    /// Is connected or not.
    fn is_connected(&self) -> bool {
        let (is_connected_gui, is_connected_csc) = self.get_connection_status();
        is_connected_gui || is_connected_csc
    }

    /// Get the connection status.
    ///
    /// # Returns
    /// Connection status. The first element is the GUI and the second element
    /// is the CSC.
    fn get_connection_status(&self) -> (bool, bool) {
        (
            self._controller.status.is_connected(Commander::GUI),
            self._controller.status.is_connected(Commander::CSC),
        )
    }

    /// Process the subsystem event.
    ///
    /// # Arguments
    /// * `events` - Events.
    fn process_subsystem_event(&mut self, events: &Vec<Value>) {
        for event in events {
            let name = get_message_name(event);
            if name == "powerSystemState" {
                if self
                    ._controller
                    .update_internal_status_power_system(event)
                    .is_none()
                {
                    debug!("Failed to update the power system status: {event}.");
                }
            } else if name == "closedLoopControlMode" {
                if self
                    ._controller
                    .update_internal_status_mode(event)
                    .is_none()
                {
                    debug!("Failed to update the closed-loop control mode: {event}.");
                }
            } else if name == "innerLoopControlMode" {
                if self
                    ._controller
                    .update_internal_status_ilc_mode(event)
                    .is_none()
                {
                    debug!("Failed to update the inner-loop control mode: {event}.");
                }
            }
        }
    }

    /// Publish the events.
    ///
    /// # Arguments
    /// * `messages` - Messages.
    ///
    /// # Returns
    /// Did the publishment or not.
    fn publish_events(&self, events: Vec<Value>) -> bool {
        // No connection, return immediately
        if !self.is_connected() {
            return false;
        }

        // Publish the events
        let (is_connected_gui, is_connected_csc) = self.get_connection_status();
        if is_connected_gui {
            let _ = self._senders_to_tcp[&Commander::GUI]
                .0
                .as_ref()
                .expect("Should have a command sender to the GUI.")
                .try_send(events.clone());
        }
        if is_connected_csc {
            let _ = self._senders_to_tcp[&Commander::CSC]
                .0
                .as_ref()
                .expect("Should have a command sender to the CSC.")
                .try_send(events);
        }

        true
    }

    /// Get the welcome messages.
    ///
    /// # Arguments
    /// * `controller` - Controller.
    ///
    /// # Returns
    /// Welcome messages.
    fn get_welcome_messages(controller: &Controller) -> Vec<Value> {
        let status = &controller.status;

        let power_system_communication = status.power_system[&PowerType::Communication];
        let power_system_motor = status.power_system[&PowerType::Motor];

        let ilc_data = &controller.error_handler.ilc;

        let config = &controller.error_handler.config_control_loop;

        let mut messages = vec![
            Event::get_message_tcp_ip_connected(true),
            Event::get_message_commandable_by_dds(controller.commander == Commander::CSC),
            Event::get_message_hardpoint_list(&config.hardpoints),
            Event::get_message_bypassed_actuator_ilcs(&config.bypassed_actuator_ilcs),
            Event::get_message_interlock(status.is_interlock_on),
            Event::get_message_inclination_telemetry_source(config.use_external_elevation_angle),
            Event::get_message_temperature_offset(&config.ref_temperature),
            Event::get_message_digital_input(status.digital_input),
            Event::get_message_digital_output(status.digital_output),
            Event::get_message_config(config),
            Event::get_message_closed_loop_control_mode(status.mode),
            Event::get_message_enabled_faults_mask(config.enabled_faults_mask),
            Event::get_message_summary_faults_status(status.summary_faults_status),
            Event::get_message_configuration_files(&config.filename),
            Event::get_message_power_system_state(
                PowerType::Communication,
                power_system_communication.is_power_on,
                power_system_communication.state,
            ),
            Event::get_message_power_system_state(
                PowerType::Motor,
                power_system_motor.is_power_on,
                power_system_motor.state,
            ),
            Event::get_message_limit_switch_status(
                &ilc_data["limit_switch_retract"],
                &ilc_data["limit_switch_extend"],
            ),
            Event::get_message_open_loop_max_limit(config.enable_open_loop_max_limit),
        ];

        status.ilc_modes.iter().enumerate().for_each(|(idx, mode)| {
            messages.push(Event::get_message_inner_loop_control_mode(idx, *mode));
        });

        messages
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use serde_json::json;
    use std::net::{Shutdown, TcpStream};
    use std::sync::atomic::Ordering;
    use std::thread::sleep;
    use std::time::Duration;

    use crate::constants::{LOCAL_HOST, NUM_INNER_LOOP_CONTROLLER, TERMINATOR};
    use crate::enums::PowerSystemState;
    use crate::mock::mock_constants::{
        TEST_DIGITAL_INPUT_NO_POWER, TEST_DIGITAL_INPUT_POWER_COMM,
        TEST_DIGITAL_INPUT_POWER_COMM_MOTOR, TEST_DIGITAL_OUTPUT_NO_POWER,
        TEST_DIGITAL_OUTPUT_POWER_COMM, TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
    };
    use crate::utility::{
        client_read_and_assert, client_read_json, client_read_specific_json,
        client_write_and_sleep, read_file_stiffness,
    };

    const MAX_TIME_WAIT_FOR_COMMAND_RESULT: i32 = 30;
    const SLEEP_TIME: u64 = 100;
    const MAX_TIMEOUT: u64 = 200;

    fn create_model() -> Model {
        let mut model = Model::new(true, LOCAL_HOST, 0, 0, 0, 0);

        model._controller.status.digital_input = TEST_DIGITAL_INPUT_NO_POWER;
        model._controller.status.digital_output = TEST_DIGITAL_OUTPUT_NO_POWER;

        model
    }

    fn create_tcp_clients(port_command: i32, port_telemetry: i32) -> (TcpStream, TcpStream) {
        (
            TcpStream::connect(format!("{}:{}", LOCAL_HOST, port_command))
                .expect("Tcp command stream should connect."),
            TcpStream::connect(format!("{}:{}", LOCAL_HOST, port_telemetry))
                .expect("Tcp telemetry stream should connect."),
        )
    }

    fn wait_for_connection(model: &mut Model) {
        for _ in 0..MAX_TIME_WAIT_FOR_COMMAND_RESULT {
            model.step();
            if model.is_connected() {
                break;
            }
        }
    }

    fn consume_welcome_messages(model: &Model, client: &mut TcpStream) -> usize {
        let welcome_messages = Model::get_welcome_messages(&model._controller);
        let num_welcome_messages = welcome_messages.len();

        for _idx in 0..num_welcome_messages {
            let _ = client_read_json(client, TERMINATOR);
        }

        num_welcome_messages
    }

    fn wait_for_command_result_and_reset(model: &mut Model) {
        for _idx in 0..MAX_TIME_WAIT_FOR_COMMAND_RESULT {
            model.step();
            if model
                ._controller
                .last_effective_telemetry
                .command_result
                .is_some()
            {
                model._controller.last_effective_telemetry.command_result = None;
                break;
            }
        }
    }

    fn run_until_expected_digital_input_output(
        model: &mut Model,
        expected_digital_input: u32,
        expected_digital_output: u8,
    ) {
        loop {
            if (model._controller.status.digital_input == expected_digital_input)
                && (model._controller.status.digital_output == expected_digital_output)
            {
                break;
            }

            model.step();
        }
    }

    #[test]
    fn test_new() {
        let model = create_model();

        assert_eq!(model._controller_command_schema.number_of_commands(), 13);
    }

    #[test]
    fn test_stop() {
        let mut model = create_model();

        model.stop();

        assert_eq!(model.stop.load(Ordering::Relaxed), true);
    }

    #[test]
    fn test_run_command_telemetry_server() {
        let mut model = create_model();

        model.run_command_telemetry_server();

        for (_, value) in &model._ports {
            assert_ne!(*value, 0);
        }

        model.stop();
    }

    #[test]
    fn test_run_control_loop() {
        let mut model = create_model();

        model.run_control_loop();

        model.stop();
    }

    #[test]
    fn test_run_power_system() {
        let mut model = create_model();

        model.run_power_system(Some(MockPlant::new(
            &read_file_stiffness(Path::new("config/stiff_matrix_m2.yaml")),
            90.0,
        )));

        model.stop();
    }

    #[test]
    fn test_publish_telemetry_connection_off() {
        let model = create_model();

        let telemetry = TelemetryPower::new();
        let did_something = model.publish_telemetry(telemetry.get_messages(model._telemetry_digit));

        assert!(!did_something);
    }

    #[test]
    fn test_publish_telemetry_connection_on() {
        let mut model = create_model();
        model.run_processes();

        // Create the clients to connect the servers
        let (_client_command, mut client_telemetry) =
            create_tcp_clients(model._ports["gui_command"], model._ports["gui_telemetry"]);

        sleep(Duration::from_millis(MAX_TIMEOUT));

        wait_for_connection(&mut model);

        // Write the telemetry message
        let telemetry = TelemetryPower::new();
        let did_something = model.publish_telemetry(telemetry.get_messages(model._telemetry_digit));

        client_read_and_assert(&mut client_telemetry,"{\"commCurrent\":0.0,\"commVoltage\":0.0,\"id\":\"powerStatusRaw\",\"motorCurrent\":0.0,\"motorVoltage\":0.0}\r\n",
        );
        client_read_and_assert(&mut client_telemetry,"{\"commCurrent\":0.0,\"commVoltage\":0.0,\"id\":\"powerStatus\",\"motorCurrent\":0.0,\"motorVoltage\":0.0}\r\n",
        );

        assert!(did_something);

        model.stop();
    }

    #[test]
    fn test_publish_events_connection_on() {
        let mut model = create_model();
        model.run_processes();

        // Create the clients to connect the servers
        let (mut client_command, _client_telemetry) =
            create_tcp_clients(model._ports["gui_command"], model._ports["gui_telemetry"]);

        sleep(Duration::from_millis(MAX_TIMEOUT));

        wait_for_connection(&mut model);
        consume_welcome_messages(&model, &mut client_command);

        // Write the event
        let did_something = model.publish_events(vec![
            Event::get_message_in_position(true),
            Event::get_message_in_position(false),
        ]);

        client_read_and_assert(
            &mut client_command,
            "{\"id\":\"m2AssemblyInPosition\",\"inPosition\":true}\r\n",
        );
        client_read_and_assert(
            &mut client_command,
            "{\"id\":\"m2AssemblyInPosition\",\"inPosition\":false}\r\n",
        );

        assert!(did_something);

        model.stop();
    }

    #[test]
    fn test_run_processes_and_power_communication() {
        let mut model = create_model();

        model.run_processes();

        // Create the clients to connect the servers
        let (mut client_command_gui, _client_telemetry_gui) =
            create_tcp_clients(model._ports["gui_command"], model._ports["gui_telemetry"]);

        sleep(Duration::from_millis(MAX_TIMEOUT));

        wait_for_connection(&mut model);
        consume_welcome_messages(&model, &mut client_command_gui);

        // Issue a command to the power on the system
        client_write_and_sleep(
            &mut client_command_gui,
            "{\"id\":\"cmd_power\",\"sequence_id\":1,\"status\":true,\"powerType\":2}\r\n",
            SLEEP_TIME,
        );

        client_read_and_assert(
            &mut client_command_gui,
            "{\"id\":\"ack\",\"sequence_id\":1}\r\n",
        );

        wait_for_command_result_and_reset(&mut model);

        client_read_and_assert(
            &mut client_command_gui,
            "{\"id\":\"success\",\"sequence_id\":1}\r\n",
        );

        client_read_and_assert(
            &mut client_command_gui,
            "{\"id\":\"powerSystemState\",\"powerType\":2,\"state\":3,\"status\":true}\r\n",
        );
        client_read_and_assert(
            &mut client_command_gui,
            "{\"id\":\"powerSystemState\",\"powerType\":2,\"state\":5,\"status\":true}\r\n",
        );

        run_until_expected_digital_input_output(
            &mut model,
            TEST_DIGITAL_INPUT_POWER_COMM,
            TEST_DIGITAL_OUTPUT_POWER_COMM,
        );

        client_read_and_assert(
            &mut client_command_gui,
            &format!("{{\"id\":\"digitalOutput\",\"value\":{TEST_DIGITAL_OUTPUT_POWER_COMM}}}\r\n"),
        );

        client_read_and_assert(
            &mut client_command_gui,
            &format!("{{\"id\":\"digitalInput\",\"value\":{TEST_DIGITAL_INPUT_POWER_COMM}}}\r\n"),
        );

        // Check the power system status
        let power_system = model
            ._controller
            .status
            .power_system
            .get(&PowerType::Communication)
            .unwrap();

        assert!(power_system.is_power_on);
        assert_eq!(power_system.state, PowerSystemState::PoweredOn);

        // Issue a command to the power off the system
        client_write_and_sleep(
            &mut client_command_gui,
            "{\"id\":\"cmd_power\",\"sequence_id\":2,\"status\":false,\"powerType\":2}\r\n",
            SLEEP_TIME,
        );

        client_read_and_assert(
            &mut client_command_gui,
            "{\"id\":\"ack\",\"sequence_id\":2}\r\n",
        );

        wait_for_command_result_and_reset(&mut model);

        client_read_and_assert(
            &mut client_command_gui,
            "{\"id\":\"success\",\"sequence_id\":2}\r\n",
        );

        client_read_and_assert(
            &mut client_command_gui,
            "{\"id\":\"powerSystemState\",\"powerType\":2,\"state\":6,\"status\":false}\r\n",
        );
        client_read_and_assert(
            &mut client_command_gui,
            "{\"id\":\"powerSystemState\",\"powerType\":2,\"state\":2,\"status\":false}\r\n",
        );

        client_read_and_assert(
            &mut client_command_gui,
            &format!("{{\"id\":\"digitalOutput\",\"value\":{TEST_DIGITAL_OUTPUT_NO_POWER}}}\r\n"),
        );

        client_read_and_assert(
            &mut client_command_gui,
            &format!("{{\"id\":\"digitalInput\",\"value\":{TEST_DIGITAL_INPUT_NO_POWER}}}\r\n"),
        );

        // Check the power system status
        let power_system = model
            ._controller
            .status
            .power_system
            .get(&PowerType::Communication)
            .unwrap();

        assert!(!power_system.is_power_on);
        assert_eq!(power_system.state, PowerSystemState::PoweredOff);

        model.stop();
    }

    #[test]
    fn test_run_processes_and_transition_to_closed_loop_control() {
        let mut model = create_model();

        model.run_processes();

        // Create the clients to connect the servers
        let (mut client_command_gui, _client_telemetry_gui) =
            create_tcp_clients(model._ports["gui_command"], model._ports["gui_telemetry"]);

        sleep(Duration::from_millis(MAX_TIMEOUT));

        wait_for_connection(&mut model);
        consume_welcome_messages(&model, &mut client_command_gui);

        // Issue a command to the power on the communication system
        client_write_and_sleep(
            &mut client_command_gui,
            "{\"id\":\"cmd_power\",\"sequence_id\":1,\"status\":true,\"powerType\":2}\r\n",
            SLEEP_TIME,
        );

        wait_for_command_result_and_reset(&mut model);

        // Check the communication power system status
        assert!(model._controller.status.power_system[&PowerType::Communication].is_power_on());

        // Issue a command to the power on the communication system
        client_write_and_sleep(
            &mut client_command_gui,
            "{\"id\":\"cmd_power\",\"sequence_id\":2,\"status\":true,\"powerType\":1}\r\n",
            SLEEP_TIME,
        );

        wait_for_command_result_and_reset(&mut model);

        // Check the motor power system status
        assert!(model._controller.status.power_system[&PowerType::Motor].is_power_on());

        run_until_expected_digital_input_output(
            &mut model,
            TEST_DIGITAL_INPUT_POWER_COMM_MOTOR,
            TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
        );

        // Check the system staus
        let status = &model._controller.status;

        assert_eq!(status.digital_output, TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR);
        assert_eq!(status.digital_input, TEST_DIGITAL_INPUT_POWER_COMM_MOTOR);
        assert!(!status.is_interlock_on);

        assert_eq!(
            client_read_specific_json(&mut client_command_gui, TERMINATOR, "interlock"),
            json!({"id": "interlock", "state": false})
        );

        // Enable the ILCs (skip the state transtions)
        let addresses: Vec<usize> = (0..NUM_INNER_LOOP_CONTROLLER).collect();

        client_write_and_sleep(
            &mut client_command_gui,
            &format!(
                "{{\"id\":\"cmd_setInnerLoopControlMode\",\"sequence_id\":3,\"mode\":3,\"addresses\":{:?}}}\r\n", addresses
            ),
            SLEEP_TIME,
        );

        wait_for_command_result_and_reset(&mut model);

        let bypassed_ilcs = &model
            ._controller
            .error_handler
            .config_control_loop
            .bypassed_actuator_ilcs;
        assert!(model._controller.status.are_ilc_enabled(bypassed_ilcs));

        // Set the open-loop mode (skip the telemetry mode)
        client_write_and_sleep(
            &mut client_command_gui,
            "{\"id\":\"cmd_setClosedLoopControlMode\",\"sequence_id\":4,\"mode\":3}\r\n",
            SLEEP_TIME,
        );

        wait_for_command_result_and_reset(&mut model);

        assert_eq!(
            model._controller.status.mode,
            ClosedLoopControlMode::OpenLoop
        );

        // Turn on the force balance system
        client_write_and_sleep(
            &mut client_command_gui,
            "{\"id\":\"cmd_switchForceBalanceSystem\",\"sequence_id\":5,\"status\":true}\r\n",
            SLEEP_TIME,
        );

        wait_for_command_result_and_reset(&mut model);

        assert_eq!(
            model._controller.status.mode,
            ClosedLoopControlMode::ClosedLoop
        );

        // Wait for the mirror to be in position
        loop {
            model.step();

            if let Some(telemetry) = &model._controller.last_effective_telemetry.control_loop {
                if telemetry.is_in_position {
                    break;
                }
            }
        }

        assert_eq!(
            client_read_specific_json(&mut client_command_gui, TERMINATOR, "m2AssemblyInPosition"),
            json!({"id": "m2AssemblyInPosition", "inPosition": true})
        );

        model.stop();
    }

    #[test]
    fn test_run_processes_and_update_elevation_anle() {
        let mut model = create_model();

        model.run_processes();

        // Create the clients to connect the servers
        let (mut client_command_csc, mut client_telemetry_csc) =
            create_tcp_clients(model._ports["csc_command"], model._ports["csc_telemetry"]);

        sleep(Duration::from_millis(MAX_TIMEOUT));

        wait_for_connection(&mut model);

        // Issue a command to the power on the communication system
        client_write_and_sleep(
            &mut client_command_csc,
            "{\"id\":\"cmd_power\",\"sequence_id\":1,\"status\":true,\"powerType\":2}\r\n",
            SLEEP_TIME,
        );

        wait_for_command_result_and_reset(&mut model);

        // Write the telemetry of the external elevation angle.
        client_write_and_sleep(
            &mut client_telemetry_csc,
            "{\"id\":\"tel_elevation\",\"compName\":\"mtmount\",\"actualPosition\":12.34}\r\n",
            SLEEP_TIME,
        );

        let max_num = 100;
        let mut idx = 0;
        for _ in 0..max_num {
            idx += 1;
            model.step();

            let elevation = model
                ._controller
                .last_effective_telemetry
                .control_loop
                .clone()
                .unwrap()
                .inclinometer["external"];

            if elevation == 12.34 {
                break;
            }
        }

        assert!(idx < max_num);

        model.stop();
    }

    #[test]
    fn test_run_processes_and_step_gui_become_commander() {
        let mut model = create_model();

        model.run_processes();

        // Create the clients to connect the servers
        let (mut client_command_gui, _client_telemetry_gui) =
            create_tcp_clients(model._ports["gui_command"], model._ports["gui_telemetry"]);

        let (mut client_command_csc, _client_telemetry_csc) =
            create_tcp_clients(model._ports["csc_command"], model._ports["csc_telemetry"]);

        sleep(Duration::from_millis(MAX_TIMEOUT));

        wait_for_connection(&mut model);
        consume_welcome_messages(&model, &mut client_command_gui);
        consume_welcome_messages(&model, &mut client_command_csc);

        // Issue a command
        client_write_and_sleep(
            &mut client_command_gui,
            "{\"id\":\"cmd_clearErrors\",\"sequence_id\":1}\r\n",
            SLEEP_TIME,
        );

        client_read_and_assert(
            &mut client_command_gui,
            "{\"id\":\"ack\",\"sequence_id\":1}\r\n",
        );

        wait_for_command_result_and_reset(&mut model);

        client_read_and_assert(
            &mut client_command_gui,
            "{\"id\":\"success\",\"sequence_id\":1}\r\n",
        );

        // Command should change to the GUI.
        assert_eq!(model._controller.commander, Commander::GUI);

        // Command from the CSC should fail.
        client_write_and_sleep(
            &mut client_command_csc,
            "{\"id\":\"cmd_clearErrors\",\"sequence_id\":2}\r\n",
            SLEEP_TIME,
        );

        client_read_and_assert(
            &mut client_command_csc,
            "{\"id\":\"ack\",\"sequence_id\":2}\r\n",
        );

        wait_for_command_result_and_reset(&mut model);

        client_read_and_assert(
            &mut client_command_csc,
            "{\"id\":\"fail\",\"sequence_id\":2}\r\n",
        );

        model.stop();
    }

    #[test]
    fn test_update_status_telemetry_power() {
        let mut telemetry = TelemetryPower::new();
        telemetry.digital_output = TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR;
        telemetry.digital_input = TEST_DIGITAL_INPUT_POWER_COMM_MOTOR;

        let mut model = create_model();

        let events = model.update_status_telemetry_power(&telemetry);

        assert_eq!(
            events,
            vec![
                Event::get_message_digital_output(TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR),
                Event::get_message_digital_input(TEST_DIGITAL_INPUT_POWER_COMM_MOTOR),
                Event::get_message_interlock(false),
            ]
        );
    }

    #[test]
    fn test_update_status_telemetry_control_loop() {
        let mut telemetry = TelemetryControlLoop::new();
        telemetry.is_in_position = true;

        let mut model = create_model();

        let events = model.update_status_telemetry_control_loop(&telemetry);

        assert_eq!(events, vec![Event::get_message_in_position(true),]);
    }

    #[test]
    fn test_get_welcome_messages() {
        let model = create_model();

        let messages = Model::get_welcome_messages(&model._controller);

        assert_eq!(messages.len(), 102);
    }

    #[test]
    fn test_get_welcome_messages_multiple_times_connection() {
        let mut model = create_model();
        model.run_processes();

        for _ in 0..2 {
            // Create the clients to connect the servers
            let (mut client_command, _client_telemetry) =
                create_tcp_clients(model._ports["gui_command"], model._ports["gui_telemetry"]);

            sleep(Duration::from_millis(MAX_TIMEOUT));

            wait_for_connection(&mut model);

            assert_eq!(consume_welcome_messages(&model, &mut client_command), 102);

            // Client disconnects the servers
            let _ = client_command.shutdown(Shutdown::Both);
            let _ = _client_telemetry.shutdown(Shutdown::Both);
        }

        model.stop();
    }
}
