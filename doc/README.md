# Software Architecture

This is a multiple-thread application.
After starting the application, the [Model](../src/model.rs) instance will run the processes of power system, control loop, and TCP/IP communication in the background.
The **Model** is running on the main thread of [application](../src/application.rs) with the `Model.step()` function, which has a blocking call (`recv()`) inside to save the CPU resource.
This blocking call is triggered at each time when receiving a new telemetry from the power system or control loop.

There are two clients to connect to the application, which serves as a TCP/IP server.
They are the graphical user interface (GUI) and commandable SAL component (CSC).
The communication between the server and client is by the [JSON](https://www.json.org/json-en.html) packet.
By default, the CSC is the commander to control the system.
However, the GUI can always take over the control from the CSC.
This assures the engineer's ability to support and debug the system in night.

## Command Factory Pattern

The [command factory pattern](../src/command/command_schema.rs) is applied in the system.
There are three categories:

- [controller](#command-of-the-controller)
- [power system](#command-of-the-power-system)
- [control loop](#command-of-the-control-loop)

### Command of the Controller

The controller's command plays the toppest level.
It might trigger the command of power system or control loop internally.
If the received command belongs to the subsystem (e.g. power system or control loop), the `Model` class will pass to the related subsystem directly.

List the commands in the followings:

- CommandClearErrors
- CommandSwitchForceBalanceSystem
- CommandSetTemperatureOffset
- CommandSwitchCommandSource
- CommandEnableOpenLoopMaxLimit
- CommandSaveMirrorPosition
- CommandLoadConfiguration
- CommandSetControlParameters
- CommandSetEnabledFaultsMask
- CommandSetConfigurationFile
- CommandSetHardpointList
- CommandRunScript

See [here](../src/command/command_controller.rs) for the details.

### Command of the Power System

The followings are the commands:

- CommandPower
- CommandResetBreakers
- CommandToggleBitClosedLoopControl
- CommandSwitchDigitalOutput

See [here](../src/command/command_power_system.rs) for the details.

### Command of the Control Loop

The followings are the commands:

- CommandSetClosedLoopControlMode
- CommandApplyForces
- CommandResetForceOffsets
- CommandPositionMirror
- CommandResetActuatorSteps
- CommandMoveActuators
- CommandSetConfig
- CommandSetExternalElevation
- CommandSetMirrorHome
- CommandGetInnerLoopControlMode
- CommandSetInnerLoopControlMode

See [here](../src/command/command_control_loop.rs) for the details.

## Communication Diagram

See [here](communication_diagram.md) for the communication diagram.

## Class Diagram

See [here](class_diagram.md) for the class diagram.
