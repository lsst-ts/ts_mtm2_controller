# Class Diagram

The [Model](../src/model.rs) holds the [Controller](../src/controller.rs) to do the main login in the application.
There are the following modules in the control system:

- [power](#power)
- [control](#control)
- [interface](#interface)
- [telemetry](#telemetry)
- [command](#command)
- [mock](#mock)

Show the main class diagram below:

```mermaid
classDiagram

namespace telemetry {
  class Event
  class Telemetry
  class TelemetryPower
  class TelemetryControlLoop
}

Telemetry o-- TelemetryPower
Telemetry o-- TelemetryControlLoop

namespace command {
  class CommandSchema
  class CommandController
  class CommandPowerSystem
  class CommandControlLoop
}

CommandSchema "1" *-- "n" CommandController

namespace control {
  class Lut
  class ControlLoopProcess
  class Actuator
}

namespace power {
  class ConfigPower
  class SubPowerSystem
  class PowerSystemProcess
}

namespace interface {
  class CommandTelemetryServer
}

namespace mock {
  class MockPlant
}

Model *-- Controller
Model *-- CommandSchema
Model ..> CommandController
Model ..> CommandPowerSystem
Model ..> CommandControlLoop
Model ..> Event
Model ..> Telemetry
Model ..> TelemetryPower
Model ..> TelemetryControlLoop
Model ..> MockPlant
Model ..> ControlLoopProcess
Model ..> PowerSystemProcess
Model ..> CommandTelemetryServer

Controller *-- Status
Controller *-- ErrorHandler
Controller *-- EventQueue
Controller *-- Telemetry
Controller ..> Event
Controller ..> Config
Controller ..> Lut

Status *-- ConnectionStatus
Status "1" *-- "2" SubPowerSystem

Config *-- Lut

ErrorHandler *-- Config
ErrorHandler *-- ConfigPower
ErrorHandler "1" *-- "78" Actuator
ErrorHandler ..> TelemetryControlLoop
```

## Power

The [power](../src/power/) module implements the power system:

```mermaid
classDiagram

namespace main {
  class EventQueue
  class ErrorHandler
}

namespace mock {
  class MockPlant
}

namespace telemetry {
  class Event
  class Telemetry
  class TelemetryPower
}

Telemetry o-- TelemetryPower

namespace command {
  class CommandSchema
  class CommandPowerSystem
}

CommandSchema "1" *-- "n" CommandPowerSystem

PowerSystemProcess *-- PowerSystem
PowerSystemProcess *-- CommandSchema
PowerSystemProcess ..> CommandPowerSystem
PowerSystemProcess ..> Telemetry

CommandPowerSystem --> PowerSystem

PowerSystem *-- ConfigPower
PowerSystem "1" *-- "2" SubPowerSystem
PowerSystem o-- MockPlant
PowerSystem *-- EventQueue
PowerSystem ..> Event
PowerSystem --> TelemetryPower
PowerSystem ..> ErrorHandler
```

## Control

The [control](../src/control/) module implements the control loop:

```mermaid
classDiagram

namespace main {
  class Config
  class EventQueue
}

namespace mock {
  class MockPlant
}

namespace telemetry {
  class Event
  class Telemetry
  class TelemetryControlLoop
}

Telemetry o-- TelemetryControlLoop

namespace command {
  class CommandSchema
  class CommandControlLoop
}

CommandSchema "1" *-- "n" CommandControlLoop

ControlLoopProcess ..> Config
ControlLoopProcess *-- ControlLoop
ControlLoopProcess *-- CommandSchema
ControlLoopProcess ..> CommandControlLoop
ControlLoopProcess ..> Telemetry

CommandControlLoop --> ControlLoop

ControlLoop *-- Config
ControlLoop *-- ClosedLoop
ControlLoop *-- OpenLoop
ControlLoop *-- TelemetryControlLoop
ControlLoop *-- EventQueue
ControlLoop o-- MockPlant
ControlLoop ..> Event

Config *-- Lut

ClosedLoop "1" *-- "4" BiquadraticFilter
ClosedLoop "1" *-- "3" SimpleDelayFilter
ClosedLoop "1" *-- "2" DeadbandControl
ClosedLoop *-- GainSchedular
ClosedLoop *-- InPosition

BiquadraticFilter "1" *-- "n" SingleBiquadraticFilter

OpenLoop "1" *-- "78" Actuator
```

## Interface

The [interface](../src/interface/) module supports the TCP/IP communication:

```mermaid
classDiagram

namespace telemetry {
  class Event
}

CommandTelemetryServer --> CommandServer
CommandTelemetryServer --> TelemetryServer
CommandTelemetryServer --> TcpServer
CommandTelemetryServer ..> Event

CommandServer --> TcpServer
TelemetryServer --> TcpServer
```

## Telemetry

The [telemetry](../src/telemetry/) module supports the telemetry and event:

```mermaid
classDiagram

namespace main {
  class Config
}

Event ..> Config

TelemetryDefault <|-- TelemetryPower
TelemetryDefault <|-- TelemetryControlLoop

Telemetry o-- TelemetryPower
Telemetry o-- TelemetryControlLoop
```

## Command

The [command](../src/command/) module implements the command factory:

```mermaid
classDiagram

namespace main {
  class Config
  class Controller
}

namespace power {
  class PowerSystem
}

namespace control {
  class ControlLoop
}

CommandSchema "1" *-- "n" Command

Command ..> Config
Command --> Controller
Command --> PowerSystem
Command --> ControlLoop
Command <|-- CommandController
Command <|-- CommandPowerSystem
Command <|-- CommandControlLoop
```

## Mock

The [mock](../src/mock/) module supports the simulation mode:

```mermaid
classDiagram

namespace power {
  class ConfigPower
}

MockPlant ..> ConfigPower
MockPlant "1" *-- "2" MockPowerSystem
MockPlant "1" *-- "84" MockInnerLoopController
```
