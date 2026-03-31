# Class Diagram

The [Model](../src/model.rs) holds the [Controller](../src/controller.rs) to do the main login in the application.
There are the following modules in the control system:

- [daq](#daq)
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
  class CommandDataAcquisition
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

namespace daq {
  class DataAcquisitionProcess
  class InnerLoopController
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
Model ..> CommandDataAcquisition
Model ..> Event
Model ..> Telemetry
Model ..> TelemetryPower
Model ..> TelemetryControlLoop
Model ..> MockPlant
Model ..> ControlLoopProcess
Model ..> PowerSystemProcess
Model ..> CommandTelemetryServer
Model ..> DataAcquisitionProcess

Controller *-- Status
Controller *-- ErrorHandler
Controller *-- EventQueue
Controller *-- Telemetry
Controller ..> Event
Controller ..> Config
Controller ..> Lut
Controller ..> CommandPowerSystem
Controller ..> CommandControlLoop
Controller ..> CommandDataAcquisition

Status *-- ConnectionStatus
Status "1" *-- "2" SubPowerSystem

Config *-- Lut

ErrorHandler *-- Config
ErrorHandler *-- ConfigPower
ErrorHandler "1" *-- "78" Actuator
ErrorHandler ..> TelemetryControlLoop
ErrorHandler ..> InnerLoopController
```

## Daq

The [daq](../src/daq/) module implements the data acquisition process:

```mermaid
classDiagram

namespace main {
  class EventQueue
}

namespace mock {
  class MockPlant
}

namespace telemetry {
  class Event
  class TelemetryPower
  class TelemetryControlLoop
}

Telemetry o-- TelemetryPower
Telemetry o-- TelemetryControlLoop

namespace command {
  class CommandSchema
  class CommandDataAcquisition
}

CommandSchema "1" *-- "n" CommandDataAcquisition

DataAcquisitionProcess *-- DataAcquisition
DataAcquisitionProcess *-- CommandSchema
DataAcquisitionProcess ..> CommandDataAcquisition
DataAcquisitionProcess ..> Telemetry

CommandDataAcquisition --> DataAcquisition

DataAcquisition *-- ConfigDataAcquisition
DataAcquisition o-- MockPlant
DataAcquisition *-- EventQueue
DataAcquisition *-- InnerLoopController
DataAcquisition *-- TelemetryControlLoop
DataAcquisition ..> Event
DataAcquisition ..> TelemetryPower
```

## Power

The [power](../src/power/) module implements the power system:

```mermaid
classDiagram

namespace main {
  class EventQueue
  class ErrorHandler
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
  class CommandDataAcquisition
}

CommandSchema "1" *-- "n" CommandPowerSystem

PowerSystemProcess *-- PowerSystem
PowerSystemProcess *-- CommandSchema
PowerSystemProcess ..> CommandPowerSystem
PowerSystemProcess ..> CommandDataAcquisition
PowerSystemProcess ..> Telemetry

CommandPowerSystem --> PowerSystem

PowerSystem *-- ConfigPower
PowerSystem "1" *-- "2" SubPowerSystem
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
  class CommandDataAcquisition
}

CommandSchema "1" *-- "n" CommandControlLoop

ControlLoopProcess ..> Config
ControlLoopProcess *-- ControlLoop
ControlLoopProcess *-- CommandSchema
ControlLoopProcess ..> CommandControlLoop
ControlLoopProcess ..> CommandDataAcquisition
ControlLoopProcess ..> Telemetry

CommandControlLoop --> ControlLoop

ControlLoop *-- Config
ControlLoop *-- ClosedLoop
ControlLoop *-- OpenLoop
ControlLoop *-- TelemetryControlLoop
ControlLoop *-- EventQueue
ControlLoop ..> MockPlant
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

namespace daq {
  class DataAcquisition
}

CommandSchema "1" *-- "n" Command

Command ..> Config
Command --> Controller
Command --> PowerSystem
Command --> ControlLoop
Command --> DataAcquisition
Command <|-- CommandController
Command <|-- CommandPowerSystem
Command <|-- CommandControlLoop
Command <|-- CommandDataAcquisition
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
