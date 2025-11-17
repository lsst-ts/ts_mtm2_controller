# Communication Diagram

The communication among threads is mainly by the [multi-producer, single-consumer FIFO queue communication](https://doc.rust-lang.org/std/sync/mpsc/index.html).
The only exception are:

- stopping of threads
- connection status (see [CommandTelemetryServer](../src/interface/command_telemetry_server.rs))

, which are by the thread-safe reference-counting pointer, [Arc (atomically reference counted)](https://doc.rust-lang.org/std/sync/struct.Arc.html).

The main thread is the [application](../src/application.rs) that runs the `Model.step()`.
The [Model](../src/model.rs) class holds the [Controller](../src/controller.rs) instance to send the commands to the [PowerSystemProcess](../src/power/power_system_process.rs), [ControlLoopProcess](../src/control/control_loop_process.rs), and [DataAcquisitionProcess](../src/daq/data_acquisition_process.rs).
The **PowerSystemProcess** and **ControlLoopProcess** can send the commands to the **DataAcquisitionProcess** as well.
The [CommandTelemetryServer](../src/interface/command_telemetry_server.rs) runs the [CommandServer](../src/interface/command_server.rs) and [TelemetryServer](../src/interface/telemetry_server.rs) as the TCP/IP servers.
The **CommandTelemetryServer** runs a monitor loop to check the connection status and feedbacks to the **Model**.
The exchanged data is the [Value](https://docs.rs/serde_json/latest/serde_json/value/index.html) or [Telemetry](../src/telemetry/telemetry.rs).
For each [Sender](https://doc.rust-lang.org/std/sync/mpsc/struct.Sender.html), there is a related [Receiver](https://doc.rust-lang.org/std/sync/mpsc/struct.Receiver.html) in the class or module.

Since the Mermaid does not support the communication diagram yet, we use the class diagram to mimic the data flow with the association relationship.
Each block is a process in the application and the line's guard is the exchanged data.
Show the communication diagram in the following:

```mermaid
classDiagram

class Model {
    _senders_to_tcp
    _receivers_from_tcp
    _sender_to_model
    _receiver_to_model
    Controller: sender_to_power_system
    Controller: sender_to_control_loop
    Controller: sender_to_daq
}

class PowerSystemProcess {
    _sender_to_daq
    _sender_to_model
    _sender_to_power_system
    _receiver_to_power_system
    _sender_telemetry_to_power_system
    _receiver_telemetry_to_power_system
}

class ControlLoopProcess {
    _sender_to_daq
    _sender_to_model
    _sender_to_control_loop
    _receiver_to_control_loop
    _sender_telemetry_to_control_loop
    _receiver_telemetry_to_control_loop
}

class DataAcquisitionProcess {
    _sender_telemetry_to_control_loop
    _sender_telemetry_to_power
    _sender_to_model
    _sender_to_daq
    _receiver_to_daq
}

class CommandTelemetryServer {
    senders_to_tcp
    _sender_from_tcp
}

class CommandServer {
    _sender_to_tcp
    receiver_to_tcp
    sender_from_tcp
}

class TelemetryServer {
    _sender_to_tcp
    receiver_to_tcp
    sender_from_tcp
}

Model --> PowerSystemProcess : Value
Model --> ControlLoopProcess : Value
Model --> CommandServer : Value
Model --> TelemetryServer : Value
Model --> DataAcquisitionProcess : Value

PowerSystemProcess --> Model : Telemetry
PowerSystemProcess --> DataAcquisitionProcess : Value

ControlLoopProcess --> Model : Telemetry
ControlLoopProcess --> DataAcquisitionProcess : Value

DataAcquisitionProcess --> Model : Telemetry

DataAcquisitionProcess --> PowerSystemProcess : TelemetryPower
DataAcquisitionProcess --> ControlLoopProcess : TelemetryControlLoop

CommandTelemetryServer --> Model : Value

CommandServer --> Model : Value
TelemetryServer --> Model : Value
```

The **CommandServer** receives the command and event from the clients and sends to the **Model** to process.
The **TelemetryServer** subscribes the current elevation angle of telescope mount assemble (TMA) and sends to the **Model**.
The **Model** processes the received command, event, and telemetry, and sends to the **PowerSystemProcess**, **ControlLoopProcess**, and **DataAcquisitionProcess** to do the further processing.
They send the command's result, internal event, and telemetry (except the **DataAcquisitionProcess**) to the **Model** to forward to the **CommandServer** and **TelemetryServer**.

For the telemetry, the **DataAcquisitionProcess** sends the raw **TelemetryPower** to the **PowerSystemProcess** to do the processing.
The **PowerSystemProcess** sends the processed **Telemetry** to **Model** to publish the data.
The **DataAcquisitionProcess** also sends the raw inner-loop-controller (ILC) **TelemetryControlLoop** to **ControlLoopProcess** to do the processing.
The **ControlLoopProcess** sends the processed **Telemetry** to **Model** to publish the data.
