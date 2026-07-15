# M2 Controller

This is the M2 control system.

## Features

The available features are:

- fpga: Run the application with the use of the NI FPGA dynamic library.
- realtime: Run the application with the realtime feature (in Linux only).

Note: When doing the `cargo check`, make sure to check each feature under the required software/hardware requirement.

## Development Environment

You can develop the code under the Windows, Mac, and Linux.
For the Windows and Mac, disable all the features by default.
For the Linux, if you have the realtime support in OS, you can enable the `realtime` feature.
To enable the `fpga` feature, make sure you have the expected hardware environment to load the FPGA bitfile (see the [ts_mtm2_cell](https://github.com/lsst-ts/ts_mtm2_cell)).

## Install the Rust in cRIO and Run the Application

Follow [here](https://www.rust-lang.org/tools/install) to install the Rust in cRIO.

To avoid the stack overflow, do the following in the cRIO:

```bash
ulimit -s unlimited
```

To run the application in the simulation mode by `cargo`, do:

```bash
cargo run --bin run_m2 -- -s
```

If you are using the cRIO simulator, you can do the following instead:

```bash
cargo run --features realtime --bin run_m2 -- -s
```

You can interrupt the running application by `ctrl` + `c`.

To run the application in the hardware mode with FPGA by `cargo`, do:

```bash
cargo run --features fpga --bin run_m2
```

Since the FPGA bitfile comes from [ts_mtm2_cell](https://github.com/lsst-ts/ts_mtm2_cell), you need to make sure your target model is the same.

To get more information, do:

```bash
cargo run --bin run_m2 -- -h
```

To run the test FPGA code in the cRIO, do:

```bash
cargo run --features fpga --bin test_fpga
```

The system should look for the `/usr/lib/x86_64-linux-gnu/libNiFpga.so` by itself at compile time.
See the [build.rs](build.rs).

## Executables

The followings are the executables:

- [main](src/main.rs): M2 controller.
- [test_fpga](src/bin/test_fpga.rs): Test script of FPGA for the M2 cRIO simulator in the electronic lab.
This cRIO is the same model of the current cRIO of M2.
It has the same NI modules as well.

## Build the Executable

Do the following to build the M2 controller executable:

```bash
cargo build --release --features fpga,realtime
```

For the cRIO simulator, do the following instead (to enable the `realtime` feature is optional):

```bash
cargo build --release --features realtime
```

This will generate an optimized executable in the `target/release/` directory, which is suitable for distribution.

## FPGA Files

You should put the FPGA files in the `fpga/` directory.
They are generated from the bifile of [ts_mtm2_cell](https://github.com/lsst-ts/ts_mtm2_cell).
See [FPGA Interface C API User Manual](https://www.ni.com/docs/en-US/bundle/fpga-interface-c/page/user-manual-welcome.html) for more details.
Although the raw dynamic library is used and you do not really compile the NI FPGA C code, it is good to have the generated header file ([NiFpga_portSerialMasterSlave.h](fpga/NiFpga_portSerialMasterSlave.h)) to get the register offsets.
Otherwise, you need to read the NI FPGA bitfile (an xml file) to get the required offsets.

Since the safety module needs the NI FPGA hybrid mode and the NI raw dynamic library only allows to load the bitfile of pure FPGA mode, we need to use the LabVIEW application to load the bitfile first to let this Rust-based application to be able to open the session of FPGA to control the hardware as a workaround at the moment.
Hopefully the NI can support this in the future.

## Deployment

The details can follow [deployment](doc/deployment.md).

## Configuration Files

See the [config/](config) directory for the configuration files:

- [cell/actuator](config/cell/actuator/) directory has the actuator configuration files.
- [lut/](config/lut/) directory has the look-up tables of gravity and temperature.
  - [handling/](config/lut/handling/) directory is for the handling of mirror.
  - [optical/](config/lut/optical/) directory is for the optical imaging.
- [parameters_app.yaml](config/parameters_app.yaml) is the configuration of application.
- [parameters_control.yaml](config/parameters_control.yaml) is the configuration of control loop.
- [parameters_power.yaml](config/parameters_power.yaml) is the configuration of power system.
- [cell_geom.yaml](config/cell_geom.yaml) is the cell geometry.
- [cell_actuator_mapping.yaml](config/cell/cell_actuator_mapping.yaml) is the mapping between the cell and actuators.
- [home_position.yaml](config/home_position.yaml) is the home position of mirror.
- [disp_ims.yaml](config/disp_ims.yaml) has the information of displacement sensors used in the independent measurement system (IMS).
- [stiff_matrix_m2.yaml](config/stiff_matrix_m2.yaml) is the stiffness matrix of M2 mirror.
- [stiff_matrix_surrogate.yaml](config/stiff_matrix_surrogate.yaml) is the stiffness matrix of surrogate.
- [logspecification.toml](config/logspecification.toml) assigns the log level.

## Script

Some useful scripts are in `script/` directory.

1. `m2`: Initialization file in the Linux system.
Note the run commands between the cRIO simulator and the real hardware are different.
You need to do the related modification in the script.

## Log Data

The logging files contain the mirror position are in the `log/` directory.
You can change the log level in the runtime by modifying the [logspecification.toml](config/logspecification.toml).
The logging files are rotated, and the related parameters are in the [parameters_app.yaml](config/parameters_app.yaml).
You can adjust the log level of each module individually.

To log the telemetry to the `log/` directory, put the `local_telemetry_file` to be `true` in [parameters_app.yaml](config/parameters_app.yaml).
This is for the specific use for the data analysis only if the summit engineering feasibility database (EFD) is not available or not enough.
Usually you should not need it.
Note you need to rerun the application after changing the setting.

## Read the Telemetry Binary File by Python

The [telemetry_file.py](python/telemetry_file.py) is used to decode the telemetry structure in [telemetry_file.rs](src/telemetry/telemetry_file.rs).
Their structures should be consistent with each other.
To read the telemetry binary file, do the followings under the `python/` directory:

```python
from telemetry_file import TelemetryFile
telemetry = TelemetryFile.deserialize_from_file("path_to_telemetry_binary_file")
```

The output is a list of `TelemetryFile` object defined in the above `telemetry_file.py`.

You might need to install the `msgpack` package in Python such as:

```bash
conda install conda-forge::msgpack-python
```

## Code Format

To format the code, do:

```bash
.githooks/pre-commit
```

## Docker File

The docker file is [here](dockerImage/Dockerfile) that contains the dependencies to generate the test and coverage reports to support the CI integration.

## Unit Test

Each module and function have the related unit tests.
Since the CI test is needed, you can use the [cargo-nextest](https://crates.io/crates/cargo-nextest) instead of the built-in test framework.
Do the following to run all tests:

```bash
cargo nextest run
```

To test a single module, do:

```bash
cargo nextest run --lib $module_name
```

To generate the `junit.xml` (ouput path is `target/nextest/ci/junit.xml`), do:

```bash
cargo nextest run --profile ci
```

To run the FPGA related test in cRIO, add the `--features fpga` flag when running the test.
For example, you can run the tests in the cRIO with:

```bash
cargo test fpga_hardware --features fpga -- --test-threads=1
```

Note we need to use the flag `--test-threads=1` here to make sure the system to run the test one by one.
Otherwise, you might get the error code: -52010:

```text
A required resource was not properly initialized. This could occur if NiFpga_Initialize was not called or a required NiFpga_IrqContext was not reserved.
```

## Software Architecture

See [here](doc/README.md) for the design of software.

## UML Diagrams

The UML diagrams are used to detail the system design for each subsystem in the `doc/` directory.
The GitHub supports the [Mermaid](https://github.com/mermaid-js/mermaid) natively.
You can use the [online editor](https://mermaid.live) to edit them.

## Tricky Parts of the Code Tuning with the Inner-Loop Controller (ILC)

For the actuator ILC, if you do not issue the broadcast `step()` command first, the received status/force frame data is just some garbage data.
The status and force will be 0 and the encoder value is some random huge value (out of available encoder range).
For the monitor ILC, you might get the `inf` value when just starting up the ILCs.
We always have this for the temperature ILCs.
Sometime, the displacement ILC gives the `inf` value as well.

## ILC Communication Protocol

The ILC communication protocol is defined in:

1. LSST-ILC Firmware: MODBUS Protocol Interface Control Document for M2 Support System
2. LTS-346, ILC Communications Protocol For M2 Support System.

## Realtime Thread Support

The data acquisition process is designed to support the realtime thread because:

1. It needs to send a 10 Hz signal to the safety module reliably when the mirror control system is under the closed-loop control.
If the safety module does not receive this signal, it will trigger the global interlock system (GIS) signal to stop the telescope motion.
2. It needs to send the current power status (voltage and current) to the power system process reliably to check the health of the power system, especially when powering on/off the system.

Notes:

1. The low-level PID controller is in the ILC.
The control system only sends the `step()` command to the ILC.
Therefore, in the theory, the data acquisition process does not need to be realtime for the hardware control with the ILC.
2. The control loop process will be woken up as soon as there is the new telemetry from the data acquisition process.
Therefore, although the requirement document specifies the bandwidth of control loop to be 20 Hz, we do not need to make the control loop process to be realtime thread since the data acquisition process is 20 Hz as well.
This can save the system resource significantly.

## Version History

See [here](doc/version_history.md) for the version history.
