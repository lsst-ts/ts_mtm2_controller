# M2 Controller

This is the M2 control system.

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

You can interrupt the running application by `ctrl` + `c`.

To get more information, do:

```bash
cargo run --bin run_m2 -- -h
```

To run the test FPGA code in the cRIO, do:

```bash
cargo run --features fpga --bin test_fpga
```

The system should look for the `/usr/lib/x86_64-linux-gnu/libNiFpga.so` by itself at run time.
If not, do:

```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
```

## Build the Executable

Do the following to build the executable:

```bash
cargo build --release
```

This will generate an optimized executable in the `target/release/` directory, which is suitable for distribution.

## FPGA Files

You should put the FPGA files in the `fpga/` directory.
They are generated from the bifile of [ts_mtm2_cell](https://github.com/lsst-ts/ts_mtm2_cell).
See [FPGA Interface C API User Manual](https://www.ni.com/docs/en-US/bundle/fpga-interface-c/page/user-manual-welcome.html) for more details.
You can also see the page: [A little C with your Rust](https://docs.rust-embedded.org/book/interoperability/c-with-rust.html) for the interoperation between the Rust and C library.

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

## Script

Some useful scripts are in `script/` directory.

1. `m2`: Initialization file in the Linux system.

## Log Data

The logging files contain the mirror position are in the `log/` directory.

## System Log

While the application is running, you can see the system log in the `log/application.log`.
The logging level can be changed by the `-l` option when starting the application.

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

## Version History

See [here](doc/version_history.md) for the version history.
