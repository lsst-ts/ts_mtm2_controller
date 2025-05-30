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
cargo run -- -s
```

You can interrupt the running application by `ctrl` + `c`.

To get more information, do:

```bash
cargo run -- -h
```

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
- [disp_ims.yaml](config/disp_ims.yaml) has the information of displacement sensors used in the independent measurement system (IMS).
- [stiff_matrix_m2.yaml](config/stiff_matrix_m2.yaml) is the stiffness matrix of M2 mirror.
- [stiff_matrix_surrogate.yaml](config/stiff_matrix_surrogate.yaml) is the stiffness matrix of surrogate.

## System Log

While the application is running, you can see the system log in the `application.log`.
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

## Software Architecture

See [here](doc/README.md) for the design of software.

## UML Diagrams

The UML diagrams are used to detail the system design for each subsystem in the `doc/` directory.
The GitHub supports the [Mermaid](https://github.com/mermaid-js/mermaid) natively.
You can use the [online editor](https://mermaid.live) to edit them.

## Version History

See [here](doc/version_history.md) for the version history.
