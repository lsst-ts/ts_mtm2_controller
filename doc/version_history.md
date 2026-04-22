# Version History

0.4.4

- Add the `PowerSystem.update_digital_input_based_on_voltage()`.
- Add the `warning_voltage_level` and `fault_voltage_level` fields to **ConfigPower**.
- Update the `Event.get_message_config()` to use the data in **ConfigPower**.
- Add the **MOCK_CODE_ILC_ERROR** and **MOCK_CODE_ILC_EXCEPTION**.
- Add the `InnerLoopController.get_mode_value()` and `InnerLoopController.get_mode_from_value()`.
- Support the ModBus frame in **MockInnerLoopController**.
- Update the ILC data and do the request in **MockPlant**.
- Apply the ModBus frame in **DataAcquisition**.
- Use the `MockPlant.get_actuator_forces()` in **ClosedLoop**.
- Update the `Event.get_message_config()` to use the data in **ConfigPower**.
- Update the data type in **Event**, **CommandDataAcquisition**, and **Model**.
- Remove the `frequency_send_telemetry` from `parameters_daq.yaml`.
- Remove the `ConfigDataAcquisition.frequency_send_telemetry`.
- Change the unit of `max_value_displacement_sensor` and `min_value_displacement_sensor` in `parameters_control.yaml`.
- Add the `max_value_temperature_cell`, `min_value_temperature_cell`, `max_value_temperature_mirror`, and `min_value_temperature_mirror` in `parameters_control.yaml`.
- Add the `ErrorHandler.is_temperature_out_of_range()`.
- Update the `class_diagram.md`.

0.4.3

- Update the **tangent_link_total_weight_error** to be 5000 N from 2000 N in `config/parameters_control.yaml`.

0.4.2

- Add the `crc` dependency.
- Add the ILC related constants and enums to `constants.rs` and `enums.rs`.
- Add the `inner_loop_controller.rc`.
- Update the `parameters_control.yaml` and **Config** to have the limit of raw ILC values.
- Update the `parameters_daq.yaml` and **ConfigDataAcquisition** to check the ILC stale data.
- Update the **TelemetryControlLoop** to have the `ilc_error_codes` field.
- Update the **DataAcquisition** to hold the latest ILC telemetry, which will be used when working with the real ILC data.
- Update the **DataAcquisitionProcess** to end the default digital output when shutting down the process.
- Update the **ErrorHandler** to check the raw ILC values and add the error from `TelemetryControlLoop.ilc_error_codes`.
- Update the `class_diagram.md`.
- Improve the `Jenkinsfile`.

0.4.1

- Use the **ts_control_utils**.

0.4.0

- Put the **timeout** value to be 50 ms in `parameters_app.yaml`.
- Improve the clippy format.
- Put the `MockPlant.calculate_ims_readings()` to be static.
- Add the `disp_matrix_inv` field in **Config**.
- Remove the `_is_simulation_mode` field in **OpenLoop**.
- Remove the simulation of **MockPlant** from the **ControlLoop**.
The related simulation goes to the **DataAcquisition** instead.
- Add the command of data acquisition to the **Model**.
- Use the `Receiver.recv_timeout()` instead of `Receiver.try_recv()` in **CommandServer**,  **TelemetryServer**, and **DataAcquisitionProcess** (in the test).
- Use the `Receiver.recv_timeout()` in the tests of **PowerSystemProcess**.
- Add the `seq_id_move_actuator_steps` field in **CommandMoveActuatorSteps** and **TelemetryControlLoop**.
The **DataAcquisition** will cache this value.
- Fix the switch of commander in `model.rs`.

0.3.2

- Update the dependencies.
- Format the code with the `clippy`.
- Add the `clippy` to the `pre-commit` and `Dockerfile`.
- Add the linting code stage in `Jenkinsfile`.

0.3.1

- Remove the **MockPlant** from the **PowerSystem**.
The related simulation goes to the **DataAcquisition** instead.

0.3.0

- Add the `config/parameters_daq.yaml`.
- Add the `config_data_acquisition.rs` and `data_acquisition_process.rs`.
- Run the data acquisition process in `model.rs.`
- Update the `class_diagram.md` and `communication_diagram.md`.

0.2.9

- Add the `data_acquisition.rs` and `command_data_acquisition.rs`.
- Update the `class_diagram.md`.

0.2.8

- Add the chrono dependency.
- Add the generate_log_file_name() in `main.rs`.
- Add the `m2` init file.

0.2.7

- Update the packages.
- Update the **PowerSystem** to track the communication and motor power systems individually.
- Update the **ControlLoopProcess** that it does not report the command result for the internal command.
- Remove the **ConnectionStatus**.
- Support the safe mode when the controller loses the connection with clients.

0.2.6

- Fix the digital input of interlock bit in **MockPlant**.
- Add the **is_boost_current_fault_enabled** to `parameters_power.yaml`.
- Check the power health and interlock in the power system and error handler.
If there is issue, fail the power command.
- Improve the `TcpServer.write_jsons()` and `TcpServer.flush()` to consider the system resource of **std::io::ErrorKind::WouldBlock**.
- Fix the `CommandSetExternalElevation.execute()` to use the lower case.
- Log the current closed-loop control mode.
- Improve the control loop process to make sure to process the non-telemetry command when there is the telemetry in each loop.

0.2.5

- Track the power command status.
- Check the power-on status with the breaker status (digital out value).

0.2.4

- Add the **telemetry_stable_time** to `parameters_power.yaml`.
- Update the **ConfigPower** to have the expected timing in power on/off process.
- Improve the logic of breakers in **MockPlant** to have the more realistic behavior.
- Update the `SubPowerSystem.transition_state()` with the appropriate state machine that tracks the power on/off process based on the timing and others.
- Improve the **PowerSystem** to rely on **SubPowerSystem** to do the state transition.
- Add the `disableConcurrentBuilds()` to the **Jenkinsfile**.

0.2.3

- Add the `mock_power_system.rs`.

0.2.2

- Improve the commands to log the error messages.

0.2.1

- Support the mirror position file.

0.2.0

- Read the actuator and cell mapping files.

0.1.1

- Add the Jenkinsfile, Dockerfile, and license file.

0.1.0

- Initial version to support the simulation mode.
