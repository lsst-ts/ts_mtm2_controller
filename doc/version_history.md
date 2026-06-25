# Version History

0.5.5

- Add the new ILC command codes to `constants.rs`.
- Move the `get_f32_values_from_u8_array()` to `utility.rs`.
- Add the **CalibrationData** class.
- Add the **ServerIdentifier** class.
- Add new ILC commands to **InnerLoopController**.
- Add mock constants to `mock_constants.rs`.
- Add new ILC commands to **MockInnerLoopController**.
- Update the `MockPlant.new()` to assign the unique IDs to mock ILCs.
- Add the new ILC events in `event.rs`.
- Add the new payloads and latencies to the `parameters_daq.yaml`.
- Update the `ConfigDataAcquisition.new()`.
- Add the indicator of received function code to `fpga_wrapper.rs`.
- Add the new ILC commands to **DataAcquisition**.
- Add the new ILC commands to `command_data_acquisition.rs`.
- Register new ILC commands to `Model.create_commands()`.
- Update the `class_diagram.md`.
- Update the README files.

0.5.4

- Update the log level in `Model.check_condition_control_loop()`.
- Fix the bug that the raw ILC telemetry might not be processed when the received step command sequence id is not expected in `ControlLoopProcess.run()`.
- Update the `DataAcquisitionProcess.run()` to continue to process the command after receiving the set mode command.
- Update the `README.md`.

0.5.3

- Add the `libc` dependency.
- Add the `realtime` feature to run the data acquisition process as a realtime thread.
- Update the `script/m2` script.
- Update the debug level in `FpgaWrapper.map_ilc_error_code()`.
- Update the debug level in `DataAcquisition.check_ilc_stale_data()`.
- Add modules to `logspecification.toml`.
- Fix the max counter of debug in `DataAcquisitionProcess.run()`.
- Update the `README.md`.

0.5.2

- Update the `ErrorHandler.check_condition_control_loop()` to only check the actuator ILC after broadcasting the `step()` command to actuator ILCs.
- Add the debug messages in **ErrorHandler**.
- Update the `ErrorHandler.check_cycle_time()` to pass the cycle_time as milliseconds.
- Update the `DataAcquisition.check_ilc_stale_data()` that only check the communication counter after broadcasting the `step()` command to ILCs.
- Reset the `DataAcquisition._seq_id_move_actuator_steps` when transitioning to the **Idle** mode.
- Add the `ControlLoop.get_control_mode()`.
- Use the cached ILC monitor data if the received values are inf in **DataAcquisition**.
- Reset the `seq_id_move_actuator_steps` in `ControlLoopProcess.run()`.
- Improve the error message in **SubPowerSystem**.
- Update the **output_voltage_fall_time_communication** and **output_voltage_fall_time_motor** in `parameters_power.yaml`.
- Change the unit of `TelemetryControlLoop.cycle_time` to be milliseconds.
- Add the debug message for the loop time in **DataAcquisitionProcess** and record the `cycle_time`.
- Compare and record the cycle times of **ControlLoopProcess** and **DataAcquisitionProcess**.
- Simulate the ILC latency in **DataAcquisition**.

0.5.1

- Use the `flexi_logger` to replace the `simplelog` dependency.
- Add the `logspecification.toml`.
- Support the log file rotation and the change of log level in the runtime.

0.5.0

- Add the **NUM_ILC_TEMPERATURE_MONITOR_SENSOR** to `constants.rs`.
- Add the **sleep_time_broadcast_ilc**, **sleep_time_ilc_reading**, and **bypass_check_stale_inclinometer** to `parameters_daq.yaml`.
- Rename the **actuator_ilc_stale_data_limit** to be **ilc_stale_data_limit** in `parameters_daq.yaml`.
- Update the **ConfigDataAcquisition**.
- Add the `test_calculate_crc_and_update_frame()` in **InnerLoopController** to compare with the CRC calculation in **ts_mtm2_cell**.
- Update the **DataAcquisition** to have a sleep time after calling the ILC command and deal with the failed ILC reading.
- Remove the unused fields of **ErrorCode** enum.
- Add the **CommandFault** in `command_controller.rs`.
- Add the **CommandFault** to the **Model**.
- Use the `nifpga-dll` 0.4.0 from the `crates.io`.

0.4.9

- Update the dependency of `nifpga-dll`.
- Remove the `cc` dependency and modify the `build.rs`.
- Remove the C functions in `fpga_wrapper.rs`.

0.4.8

- Update the version of `ts_control_utils`.
- Upgrade the incompatible packages.
- Update the comment in `parameters_control.yaml`.
- Add the **timeout_get_next_character**, **payload_byte_xxx**, and **latency_xxx** in `parameters_daq.yaml`.
- Update the **ConfigDataAcquisition**.
- Update the `enum.rs`.
- Support the ILC command in **FpgaWrapper**.
- Update the `init_hardware()`, `get_ilc_data_actuator()`, `get_ilc_data_temperature()`, `get_ilc_data_displacement()`, `get_ilc_data_inclinometer()`, `get_ilc_mode()`, `set_ilc_mode()`, and `move_actuator_steps()` in **DataAcquisition**.
- Update the `test_fpga.rs`.
- Update the `README.md`.

0.4.7

- Update the dependency of `nifpga-dll`.
- Fix the **PowerSystem** to return the power command result if the system is already on/off.
- Add the **NUMBER_ILC_PORT** and **IRQ_NUMBER_ILC** to `constants.rs`.
- Add the **ModbusMode** and **IlcCommand** in the `enums.rs`.
- Update the `parameters_daq.yaml`.
- Update the **ConfigDataAcquisition**.
- Support the ILC communication and get all the DAQ FIFO data in **FpgaWrapper**.
- Update the `DataAcquisition.get_telemetry_power()` to out the information that the power telemetry is valid or not and update the `DataAcquisition.init_hardware()` with the twice loop rate, IRQ and ModBus setup.
- Update the `DataAcquisitionProcess.run()` to only send the valid power telemetry.
- Update the `test_fpga.rs`.

0.4.6

- Update the dependency of `nifpga-dll`.
- Add the dependency of `fixed`.
- Update the dependencies to the latest ones.
- Fix the `Dockerfile` for the installation of `cargo-nextest`.
- Fix the path in Windows in `command_controller.rs`.
- Update the `parameters_daq.yaml` to add the FIFO configuration.
- Update the **ConfigDataAcquisition**.
- Support the FIFO and read fixed-point in **FpgaWrapper**.
- Update the **DataAcquisition** and **DataAcquisitionProcess** to read the power data.
- Update the `test_fpga.rs`.

0.4.5

- Add the FPGA files.
- Add the dependencies of `cc` and `nifpga-dll`.
- Add the `build.rs`.
- Add the FPGA informations to `parameters_daq.yaml`.
- Add the `fpga_wrapper.rs`.
- Add the `test_fpga.rs`.
- Update the `data_acquisition.rs`.
- Update the `README.md` and `class_diagram.md.`.

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
