# Version Histroy

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
