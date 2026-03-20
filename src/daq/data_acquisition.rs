// This file is part of ts_mtm2_controller.
//
// Developed for the Vera Rubin Observatory Systems.
// This product includes software developed by the LSST Project
// (https://www.lsst.org).
// See the COPYRIGHT file at the top-level directory of this distribution
// for details of code ownership.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

use log::{error, info};
use std::path::Path;

use crate::constants::{NUM_ACTUATOR, NUM_IMS};
use crate::daq::{
    config_data_acquisition::ConfigDataAcquisition, inner_loop_controller::InnerLoopController,
};
use crate::enums::{
    DataAcquisitionMode, DigitalOutput, DigitalOutputStatus, ErrorCode, InnerLoopControlMode,
    PowerType,
};
use crate::event_queue::EventQueue;
use crate::mock::mock_plant::MockPlant;
use crate::telemetry::{
    event::Event, telemetry_control_loop::TelemetryControlLoop, telemetry_power::TelemetryPower,
};
use crate::utility::read_file_stiffness;

pub struct DataAcquisition {
    // Configuration of the data acquisition
    pub config: ConfigDataAcquisition,
    // Events to publish
    pub event_queue: EventQueue,
    // Data acquisition mode
    pub mode: DataAcquisitionMode,
    // The latest telemetry data from the inner-loop controller (ILC)
    _latest_telemetry: TelemetryControlLoop,
    // Sequence ID of the last move actuator steps command. This is used to let
    // the ControlLoopProcess can synchronize the commands and telemetry (as
    // the results of commands).
    _seq_id_move_actuator_steps: i32,
    // Inner-loop controller
    _ilc: InnerLoopController,
    // The counts of stale data for each actuator ILC. This is used to check if
    // the ILC data is stale for too long.
    _actuator_ilc_stale_data_counts: Vec<i32>,
    // Plant model
    pub plant: Option<MockPlant>,
}

impl DataAcquisition {
    /// Create a new Data Acquisition instance. This communicates with the
    /// hardwares.
    ///
    /// # Arguments
    /// * `is_simulation_mode` - A boolean indicating whether to run in
    ///   simulation mode.
    ///
    /// # Returns
    /// A new instance of DataAcquisition.
    pub fn new(is_simulation_mode: bool) -> Self {
        // Plant model
        let plant = if is_simulation_mode {
            // Use the M2 stiffness matrix here intentionally to match the
            // simulation mode of ts_mtm2_cell.
            let mock_plant = MockPlant::new(
                &read_file_stiffness(Path::new("config/stiff_matrix_m2.yaml")),
                // Zenith angle by default
                90.0,
            );

            Option::Some(mock_plant)
        } else {
            Option::None
        };

        Self {
            config: ConfigDataAcquisition::new(),

            event_queue: EventQueue::new(),

            mode: DataAcquisitionMode::Idle,
            _latest_telemetry: TelemetryControlLoop::new(),

            _seq_id_move_actuator_steps: 0,

            _ilc: InnerLoopController::new(),
            _actuator_ilc_stale_data_counts: vec![0; NUM_ACTUATOR],

            plant,
        }
    }

    /// Is the simulation mode or not.
    ///
    /// # Returns
    /// True if the simulation mode is enabled. Otherwise, false.
    pub fn is_simulation_mode(&self) -> bool {
        self.plant.is_some()
    }

    /// Set the data acquisition mode.
    ///
    /// # Arguments
    /// * `mode` - Data acquisition mode.
    ///
    /// # Returns
    /// Some if the mode is set successfully. Otherwise, None.
    pub fn set_mode(&mut self, mode: DataAcquisitionMode) -> Option<()> {
        // Do nothing if the mode is the same
        if self.mode == mode {
            return Some(());
        }

        // Idle -> Telemetry -> ClosedLoopControl
        // ClosedLoopControl -> Telemetry or Idle
        // Telemetry -> Idle
        match (self.mode, mode) {
            (DataAcquisitionMode::Idle, DataAcquisitionMode::Telemetry) => {}
            (DataAcquisitionMode::Telemetry, DataAcquisitionMode::ClosedLoopControl) => {}
            (DataAcquisitionMode::Telemetry, DataAcquisitionMode::Idle) => {}
            (DataAcquisitionMode::ClosedLoopControl, DataAcquisitionMode::Telemetry) => {}
            (DataAcquisitionMode::ClosedLoopControl, DataAcquisitionMode::Idle) => {}
            _ => {
                error!(
                    "Invalid data acquisition mode transition from {:?} to {:?}.",
                    self.mode, mode
                );

                return None;
            }
        }

        self.mode = mode;
        self.event_queue
            .add_event(Event::get_message_data_acquisition_mode(mode));

        info!("Set the data acquisition mode to {:?}.", mode);

        // Reset the current actuator ILC stale data counts to 0 when switching
        // to Idle mode as we do not read the ILC data in Idle mode.
        if mode == DataAcquisitionMode::Idle {
            self._actuator_ilc_stale_data_counts = vec![0; NUM_ACTUATOR];

            info!("Reset the current actuator ILC stale data counts to 0.");
        }

        Some(())
    }

    /// Get the power telemetry data.
    ///
    /// # Returns
    /// Telemetry data.
    pub fn get_telemetry_power(&mut self) -> TelemetryPower {
        let mut telemetry = TelemetryPower::new();

        // Raw power data
        let (comm_voltage, comm_current) = self.get_power(PowerType::Communication);
        telemetry
            .power_raw
            .insert(String::from("commVoltage"), comm_voltage);
        telemetry
            .power_raw
            .insert(String::from("commCurrent"), comm_current);

        let (motor_voltage, motor_current) = self.get_power(PowerType::Motor);
        telemetry
            .power_raw
            .insert(String::from("motorVoltage"), motor_voltage);
        telemetry
            .power_raw
            .insert(String::from("motorCurrent"), motor_current);

        // Digital output and input
        telemetry.digital_output = self.get_digital_output();
        telemetry.digital_input = self.get_digital_input();

        telemetry
    }

    /// Get the power.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    ///
    /// # Returns
    /// A tuple of the power. The first element is the voltage in volt and the
    /// second element is the current in ampere.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn get_power(&mut self, power_type: PowerType) -> (f64, f64) {
        if let Some(plant) = &mut self.plant {
            if power_type == PowerType::Motor {
                plant.power_system_motor.get_voltage_and_current()
            } else {
                plant.power_system_communication.get_voltage_and_current()
            }
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Get the digital output.
    ///
    /// # Returns
    /// The digital output.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn get_digital_output(&self) -> u8 {
        if let Some(plant) = &self.plant {
            plant.digital_output
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Get the digital input.
    ///
    /// # Returns
    /// The digital input.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn get_digital_input(&self) -> u32 {
        if let Some(plant) = &self.plant {
            plant.get_digital_input()
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Get the telemetry data from the inner-loop controller (ILC).
    ///
    /// # Returns
    /// Telemetry data.
    pub fn get_telemetry_ilc(&mut self) -> TelemetryControlLoop {
        let mut telemetry = TelemetryControlLoop::new();

        // Put the sequence ID
        telemetry.seq_id_move_actuator_steps = self._seq_id_move_actuator_steps;

        // Raw ILC data
        let (ilc_status, ilc_encoders, forces) = self.get_ilc_data_actuator();
        telemetry.forces.insert(String::from("measured"), forces);
        telemetry.ilc_status = ilc_status;
        telemetry.ilc_encoders = ilc_encoders;

        let (ring, intake, exhaust) = self.get_ilc_data_temperature();
        telemetry.temperature.insert(String::from("ring"), ring);
        telemetry.temperature.insert(String::from("intake"), intake);
        telemetry
            .temperature
            .insert(String::from("exhaust"), exhaust);

        let (theta_z, delta_z) = self.get_ilc_data_displacement();
        telemetry
            .displacement_sensors
            .insert(String::from("thetaZ"), theta_z);
        telemetry
            .displacement_sensors
            .insert(String::from("deltaZ"), delta_z);

        let inclinometer_angle = self.get_ilc_data_inclinometer();
        telemetry
            .inclinometer
            .insert(String::from("raw"), inclinometer_angle);

        // Check the ILC stale data
        //
        // TODO: OSW-2036. Improve the MockInnerLoopController to be more
        // realistic. Then, remove the bypass for the ILC stale data check in
        // simulation mode.
        if !self.is_simulation_mode() {
            self.check_ilc_stale_data(&mut telemetry);
        }

        // Cache the latest telemetry data
        self._latest_telemetry = telemetry;

        self._latest_telemetry.clone()
    }

    /// Get the actuator inner-loop controller (ILC) data.
    ///
    /// # Returns
    /// A tuple containing the ILC status, actuator encoders, and actuator
    /// forces in Newton.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn get_ilc_data_actuator(&mut self) -> (Vec<u8>, Vec<i32>, Vec<f64>) {
        if let Some(plant) = &mut self.plant {
            plant.get_actuator_ilc_data()
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Get the temperature inner-loop controller (ILC) data.
    ///
    /// # Returns
    /// A tuple containing the ring, intake, and exhaust temperatures in degree
    /// Celsius.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn get_ilc_data_temperature(&self) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        if let Some(plant) = &self.plant {
            (
                plant.temperature_ring.clone(),
                plant.temperature_intake.clone(),
                plant.temperature_exhaust.clone(),
            )
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Get the displacement inner-loop controller (ILC) data.
    ///
    /// # Returns
    /// A tuple containing the theta-Z and delta-Z displacement sensor readings
    /// in micron.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn get_ilc_data_displacement(&self) -> (Vec<f64>, Vec<f64>) {
        if self.is_simulation_mode() {
            (vec![0.0; NUM_IMS], vec![0.0; NUM_IMS])
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Get the inclinometer inner-loop controller (ILC) data.
    ///
    /// # Returns
    /// The inclinometer angle in degrees.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn get_ilc_data_inclinometer(&self) -> f64 {
        if let Some(plant) = &self.plant {
            plant.inclinometer_angle
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Check the inner-loop controller (ILC) data is stale or not. Add the
    /// related error codes to the telemetry data.
    ///
    /// # Arguments
    /// * `telmetry` - The telemetry data.
    fn check_ilc_stale_data(&mut self, telmetry: &mut TelemetryControlLoop) {
        let bypassed_ilcs = &self.config.bypassed_actuator_ilcs;
        let ilc_stale_data_limit = self.config.actuator_ilc_stale_data_limit;

        let mut has_warning = false;
        let mut has_fault = false;
        for (idx, status) in telmetry.ilc_status.iter().enumerate() {
            // Bypass the stale data check for the bypassed actuator ILCs
            if bypassed_ilcs.contains(&idx) {
                continue;
            }

            if self._ilc.is_expected_communication_counter(*status) {
                // The bad ILC reading has the status to be 0.
                if *status != 0 {
                    self._actuator_ilc_stale_data_counts[idx] = 0;
                }
            } else {
                if !has_warning {
                    has_warning = true;
                }

                if self._actuator_ilc_stale_data_counts[idx] < ilc_stale_data_limit {
                    self._actuator_ilc_stale_data_counts[idx] += 1;
                } else if !has_fault {
                    has_fault = true;
                }
            }
        }

        if has_warning {
            telmetry
                .ilc_error_codes
                .push(ErrorCode::IgnoreWarnBroadcast);
        }

        if has_fault {
            telmetry
                .ilc_error_codes
                .push(ErrorCode::IgnoreFaultStaleData);
            telmetry
                .ilc_error_codes
                .push(ErrorCode::FaultActuatorIlcRead);
        } else if has_warning {
            telmetry
                .ilc_error_codes
                .push(ErrorCode::IgnoreWarnStaleData);
        }
    }

    /// Initialize the default digital output.
    pub fn init_default_digital_output(&mut self) {
        self.set_default_digital_output(DigitalOutputStatus::BinaryHighLevel);
    }

    /// Set the default digital output.
    ///
    /// # Arguments
    /// * `status` - Digital output status to set for the default digital outputs.
    fn set_default_digital_output(&mut self, status: DigitalOutputStatus) {
        [
            DigitalOutput::InterlockEnable,
            DigitalOutput::ResetMotorBreakers,
            DigitalOutput::ResetCommunicationBreakers,
        ]
        .iter()
        .for_each(|digital_output| {
            self.switch_digital_output(*digital_output, status);
        });
    }

    /// End the default digital output.
    pub fn end_default_digital_output(&mut self) {
        self.set_default_digital_output(DigitalOutputStatus::BinaryLowLevel);
    }

    /// Switch the digital output.
    ///
    /// # Arguments
    /// * `digital_output` - Digital output.
    /// * `status` - Digital output status.
    ///
    /// # Returns
    /// Some if the digital output is switched successfully.
    ///
    /// # Panics
    /// If not in simulation mode.
    pub fn switch_digital_output(
        &mut self,
        digital_output: DigitalOutput,
        status: DigitalOutputStatus,
    ) -> Option<()> {
        if let Some(plant) = &mut self.plant {
            plant.switch_digital_output(digital_output, status);

            Some(())
        } else {
            panic!("Not implemented yet.");
        }
    }

    /// Set the mode of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The address of ILC.
    /// * `mode` - The mode to be set.
    ///
    /// # Returns
    /// Always Some with the new mode or unknown if not successful.
    pub fn set_ilc_mode(
        &mut self,
        address: usize,
        mode: InnerLoopControlMode,
    ) -> Option<InnerLoopControlMode> {
        if self.is_simulation_mode() {
            if let Some(plant) = &mut self.plant {
                let new_mode = plant.set_ilc_mode(address, mode);
                self.event_queue
                    .add_event(Event::get_message_inner_loop_control_mode(
                        address, new_mode,
                    ));

                return Some(new_mode);
            }
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }

        Some(InnerLoopControlMode::Unknown)
    }

    /// Get the mode of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The address of ILC.
    ///
    /// # Returns
    /// Always Some with the current mode or unknown if not successful.
    pub fn get_ilc_mode(&mut self, address: usize) -> Option<InnerLoopControlMode> {
        if self.is_simulation_mode() {
            if let Some(plant) = &self.plant {
                let mode = plant.get_ilc_mode(address);
                self.event_queue
                    .add_event(Event::get_message_inner_loop_control_mode(address, mode));

                return Some(mode);
            }
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }

        Some(InnerLoopControlMode::Unknown)
    }

    /// Move the actuator steps.
    ///
    /// # Arguments
    /// * `seq_id` - The sequence ID of the last command to move actuator steps
    ///   (see CommandMoveActuatorSteps).
    /// * `actuator_steps` - The actuator steps to move.
    ///
    /// # Returns
    /// Some if the actuator steps are moved successfully. Otherwise, None.
    pub fn move_actuator_steps(&mut self, seq_id: i32, actuator_steps: &[i32]) -> Option<()> {
        // Check the input
        if actuator_steps.len() != NUM_ACTUATOR {
            error!(
                "The length of the actuator steps ({}) is not equal to the number of actuators ({}).",
                actuator_steps.len(),
                NUM_ACTUATOR
            );

            return None;
        }

        // Check the mode
        if self.mode == DataAcquisitionMode::Idle {
            error!("Cannot move actuator steps in {:?} mode.", self.mode);

            return None;
        }

        // Do the actuator movement.
        self._seq_id_move_actuator_steps = seq_id;
        if let Some(plant) = &mut self.plant {
            plant.move_actuator_steps(actuator_steps);

            Some(())
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Toggle the bit for the closed-loop control. This signal is used by the
    /// safety module to communicate with the global interlock system (GIS).
    pub fn toggle_bit_closed_loop_control(&mut self) {
        if self.mode == DataAcquisitionMode::ClosedLoopControl {
            self.switch_digital_output(
                DigitalOutput::ClosedLoopControl,
                DigitalOutputStatus::ToggleBit,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use approx::assert_relative_eq;
    use serde_json::json;

    use crate::mock::mock_constants::{
        PLANT_TEMPERATURE_HIGH, PLANT_TEMPERATURE_LOW, TEST_DIGITAL_INPUT_POWER_COMM_MOTOR,
        TEST_DIGITAL_OUTPUT_NO_POWER, TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
    };
    use crate::mock::mock_power_system::MockPowerSystem;
    use ts_control_utils::enums::BitEnum;

    const EPSILON: f64 = 1e-7;

    fn create_data_acquisition(is_simulation_mode: bool) -> DataAcquisition {
        let mut data_acquisition = DataAcquisition::new(is_simulation_mode);

        if is_simulation_mode {
            data_acquisition.init_default_digital_output();
        }

        data_acquisition
    }

    fn run_until_breaker_enabled(power_system: &mut MockPowerSystem) {
        power_system.is_power_on = true;
        power_system.is_breaker_on = true;

        loop {
            if power_system.is_breaker_enabled() {
                break;
            }

            power_system.get_voltage_and_current();
        }
    }

    #[test]
    fn test_is_simulation_mode() {
        let data_acquisition = create_data_acquisition(true);
        assert!(data_acquisition.is_simulation_mode());

        let data_acquisition = create_data_acquisition(false);
        assert!(!data_acquisition.is_simulation_mode());
    }

    #[test]
    fn test_set_mode() {
        let mut data_acquisition = create_data_acquisition(true);

        // Should not allow Idle -> ClosedLoopControl
        assert!(data_acquisition
            .set_mode(DataAcquisitionMode::ClosedLoopControl)
            .is_none());

        // Valid transitions
        assert!(data_acquisition
            .set_mode(DataAcquisitionMode::Telemetry)
            .is_some());
        assert_eq!(data_acquisition.mode, DataAcquisitionMode::Telemetry);
        assert_eq!(
            data_acquisition.event_queue.get_events_and_clear(),
            vec![json!({
                "id": "dataAcquisitionMode",
                "mode": DataAcquisitionMode::Telemetry as u8,
            })]
        );

        assert!(data_acquisition
            .set_mode(DataAcquisitionMode::ClosedLoopControl)
            .is_some());
        assert_eq!(
            data_acquisition.mode,
            DataAcquisitionMode::ClosedLoopControl
        );
        assert_eq!(
            data_acquisition.event_queue.get_events_and_clear(),
            vec![json!({
                "id": "dataAcquisitionMode",
                "mode": DataAcquisitionMode::ClosedLoopControl as u8,
            })]
        );

        assert!(data_acquisition
            .set_mode(DataAcquisitionMode::Telemetry)
            .is_some());
        assert_eq!(data_acquisition.mode, DataAcquisitionMode::Telemetry);
        assert_eq!(
            data_acquisition.event_queue.get_events_and_clear(),
            vec![json!({
                "id": "dataAcquisitionMode",
                "mode": DataAcquisitionMode::Telemetry as u8,
            })]
        );

        data_acquisition._actuator_ilc_stale_data_counts[0] = 5;

        assert!(data_acquisition
            .set_mode(DataAcquisitionMode::Idle)
            .is_some());
        assert_eq!(data_acquisition.mode, DataAcquisitionMode::Idle);
        assert_eq!(
            data_acquisition.event_queue.get_events_and_clear(),
            vec![json!({
                "id": "dataAcquisitionMode",
                "mode": DataAcquisitionMode::Idle as u8,
            })]
        );

        assert_eq!(data_acquisition._actuator_ilc_stale_data_counts[0], 0);

        // ClosedLoopControl -> Idle
        data_acquisition.set_mode(DataAcquisitionMode::Telemetry);
        data_acquisition.set_mode(DataAcquisitionMode::ClosedLoopControl);

        assert!(data_acquisition
            .set_mode(DataAcquisitionMode::Idle)
            .is_some());
        assert_eq!(data_acquisition.mode, DataAcquisitionMode::Idle);
    }

    #[test]
    fn test_get_telemetry_power() {
        let mut data_acquisition = create_data_acquisition(true);

        // Power on the motor and communication, and enable the breakers
        data_acquisition.switch_digital_output(
            DigitalOutput::CommunicationPower,
            DigitalOutputStatus::BinaryHighLevel,
        );
        data_acquisition.switch_digital_output(
            DigitalOutput::MotorPower,
            DigitalOutputStatus::BinaryHighLevel,
        );

        run_until_breaker_enabled(
            &mut data_acquisition
                .plant
                .as_mut()
                .unwrap()
                .power_system_communication,
        );
        run_until_breaker_enabled(&mut data_acquisition.plant.as_mut().unwrap().power_system_motor);

        // Get the telemetry data
        let telemetry = data_acquisition.get_telemetry_power();

        assert_eq!(
            telemetry.digital_output,
            TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR
        );
        assert_eq!(telemetry.digital_input, TEST_DIGITAL_INPUT_POWER_COMM_MOTOR);

        assert_eq!(
            telemetry.power_raw["commVoltage"],
            data_acquisition.get_power(PowerType::Communication).0
        );
        assert!(telemetry.power_raw["commCurrent"] > 0.0);

        assert_eq!(
            telemetry.power_raw["motorVoltage"],
            data_acquisition.get_power(PowerType::Motor).0
        );
        assert!(telemetry.power_raw["motorCurrent"] > 0.0);
    }

    #[test]
    fn test_get_power() {
        let mut data_acquisition = create_data_acquisition(true);

        // Power is off
        let mut voltage = data_acquisition.get_power(PowerType::Motor).0;

        assert_eq!(voltage, 0.0);

        // Power is on
        data_acquisition.switch_digital_output(
            DigitalOutput::MotorPower,
            DigitalOutputStatus::BinaryHighLevel,
        );

        for _ in 0..2 {
            voltage = data_acquisition.get_power(PowerType::Motor).0;
        }

        assert_ne!(voltage, 0.0);
    }

    #[test]
    fn test_get_telemetry_ilc() {
        let mut data_acquisition = create_data_acquisition(true);
        data_acquisition._seq_id_move_actuator_steps = 1;
        if let Some(plant) = &mut data_acquisition.plant {
            plant.power_system_communication.is_power_on = true;
        }

        data_acquisition.get_telemetry_ilc();
        let telemetry = data_acquisition.get_telemetry_ilc();

        assert_eq!(telemetry.seq_id_move_actuator_steps, 1);

        assert_relative_eq!(
            telemetry.forces["measured"][0],
            216.2038166,
            epsilon = EPSILON
        );
        assert_eq!(telemetry.ilc_status[0], 16);

        assert_eq!(telemetry.temperature["ring"][0], PLANT_TEMPERATURE_LOW);
        assert_eq!(telemetry.temperature["intake"][0], PLANT_TEMPERATURE_LOW);
        assert_eq!(telemetry.temperature["exhaust"][0], PLANT_TEMPERATURE_HIGH);

        assert_eq!(telemetry.inclinometer["raw"], 90.0);

        assert_eq!(data_acquisition._latest_telemetry, telemetry);
    }

    #[test]
    fn test_check_ilc_stale_data() {
        let mut data_acquisition = create_data_acquisition(true);
        data_acquisition.config.bypassed_actuator_ilcs = vec![1, 2];

        let mut telemetry = TelemetryControlLoop::new();

        // Bypassed actuator ILCs should not be checked for stale data
        // Bit 4-7 is the broadcast communication counter
        let current_communication_counter = data_acquisition._ilc.communication_counter << 4;
        telemetry.ilc_status = vec![current_communication_counter; NUM_ACTUATOR];
        telemetry.ilc_status[1] = 0x00;
        telemetry.ilc_status[2] = 0x00;

        let actuator_ilc_stale_data_limit = data_acquisition.config.actuator_ilc_stale_data_limit;
        for _ in 0..actuator_ilc_stale_data_limit {
            data_acquisition.check_ilc_stale_data(&mut telemetry);
        }

        assert_eq!(
            data_acquisition._actuator_ilc_stale_data_counts,
            vec![0; NUM_ACTUATOR]
        );
        assert!(telemetry.ilc_error_codes.is_empty());

        // Stale data for one cycle
        telemetry.ilc_status[0] = 0x00;
        data_acquisition.check_ilc_stale_data(&mut telemetry);

        assert_eq!(data_acquisition._actuator_ilc_stale_data_counts[0], 1);
        assert_eq!(
            telemetry.ilc_error_codes,
            vec![
                ErrorCode::IgnoreWarnBroadcast,
                ErrorCode::IgnoreWarnStaleData
            ]
        );

        // Stale data for too long
        telemetry.ilc_error_codes.clear();

        for _ in 0..actuator_ilc_stale_data_limit {
            data_acquisition.check_ilc_stale_data(&mut telemetry);
        }

        assert_eq!(
            data_acquisition._actuator_ilc_stale_data_counts[0],
            actuator_ilc_stale_data_limit
        );
        assert!(telemetry
            .ilc_error_codes
            .contains(&ErrorCode::IgnoreFaultStaleData));
        assert!(telemetry
            .ilc_error_codes
            .contains(&ErrorCode::FaultActuatorIlcRead));

        // No stale data
        telemetry.ilc_status[0] = current_communication_counter;
        telemetry.ilc_error_codes.clear();

        data_acquisition.check_ilc_stale_data(&mut telemetry);

        assert_eq!(
            data_acquisition._actuator_ilc_stale_data_counts,
            vec![0; NUM_ACTUATOR]
        );
        assert!(telemetry.ilc_error_codes.is_empty());
    }

    #[test]
    fn test_init_default_digital_output() {
        let data_acquisition = create_data_acquisition(true);

        // Note the create_data_acquisition() already called
        // init_default_digital_output()
        assert_eq!(
            data_acquisition.get_digital_output(),
            TEST_DIGITAL_OUTPUT_NO_POWER
        );
    }

    #[test]
    fn test_end_default_digital_output() {
        let mut data_acquisition = create_data_acquisition(true);

        data_acquisition.end_default_digital_output();
        assert_eq!(data_acquisition.get_digital_output(), 0);
    }

    #[test]
    fn test_set_ilc_mode() {
        let mut data_acquisition = create_data_acquisition(true);

        assert_eq!(
            data_acquisition.set_ilc_mode(10, InnerLoopControlMode::Disabled),
            Some(InnerLoopControlMode::Disabled),
        );
        assert_eq!(
            data_acquisition.event_queue.get_events_and_clear(),
            vec![json!({
                "id": "innerLoopControlMode",
                "address": 10,
                "mode": 2,
            })]
        );
    }

    #[test]
    fn test_get_ilc_mode() {
        let mut data_acquisition = create_data_acquisition(true);

        assert_eq!(
            data_acquisition.get_ilc_mode(10),
            Some(InnerLoopControlMode::Standby)
        );
        assert_eq!(
            data_acquisition.event_queue.get_events_and_clear(),
            vec![json!({
                "id": "innerLoopControlMode",
                "address": 10,
                "mode": 1,
            })]
        );
    }

    #[test]
    fn test_move_actuator_steps() {
        let mut data_acquisition = create_data_acquisition(true);

        // Test with invalid actuator steps
        let actuator_steps_invalid = vec![100];

        assert!(data_acquisition
            .move_actuator_steps(1, &actuator_steps_invalid)
            .is_none());

        // Test with invalid mode
        let actuator_steps_valid = vec![100; NUM_ACTUATOR];

        assert!(data_acquisition
            .move_actuator_steps(1, &actuator_steps_valid)
            .is_none());

        // Test with valid actuator steps and mode
        data_acquisition.set_mode(DataAcquisitionMode::Telemetry);

        assert!(data_acquisition
            .move_actuator_steps(1, &actuator_steps_valid)
            .is_some());
    }

    #[test]
    fn test_toggle_bit_closed_loop_control() {
        let mut data_acquisition = create_data_acquisition(true);

        assert!(
            data_acquisition.get_digital_output() & DigitalOutput::ClosedLoopControl.bit_value()
                == 0
        );

        // Not in the closed-loop control mode
        data_acquisition.toggle_bit_closed_loop_control();

        assert!(
            data_acquisition.get_digital_output() & DigitalOutput::ClosedLoopControl.bit_value()
                == 0
        );

        // In the closed-loop control mode
        data_acquisition.mode = DataAcquisitionMode::ClosedLoopControl;

        data_acquisition.toggle_bit_closed_loop_control();

        assert!(
            data_acquisition.get_digital_output() & DigitalOutput::ClosedLoopControl.bit_value()
                != 0
        );

        data_acquisition.toggle_bit_closed_loop_control();

        assert!(
            data_acquisition.get_digital_output() & DigitalOutput::ClosedLoopControl.bit_value()
                == 0
        );
    }
}
