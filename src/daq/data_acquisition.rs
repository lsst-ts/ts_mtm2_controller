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

use log::{error, info, warn};
use std::path::Path;

use crate::constants::{
    NUM_ACTUATOR, NUM_IMS, NUM_TEMPERATURE_EXHAUST, NUM_TEMPERATURE_INTAKE, NUM_TEMPERATURE_RING,
};
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
        self.check_ilc_stale_data(&mut telemetry);

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
        // Set the ILC data for the simulation mode.
        if let Some(plant) = &mut self.plant {
            plant.set_actuator_ilc_data()
        }

        // Get the ILC data for each actuator.
        let mut statuses = vec![0; NUM_ACTUATOR];
        let mut encoder_counts = vec![0; NUM_ACTUATOR];
        let mut forces = vec![0.0; NUM_ACTUATOR];

        let mut frame_payload = Vec::new();
        for idx in 0..NUM_ACTUATOR {
            if let Some(frame_request) = self._ilc.get_frame_get_force_and_status(idx) {
                if let Some(plant) = &mut self.plant {
                    frame_payload = plant.request_ilc(frame_request);
                } else {
                    // Update the hardware.
                    panic!("Not implemented yet.");
                }

                self.record_ilc_exception_code(&frame_payload, frame_request);
            }

            if let Some((status, encoder_count, force)) =
                self._ilc.get_force_and_status_from_frame(&frame_payload)
            {
                statuses[idx] = status;
                encoder_counts[idx] = encoder_count;
                forces[idx] = force as f64;
            } else {
                // Use the cached ILC data if the ILC data is not valid.
                encoder_counts[idx] = self._latest_telemetry.ilc_encoders[idx];
                forces[idx] = self._latest_telemetry.forces["measured"][idx]
            }
        }

        (statuses, encoder_counts, forces)
    }

    /// Record the inner-loop controller (ILC) exception code.
    ///
    /// # Arguments
    /// * `frame_payload` - The payload of response frame from the ILC.
    /// * `frame_request` - The request frame sent to the ILC.
    ///
    /// # Returns
    /// `true` if an exception code was recorded, `false` otherwise.
    fn record_ilc_exception_code(&self, frame_payload: &[u8], frame_request: &[u8]) -> bool {
        if frame_payload.len() == 1 {
            warn!(
                "Received the ILC exception code: {} for the request frame {:?}.",
                frame_payload[0], frame_request
            );

            return true;
        }

        false
    }

    /// Get the temperature inner-loop controller (ILC) data.
    ///
    /// # Returns
    /// A tuple containing the ring, intake, and exhaust temperatures in degree
    /// Celsius.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn get_ilc_data_temperature(&mut self) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        // Set the ILC data for the simulation mode.
        if let Some(plant) = &mut self.plant {
            plant.set_monitor_ilc_temperature();
        }

        // Get the ILC data for each temperature sensor.
        let mut temperatures =
            [0.0; NUM_TEMPERATURE_RING + NUM_TEMPERATURE_INTAKE + NUM_TEMPERATURE_EXHAUST];
        const NUM_TEMPERATURE_PER_FRAME: usize = 4;
        const NUM_TEMPERATURE_SENSOR: usize = 4;

        let mut frame_payload = Vec::new();
        let mut successful_indices = vec![];
        for idx in 0..NUM_TEMPERATURE_SENSOR {
            if let Some(frame_request) = self._ilc.get_frame_temperature(idx) {
                if let Some(plant) = &mut self.plant {
                    frame_payload = plant.request_ilc(frame_request);
                } else {
                    // Update the hardware.
                    panic!("Not implemented yet.");
                }

                self.record_ilc_exception_code(&frame_payload, frame_request);
            }

            if let Some(temperature) = self._ilc.get_temperature_from_frame(&frame_payload) {
                temperatures
                    [idx * NUM_TEMPERATURE_PER_FRAME..(idx + 1) * NUM_TEMPERATURE_PER_FRAME]
                    .copy_from_slice(&temperature);
                successful_indices.push(idx);
            }
        }

        self.rearrange_temperature_sensor_readings(&successful_indices, &temperatures)
    }

    /// Rearrange the temperature sensor readings.
    ///
    /// # Arguments
    /// * `indices` - The indices of the successfully read temperature sensors.
    /// * `readings` - The raw readings from the frames of inner-loop
    ///   controller (ILC).
    ///
    /// # Returns
    /// A tuple containing the ring, intake, and exhaust temperatures in degree
    /// Celsius.
    fn rearrange_temperature_sensor_readings(
        &self,
        indices: &[usize],
        readings: &[f32],
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let mut ring = self._latest_telemetry.temperature["ring"].clone();
        let mut intake = self._latest_telemetry.temperature["intake"].clone();
        let mut exhaust = self._latest_telemetry.temperature["exhaust"].clone();

        // The mapping is:
        // 0: Mirror (LG2): LG2-1, LG2-2, LG2-3, LG2-4
        // 1: Cell: Intake-1, Exhaust-1, Exhaust-2, Intake-2
        // 2: Mirror (LG4): LG4-1, LG4-2, LG4-3, LG4-4
        // 3: Mirror (LG3): LG3-1, LG3-2, LG3-3, LG3-4
        indices.iter().for_each(|idx| match *idx {
            0 => {
                ring[0] = readings[0] as f64;
                ring[1] = readings[1] as f64;
                ring[2] = readings[2] as f64;
                ring[3] = readings[3] as f64;
            }
            1 => {
                intake[0] = readings[4] as f64;
                intake[1] = readings[7] as f64;

                exhaust[0] = readings[5] as f64;
                exhaust[1] = readings[6] as f64;
            }
            2 => {
                ring[8] = readings[8] as f64;
                ring[9] = readings[9] as f64;
                ring[10] = readings[10] as f64;
                ring[11] = readings[11] as f64;
            }
            3 => {
                ring[4] = readings[12] as f64;
                ring[5] = readings[13] as f64;
                ring[6] = readings[14] as f64;
                ring[7] = readings[15] as f64;
            }
            _ => {}
        });

        (ring, intake, exhaust)
    }

    /// Get the displacement inner-loop controller (ILC) data.
    ///
    /// # Returns
    /// A tuple containing the theta-Z and delta-Z displacement sensor readings
    /// in micron.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn get_ilc_data_displacement(&mut self) -> (Vec<f64>, Vec<f64>) {
        let frame_request = self._ilc.get_frame_displacement();

        let frame_payload;
        if let Some(plant) = &mut self.plant {
            // Set the displacement sensor values for the simulation mode.
            plant.set_monitor_ilc_displacement();

            // Get the displacement sensor values as a frame.
            frame_payload = plant.request_ilc(frame_request);
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }

        self.record_ilc_exception_code(&frame_payload, frame_request);

        if let Some(displacement) = self._ilc.get_displacement_from_frame(&frame_payload) {
            self.rearrange_displacement_sensor_readings_and_unit(&displacement)
        } else {
            // Use the cached displacement values if the ILC data is not valid.
            (
                self._latest_telemetry.displacement_sensors["thetaZ"].clone(),
                self._latest_telemetry.displacement_sensors["deltaZ"].clone(),
            )
        }
    }

    /// Rearrange the displacement sensor readings and convert the unit from mm
    /// to micron.
    ///
    /// # Arguments
    /// * `readings` - The raw readings from the frame of inner-loop controller
    ///   (ILC).
    ///
    /// # Returns
    /// A tuple containing the theta-Z and delta-Z displacement sensor readings
    /// in micron.
    fn rearrange_displacement_sensor_readings_and_unit(
        &self,
        readings: &[f32],
    ) -> (Vec<f64>, Vec<f64>) {
        // The displacement sensors are stored in the array. The order is:
        // A5TZ (0), A5DZ (1), A6TZ (2), A6DZ (3), A3TZ (4), A3DZ (5)
        // A4TZ (6), A4DZ (7), A1TZ (8), A1DZ (9), A2TZ (10), A2DZ (11)
        const MM_TO_UM: f64 = 1000.0;

        let mut theta_z = vec![0.0; NUM_IMS];
        let mut delta_z = vec![0.0; NUM_IMS];
        let order_theta_z = [8, 10, 4, 6, 0, 2];
        for (idx, idx_theta_z) in order_theta_z.iter().enumerate() {
            theta_z[idx] = (readings[*idx_theta_z] as f64) * MM_TO_UM;
            delta_z[idx] = (readings[*idx_theta_z + 1] as f64) * MM_TO_UM;
        }

        (theta_z, delta_z)
    }

    /// Get the inclinometer inner-loop controller (ILC) data.
    ///
    /// # Returns
    /// The inclinometer angle in degrees.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn get_ilc_data_inclinometer(&mut self) -> f64 {
        let frame_request = self._ilc.get_frame_inclinometer();

        let frame_payload;
        if let Some(plant) = &mut self.plant {
            // Set the inclinometer value for the simulation mode.
            plant.set_monitor_ilc_inclinometer();

            // Get the inclinometer as a frame.
            frame_payload = plant.request_ilc(frame_request);
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }

        self.record_ilc_exception_code(&frame_payload, frame_request);

        if let Some(inclinometer_angle) = self._ilc.get_inclinometer_from_frame(&frame_payload) {
            inclinometer_angle as f64
        } else {
            // Use the cached inclinometer value if the ILC data is not valid.
            self._latest_telemetry.inclinometer["raw"]
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
        address: u8,
        mode: InnerLoopControlMode,
    ) -> Option<InnerLoopControlMode> {
        let frame_request = self._ilc.create_frame_set_mode(address, mode);

        let mut frame_payload = Vec::new();
        if self.is_simulation_mode() {
            if let Some(plant) = &mut self.plant {
                frame_payload = plant.request_ilc(&frame_request);
            };
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }

        self.record_ilc_exception_code(&frame_payload, &frame_request);

        self.get_ilc_mode_and_add_event(address, &frame_payload)
    }

    /// Get the mode of the inner-loop controller (ILC) and add the related
    /// event.
    ///
    /// # Arguments
    /// * `address` - The address of ILC.
    /// * `frame` - Response frame that contains the ILC mode.
    ///
    /// # Returns
    /// Some with the current mode if successful. Otherwise, Some with
    /// unknown.
    fn get_ilc_mode_and_add_event(
        &mut self,
        address: u8,
        frame: &[u8],
    ) -> Option<InnerLoopControlMode> {
        let ilc_mode = match self._ilc.get_ilc_mode_from_frame(frame) {
            Some(mode) => mode,
            None => InnerLoopControlMode::Unknown,
        };

        // Publish the event for the ILC mode change.
        self.event_queue
            .add_event(Event::get_message_inner_loop_control_mode(
                address, ilc_mode,
            ));

        Some(ilc_mode)
    }

    /// Get the mode of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The address of ILC.
    ///
    /// # Returns
    /// Always Some with the current mode or unknown if not successful.
    pub fn get_ilc_mode(&mut self, address: u8) -> Option<InnerLoopControlMode> {
        let frame_request = self._ilc.create_frame_get_mode(address);

        let mut frame_payload = Vec::new();
        if self.is_simulation_mode() {
            if let Some(plant) = &mut self.plant {
                frame_payload = plant.request_ilc(&frame_request);
            }
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }

        self.record_ilc_exception_code(&frame_payload, &frame_request);

        self.get_ilc_mode_and_add_event(address, &frame_payload)
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
    pub fn move_actuator_steps(&mut self, seq_id: i32, actuator_steps: &[i8]) -> Option<()> {
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
        let frame_request = self._ilc.create_frame_move_steps(actuator_steps);
        if let Some(plant) = &mut self.plant {
            plant.request_ilc(&frame_request);

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
    use ts_control_utils::utility::assert_relative_eq_vector;

    use crate::mock::mock_constants::{
        PLANT_TEMPERATURE_HIGH, PLANT_TEMPERATURE_LOW, TEST_DIGITAL_INPUT_POWER_COMM_MOTOR,
        TEST_DIGITAL_OUTPUT_NO_POWER, TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
    };
    use crate::mock::mock_power_system::MockPowerSystem;
    use ts_control_utils::enums::BitEnum;

    const EPSILON: f64 = 1e-7;
    const EPSILON_F32: f64 = 1e-6;

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

        let telemetry = data_acquisition.get_telemetry_ilc();

        assert_eq!(telemetry.seq_id_move_actuator_steps, 1);

        assert_relative_eq!(
            telemetry.forces["measured"][0],
            216.2038116,
            epsilon = EPSILON
        );
        assert_eq!(telemetry.ilc_status[0], 0);

        assert_relative_eq!(
            telemetry.temperature["ring"][0],
            PLANT_TEMPERATURE_LOW,
            epsilon = EPSILON_F32
        );
        assert_relative_eq!(
            telemetry.temperature["intake"][0],
            PLANT_TEMPERATURE_LOW,
            epsilon = EPSILON_F32
        );
        assert_relative_eq!(
            telemetry.temperature["exhaust"][0],
            PLANT_TEMPERATURE_HIGH,
            epsilon = EPSILON_F32
        );

        assert_eq!(telemetry.inclinometer["raw"], 90.0);

        assert_eq!(data_acquisition._latest_telemetry, telemetry);
    }

    #[test]
    fn test_record_ilc_exception_code() {
        let data_acquisition = create_data_acquisition(true);

        let frame_request = [0x01, 0x02];

        assert!(data_acquisition.record_ilc_exception_code(&[0x01], &frame_request));
        assert!(!data_acquisition.record_ilc_exception_code(&[0x01, 0x02], &frame_request));
    }

    #[test]
    fn test_rearrange_temperature_sensor_readings() {
        let data_acquisition = create_data_acquisition(true);

        let readings = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        ];
        let indices = [0, 1, 2, 3];
        let (ring, intake, exhaust) =
            data_acquisition.rearrange_temperature_sensor_readings(&indices, &readings);

        assert_relative_eq_vector(
            &ring,
            &vec![
                1.0, 2.0, 3.0, 4.0, 13.0, 14.0, 15.0, 16.0, 9.0, 10.0, 11.0, 12.0,
            ],
            EPSILON,
        );
        assert_relative_eq_vector(&intake, &vec![5.0, 8.0], EPSILON);
        assert_relative_eq_vector(&exhaust, &vec![6.0, 7.0], EPSILON);
    }

    #[test]
    fn test_rearrange_displacement_sensor_readings_and_unit() {
        let data_acquisition = create_data_acquisition(true);

        let readings = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
        ];
        let (theta_z, delta_z) =
            data_acquisition.rearrange_displacement_sensor_readings_and_unit(&readings);

        assert_relative_eq_vector(
            &theta_z,
            &vec![9000.0, 11000.0, 5000.0, 7000.0, 1000.0, 3000.0],
            EPSILON,
        );
        assert_relative_eq_vector(
            &delta_z,
            &vec![10000.0, 12000.0, 6000.0, 8000.0, 2000.0, 4000.0],
            EPSILON,
        );
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
