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

use std::path::Path;
use std::vec;

use nalgebra::{DMatrix, SMatrix, SVector};

use crate::constants::{
    CODE_STEP_MOTOR_BROADCAST, NUM_ACTUATOR, NUM_AXIAL_ACTUATOR, NUM_IMS_READING,
    NUM_INNER_LOOP_CONTROLLER, NUM_SPACE_DEGREE_OF_FREEDOM, NUM_TEMPERATURE_EXHAUST,
    NUM_TEMPERATURE_INTAKE, NUM_TEMPERATURE_RING,
};
use crate::control::math_tool::correct_inclinometer_angle;
use crate::enums::{
    DigitalInput, DigitalOutput, DigitalOutputStatus, InnerLoopControlMode, PowerType,
};
use crate::mock::mock_constants::{
    PLANT_CURRENT_COMMUNICATION, PLANT_CURRENT_MOTOR, PLANT_STEP_TO_ENCODER,
    PLANT_TEMPERATURE_HIGH, PLANT_TEMPERATURE_LOW, PLANT_VOLTAGE,
};
use crate::mock::mock_inner_loop_controller::MockInnerLoopController;
use crate::mock::mock_power_system::MockPowerSystem;
use crate::power::config_power::ConfigPower;
use ts_control_utils::{enums::BitEnum, utility::get_parameter};

#[derive(Clone)]
pub struct MockPlant {
    _static_transfer_matrix: SMatrix<f64, NUM_ACTUATOR, NUM_ACTUATOR>,
    // Current positions of the actuator in steps referenced to the home
    // position.
    pub actuator_steps: Vec<i32>,
    // Actuator force based on the mirror's weight in Newton.
    _actuator_force_weight: Vec<f64>,
    // Actuator force based on the step in Newton
    _actuator_force_step: Vec<f64>,
    // Inclinometer angle in degree.
    pub inclinometer_angle: f64,
    _inclinometer_offset: f64,
    _mirror_weight_kg: f64,
    // Ring temperature in degree Celsius. The order is: [LG2-1, LG2-2, LG2-3,
    // LG2-4, LG3-1, LG3-2, LG3-3, LG3-4, LG4-1, LG4-2, LG4-3, LG4-4].
    pub temperature_ring: Vec<f64>,
    // Intake temperature in degree Celsius.
    pub temperature_intake: Vec<f64>,
    // Exhaust temperature in degree Celsius.
    pub temperature_exhaust: Vec<f64>,
    // Power systems.
    pub power_system_communication: MockPowerSystem,
    pub power_system_motor: MockPowerSystem,
    _ilcs: Vec<MockInnerLoopController>,
    // Digital output.
    pub digital_output: u8,
}

impl MockPlant {
    /// Mock plant model to simulate the actuator's force feedback and sensor
    /// data.
    ///
    /// # Arguments
    /// * `static_transfer_matrix` - Static transfer matrix used to simulate the
    ///   delta force from the change of steps. This is a (NUM_ACTUATOR x
    ///   NUM_ACTUATOR) matrix. The unit of row is the Newton and the unit of
    ///   column is the actuator's step.
    /// * `inclinometer_angle` - The inclinometer angle in degree.
    ///
    /// # Returns
    /// A new mock plant.
    pub fn new(static_transfer_matrix: &[Vec<f64>], inclinometer_angle: f64) -> Self {
        let matrix = SMatrix::from_row_iterator(
            static_transfer_matrix
                .iter()
                .flat_map(|row| row.iter().copied()),
        );

        let filepath = Path::new("config/parameters_control.yaml");
        let inclinometer_offset = get_parameter(filepath, "inclinometer_offset");
        let mirror_weight_kg = get_parameter(filepath, "mirror_weight_kg");

        let mut temperature_ring = vec![PLANT_TEMPERATURE_LOW; NUM_TEMPERATURE_RING];

        // Indexes of the ring temperatures that have a higher temperature.
        let indexes: [usize; 4] = [3, 6, 7, 11];
        indexes.iter().for_each(|&idx| {
            temperature_ring[idx] = PLANT_TEMPERATURE_HIGH;
        });

        // Setup the power systems.
        let config_power = ConfigPower::new();
        let power_system_communication = MockPowerSystem::new(
            PLANT_VOLTAGE,
            PLANT_CURRENT_COMMUNICATION,
            config_power.breaker_operating_voltage,
            config_power.loop_time as i32,
            config_power.get_time_power_on(PowerType::Communication),
            config_power.get_time_power_off(PowerType::Communication),
            config_power.get_time_breaker_on(PowerType::Communication),
            config_power.get_time_breaker_off(PowerType::Communication),
        );
        let power_system_motor = MockPowerSystem::new(
            PLANT_VOLTAGE,
            PLANT_CURRENT_MOTOR,
            config_power.breaker_operating_voltage,
            config_power.loop_time as i32,
            config_power.get_time_power_on(PowerType::Motor),
            config_power.get_time_power_off(PowerType::Motor),
            config_power.get_time_breaker_on(PowerType::Motor),
            config_power.get_time_breaker_off(PowerType::Motor),
        );

        Self {
            _static_transfer_matrix: matrix,

            actuator_steps: vec![0; NUM_ACTUATOR],
            _actuator_force_weight: Self::get_forces_mirror_weight(
                inclinometer_angle,
                inclinometer_offset,
                mirror_weight_kg,
            ),
            _actuator_force_step: vec![0.0; NUM_ACTUATOR],

            inclinometer_angle,
            _inclinometer_offset: inclinometer_offset,

            _mirror_weight_kg: mirror_weight_kg,

            temperature_ring,
            temperature_intake: vec![PLANT_TEMPERATURE_LOW; NUM_TEMPERATURE_INTAKE],
            temperature_exhaust: vec![PLANT_TEMPERATURE_HIGH; NUM_TEMPERATURE_EXHAUST],

            power_system_communication,
            power_system_motor,

            _ilcs: vec![MockInnerLoopController::new(); NUM_INNER_LOOP_CONTROLLER],

            digital_output: 0,
        }
    }

    /// Switch the digital output with the specific bit.
    ///
    /// # Arguments
    /// * `bit` - The digital output bit to switch.
    /// * `status` - The status of the digital output.
    pub fn switch_digital_output(&mut self, bit: DigitalOutput, status: DigitalOutputStatus) {
        // Check if the current motor/communication breakers were on.
        let were_motor_breakers_on =
            (self.digital_output & DigitalOutput::ResetMotorBreakers.bit_value()) != 0;
        let were_communication_breakers_on =
            (self.digital_output & DigitalOutput::ResetCommunicationBreakers.bit_value()) != 0;

        // Update the digital output.
        match status {
            DigitalOutputStatus::BinaryLowLevel => {
                self.digital_output &= !bit.bit_value();
            }
            DigitalOutputStatus::BinaryHighLevel => {
                self.digital_output |= bit.bit_value();
            }
            DigitalOutputStatus::ToggleBit => {
                self.digital_output ^= bit.bit_value();
            }
        }

        // Update the power systems based on the digital output.
        if (self.digital_output & DigitalOutput::MotorPower.bit_value()) != 0 {
            self.power_system_motor.is_power_on = true;
        } else {
            self.power_system_motor.is_power_on = false;
            self.power_system_motor.is_breaker_on = false;
        }

        if (self.digital_output & DigitalOutput::CommunicationPower.bit_value()) != 0 {
            self.power_system_communication.is_power_on = true;
        } else {
            self.power_system_communication.is_power_on = false;
            self.power_system_communication.is_breaker_on = false;
        }

        if (self.digital_output & DigitalOutput::ResetMotorBreakers.bit_value()) != 0 {
            if self.power_system_motor.is_power_on && (!were_motor_breakers_on) {
                self.power_system_motor.is_breaker_on = true;
            }
        } else {
            self.power_system_motor.is_breaker_on = false;
        }

        if (self.digital_output & DigitalOutput::ResetCommunicationBreakers.bit_value()) != 0 {
            if self.power_system_communication.is_power_on && (!were_communication_breakers_on) {
                self.power_system_communication.is_breaker_on = true;
            }
        } else {
            self.power_system_communication.is_breaker_on = false;
        }
    }

    /// Get the digital input.
    ///
    /// # Returns
    /// The digital input.
    pub fn get_digital_input(&self) -> u32 {
        let mut bits = vec![
            DigitalInput::RedundancyOK,
            DigitalInput::LoadDistributionOK,
            DigitalInput::PowerSupplyDC2OK,
            DigitalInput::PowerSupplyDC1OK,
            DigitalInput::PowerSupplyCurrent2OK,
            DigitalInput::PowerSupplyCurrent1OK,
        ];

        if !self.power_system_communication.is_breaker_enabled() {
            bits.extend_from_slice(&[
                DigitalInput::J1W12N1CommunicationPowerBreaker,
                DigitalInput::J1W12N2CommunicationPowerBreaker,
                DigitalInput::J2W13N1CommunicationPowerBreaker,
                DigitalInput::J2W13N2CommunicationPowerBreaker,
                DigitalInput::J3W14N1CommunicationPowerBreaker,
                DigitalInput::J3W14N2CommunicationPowerBreaker,
            ]);
        }

        if !self.power_system_motor.is_power_on {
            bits.push(DigitalInput::InterlockPowerRelay);
        }

        if !self.power_system_motor.is_breaker_enabled() {
            bits.extend_from_slice(&[
                DigitalInput::J1W9N1MotorPowerBreaker,
                DigitalInput::J1W9N2MotorPowerBreaker,
                DigitalInput::J1W9N3MotorPowerBreaker,
                DigitalInput::J2W10N1MotorPowerBreaker,
                DigitalInput::J2W10N2MotorPowerBreaker,
                DigitalInput::J2W10N3MotorPowerBreaker,
                DigitalInput::J3W11N1MotorPowerBreaker,
                DigitalInput::J3W11N2MotorPowerBreaker,
                DigitalInput::J3W11N3MotorPowerBreaker,
            ]);
        }

        bits.iter().fold(0, |acc, x| acc | x.bit_value())
    }

    /// Get the forces that bear the weight of mirror.
    ///
    /// # Parameters
    /// * `angle` - Inclinometer angle in degree.
    /// * `inclinometer_offset` - Inclinometer offset in degree.
    /// * `mirror_weight_kg` - The weight of the mirror in kilogram.
    ///
    /// # Returns
    /// Forces to support the mirror in Newton.
    fn get_forces_mirror_weight(
        angle: f64,
        inclinometer_offset: f64,
        mirror_weight_kg: f64,
    ) -> Vec<f64> {
        let mut forces = vec![0.0; NUM_ACTUATOR];

        let angle_correct = correct_inclinometer_angle(angle, inclinometer_offset);

        let gravitation_acceleration = 9.8;
        let force_mirror_weight = mirror_weight_kg * gravitation_acceleration;

        let force_axial =
            force_mirror_weight * angle_correct.to_radians().sin() / (NUM_AXIAL_ACTUATOR as f64);
        forces[..NUM_AXIAL_ACTUATOR]
            .iter_mut()
            .for_each(|x| *x = force_axial);

        // Tangent actuators A1 and A4 do not bear the weight of mirror.
        // A2 and A3 have the reversed direction compared with A5 and A6.
        let mut index_tangent_link = [1, 2, 4, 5];
        index_tangent_link
            .iter_mut()
            .for_each(|x| *x += NUM_AXIAL_ACTUATOR);

        let tangent_force_direction = [-1.0, -1.0, 1.0, 1.0];
        let force_tangent = force_mirror_weight * angle_correct.to_radians().cos()
            / (index_tangent_link.len() as f64);

        index_tangent_link
            .iter()
            .zip(tangent_force_direction.iter())
            .for_each(|(index, direction)| {
                forces[*index] = force_tangent * (*direction);
            });

        forces
    }

    /// Set the inclinometer angle. The actuator force related to the mirror's
    /// weight will be updated according to the new angle.
    ///
    /// # Arguments
    /// * `inclinometer_angle` - The inclinometer angle in degree.
    pub fn set_inclinometer_angle(&mut self, inclinometer_angle: f64) {
        self.inclinometer_angle = inclinometer_angle;
        self._actuator_force_weight = Self::get_forces_mirror_weight(
            inclinometer_angle,
            self._inclinometer_offset,
            self._mirror_weight_kg,
        );
    }

    /// Set the inclinometer value in the monitor inner-loop controller (ILC)
    /// data.
    pub fn set_monitor_ilc_inclinometer(&mut self) {
        self._ilcs[NUM_INNER_LOOP_CONTROLLER - 1].monitor_values =
            vec![self.inclinometer_angle as f32];
    }

    /// Set the displacement values in the monitor inner-loop controller (ILC)
    /// data.
    pub fn set_monitor_ilc_displacement(&mut self) {
        self._ilcs[NUM_INNER_LOOP_CONTROLLER - 2].monitor_values = vec![0.0; NUM_IMS_READING];
    }

    /// Set the temperature values in the monitor inner-loop controller (ILC)
    /// data.
    pub fn set_monitor_ilc_temperature(&mut self) {
        let ring = &self.temperature_ring;
        let intake = &self.temperature_intake;
        let exhaust = &self.temperature_exhaust;

        // The mapping is:
        // 0: Mirror (LG2): LG2-1, LG2-2, LG2-3, LG2-4
        // 1: Cell: Intake-1, Exhaust-1, Exhaust-2, Intake-2
        // 2: Mirror (LG4): LG4-1, LG4-2, LG4-3, LG4-4
        // 3: Mirror (LG3): LG3-1, LG3-2, LG3-3, LG3-4
        self._ilcs[NUM_ACTUATOR].monitor_values = vec![
            ring[0] as f32,
            ring[1] as f32,
            ring[2] as f32,
            ring[3] as f32,
        ];
        self._ilcs[NUM_ACTUATOR + 1].monitor_values = vec![
            intake[0] as f32,
            exhaust[0] as f32,
            exhaust[1] as f32,
            intake[1] as f32,
        ];
        self._ilcs[NUM_ACTUATOR + 2].monitor_values = vec![
            ring[8] as f32,
            ring[9] as f32,
            ring[10] as f32,
            ring[11] as f32,
        ];
        self._ilcs[NUM_ACTUATOR + 3].monitor_values = vec![
            ring[4] as f32,
            ring[5] as f32,
            ring[6] as f32,
            ring[7] as f32,
        ];
    }

    /// Set the inner-loop controller (ILC) data of 78 actuators.
    pub fn set_actuator_ilc_data(&mut self) {
        let encoders = self.get_actuator_encoders();
        let forces = self.get_actuator_forces();
        for idx in 0..NUM_ACTUATOR {
            self._ilcs[idx].update_encoder_and_force(encoders[idx], forces[idx]);
        }
    }

    /// Get the actuator encoder values.
    ///
    /// # Returns
    /// A vector of actuator encoder values in count.
    fn get_actuator_encoders(&self) -> Vec<i32> {
        self.actuator_steps
            .iter()
            .map(|x| ((*x as f64) * PLANT_STEP_TO_ENCODER) as i32)
            .collect()
    }

    /// Get the actuator forces.
    ///
    /// # Returns
    /// A vector of actuator forces in Newton.
    pub fn get_actuator_forces(&self) -> Vec<f64> {
        let forces = self
            ._actuator_force_weight
            .iter()
            .zip(&self._actuator_force_step)
            .map(|(x, y)| x + y)
            .collect();

        forces
    }

    /// Reset the actuator steps to zero.
    pub fn reset_actuator_steps(&mut self) {
        self.actuator_steps = vec![0; NUM_ACTUATOR];
        self._actuator_force_step = vec![0.0; NUM_ACTUATOR];
    }

    /// Request the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `frame_request` - Request frame containing the command and
    ///   parameters.
    ///
    /// # Returns
    /// The payload of the response frame from the mock ILC.
    pub fn request_ilc(&mut self, frame_request: &[u8]) -> Vec<u8> {
        if frame_request[1] == CODE_STEP_MOTOR_BROADCAST {
            for idx in 0..NUM_ACTUATOR {
                self._ilcs[idx].request(frame_request);
            }

            // Apply the steps. Convert the u8 to i8 first to get the negative
            // values and then convert to i32 for calculation.
            let start_index = 3;
            let steps = (0..NUM_ACTUATOR)
                .map(|idx| (frame_request[start_index + idx] as i8) as i32)
                .collect::<Vec<i32>>();

            self.move_actuator_steps(&steps);

            Vec::new()
        } else {
            // To zero-based address by subtracting 1 from the first byte of the
            // request frame.
            let frame_response = self._ilcs[(frame_request[0] - 1) as usize].request(frame_request);

            // Only send the payload.
            self.get_ilc_payload(&frame_response)
        }
    }

    /// Get the payload of the mock inner-loop controller (ILC) response frame.
    ///
    /// # Notes
    /// This is based on the Rec_Data_to_Host.vi in the ts_mtm2_cell.
    ///
    /// # Arguments
    /// * `frame_response` - The response frame from the mock ILC.
    ///
    /// # Returns
    /// The payload of the mock ILC response frame.
    fn get_ilc_payload(&self, frame_response: &[u8]) -> Vec<u8> {
        // Do not return the first 2 bytes (address and function code) and the
        // final 2 bytes (CRC checksum).
        frame_response[2..frame_response.len() - 2].to_vec()
    }

    /// Move the actuator steps.
    ///
    /// # Arguments
    /// * `actuator_steps` - The actuator steps to move.
    ///
    /// # Panics
    /// If the length of the actuator steps is not equal to the number of
    /// actuators.
    pub fn move_actuator_steps(&mut self, actuator_steps: &[i32]) {
        assert!(
            actuator_steps.len() == NUM_ACTUATOR,
            "The length of the actuator steps is not equal to the number of actuators."
        );

        // Update the steps
        self.actuator_steps
            .iter_mut()
            .zip(actuator_steps.iter())
            .for_each(|(x, y)| {
                *x += *y;
            });

        // Update the force by multiplying the static transfer matrix with the
        // change of steps and add to the current force.
        let steps = DMatrix::from_vec(
            actuator_steps.len(),
            1,
            actuator_steps.iter().map(|&x| x as f64).collect(),
        );

        let force = self._static_transfer_matrix * steps;
        self._actuator_force_step
            .iter_mut()
            .zip(force.iter())
            .for_each(|(x, y)| {
                *x += *y;
            });
    }

    /// Get the mode of the inner-loop controller (ILC) by the 0-based address.
    ///
    /// # Arguments
    /// * `address` - The 0-based address of the ILC.
    ///
    /// Returns
    /// The mode of the ILC.
    pub fn get_ilc_mode_by_address(&self, address: usize) -> InnerLoopControlMode {
        self._ilcs[address].mode
    }

    /// Calculate the independent measurement system (IMS) readings based on the
    /// rigid body position.
    ///
    /// # Arguments
    /// * `disp_matrix_inv` - The pseudo-inverse of the displacement matrix.
    /// * `disp_offset` - The displacement offset.
    /// * `x` - The x position in micron.
    /// * `y` - The y position in micron.
    /// * `z` - The z position in micron.
    /// * `rx` - The rotation around x-axis in arcsec.
    /// * `ry` - The rotation around y-axis in arcsec.
    /// * `rz` - The rotation around z-axis in arcsec.
    ///
    /// # Returns
    /// A tuple of the theta_z and delta_z readings in micron.
    #[allow(clippy::too_many_arguments)]
    pub fn calculate_ims_readings(
        disp_matrix_inv: &SMatrix<f64, NUM_IMS_READING, NUM_SPACE_DEGREE_OF_FREEDOM>,
        disp_offset: &SVector<f64, NUM_IMS_READING>,
        x: f64,
        y: f64,
        z: f64,
        rx: f64,
        ry: f64,
        rz: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        // Change the unit from mm to um
        let reading_ims =
            (disp_matrix_inv * SVector::from_vec(vec![x, y, z, rx, ry, rz]) + disp_offset) * 1e3;

        let theta_z = [8, 10, 4, 6, 0, 2]
            .iter()
            .map(|idx| reading_ims[*idx])
            .collect();
        let delta_z = [9, 11, 5, 7, 1, 3]
            .iter()
            .map(|idx| reading_ims[*idx])
            .collect();

        (theta_z, delta_z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    use crate::control::math_tool::calculate_position_ims;
    use crate::daq::inner_loop_controller::InnerLoopController;
    use crate::mock::mock_constants::{
        TEST_DIGITAL_INPUT_NO_POWER, TEST_DIGITAL_INPUT_POWER_COMM,
        TEST_DIGITAL_INPUT_POWER_COMM_MOTOR,
    };
    use crate::utility::{read_file_disp_ims, read_file_stiffness};

    const EPSILON: f64 = 1e-7;

    fn create_mock_plant() -> MockPlant {
        let filepath = Path::new("config/stiff_matrix_m2.yaml");
        let stiffness = read_file_stiffness(filepath);

        MockPlant::new(&stiffness, 0.0)
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
    fn test_new() {
        let mock_plant = create_mock_plant();

        for (idx, value) in mock_plant.temperature_ring.iter().enumerate() {
            if [3, 6, 7, 11].contains(&idx) {
                assert_eq!(*value, PLANT_TEMPERATURE_HIGH);
            } else {
                assert_eq!(*value, PLANT_TEMPERATURE_LOW);
            }
        }
    }

    #[test]
    fn test_switch_digital_output() {
        let mut mock_plant = create_mock_plant();

        // Binary high level
        mock_plant.switch_digital_output(
            DigitalOutput::InterlockEnable,
            DigitalOutputStatus::BinaryHighLevel,
        );
        assert!(mock_plant.digital_output & DigitalOutput::InterlockEnable.bit_value() != 0);

        // Binary low level
        mock_plant.switch_digital_output(
            DigitalOutput::InterlockEnable,
            DigitalOutputStatus::BinaryLowLevel,
        );
        assert!(mock_plant.digital_output & DigitalOutput::InterlockEnable.bit_value() == 0);

        // Toggle bit
        mock_plant.switch_digital_output(
            DigitalOutput::InterlockEnable,
            DigitalOutputStatus::ToggleBit,
        );
        assert!(mock_plant.digital_output & DigitalOutput::InterlockEnable.bit_value() != 0);

        mock_plant.switch_digital_output(
            DigitalOutput::InterlockEnable,
            DigitalOutputStatus::ToggleBit,
        );
        assert!(mock_plant.digital_output & DigitalOutput::InterlockEnable.bit_value() == 0);
    }

    #[test]
    fn test_switch_digital_output_power_communication() {
        let mut mock_plant = create_mock_plant();

        // Reset communication breakers (should fail because no power yet)
        mock_plant.switch_digital_output(
            DigitalOutput::ResetCommunicationBreakers,
            DigitalOutputStatus::BinaryHighLevel,
        );

        assert!(!mock_plant.power_system_communication.is_power_on);
        assert!(!mock_plant.power_system_communication.is_breaker_on);

        // Turn on the communication power
        mock_plant.switch_digital_output(
            DigitalOutput::CommunicationPower,
            DigitalOutputStatus::BinaryHighLevel,
        );

        assert!(mock_plant.power_system_communication.is_power_on);
        assert!(!mock_plant.power_system_communication.is_breaker_on);

        // Reset communication breakers (should succeed now). Note we have a
        // reset process here (off the breakers and then on again).
        mock_plant.switch_digital_output(
            DigitalOutput::ResetCommunicationBreakers,
            DigitalOutputStatus::BinaryLowLevel,
        );
        mock_plant.switch_digital_output(
            DigitalOutput::ResetCommunicationBreakers,
            DigitalOutputStatus::BinaryHighLevel,
        );

        assert!(mock_plant.power_system_communication.is_power_on);
        assert!(mock_plant.power_system_communication.is_breaker_on);

        // Unset communication breakers
        mock_plant.switch_digital_output(
            DigitalOutput::ResetCommunicationBreakers,
            DigitalOutputStatus::BinaryLowLevel,
        );

        assert!(mock_plant.power_system_communication.is_power_on);
        assert!(!mock_plant.power_system_communication.is_breaker_on);

        // Turn off the communication power
        mock_plant.switch_digital_output(
            DigitalOutput::ResetCommunicationBreakers,
            DigitalOutputStatus::BinaryHighLevel,
        );

        assert!(mock_plant.power_system_communication.is_power_on);
        assert!(mock_plant.power_system_communication.is_breaker_on);

        mock_plant.switch_digital_output(
            DigitalOutput::CommunicationPower,
            DigitalOutputStatus::BinaryLowLevel,
        );

        assert!(!mock_plant.power_system_communication.is_power_on);
        assert!(!mock_plant.power_system_communication.is_breaker_on);
    }

    #[test]
    fn test_switch_digital_output_power_motor() {
        let mut mock_plant = create_mock_plant();

        // Reset motor breakers (should fail because no power yet)
        mock_plant.switch_digital_output(
            DigitalOutput::ResetMotorBreakers,
            DigitalOutputStatus::BinaryHighLevel,
        );

        assert!(!mock_plant.power_system_motor.is_power_on);
        assert!(!mock_plant.power_system_motor.is_breaker_on);

        // Turn on the motor power
        mock_plant.switch_digital_output(
            DigitalOutput::MotorPower,
            DigitalOutputStatus::BinaryHighLevel,
        );

        assert!(mock_plant.power_system_motor.is_power_on);
        assert!(!mock_plant.power_system_motor.is_breaker_on);

        // Reset motor breakers (should succeed now). Note we have a reset
        // process here (off the breakers and then on again).
        mock_plant.switch_digital_output(
            DigitalOutput::ResetMotorBreakers,
            DigitalOutputStatus::BinaryLowLevel,
        );
        mock_plant.switch_digital_output(
            DigitalOutput::ResetMotorBreakers,
            DigitalOutputStatus::BinaryHighLevel,
        );

        assert!(mock_plant.power_system_motor.is_power_on);
        assert!(mock_plant.power_system_motor.is_breaker_on);

        // Unset motor breakers
        mock_plant.switch_digital_output(
            DigitalOutput::ResetMotorBreakers,
            DigitalOutputStatus::BinaryLowLevel,
        );

        assert!(mock_plant.power_system_motor.is_power_on);
        assert!(!mock_plant.power_system_motor.is_breaker_on);

        // Turn off the motor power
        mock_plant.switch_digital_output(
            DigitalOutput::ResetMotorBreakers,
            DigitalOutputStatus::BinaryHighLevel,
        );

        assert!(mock_plant.power_system_motor.is_power_on);
        assert!(mock_plant.power_system_motor.is_breaker_on);

        mock_plant.switch_digital_output(
            DigitalOutput::MotorPower,
            DigitalOutputStatus::BinaryLowLevel,
        );

        assert!(!mock_plant.power_system_motor.is_power_on);
        assert!(!mock_plant.power_system_motor.is_breaker_on);
    }

    #[test]
    fn test_get_digital_input() {
        let mut mock_plant = create_mock_plant();

        // Default
        assert_eq!(mock_plant.get_digital_input(), TEST_DIGITAL_INPUT_NO_POWER);

        // Communication power
        run_until_breaker_enabled(&mut mock_plant.power_system_communication);

        assert_eq!(
            mock_plant.get_digital_input(),
            TEST_DIGITAL_INPUT_POWER_COMM
        );

        // Check the interlock bit
        assert!(
            (mock_plant.get_digital_input() & DigitalInput::InterlockPowerRelay.bit_value()) != 0
        );

        mock_plant.power_system_motor.is_power_on = true;

        assert!(
            (mock_plant.get_digital_input() & DigitalInput::InterlockPowerRelay.bit_value()) == 0
        );

        // Motor power
        mock_plant.power_system_motor.is_power_on = false;
        run_until_breaker_enabled(&mut mock_plant.power_system_motor);

        assert_eq!(
            mock_plant.get_digital_input(),
            TEST_DIGITAL_INPUT_POWER_COMM_MOTOR
        );
    }

    #[test]
    fn test_get_forces_mirror_weight() {
        let forces = MockPlant::get_forces_mirror_weight(120.0, 0.94, 1588.65);

        const EPSILON: f64 = 1e-7;
        assert_relative_eq!(forces[0], 185.4643083, epsilon = EPSILON);
        assert_relative_eq!(forces[1], 185.4643083, epsilon = EPSILON);

        assert_eq!(forces[72], 0.0);
        assert_relative_eq!(forces[73], -2001.1325104, epsilon = EPSILON);
        assert_eq!(forces[75], 0.0);
        assert_relative_eq!(forces[76], 2001.1325104, epsilon = EPSILON);
    }

    #[test]
    fn test_set_inclinometer_angle() {
        let mut mock_plant = create_mock_plant();
        mock_plant.set_inclinometer_angle(89.06);

        assert_eq!(mock_plant.inclinometer_angle, 89.06);

        assert_relative_eq!(
            mock_plant._actuator_force_weight[0],
            216.2329167,
            epsilon = EPSILON
        );
    }

    #[test]
    fn test_set_monitor_ilc_inclinometer() {
        let mut mock_plant = create_mock_plant();
        mock_plant.inclinometer_angle = 123.45;

        mock_plant.set_monitor_ilc_inclinometer();

        assert_eq!(
            mock_plant._ilcs[NUM_INNER_LOOP_CONTROLLER - 1].monitor_values[0],
            mock_plant.inclinometer_angle as f32
        );
    }

    #[test]
    fn test_set_monitor_ilc_displacement() {
        let mut mock_plant = create_mock_plant();

        mock_plant.set_monitor_ilc_displacement();

        assert_eq!(
            mock_plant._ilcs[NUM_INNER_LOOP_CONTROLLER - 2].monitor_values,
            vec![0.0; NUM_IMS_READING]
        );
    }

    #[test]
    fn test_set_monitor_ilc_temperature() {
        let mut mock_plant = create_mock_plant();
        mock_plant.temperature_ring = vec![
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
        ];
        mock_plant.temperature_intake = vec![13.0, 14.0];
        mock_plant.temperature_exhaust = vec![15.0, 16.0];

        mock_plant.set_monitor_ilc_temperature();

        assert_eq!(
            mock_plant._ilcs[NUM_ACTUATOR].monitor_values,
            vec![1.0, 2.0, 3.0, 4.0]
        );
        assert_eq!(
            mock_plant._ilcs[NUM_ACTUATOR + 1].monitor_values,
            vec![13.0, 15.0, 16.0, 14.0]
        );
        assert_eq!(
            mock_plant._ilcs[NUM_ACTUATOR + 2].monitor_values,
            vec![9.0, 10.0, 11.0, 12.0]
        );
        assert_eq!(
            mock_plant._ilcs[NUM_ACTUATOR + 3].monitor_values,
            vec![5.0, 6.0, 7.0, 8.0]
        );
    }

    #[test]
    fn test_set_actuator_ilc_data() {
        let mut mock_plant = create_mock_plant();

        mock_plant.set_actuator_ilc_data();

        let encoders = mock_plant.get_actuator_encoders();
        let forces = mock_plant.get_actuator_forces();
        for idx in 0..NUM_ACTUATOR {
            assert_eq!(mock_plant._ilcs[idx].data.encoder_count, encoders[idx]);
            assert_eq!(mock_plant._ilcs[idx].data.force, -forces[idx] as f32);
        }
    }

    #[test]
    fn test_get_actuator_encoders() {
        let mut mock_plant = create_mock_plant();
        mock_plant.actuator_steps[0] = 1;
        mock_plant.actuator_steps[1] = -1;

        let encoders = mock_plant.get_actuator_encoders();

        assert_eq!(encoders[0], PLANT_STEP_TO_ENCODER as i32);
        assert_eq!(encoders[1], -PLANT_STEP_TO_ENCODER as i32);
    }

    #[test]
    fn test_get_actuator_forces() {
        let mut mock_plant = create_mock_plant();
        mock_plant.set_inclinometer_angle(89.06);
        mock_plant._actuator_force_step[0] = 2.1;

        let forces = mock_plant.get_actuator_forces();

        assert_relative_eq!(forces[0], 218.3329167, epsilon = EPSILON);
        assert_relative_eq!(forces[1], 216.2329167, epsilon = EPSILON);
    }

    #[test]
    fn test_request_ilc_move_actuator_steps() {
        let mut mock_plant = create_mock_plant();
        let mut ilc = InnerLoopController::new();

        // Test the data type conversion from i8 to u8 and then to i32
        let mut steps = vec![0; NUM_ACTUATOR];
        steps[0] = 1;
        steps[1] = -1;
        steps[2] = 127;
        steps[3] = -128;
        steps[4] = -127;
        steps[NUM_ACTUATOR - 1] = 1;

        let mut frame = ilc.create_frame_move_steps(&steps);
        mock_plant.request_ilc(&frame);

        assert_eq!(mock_plant.actuator_steps[0], 1);
        assert_eq!(mock_plant.actuator_steps[1], -1);
        assert_eq!(mock_plant.actuator_steps[2], 127);
        assert_eq!(mock_plant.actuator_steps[3], -128);
        assert_eq!(mock_plant.actuator_steps[4], -127);
        assert_eq!(mock_plant.actuator_steps[NUM_ACTUATOR - 1], 1);

        for idx in 1..20 {
            frame = ilc.create_frame_move_steps(&steps);
            mock_plant.request_ilc(&frame);

            let expected_communication_counter = (idx % 16) << 4;
            assert_eq!(
                mock_plant._ilcs[0].data.status,
                expected_communication_counter
            );
            assert_eq!(
                mock_plant._ilcs[NUM_ACTUATOR - 1].data.status,
                expected_communication_counter
            );
        }
    }

    #[test]
    fn test_request_ilc_set_mode() {
        let mut mock_plant = create_mock_plant();
        let ilc = InnerLoopController::new();

        let address = 3;
        let mode = InnerLoopControlMode::Disabled;
        assert_eq!(
            mock_plant.request_ilc(&ilc.create_frame_set_mode(address, mode)),
            [0, InnerLoopController::get_mode_value(mode) as u8]
        );
        assert_eq!(mock_plant.get_ilc_mode_by_address(address as usize), mode);
    }

    #[test]
    fn test_request_ilc_get_mode() {
        let mut mock_plant = create_mock_plant();
        let ilc = InnerLoopController::new();

        let address = 3;
        let mode = InnerLoopControlMode::Disabled;
        mock_plant.request_ilc(&ilc.create_frame_set_mode(address, mode));

        assert_eq!(
            mock_plant.request_ilc(&ilc.create_frame_get_mode(address)),
            [0, InnerLoopController::get_mode_value(mode) as u8]
        );
        assert_eq!(mock_plant.get_ilc_mode_by_address(address as usize), mode);
    }

    #[test]
    fn test_request_ilc_actuator_data() {
        let mut mock_plant = create_mock_plant();

        let address = 1;
        mock_plant._ilcs[address].update_communication_counter(1);
        mock_plant._ilcs[address].update_encoder_and_force(123, 456.78);

        let ilc = InnerLoopController::new();
        let frame_request = ilc.get_frame_get_force_and_status(address).unwrap();
        let frame_response = mock_plant.request_ilc(frame_request);

        let (status, encoder, force) = ilc
            .get_force_and_status_from_frame(&frame_response)
            .unwrap();

        assert_eq!(status, 0b0001_0000);
        assert_eq!(encoder, 123);
        assert_eq!(force, 456.78);
    }

    #[test]
    fn test_request_ilc_monitor_data() {
        let mut mock_plant = create_mock_plant();
        mock_plant.inclinometer_angle = 12.34;
        mock_plant.set_monitor_ilc_inclinometer();

        let ilc = InnerLoopController::new();
        let monitor_values = mock_plant.request_ilc(ilc.get_frame_inclinometer());

        assert_eq!(
            ilc.get_inclinometer_from_frame(&monitor_values).unwrap(),
            mock_plant.inclinometer_angle as f32
        );
    }

    #[test]
    fn test_get_ilc_payload() {
        let mock_plant = create_mock_plant();

        let frame_response = [1, 2, 3, 4, 5];
        let payload = mock_plant.get_ilc_payload(&frame_response);

        assert_eq!(payload, vec![3]);
    }

    #[test]
    #[should_panic(
        expected = "The length of the actuator steps is not equal to the number of actuators."
    )]
    fn test_move_actuator_steps_panic() {
        let mut mock_plant = create_mock_plant();
        mock_plant.move_actuator_steps(&vec![1, 2]);
    }

    #[test]
    fn test_move_actuator_steps() {
        let mut mock_plant = create_mock_plant();

        // Move
        move_actuator_steps(&mut mock_plant);

        assert_eq!(mock_plant.actuator_steps[0], 1);
        assert_relative_eq!(
            mock_plant._actuator_force_step[0],
            -0.0571288,
            epsilon = EPSILON
        );

        // Move again
        move_actuator_steps(&mut mock_plant);

        assert_eq!(mock_plant.actuator_steps[0], 2);
        assert_relative_eq!(
            mock_plant._actuator_force_step[0],
            -0.1142577,
            epsilon = EPSILON
        );
    }

    fn move_actuator_steps(mock_plant: &mut MockPlant) {
        let mut actuator_steps = vec![0; NUM_ACTUATOR];
        actuator_steps[0] = 1;
        mock_plant.move_actuator_steps(&actuator_steps);
    }

    #[test]
    fn test_reset_actuator_steps() {
        let mut mock_plant = create_mock_plant();
        move_actuator_steps(&mut mock_plant);

        mock_plant.reset_actuator_steps();

        assert_eq!(mock_plant.actuator_steps[0], 0);
        assert_eq!(mock_plant._actuator_force_step[0], 0.0);
    }

    #[test]
    fn test_calculate_ims_readings() {
        let (matrix, offset) = read_file_disp_ims(Path::new("config/disp_ims.yaml"));
        let disp_matrix: SMatrix<f64, NUM_SPACE_DEGREE_OF_FREEDOM, NUM_IMS_READING> =
            SMatrix::from_row_iterator(matrix.iter().flat_map(|row| row.iter().copied()));
        let disp_offset: SVector<f64, NUM_IMS_READING> =
            SVector::from_iterator(offset.iter().copied());

        let disp_matrix_inv = disp_matrix
            .pseudo_inverse(f64::EPSILON)
            .expect("Should be able to compute the pseudo inverse of displacement matrix.");

        // Zero position
        let (theta_z, delta_z) = MockPlant::calculate_ims_readings(
            &disp_matrix_inv,
            &disp_offset,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        );

        let (x, y, z, rx, ry, rz) =
            calculate_position_ims(&disp_matrix, &disp_offset, &theta_z, &delta_z);

        assert_relative_eq!(x, 0.0, epsilon = EPSILON);
        assert_relative_eq!(y, 0.0, epsilon = EPSILON);
        assert_relative_eq!(z, 0.0, epsilon = EPSILON);
        assert_relative_eq!(rx, 0.0, epsilon = EPSILON);
        assert_relative_eq!(ry, 0.0, epsilon = EPSILON);
        assert_relative_eq!(rz, 0.0, epsilon = EPSILON);

        // Non-zero position
        let (theta_z, delta_z) = MockPlant::calculate_ims_readings(
            &disp_matrix_inv,
            &disp_offset,
            10.0,
            20.0,
            30.0,
            40.0,
            50.0,
            60.0,
        );

        let (x, y, z, rx, ry, rz) =
            calculate_position_ims(&disp_matrix, &disp_offset, &theta_z, &delta_z);

        assert_relative_eq!(x, 10.0, epsilon = EPSILON);
        assert_relative_eq!(y, 20.0, epsilon = EPSILON);
        assert_relative_eq!(z, 30.0, epsilon = EPSILON);
        assert_relative_eq!(rx, 40.0, epsilon = EPSILON);
        assert_relative_eq!(ry, 50.0, epsilon = EPSILON);
        assert_relative_eq!(rz, 60.0, epsilon = EPSILON);
    }
}
