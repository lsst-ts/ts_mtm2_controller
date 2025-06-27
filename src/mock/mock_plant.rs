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
    NUM_ACTUATOR, NUM_AXIAL_ACTUATOR, NUM_IMS_READING, NUM_INNER_LOOP_CONTROLLER,
    NUM_SPACE_DEGREE_OF_FREEDOM, NUM_TEMPERATURE_EXHAUST, NUM_TEMPERATURE_INTAKE,
    NUM_TEMPERATURE_RING,
};
use crate::control::math_tool::correct_inclinometer_angle;
use crate::enums::{
    BitEnum, DigitalInput, DigitalOutput, DigitalOutputStatus, InnerLoopControlMode, PowerType,
};
use crate::mock::mock_constants::{
    PLANT_CURRENT_COMMUNICATION, PLANT_CURRENT_MOTOR, PLANT_STEP_TO_ENCODER,
    PLANT_TEMPERATURE_HIGH, PLANT_TEMPERATURE_LOW, PLANT_VOLTAGE,
};
use crate::mock::mock_inner_loop_controller::MockInnerLoopController;
use crate::mock::mock_power_system::MockPowerSystem;
use crate::power::config_power::ConfigPower;
use crate::utility::{get_parameter, read_file_disp_ims};

#[derive(Clone)]
pub struct MockPlant {
    _static_transfer_matrix: SMatrix<f64, NUM_ACTUATOR, NUM_ACTUATOR>,
    _disp_matrix_inv: SMatrix<f64, NUM_IMS_READING, NUM_SPACE_DEGREE_OF_FREEDOM>,
    _disp_offset: SVector<f64, NUM_IMS_READING>,
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
    // Ring temperature in degree Celsius.
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
    /// delta force from the change of steps. This is a (NUM_ACTUATOR x
    /// NUM_ACTUATOR) matrix. The unit of row is the Newton and the unit of
    /// column is the actuator's step.
    /// * `inclinometer_angle` - The inclinometer angle in degree.
    ///
    /// # Returns
    /// A new mock plant.
    pub fn new(static_transfer_matrix: &Vec<Vec<f64>>, inclinometer_angle: f64) -> Self {
        let matrix = SMatrix::from_row_iterator(
            static_transfer_matrix
                .iter()
                .flat_map(|row| row.iter().copied()),
        );

        let (disp_matrix, disp_offset) = read_file_disp_ims(Path::new("config/disp_ims.yaml"));
        let disp_smatrix: SMatrix<f64, NUM_SPACE_DEGREE_OF_FREEDOM, NUM_IMS_READING> =
            SMatrix::from_row_iterator(disp_matrix.iter().flat_map(|row| row.iter().copied()));

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
            config_power.get_time_breaker_on(),
            config_power.get_time_breaker_off(),
        );
        let power_system_motor = MockPowerSystem::new(
            PLANT_VOLTAGE,
            PLANT_CURRENT_MOTOR,
            config_power.breaker_operating_voltage,
            config_power.loop_time as i32,
            config_power.get_time_power_on(PowerType::Motor),
            config_power.get_time_power_off(PowerType::Motor),
            config_power.get_time_breaker_on(),
            config_power.get_time_breaker_off(),
        );

        Self {
            _static_transfer_matrix: matrix,

            _disp_matrix_inv: disp_smatrix
                .pseudo_inverse(f64::EPSILON)
                .expect("Should be able to compute the pseudo inverse of displacement matrix."),
            _disp_offset: SVector::from_iterator(disp_offset.iter().copied()),

            actuator_steps: vec![0; NUM_ACTUATOR],
            _actuator_force_weight: Self::get_forces_mirror_weight(
                inclinometer_angle,
                inclinometer_offset,
                mirror_weight_kg,
            ),
            _actuator_force_step: vec![0.0; NUM_ACTUATOR],

            inclinometer_angle: inclinometer_angle,
            _inclinometer_offset: inclinometer_offset,

            _mirror_weight_kg: mirror_weight_kg,

            temperature_ring: temperature_ring,
            temperature_intake: vec![PLANT_TEMPERATURE_LOW; NUM_TEMPERATURE_INTAKE],
            temperature_exhaust: vec![PLANT_TEMPERATURE_HIGH; NUM_TEMPERATURE_EXHAUST],

            power_system_communication: power_system_communication,
            power_system_motor: power_system_motor,

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
                DigitalInput::InterlockPowerRelay,
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
        let mut index_tangent_link = vec![1, 2, 4, 5];
        index_tangent_link
            .iter_mut()
            .for_each(|x| *x += NUM_AXIAL_ACTUATOR);

        let tangent_force_direction = vec![-1.0, -1.0, 1.0, 1.0];
        let force_tangent = force_mirror_weight * angle_correct.to_radians().cos()
            / (index_tangent_link.len() as f64);

        index_tangent_link
            .iter()
            .zip(tangent_force_direction.iter())
            .for_each(|(index, direction)| {
                forces[*index as usize] = force_tangent * (*direction);
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

    /// Get the inner-loop controller (ILC) data of 78 actuators.
    ///
    /// # Returns
    /// A tuple containing the ILC status, actuator encoders, and actuator
    /// forces.
    pub fn get_actuator_ilc_data(&mut self) -> (Vec<u8>, Vec<i32>, Vec<f64>) {
        (
            self.get_actuator_ilc_status(),
            self.get_actuator_encoders(),
            self.get_actuator_forces(),
        )
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
    fn get_actuator_forces(&self) -> Vec<f64> {
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

    /// Move the actuator steps.
    ///
    /// # Arguments
    /// * `actuator_steps` - The actuator steps to move.
    ///
    /// # Panics
    /// If the length of the actuator steps is not equal to the number of
    /// actuators.
    pub fn move_actuator_steps(&mut self, actuator_steps: &Vec<i32>) {
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

    /// Get the status of 78 actuator inner-loop controllers.
    ///
    /// # Returns
    /// A vector of the status of 78 actuator inner-loop controllers.
    fn get_actuator_ilc_status(&mut self) -> Vec<u8> {
        let mut ilc_status = vec![0; NUM_ACTUATOR];
        if self.power_system_communication.is_power_on {
            for (idx, ilc) in self._ilcs.iter_mut().enumerate() {
                if idx < NUM_ACTUATOR {
                    ilc_status[idx] = ilc.get_status();
                }
            }
        }

        ilc_status
    }

    /// Set the mode of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The address of ILC.
    /// * `mode` - The mode to be set.
    ///
    /// # Returns
    /// Current ILC mode.
    pub fn set_ilc_mode(
        &mut self,
        address: usize,
        mode: InnerLoopControlMode,
    ) -> InnerLoopControlMode {
        self._ilcs[address].set_mode(mode)
    }

    /// Get the mode of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The address of ILC.
    ///
    /// # Returns
    /// ILC mode.
    pub fn get_ilc_mode(&self, address: usize) -> InnerLoopControlMode {
        self._ilcs[address].mode
    }

    /// Calculate the independent measurement system (IMS) readings based on the
    /// rigid body position.
    ///
    /// # Arguments
    /// * `x` - The x position in micron.
    /// * `y` - The y position in micron.
    /// * `z` - The z position in micron.
    /// * `rx` - The rotation around x-axis in arcsec.
    /// * `ry` - The rotation around y-axis in arcsec.
    /// * `rz` - The rotation around z-axis in arcsec.
    ///
    /// # Returns
    /// A tuple of the theta_z and delta_z readings in micron.
    pub fn calculate_ims_readings(
        &self,
        x: f64,
        y: f64,
        z: f64,
        rx: f64,
        ry: f64,
        rz: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        // Change the unit from mm to um
        let reading_ims = (self._disp_matrix_inv * SVector::from_vec(vec![x, y, z, rx, ry, rz])
            + self._disp_offset)
            * 1e3;

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
    use crate::mock::mock_constants::{
        TEST_DIGITAL_INPUT_NO_POWER, TEST_DIGITAL_INPUT_POWER_COMM,
        TEST_DIGITAL_INPUT_POWER_COMM_MOTOR,
    };
    use crate::utility::read_file_stiffness;

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
        mock_plant.digital_output = 0;

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

        // Motor power
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
    fn test_get_actuator_ilc_data() {
        let mut mock_plant = create_mock_plant();

        let (ilc_status, encoders, forces) = mock_plant.get_actuator_ilc_data();

        assert_eq!(ilc_status.len(), NUM_ACTUATOR);
        assert_eq!(encoders.len(), NUM_ACTUATOR);
        assert_eq!(forces.len(), NUM_ACTUATOR);
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
    fn test_get_actuator_ilc_status() {
        let mut mock_plant = create_mock_plant();

        // No communication power
        assert_eq!(mock_plant.get_actuator_ilc_status(), vec![0; NUM_ACTUATOR]);

        // With communication power
        mock_plant.power_system_communication.is_power_on = true;

        assert_eq!(mock_plant.get_actuator_ilc_status(), vec![0; NUM_ACTUATOR]);
        assert_eq!(mock_plant.get_actuator_ilc_status(), vec![16; NUM_ACTUATOR]);
    }

    #[test]
    fn test_set_ilc_mode() {
        let mut mock_plant = create_mock_plant();

        let mode = InnerLoopControlMode::Disabled;

        assert_eq!(mock_plant.set_ilc_mode(3, mode), mode);
    }

    #[test]
    fn test_get_ilc_mode() {
        let mut mock_plant = create_mock_plant();

        let address = 3;
        let mode = InnerLoopControlMode::Disabled;
        mock_plant.set_ilc_mode(address, mode);

        assert_eq!(mock_plant.get_ilc_mode(address), mode);
    }

    #[test]
    fn test_calculate_ims_readings() {
        let mock_plant = create_mock_plant();

        let (disp_matrix, disp_offset) = read_file_disp_ims(Path::new("config/disp_ims.yaml"));
        let matrix: SMatrix<f64, NUM_SPACE_DEGREE_OF_FREEDOM, NUM_IMS_READING> =
            SMatrix::from_row_iterator(disp_matrix.iter().flat_map(|row| row.iter().copied()));
        let offset: SVector<f64, NUM_IMS_READING> =
            SVector::from_iterator(disp_offset.iter().copied());

        // Zero position
        let (theta_z, delta_z) = mock_plant.calculate_ims_readings(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        let (x, y, z, rx, ry, rz) = calculate_position_ims(&matrix, &offset, &theta_z, &delta_z);

        assert_relative_eq!(x, 0.0, epsilon = EPSILON);
        assert_relative_eq!(y, 0.0, epsilon = EPSILON);
        assert_relative_eq!(z, 0.0, epsilon = EPSILON);
        assert_relative_eq!(rx, 0.0, epsilon = EPSILON);
        assert_relative_eq!(ry, 0.0, epsilon = EPSILON);
        assert_relative_eq!(rz, 0.0, epsilon = EPSILON);

        // Non-zero position
        let (theta_z, delta_z) =
            mock_plant.calculate_ims_readings(10.0, 20.0, 30.0, 40.0, 50.0, 60.0);

        let (x, y, z, rx, ry, rz) = calculate_position_ims(&matrix, &offset, &theta_z, &delta_z);

        assert_relative_eq!(x, 10.0, epsilon = EPSILON);
        assert_relative_eq!(y, 20.0, epsilon = EPSILON);
        assert_relative_eq!(z, 30.0, epsilon = EPSILON);
        assert_relative_eq!(rx, 40.0, epsilon = EPSILON);
        assert_relative_eq!(ry, 50.0, epsilon = EPSILON);
        assert_relative_eq!(rz, 60.0, epsilon = EPSILON);
    }
}
