use nalgebra::{SMatrix, SVector};

use crate::constants::{
    NUM_ACTIVE_ACTUATOR, NUM_ACTIVE_ACTUATOR_AXIAL, NUM_ACTIVE_ACTUATOR_TANGENT,
    NUM_AXIAL_ACTUATOR, NUM_HARDPOINTS, NUM_HARDPOINTS_AXIAL, NUM_TANGENT_LINK,
};
use crate::control::biquadratic_filter::BiquadraticFilter;
use crate::control::deadband_control::DeadbandControl;
use crate::control::gain_schedular::GainSchedular;
use crate::control::in_position::InPosition;
use crate::control::math_tool::clip;
use crate::control::simple_delay_filter::SimpleDelayFilter;

pub struct ClosedLoop {
    _prefilter_axial: BiquadraticFilter,
    _prefilter_tangent: BiquadraticFilter,
    _command_delay_axial: SimpleDelayFilter,
    _command_delay_tangent: SimpleDelayFilter,
    _control_filter_axial: BiquadraticFilter,
    _control_filter_tangent: BiquadraticFilter,
    _feedforward_delay_filter: SimpleDelayFilter,
    // Decoupling matrix
    pub kdc: SMatrix<f64, NUM_ACTIVE_ACTUATOR, NUM_ACTIVE_ACTUATOR>,
    // Influence matrix
    pub kinfl: SMatrix<f64, NUM_ACTIVE_ACTUATOR, NUM_ACTIVE_ACTUATOR>,
    // Hardpoint compensation matrix
    pub hd_comp: SMatrix<f64, NUM_ACTIVE_ACTUATOR, NUM_HARDPOINTS>,
    _deadband_control_axial: DeadbandControl,
    _deadband_control_tangent: DeadbandControl,
    _gain_schedular: GainSchedular,
    _step_limit_axial: i32,
    _step_limit_tangent: i32,
    _in_position: InPosition,
    _is_feedforward: bool,
    _is_feedback: bool,
    _is_deadzone_enabled_axial: bool,
    _is_deadzone_enabled_tangent: bool,
}

impl ClosedLoop {
    /// Closed-loop control that implements the force control algorithm.
    ///
    /// # Arguments
    /// * `gain_prefilter_axial` - Gain of the prefilter for the axial
    /// actuators.
    /// * `params_prefilter_axial` - Parameters of the prefilter for the axial
    /// actuators: [a11 a21 b11 b21 a12 a22 b12 b22 ... a1N a2N b1N b2N].
    /// * `gain_prefilter_tangent` - Gain of the prefilter for the tangent
    /// actuators.
    /// * `params_prefilter_tangent` - Parameters of the prefilter for the
    /// tangent actuators: [a11 a21 b11 b21 a12 a22 b12 b22 ... a1N a2N b1N
    /// b2N].
    /// * `params_cmd_delay_axial` - Parameters of the command delay filter for
    /// the axial actuators: [b0, b1, b2, ..., bN].
    /// * `params_cmd_delay_tangent` - Parameters of the command delay filter
    /// for the tangent actuators: [b0, b1, b2, ..., bN].
    /// * `gain_control_filter_axial` - Gain of the control filter for the axial
    /// actuators.
    /// * `params_control_filter_axial` - Parameters of the control filter for
    /// the axial actuators: [a11 a21 b11 b21 a12 a22 b12 b22 ... a1N a2N b1N
    /// b2N].
    /// * `gain_control_filter_tangent` - Gain of the control filter for the
    /// tangent actuators.
    /// * `params_control_filter_tangent` - Parameters of the control filter for
    /// the tangent actuators: [a11 a21 b11 b21 a12 a22 b12 b22 ... a1N a2N b1N
    /// b2N].
    /// * `kdc` - Decoupling matrix.
    /// * `kinfl` - Influence matrix.
    /// * `hd_comp` - Hardpoint compensation matrix.
    /// * `thresholds_deadzone_axial` - The [lower, upper] thresholds of the
    /// axial hardpoint error in deadzone. The unit is Newton.
    /// * `thresholds_deadzone_tangent` - The [lower, upper] thresholds of the
    /// tangent hardpoint error in deadzone. The unit is Newton.
    /// * `min_gain_schedular_axial` - Minimum gain for the axial actuators in
    /// the gain schedular. The value should be in (0.0, 1.0).
    /// * `min_gain_schedular_tangent` - Minimum gain for the tangent actuators
    /// in the gain schedular. The value should be in (0.0, 1.0).
    /// * `num_sample_ramp_up` - Number of samples in the ramping up process in
    /// the gain schedular. This value can not be 0.
    /// * `num_sample_ramp_down` - Number of samples in the ramping down process
    /// in the gain schedular. This value can not be 0.
    /// * `max_sample_settle` - Maximum number of samples in the settling
    /// process in the gain schedular. This value can not be 0.
    /// * `step_limit_axial` - Step limit of the axial actuator in each control
    /// cycle.
    /// * `step_limit_tangent` - Step limit of the tangent actuator in each
    /// control cycle.
    /// * `in_position_window_size` - Window size in second to judge the mirror
    /// is in position or not.
    /// * `in_position_control_frequency` - Control frequency in Hz to judge
    /// the mirror is in position or not.
    /// * `in_position_threshold_axial` - Threshold of the force error of axial
    /// actuator in Newton to judge the mirror is in position or not.
    /// * `in_position_threshold_tangent` - Threshold of the force error of
    /// tangent actuator in Newton to judge the mirror is in position or not.
    /// * `is_feedforward` - The feedforward is on or not.
    /// * `is_feedback` - The feedback is on or not.
    /// * `is_deadzone_enabled_axial` - Deadzone is enabled or not for the axial
    /// hardpoints.
    /// * `is_deadzone_enabled_tangent` - Deadzone is enabled or not for the
    /// tangent hardpoints.
    ///
    /// # Returns
    /// A new instance of the closed-loop.
    pub fn new(
        gain_prefilter_axial: f64,
        params_prefilter_axial: &Vec<Vec<f64>>,
        gain_prefilter_tangent: f64,
        params_prefilter_tangent: &Vec<Vec<f64>>,
        params_cmd_delay_axial: &Vec<f64>,
        params_cmd_delay_tangent: &Vec<f64>,
        gain_control_filter_axial: f64,
        params_control_filter_axial: &Vec<Vec<f64>>,
        gain_control_filter_tangent: f64,
        params_control_filter_tangent: &Vec<Vec<f64>>,
        kdc: &SMatrix<f64, NUM_ACTIVE_ACTUATOR, NUM_ACTIVE_ACTUATOR>,
        kinfl: &SMatrix<f64, NUM_ACTIVE_ACTUATOR, NUM_ACTIVE_ACTUATOR>,
        hd_comp: &SMatrix<f64, NUM_ACTIVE_ACTUATOR, NUM_HARDPOINTS>,
        thresholds_deadzone_axial: &Vec<f64>,
        thresholds_deadzone_tangent: &Vec<f64>,
        min_gain_schedular_axial: f64,
        min_gain_schedular_tangent: f64,
        num_sample_ramp_up: i32,
        num_sample_ramp_down: i32,
        max_sample_settle: i32,
        step_limit_axial: i32,
        step_limit_tangent: i32,
        in_position_window_size: f64,
        in_position_control_frequency: f64,
        in_position_threshold_axial: f64,
        in_position_threshold_tangent: f64,
        is_feedforward: bool,
        is_feedback: bool,
        is_deadzone_enabled_axial: bool,
        is_deadzone_enabled_tangent: bool,
    ) -> Self {
        Self {
            _prefilter_axial: BiquadraticFilter::new(
                gain_prefilter_axial,
                params_prefilter_axial,
                NUM_AXIAL_ACTUATOR,
            ),
            _prefilter_tangent: BiquadraticFilter::new(
                gain_prefilter_tangent,
                params_prefilter_tangent,
                NUM_TANGENT_LINK,
            ),

            _command_delay_axial: SimpleDelayFilter::new(
                params_cmd_delay_axial,
                NUM_AXIAL_ACTUATOR,
            ),
            _command_delay_tangent: SimpleDelayFilter::new(
                params_cmd_delay_tangent,
                NUM_TANGENT_LINK,
            ),

            _control_filter_axial: BiquadraticFilter::new(
                gain_control_filter_axial,
                params_control_filter_axial,
                NUM_ACTIVE_ACTUATOR_AXIAL,
            ),
            _control_filter_tangent: BiquadraticFilter::new(
                gain_control_filter_tangent,
                params_control_filter_tangent,
                NUM_ACTIVE_ACTUATOR_TANGENT,
            ),

            _feedforward_delay_filter: SimpleDelayFilter::new(
                &vec![1.0, -1.0],
                NUM_ACTIVE_ACTUATOR,
            ),

            kdc: kdc.clone(),
            kinfl: kinfl.clone(),

            hd_comp: hd_comp.clone(),

            _deadband_control_axial: DeadbandControl::new(
                thresholds_deadzone_axial[0],
                thresholds_deadzone_axial[1],
            ),
            _deadband_control_tangent: DeadbandControl::new(
                thresholds_deadzone_tangent[0],
                thresholds_deadzone_tangent[1],
            ),

            _gain_schedular: GainSchedular::new(
                min_gain_schedular_axial,
                min_gain_schedular_tangent,
                num_sample_ramp_up,
                num_sample_ramp_down,
                max_sample_settle,
            ),

            _step_limit_axial: step_limit_axial,
            _step_limit_tangent: step_limit_tangent,

            _in_position: InPosition::new(
                in_position_window_size,
                in_position_control_frequency,
                in_position_threshold_axial,
                in_position_threshold_tangent,
            ),

            _is_feedforward: is_feedforward,
            _is_feedback: is_feedback,

            _is_deadzone_enabled_axial: is_deadzone_enabled_axial,
            _is_deadzone_enabled_tangent: is_deadzone_enabled_tangent,
        }
    }

    /// Reset the history.
    ///
    /// # Arguments
    /// * `reset_all` - Reset all of the internal data or not.
    pub fn reset(&mut self, reset_all: bool) {
        self._prefilter_axial.reset();
        self._prefilter_tangent.reset();

        self._command_delay_axial.reset();
        self._command_delay_tangent.reset();

        self._control_filter_axial.reset();
        self._control_filter_tangent.reset();

        self._feedforward_delay_filter.reset();

        self._deadband_control_axial.reset(reset_all);
        self._deadband_control_tangent.reset(reset_all);

        self._gain_schedular.reset();

        self._in_position.reset();
    }

    /// Calculate the actuator steps to move in the next control cycle.
    ///
    /// # Arguments
    /// * `force_demanded` - Demanded force in Newton.
    /// * `force_measured` - Measured force in Newton.
    /// * `hardpoints` - Six 0-based hardpoints. The order is from low to high.
    /// * `is_in_position` - Mirror is in position or not.
    ///
    /// # Returns
    /// The first one is the 78 actuator steps to move in the next control
    /// cycle. The second one is the 78 hardpoint correction in Newton. The
    /// third one is the mirror is in position or not.
    pub fn calc_actuator_steps(
        &mut self,
        force_demanded: &[f64],
        force_measured: &[f64],
        hardpoints: &[usize],
        is_in_position: bool,
    ) -> (Vec<i32>, Vec<f64>, bool) {
        // Filter the demanded forces
        let demand_filtered_axial =
            self.filter_force_demanded(&force_demanded[..NUM_AXIAL_ACTUATOR], true);
        let demand_filtered_tangent =
            self.filter_force_demanded(&force_demanded[NUM_AXIAL_ACTUATOR..], false);

        // Get the active and passive demanded forces
        let (demand_axial_passive, demand_axial_active) =
            Self::split_1d_array(&demand_filtered_axial, &hardpoints[..NUM_HARDPOINTS_AXIAL]);

        let hardpoints_tangent_reindexed = hardpoints[NUM_HARDPOINTS_AXIAL..]
            .iter()
            .map(|hardpoint| hardpoint - NUM_AXIAL_ACTUATOR)
            .collect::<Vec<usize>>();
        let (demand_tangent_passive, demand_tangent_active) =
            Self::split_1d_array(&demand_filtered_tangent, &hardpoints_tangent_reindexed);

        // Do the feedback
        let (feedback_axial, feedback_tangent, hardpoint_correction) = self.calc_force_feedback(
            force_measured,
            &demand_axial_passive,
            &demand_tangent_passive,
            hardpoints,
        );

        let error_axial_active = if self._is_feedback {
            demand_axial_active
                .iter()
                .zip(feedback_axial.iter())
                .map(|(demand, feedback)| demand - feedback)
                .collect()
        } else {
            demand_axial_active
        };

        let error_tangent_active = if self._is_feedback {
            demand_tangent_active
                .iter()
                .zip(feedback_tangent.iter())
                .map(|(demand, feedback)| demand - feedback)
                .collect()
        } else {
            demand_tangent_active
        };

        // Apply the control filter
        let error_axial_active_filter = self._control_filter_axial.filter(&error_axial_active);
        let error_tangent_active_filter =
            self._control_filter_tangent.filter(&error_tangent_active);

        // Multiply with the decoupling matrix and gains
        let mut error_active_filter = error_axial_active_filter;
        error_active_filter.extend(error_tangent_active_filter);

        let mut steps_active: SVector<f64, NUM_ACTIVE_ACTUATOR> =
            self.kdc * SVector::from_iterator(error_active_filter);

        let (gain_axial, gain_tangent) = self._gain_schedular.get_gain(is_in_position);
        steps_active
            .row_iter_mut()
            .enumerate()
            .for_each(|(index, mut step)| {
                if index < NUM_ACTIVE_ACTUATOR_AXIAL {
                    step *= gain_axial;
                } else {
                    step *= gain_tangent;
                }
            });

        // Do the feedforward

        // Note that we are using the demanded force instead of the
        // "pre-filtered" demanded force to do the feedforward. This is
        // implemented in the ts_mtm2_cell, which is different from the
        // vendor's documentation.
        // I do not understand the reason for this inconsistency and the
        // original LabVIEW developer only commented this is just for the
        // temporary use in "Control Loop (FIFO).vi".
        let steps_feedforward = self.calc_steps_feedforward(&force_demanded, hardpoints);
        let steps = if self._is_feedforward {
            steps_active + SVector::<f64, NUM_ACTIVE_ACTUATOR>::from_iterator(steps_feedforward)
        } else {
            steps_active
        };

        // Insert the hardpoints
        let mut steps_all = steps.as_slice().to_vec();
        let mut hardpoint_correction_all = hardpoint_correction.clone();

        hardpoints.iter().for_each(|hardpoint| {
            steps_all.insert(*hardpoint, 0.0);
            hardpoint_correction_all.insert(*hardpoint, 0.0);
        });

        // Saturate the steps in each control cycle
        // Multiply the hardpoint_correction_all with -1 to make sure we have:
        // "force_measured = force_demanded + hardpoint_correction_all'"
        // to make the data analysis easier
        let steps_all_saturated = self.saturate_actuator_steps(
            &steps_all
                .iter()
                .map(|&step| step as i32)
                .collect::<Vec<i32>>(),
        );

        hardpoint_correction_all
            .iter_mut()
            .for_each(|correction| *correction *= -1.0);

        let mut error_active = error_axial_active;
        error_active.extend(error_tangent_active);
        let is_in_position = self._in_position.is_in_position(&error_active);

        (
            steps_all_saturated,
            hardpoint_correction_all,
            is_in_position,
        )
    }

    /// Filter the demanded force.
    ///
    /// # Arguments
    /// * `force_demanded` - Demanded force in Newton.
    /// * `is_axail` - Is the axial actuator or not.
    ///
    /// # Returns
    /// Filtered demanded force.
    fn filter_force_demanded(&mut self, force_demanded: &[f64], is_axail: bool) -> Vec<f64> {
        let prefilter = if is_axail {
            &mut self._prefilter_axial
        } else {
            &mut self._prefilter_tangent
        };
        let command_delay_filter = if is_axail {
            &mut self._command_delay_axial
        } else {
            &mut self._command_delay_tangent
        };

        let force_demanded_prefilter = prefilter.filter(force_demanded);
        command_delay_filter.filter(&force_demanded_prefilter)
    }

    /// Split the 1D array.
    ///
    /// # Arguments
    /// * `array` - 1D array.
    /// * `indices` - Specific indices in array.
    ///
    /// # Returns
    /// The splited arrays. The first one is the splitted array with the
    /// specified indices. The second one is the remaining array.
    fn split_1d_array(array: &[f64], indices: &[usize]) -> (Vec<f64>, Vec<f64>) {
        let mut array1 = Vec::new();
        let mut array2 = Vec::new();
        array.iter().enumerate().for_each(|(idx, value)| {
            if indices.contains(&idx) {
                array1.push(*value);
            } else {
                array2.push(*value);
            }
        });

        (array1, array2)
    }

    /// Calculate the feedbacked force.
    ///
    /// # Arguments
    /// * `force_measured` - Measured force in Newton.
    /// * `demand_hardpoint_axial` - Demanded force of the axial hardpoints in
    /// Newton.
    /// * `demand_hardpoint_tangent` - Demanded force of the tangent hardpoints
    /// in Newton.
    /// * `hardpoints` - Six 0-based hardpoints. The order is from low to high.
    ///
    /// # Returns
    /// The first one is the feedbacked force of the axial actuators in Newton.
    /// The second one is the feedbacked force of the tangent actuators in
    /// Newton. The third one is the hardpoint correction in Newton (for the
    /// active actuators).
    fn calc_force_feedback(
        &mut self,
        force_measured: &[f64],
        demand_hardpoint_axial: &[f64],
        demand_hardpoint_tangent: &[f64],
        hardpoints: &[usize],
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        // Select the latched forces of hardpoints
        let (measured_passive, measured_active) = Self::split_1d_array(force_measured, hardpoints);

        let latched_passive_axial = self._deadband_control_axial.select(
            &demand_hardpoint_axial
                .iter()
                .zip(measured_passive[..NUM_HARDPOINTS_AXIAL].iter())
                .map(|(demand, measured)| demand - measured)
                .collect::<Vec<f64>>(),
            self._is_deadzone_enabled_axial,
        );
        let latched_passive_tangent = self._deadband_control_tangent.select(
            &demand_hardpoint_tangent
                .iter()
                .zip(measured_passive[NUM_HARDPOINTS_AXIAL..].iter())
                .map(|(demand, measured)| demand - measured)
                .collect::<Vec<f64>>(),
            self._is_deadzone_enabled_tangent,
        );

        // The feedbacked forces of active actutors need to consider the
        // hardpoint compensation.
        let mut latched_passive = latched_passive_axial;
        latched_passive.extend(latched_passive_tangent);

        let hardpoint_correction = (self.hd_comp * SVector::from_iterator(latched_passive))
            .as_slice()
            .to_vec();

        let feedback_axial = measured_active[..NUM_ACTIVE_ACTUATOR_AXIAL]
            .iter()
            .zip(hardpoint_correction[..NUM_ACTIVE_ACTUATOR_AXIAL].iter())
            .map(|(measured, correction)| measured + correction)
            .collect();
        let feedback_tangent = measured_active[NUM_ACTIVE_ACTUATOR_AXIAL..]
            .iter()
            .zip(hardpoint_correction[NUM_ACTIVE_ACTUATOR_AXIAL..].iter())
            .map(|(measured, correction)| measured + correction)
            .collect();

        (feedback_axial, feedback_tangent, hardpoint_correction)
    }

    /// Calculate the feedforward steps.
    ///
    /// # Arguments
    /// * `force_demanded` - Demanded force in Newton.
    /// * `hardpoints` - Six 0-based hardpoints. The order is from low to high.
    ///
    /// # Returns
    /// Feedforward steps.
    fn calc_steps_feedforward(&mut self, force_demanded: &[f64], hardpoints: &[usize]) -> Vec<f64> {
        // Calculate the feedforward matrix
        let (force_demanded_passive, force_demanded_active) =
            Self::split_1d_array(force_demanded, hardpoints);

        let force_demanded_hd_comp: SVector<f64, NUM_ACTIVE_ACTUATOR> =
            self.hd_comp * SVector::from_iterator(force_demanded_passive);

        let force_demanded_active_diff: SVector<f64, NUM_ACTIVE_ACTUATOR> =
            SVector::from_iterator(force_demanded_active) - force_demanded_hd_comp;

        let steps: SVector<f64, NUM_ACTIVE_ACTUATOR> = self.kinfl * force_demanded_active_diff;

        // Pass the delay filter
        self._feedforward_delay_filter.filter(&steps.as_slice())
    }

    /// Saturate the actuator steps in each control cycle.
    ///
    /// # Arguments
    /// * `actuator_steps` - The 78 actuator steps to move in each control
    /// cycle.
    ///
    /// # Returns
    /// The saturated 78 actuator steps to move in each control cycle.
    fn saturate_actuator_steps(&self, actuator_steps: &[i32]) -> Vec<i32> {
        actuator_steps
            .iter()
            .enumerate()
            .map(|(idx, step)| {
                if idx < NUM_AXIAL_ACTUATOR {
                    clip(*step, -self._step_limit_axial, self._step_limit_axial)
                } else {
                    clip(*step, -self._step_limit_tangent, self._step_limit_tangent)
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::path::Path;
    use std::vec;

    use approx::assert_relative_eq;

    use crate::constants::{NUM_ACTUATOR, NUM_TEMPERATURE_RING};
    use crate::control::lut::Lut;
    use crate::control::math_tool::{
        calc_cmd_delay_filter_params, calc_hp_comp_matrix, calc_kinetic_decoupling_matrix,
        correct_inclinometer_angle,
    };
    use crate::mock::mock_constants::{PLANT_TEMPERATURE_HIGH, PLANT_TEMPERATURE_LOW};
    use crate::mock::mock_plant::MockPlant;
    use crate::utility::{read_file_cell_geom, read_file_stiffness};

    const EPSILON: f64 = 1e-7;

    fn create_closed_loop() -> ClosedLoop {
        let params_prefilter = vec![vec![0.0; 4]; 8];

        let control_frequency = 20.0;
        let params_cmd_delay = calc_cmd_delay_filter_params(false, control_frequency, false, 5);

        // Surrogate values for the closed loop parameters.
        let gain_control_filter = 0.3387225;

        // Decoupling matrix
        let loc_act_axial = read_file_cell_geom(Path::new("config/cell_geom.yaml")).0;
        let stiffness = read_file_stiffness(Path::new("config/stiff_matrix_surrogate.yaml"));

        let hardpoints = [5, 15, 25, 73, 75, 77];
        let kdc = calc_kinetic_decoupling_matrix(
            &loc_act_axial,
            &hardpoints[..NUM_HARDPOINTS_AXIAL],
            &hardpoints[NUM_HARDPOINTS_AXIAL..],
            &stiffness,
        );

        // Hardpoint compensation matrix
        let (hd_comp_axial, hd_comp_tangent) = calc_hp_comp_matrix(
            &loc_act_axial,
            &hardpoints[..NUM_HARDPOINTS_AXIAL],
            &hardpoints[NUM_HARDPOINTS_AXIAL..],
        );
        let mut hd_comp: SMatrix<f64, NUM_ACTIVE_ACTUATOR, NUM_HARDPOINTS> = SMatrix::zeros();
        hd_comp
            .view_mut((0, 0), hd_comp_axial.shape())
            .copy_from(&hd_comp_axial);
        hd_comp
            .view_mut(hd_comp_axial.shape(), hd_comp_tangent.shape())
            .copy_from(&hd_comp_tangent);

        ClosedLoop::new(
            1.0,
            &params_prefilter,
            1.0,
            &params_prefilter,
            &params_cmd_delay,
            &params_cmd_delay,
            gain_control_filter,
            &params_prefilter,
            gain_control_filter,
            &params_prefilter,
            &kdc,
            &kdc,
            &hd_comp,
            &vec![0.0, 0.0],
            &vec![0.0, 0.0],
            0.28,
            0.1,
            1,
            10,
            40,
            75,
            75,
            1.0,
            control_frequency,
            0.2,
            1.1,
            true,
            true,
            true,
            true,
        )
    }

    fn create_mock_plant(inclinometer_angle: f64) -> MockPlant {
        let filepath = Path::new("config/stiff_matrix_m2.yaml");
        let stiffness = read_file_stiffness(filepath);

        MockPlant::new(&stiffness, inclinometer_angle)
    }

    fn get_demanded_force(inclinometer_angle: f64) -> Vec<f64> {
        let inclinometer_offset = 0.94;
        let lut_angle = correct_inclinometer_angle(inclinometer_angle, inclinometer_offset);

        let lut = Lut::new(Path::new("config/lut/handling"));
        let (forces_gravity, forces_temperature) = lut.get_lut_forces(
            lut_angle,
            &get_ring_temperature(),
            &vec![21.0; NUM_TEMPERATURE_RING],
            true,
        );

        forces_gravity
            .iter()
            .zip(forces_temperature.iter())
            .map(|(f1, f2)| f1 + f2)
            .collect()
    }

    fn get_ring_temperature() -> Vec<f64> {
        let mut temperature = vec![PLANT_TEMPERATURE_LOW; NUM_TEMPERATURE_RING];

        // Indexes of the ring temperatures that have a higher temperature.
        let indexes: [usize; 4] = [3, 6, 7, 11];
        indexes.iter().for_each(|&idx| {
            temperature[idx] = PLANT_TEMPERATURE_HIGH;
        });

        temperature
    }

    #[test]
    fn test_calc_actuator_steps() {
        let inclinometer_angle = 120.0;
        let force_demanded = get_demanded_force(inclinometer_angle);

        let mut plant = create_mock_plant(inclinometer_angle);
        let mut closed_loop = create_closed_loop();

        let mut steps = vec![0; NUM_ACTUATOR];
        let mut hardpoint_correction = vec![0.0; NUM_ACTUATOR];
        let mut is_in_position = false;
        for idx in 0..70 {
            let force_measured = plant.get_actuator_forces();
            (steps, hardpoint_correction, is_in_position) = closed_loop.calc_actuator_steps(
                &force_demanded,
                &force_measured,
                &vec![5, 15, 25, 73, 75, 77],
                is_in_position,
            );
            // Check the saturated hardpoint correction before the integration
            // with the plant model.
            if idx == 10 {
                assert_relative_eq!(hardpoint_correction[0], 58.5897286, epsilon = EPSILON);
                assert_relative_eq!(hardpoint_correction[1], 58.0630028, epsilon = EPSILON);
                assert_eq!(hardpoint_correction[NUM_ACTUATOR - 1], 0.0);
                assert_relative_eq!(
                    hardpoint_correction[NUM_ACTUATOR - 2],
                    1996.7958437,
                    epsilon = EPSILON
                );
                assert_eq!(hardpoint_correction[NUM_ACTUATOR - 3], 0.0);
                assert_relative_eq!(
                    hardpoint_correction[NUM_ACTUATOR - 4],
                    -1902.7131771,
                    epsilon = EPSILON
                );
            }

            // We need to let the calculation of hardpoint_correction saturates
            // first before the integration with the plant model.
            if idx >= 10 {
                plant.move_actuator_steps(&steps);
            }
        }

        assert_relative_eq!(hardpoint_correction[0], -19.9076253, epsilon = EPSILON);
        assert_relative_eq!(hardpoint_correction[1], -19.5104835, epsilon = EPSILON);
        assert_eq!(hardpoint_correction[NUM_ACTUATOR - 1], 0.0);
        assert_relative_eq!(
            hardpoint_correction[NUM_ACTUATOR - 2],
            2002.7442391,
            epsilon = EPSILON
        );
        assert_eq!(hardpoint_correction[NUM_ACTUATOR - 3], 0.0);
        assert_relative_eq!(
            hardpoint_correction[NUM_ACTUATOR - 4],
            -1947.9413411,
            epsilon = EPSILON
        );

        let actuator_steps = plant.actuator_steps;
        assert_eq!(actuator_steps[0], -236);
        assert_eq!(actuator_steps[1], -743);
        assert_eq!(actuator_steps[NUM_ACTUATOR - 1], 0);
        assert_eq!(actuator_steps[NUM_ACTUATOR - 2], -100);
        assert_eq!(actuator_steps[NUM_ACTUATOR - 3], 0);
        assert_eq!(actuator_steps[NUM_ACTUATOR - 4], 573);

        assert_eq!(steps.iter().map(|step: &i32| step.abs()).sum::<i32>(), 0);

        assert!(is_in_position);
    }

    #[test]
    fn test_split_1d_array() {
        let (array_idx, array_no_idx) =
            ClosedLoop::split_1d_array(&vec![1.0, 2.0, 3.0, 4.0], &vec![1, 2]);

        assert_eq!(array_idx, vec![2.0, 3.0]);
        assert_eq!(array_no_idx, vec![1.0, 4.0]);
    }

    #[test]
    fn test_saturate_actuator_steps() {
        let closed_loop = create_closed_loop();

        let mut actuator_steps = vec![0; NUM_ACTUATOR];
        actuator_steps[0] = 100;
        actuator_steps[1] = -100;
        actuator_steps[NUM_ACTUATOR - 1] = 100;
        actuator_steps[NUM_ACTUATOR - 2] = -100;

        let actuator_steps_saturated = closed_loop.saturate_actuator_steps(&actuator_steps);

        assert_eq!(actuator_steps_saturated[0], closed_loop._step_limit_axial);
        assert_eq!(actuator_steps_saturated[1], -closed_loop._step_limit_axial);

        assert_eq!(
            actuator_steps_saturated[NUM_ACTUATOR - 1],
            closed_loop._step_limit_tangent
        );
        assert_eq!(
            actuator_steps_saturated[NUM_ACTUATOR - 2],
            -closed_loop._step_limit_tangent
        );
    }
}
