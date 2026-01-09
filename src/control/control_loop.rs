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

use log::info;
use nalgebra::SMatrix;
use std::path::Path;

use crate::config::Config;
use crate::constants::{
    NUM_ACTIVE_ACTUATOR, NUM_ACTUATOR, NUM_AXIAL_ACTUATOR, NUM_HARDPOINTS, NUM_HARDPOINTS_AXIAL,
};
use crate::control::closed_loop::ClosedLoop;
use crate::control::math_tool::{
    calc_cmd_delay_filter_params, calc_hp_comp_matrix, calc_kinetic_decoupling_matrix,
    calculate_position_ims, clip, correct_inclinometer_angle, hardpoint_to_rigid_body,
    rigid_body_to_actuator_displacement,
};
use crate::control::open_loop::OpenLoop;
use crate::enums::{ActuatorDisplacementUnit, ClosedLoopControlMode, CommandActuator};
use crate::event_queue::EventQueue;
use crate::mock::mock_plant::MockPlant;
use crate::telemetry::{event::Event, telemetry_control_loop::TelemetryControlLoop};
use crate::utility::{
    get_parameter, get_parameter_array, get_parameter_matrix, read_file_stiffness,
};

pub struct ControlLoop {
    // Is the mirror or the surrogate.
    pub is_mirror: bool,
    // Control loops
    _closed_loop: ClosedLoop,
    _open_loop: OpenLoop,
    // Closed-loop control mode.
    _mode: ClosedLoopControlMode,
    // Configuration.
    pub config: Config,
    // Mirror is in position or not.
    pub is_in_position: bool,
    // External elevation angle in degree.
    pub external_elevation_angle: f64,
    // Applied force in Newton.
    pub applied_force: Vec<f64>,
    // Current hardpoint displacements in meter.
    _current_hardpoint_displacement: Vec<f64>,
    // Steps to position the mirror.
    pub steps_position_mirror: Vec<i32>,
    // Events to publish
    pub event_queue: EventQueue,
    // Steps to move the actuators
    _steps_to_move_actuators: Option<Vec<i32>>,
    // Is the simulation mode enabled or not.
    _is_simulation_mode: bool,
}

impl ControlLoop {
    /// Create a new control loop.
    ///
    /// # Arguments
    /// * `config` - The configuration.
    /// * `is_mirror` - Is the mirror or the surrogate.
    /// * `is_simulation_mode` - Is the simulation mode or not.
    ///   the loop.
    ///
    /// # Returns
    /// A new control loop.
    pub fn new(config: &Config, is_mirror: bool, is_simulation_mode: bool) -> Self {
        Self {
            is_mirror,

            _closed_loop: Self::create_closed_loop(
                config.control_frequency,
                is_mirror,
                Path::new(config.filename.as_str()),
                &config.hardpoints,
                &config.cell_geometry.loc_act_axial,
            ),
            _open_loop: OpenLoop::new(),

            _mode: ClosedLoopControlMode::Idle,

            config: config.clone(),

            is_in_position: false,
            external_elevation_angle: 0.0,
            applied_force: vec![0.0; NUM_ACTUATOR],

            _current_hardpoint_displacement: vec![0.0; NUM_HARDPOINTS],

            steps_position_mirror: vec![0; NUM_ACTUATOR],

            event_queue: EventQueue::new(),

            _steps_to_move_actuators: None,

            _is_simulation_mode: is_simulation_mode,
        }
    }

    /// Update the control mode.
    ///
    /// # Arguments
    /// * `mode` - The control mode to be set.
    pub fn update_control_mode(&mut self, mode: ClosedLoopControlMode) {
        self._mode = mode;
        info!("Control mode is set to: {:?}.", mode);

        self.event_queue
            .add_event(Event::get_message_closed_loop_control_mode(mode));
        self.event_queue
            .add_event(Event::get_message_force_balance_system_status(
                mode == ClosedLoopControlMode::ClosedLoop,
            ));
    }

    /// Create a closed-loop.
    ///
    /// # Arguments
    /// * `control_frequency` - Control frequency in Hz.
    /// * `is_mirror` - Is the mirror or the surrogate.
    /// * `filepath_parameters_control` - The path to the control parameters
    ///   file.
    /// * `hardpoints` - Six 0-based hardpoints. The order is from low to high.
    /// * `loc_act_axial` - Location of the axial actuators: (x, y). This
    ///   should be a 72 x 2 matrix.
    ///
    /// # Returns
    /// A closed-loop.
    fn create_closed_loop(
        control_frequency: f64,
        is_mirror: bool,
        filepath_parameters_control: &Path,
        hardpoints: &[usize],
        loc_act_axial: &[Vec<f64>],
    ) -> ClosedLoop {
        // Calculate the command delay filter parameters.
        let params_cmd_delay = calc_cmd_delay_filter_params(is_mirror, control_frequency, false, 5);

        // Read the gain of control filter from the configuration file.
        let gain_control_filter: f64 = if is_mirror {
            get_parameter(
                filepath_parameters_control,
                "force_control_filter_params_gain_mirror",
            )
        } else {
            get_parameter(
                filepath_parameters_control,
                "force_control_filter_params_gain_surrogate",
            )
        };

        // Read the prefilter parameters from the configuration file.
        let gain_prefilter: f64 =
            get_parameter(filepath_parameters_control, "command_prefilter_params_gain");
        let params_prefilter: Vec<Vec<f64>> = get_parameter_matrix(
            filepath_parameters_control,
            "command_prefilter_params_coefficients",
        );

        // Read the control filter parameters from the configuration file.
        let params_control_filter: Vec<Vec<f64>> = get_parameter_matrix(
            filepath_parameters_control,
            "force_control_filter_params_coefficients",
        );

        // Calculate the hardpoint related matrices.
        let (kdc, hd_comp) =
            Self::calculate_matrices_hardpoints(is_mirror, hardpoints, loc_act_axial);

        ClosedLoop::new(
            gain_prefilter,
            &params_prefilter,
            gain_prefilter,
            &params_prefilter,
            &params_cmd_delay,
            &params_cmd_delay,
            gain_control_filter,
            &params_control_filter,
            gain_control_filter,
            &params_control_filter,
            &kdc,
            &kdc,
            &hd_comp,
            &get_parameter_array(filepath_parameters_control, "thresholds_deadzone_axial"),
            &get_parameter_array(filepath_parameters_control, "thresholds_deadzone_tangent"),
            get_parameter(filepath_parameters_control, "min_gain_schedular_axial"),
            get_parameter(filepath_parameters_control, "min_gain_schedular_tangent"),
            get_parameter(filepath_parameters_control, "num_sample_ramp_up"),
            get_parameter(filepath_parameters_control, "num_sample_ramp_down"),
            get_parameter(filepath_parameters_control, "max_sample_settle"),
            get_parameter(filepath_parameters_control, "step_limit_axial_passive"),
            get_parameter(filepath_parameters_control, "step_limit_tangent_passive"),
            get_parameter(filepath_parameters_control, "in_position_window_size"),
            control_frequency,
            get_parameter(filepath_parameters_control, "in_position_threshold_axial"),
            get_parameter(filepath_parameters_control, "in_position_threshold_tangent"),
            get_parameter(filepath_parameters_control, "is_feedforward"),
            get_parameter(filepath_parameters_control, "is_feedback"),
            get_parameter(filepath_parameters_control, "is_deadzone_enabled_axial"),
            get_parameter(filepath_parameters_control, "is_deadzone_enabled_tangent"),
        )
    }

    /// Calculate the matrices related to the hardpoints.
    ///
    /// # Arguments
    /// * `is_mirror` - Is the mirror or the surrogate.
    /// * `hardpoints` - Six 0-based hardpoints. The order is from low to high.
    /// * `loc_act_axial` - Location of the axial actuators: (x, y). This should
    ///   be a 72 x 2 matrix.
    ///
    /// # Returns
    /// The kinetic decoupling matrix and the hardpoint compensation matrix.
    fn calculate_matrices_hardpoints(
        is_mirror: bool,
        hardpoints: &[usize],
        loc_act_axial: &[Vec<f64>],
    ) -> (
        SMatrix<f64, NUM_ACTIVE_ACTUATOR, NUM_ACTIVE_ACTUATOR>,
        SMatrix<f64, NUM_ACTIVE_ACTUATOR, NUM_HARDPOINTS>,
    ) {
        // Read the stiffness matrix from the configuration file.
        let filepath_stiffness = if is_mirror {
            "config/stiff_matrix_m2.yaml"
        } else {
            "config/stiff_matrix_surrogate.yaml"
        };

        let stiffness = read_file_stiffness(Path::new(filepath_stiffness));
        let kdc = calc_kinetic_decoupling_matrix(
            loc_act_axial,
            &hardpoints[..NUM_HARDPOINTS_AXIAL],
            &hardpoints[NUM_HARDPOINTS_AXIAL..],
            &stiffness,
        );

        // Hardpoint compensation matrix
        let (hd_comp_axial, hd_comp_tangent) = calc_hp_comp_matrix(
            loc_act_axial,
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

        (kdc, hd_comp)
    }

    /// Update the configuration of the control loop.
    ///
    /// # Arguments
    /// * `config` - The new configuration.
    ///
    /// # Returns
    /// Ok if the configuration is updated successfully. Otherwise, an error
    /// message.
    pub fn update_config(&mut self, config: Config) -> Result<(), &'static str> {
        if self._mode == ClosedLoopControlMode::ClosedLoop {
            return Err("The control loop can not be in closed-loop mode.");
        }

        let hardpoints_are_changed = self.config.hardpoints != config.hardpoints;

        self.config = config;

        if hardpoints_are_changed {
            self.update_matrices_hardpoints();
        }

        Ok(())
    }

    /// Update the matrices related to the hardpoints.
    pub fn update_matrices_hardpoints(&mut self) {
        let (kdc, hd_comp) = Self::calculate_matrices_hardpoints(
            self.is_mirror,
            &self.config.hardpoints,
            &self.config.cell_geometry.loc_act_axial,
        );

        self._closed_loop.kinfl = kdc;
        self._closed_loop.kdc = kdc;

        self._closed_loop.hd_comp = hd_comp;
    }

    /// Step the control loop.
    ///
    /// # Arguments
    /// * `telemetry` - The processed telemetry data. This function will update
    ///   the telemetry data as well.
    ///
    /// # Panics
    /// If not in simulation mode.
    pub fn step(&mut self, telemetry: &mut TelemetryControlLoop) {
        // In idle mode, do nothing.
        if self._mode == ClosedLoopControlMode::Idle {
            return;
        }

        self.update_lut_forces(telemetry);

        // Calculate the hardpoint correction and the is_in_position flag.
        let demanded_force = self.get_demanded_force(telemetry);
        let config = &self.config;
        let (steps_closed_loop_control, hardpoint_correction, is_in_position) =
            self._closed_loop.calc_actuator_steps(
                &demanded_force,
                &telemetry.forces["measured"],
                &config.hardpoints,
                self.is_in_position,
            );

        self.is_in_position = is_in_position;

        // Update the telemetry data for the hardpoint correction.
        self.update_hardpoint_correction_to_telemetry(telemetry, &hardpoint_correction);

        // In telemetry only mode or there is still the steps to move, do
        // nothing after updating the telemetry.
        if (self._mode == ClosedLoopControlMode::TelemetryOnly)
            || self._steps_to_move_actuators.is_some()
        {
            return;
        }

        // Do the open-loop control.
        let mut steps: Vec<i32> = vec![0; NUM_ACTUATOR];
        if self._mode == ClosedLoopControlMode::OpenLoop {
            if let Ok(actuator_steps) = self
                ._open_loop
                .get_steps_to_move(config.step_limit["axial"], config.step_limit["tangent"])
            {
                steps = actuator_steps;
            }
        }

        // Do the closed-loop control.
        if self._mode == ClosedLoopControlMode::ClosedLoop {
            // Do the clipping for the active movement steps.
            for (idx, step) in steps.iter_mut().enumerate().take(NUM_ACTUATOR) {
                if idx < NUM_AXIAL_ACTUATOR {
                    *step = clip(
                        self.steps_position_mirror[idx],
                        -config.step_limit["axial"],
                        config.step_limit["axial"],
                    );
                } else {
                    *step = clip(
                        self.steps_position_mirror[idx],
                        -config.step_limit["tangent"],
                        config.step_limit["tangent"],
                    );
                }

                self.steps_position_mirror[idx] -= *step;
            }

            // Add with the closed-loop control steps in closed-loop control
            // mode.
            steps
                .iter_mut()
                .zip(steps_closed_loop_control.iter())
                .for_each(|(a, b)| {
                    *a += *b;
                });
        }

        // Cache the steps of actuator movement
        self._steps_to_move_actuators = Some(steps);
    }

    /// Take the steps to move the actuators.
    ///
    /// # Returns
    /// Option of the vector of steps to move the actuators.
    pub fn take_steps_to_move_actuators(&mut self) -> Option<Vec<i32>> {
        self._steps_to_move_actuators.take()
    }

    /// Process the telemetry data.
    ///
    /// # Arguments
    /// * `telemetry` - The telemetry data to be processed.
    pub fn process_telemetry_data(&mut self, telemetry: &mut TelemetryControlLoop) {
        // Set the is_in_position flag.
        telemetry.is_in_position = self.is_in_position;

        // Set the external elevation angle.
        telemetry
            .inclinometer
            .insert(String::from("external"), self.external_elevation_angle);

        // Set the applied force.
        telemetry
            .forces
            .insert(String::from("applied"), self.applied_force.clone());

        // Get the actuator step and position based on the encoder
        telemetry
            .ilc_encoders
            .iter()
            .enumerate()
            .for_each(|(idx, encoder)| {
                let (step, position) =
                    self._open_loop.actuators[idx].encoder_to_step_and_position(*encoder);
                telemetry.actuator_steps[idx] = step;
                telemetry.actuator_positions[idx] = position;
            });

        // Process the inclinometer angle
        telemetry.inclinometer.insert(
            String::from("processed"),
            correct_inclinometer_angle(
                telemetry.inclinometer["raw"],
                self.config.inclinometer_offset,
            ),
        );
        telemetry.inclinometer.insert(
            String::from("zenith"),
            90.0 - telemetry.inclinometer["processed"],
        );

        // Calculate the net total forces.
        let (fx, fy, fz) = self.calculate_xyz_net_forces(&telemetry.forces["measured"]);
        telemetry.net_total_forces.insert(String::from("fx"), fx);
        telemetry.net_total_forces.insert(String::from("fy"), fy);
        telemetry.net_total_forces.insert(String::from("fz"), fz);

        // Calculate the net total moments.
        let (mx, my, mz) = self.calculate_xyz_net_moments(&telemetry.forces["measured"]);
        telemetry.net_total_moments.insert(String::from("mx"), mx);
        telemetry.net_total_moments.insert(String::from("my"), my);
        telemetry.net_total_moments.insert(String::from("mz"), mz);

        // Calculate the tangent force error.
        telemetry.tangent_force_error = self.calculate_tangent_force_error(
            &(telemetry.forces["measured"][NUM_AXIAL_ACTUATOR..]),
            telemetry.inclinometer["zenith"],
        );

        // Calculate the rigid body position (hardpoints).
        self.set_current_hardpoint_displacement(&telemetry.actuator_positions);
        if let Ok((x, y, z, rx, ry, rz)) = hardpoint_to_rigid_body(
            &self.config.cell_geometry.loc_act_axial,
            &self.config.cell_geometry.loc_act_tangent,
            self.config.cell_geometry.radius_act_tangent,
            &self.config.hardpoints,
            &self._current_hardpoint_displacement,
            &self.config.disp_hardpoint_home,
        ) {
            // Need to update the unit from "m to um" and "rad to arcsec".
            let m_to_um = 1e6;

            // 1 rad × (180/pi) x 3600 arcsec ~ 206265
            let radian_to_arcsec = 206265.0;

            telemetry
                .mirror_position
                .insert(String::from("x"), x * m_to_um);
            telemetry
                .mirror_position
                .insert(String::from("y"), y * m_to_um);
            telemetry
                .mirror_position
                .insert(String::from("z"), z * m_to_um);
            telemetry
                .mirror_position
                .insert(String::from("xRot"), rx * radian_to_arcsec);
            telemetry
                .mirror_position
                .insert(String::from("yRot"), ry * radian_to_arcsec);
            telemetry
                .mirror_position
                .insert(String::from("zRot"), rz * radian_to_arcsec);
        }

        // Simulate IMS readings
        if self._is_simulation_mode {
            let (theta_z, delta_z) = MockPlant::calculate_ims_readings(
                &self.config.disp_matrix_inv,
                &self.config.disp_offset,
                telemetry.mirror_position["x"],
                telemetry.mirror_position["y"],
                telemetry.mirror_position["z"],
                telemetry.mirror_position["xRot"],
                telemetry.mirror_position["yRot"],
                telemetry.mirror_position["zRot"],
            );

            telemetry
                .displacement_sensors
                .insert(String::from("thetaZ"), theta_z);
            telemetry
                .displacement_sensors
                .insert(String::from("deltaZ"), delta_z);
        }

        // Calculate the rigid body position (IMS).
        let (x, y, z, rx, ry, rz) = calculate_position_ims(
            &self.config.disp_matrix,
            &self.config.disp_offset,
            &telemetry.displacement_sensors["thetaZ"],
            &telemetry.displacement_sensors["deltaZ"],
        );

        telemetry.mirror_position_ims.insert(String::from("x"), x);
        telemetry.mirror_position_ims.insert(String::from("y"), y);
        telemetry.mirror_position_ims.insert(String::from("z"), z);
        telemetry
            .mirror_position_ims
            .insert(String::from("xRot"), rx);
        telemetry
            .mirror_position_ims
            .insert(String::from("yRot"), ry);
        telemetry
            .mirror_position_ims
            .insert(String::from("zRot"), rz);
    }

    /// Update the hardpoint correction to the telemetry data.
    ///
    /// # Arguments
    /// * `telemetry` - The telemetry data to be updated.
    /// * `hardpoint_correction` - The hardpoint correction in Newton.
    fn update_hardpoint_correction_to_telemetry(
        &self,
        telemetry: &mut TelemetryControlLoop,
        hardpoint_correction: &[f64],
    ) {
        // Calculate the force balance.
        let (fx, fy, fz) = self.calculate_xyz_net_forces(hardpoint_correction);
        let (mx, my, mz) = self.calculate_xyz_net_moments(hardpoint_correction);

        // Update the telemetry data.
        telemetry.forces.insert(
            String::from("hardpointCorrection"),
            hardpoint_correction.to_vec(),
        );
        telemetry.force_balance.insert(String::from("fx"), fx);
        telemetry.force_balance.insert(String::from("fy"), fy);
        telemetry.force_balance.insert(String::from("fz"), fz);
        telemetry.force_balance.insert(String::from("mx"), mx);
        telemetry.force_balance.insert(String::from("my"), my);
        telemetry.force_balance.insert(String::from("mz"), mz);
    }

    /// Calculate the net forces in the x, y, and z directions.
    ///
    /// # Arguments
    /// * `forces` - 78 actuator forces in Newton.
    ///
    /// # Returns
    /// The net forces in the x, y, and z directions in Newton.
    fn calculate_xyz_net_forces(&self, forces: &[f64]) -> (f64, f64, f64) {
        let angles: Vec<f64> = self
            .config
            .cell_geometry
            .loc_act_tangent
            .iter()
            .map(|x| x.to_radians())
            .collect();

        let fx = angles
            .iter()
            .zip(forces[NUM_AXIAL_ACTUATOR..].iter())
            .map(|(angle, force)| angle.cos() * force)
            .sum();
        let fy = angles
            .iter()
            .zip(forces[NUM_AXIAL_ACTUATOR..].iter())
            .map(|(angle, force)| angle.sin() * force)
            .sum();
        let fz = forces[..NUM_AXIAL_ACTUATOR].iter().sum();

        (fx, fy, fz)
    }

    /// Calculate the net moments (Newton * meter) in the x, y, and z
    /// directions.
    ///
    /// # Arguments
    /// * `forces` - 78 actuator forces in Newton.
    ///
    /// # Returns
    /// The net moments in the x, y, and z directions in Newton * meter.
    fn calculate_xyz_net_moments(&self, forces: &[f64]) -> (f64, f64, f64) {
        let cell_geometry = &self.config.cell_geometry;

        let mut mx = 0.0;
        let mut my = 0.0;
        for (idx, force) in forces.iter().enumerate().take(NUM_AXIAL_ACTUATOR) {
            mx += *force * cell_geometry.loc_act_axial[idx][1];
            my += *force * cell_geometry.loc_act_axial[idx][0];
        }

        let mz =
            forces[NUM_AXIAL_ACTUATOR..].iter().sum::<f64>() * cell_geometry.radius_act_tangent;

        (mx, my, mz)
    }

    /// Calculate the tangent force error.
    ///
    /// # Arguments
    /// * `tangent_forces` - Currant tangent force (A1-A6) in Newton.
    /// * `zenith_angle` - The zenith angle in degree.
    ///
    /// # Returns
    /// The tangent force error.
    fn calculate_tangent_force_error(&self, tangent_forces: &[f64], zenith_angle: f64) -> Vec<f64> {
        // Calculate the individual tangent force error

        // When the mirror is on tilt orientation, tangential forces (A2, A3,
        // A5, A6) have to compensate gravity force of mirror.
        let indexes = [1, 2, 4, 5];

        // The orientation of tangent links is hexagon. Therefore, the
        // projection angle is 30 degree.

        let projection_angle: f64 = 30.0;
        let tangent_force_support: Vec<f64> = indexes
            .iter()
            .map(|idx| tangent_forces[*idx] * projection_angle.to_radians().cos())
            .collect();

        let gravitational_acceleration = 9.8;
        let mirror_weight_projection = self.config.mirror_weight_kg
            * gravitational_acceleration
            * zenith_angle.to_radians().sin();
        let divided_mirror_division = mirror_weight_projection / (indexes.len() as f64);

        let directions = [1.0, 1.0, -1.0, -1.0];
        let weight_error: Vec<f64> = tangent_force_support
            .iter()
            .zip(directions.iter())
            .map(|(force, direction)| force + direction * divided_mirror_division)
            .collect();

        let mut tangent_force_error = weight_error.clone();
        tangent_force_error.insert(0, tangent_forces[0]);
        tangent_force_error.insert(3, tangent_forces[3]);

        // Calculate the total weight error
        let total_weight_error = -tangent_force_support
            .iter()
            .zip(directions.iter())
            .map(|(force, direction)| direction * force)
            .sum::<f64>()
            - mirror_weight_projection;
        tangent_force_error.push(total_weight_error);

        // Sum of the tangent forces
        tangent_force_error.push(tangent_forces.iter().sum());

        tangent_force_error
    }

    /// Set the current hardpoint displacements in meter.
    ///
    /// # Arguments
    /// * `actuator_positions` - The actuator positions in millimeter.
    ///
    /// # Returns
    /// Current hardpoint displacements in meter.
    fn set_current_hardpoint_displacement(&mut self, actuator_positions: &[f64]) {
        // Change the unit from millimeter to meter.
        self._current_hardpoint_displacement = self
            .config
            .hardpoints
            .iter()
            .map(|idx| actuator_positions[*idx] * 1e-3)
            .collect();
    }

    /// Update the look-up table (LUT) forces.
    ///
    /// # Arguments
    /// * `telemetry` - The telemetry data to be updated.
    fn update_lut_forces(&mut self, telemetry: &mut TelemetryControlLoop) {
        let config = &self.config;

        // Calculate the LUT forces.
        let lut_angle = if config.use_external_elevation_angle {
            telemetry.inclinometer["external"]
        } else {
            telemetry.inclinometer["processed"]
        };

        let (lut_gravity, lut_temperature) = config.lut.get_lut_forces(
            lut_angle,
            &telemetry.temperature["ring"],
            &config.ref_temperature,
            config.enable_lut_temperature,
        );

        // Update the telemetry data.
        telemetry
            .forces
            .insert(String::from("lutGravity"), lut_gravity);
        telemetry
            .forces
            .insert(String::from("lutTemperature"), lut_temperature);
    }

    /// Get the demanded force.
    ///
    /// # Arguments
    /// * `telemetry` - The telemetry data.
    ///
    /// # Returns
    /// The demanded force in Newton.
    pub fn get_demanded_force(&self, telemetry: &TelemetryControlLoop) -> Vec<f64> {
        let forces = &telemetry.forces;
        (0..NUM_ACTUATOR)
            .map(|idx| {
                forces["applied"][idx] + forces["lutGravity"][idx] + forces["lutTemperature"][idx]
            })
            .collect()
    }

    /// Reset the active movement steps.
    pub fn reset_steps(&mut self) {
        self._open_loop.stop();
        self.steps_position_mirror = vec![0; NUM_ACTUATOR];
    }

    /// Handle positioning the mirror.
    ///
    /// # Arguments
    /// * `x` - The x position in micrometer.
    /// * `y` - The y position in micrometer.
    /// * `z` - The z position in micrometer.
    /// * `rx` - The x rotation in arcsec.
    /// * `ry` - The y rotation in arcsec.
    /// * `rz` - The z rotation in arcsec.
    ///
    /// # Panics
    /// If not in simulation mode.
    ///
    /// # Returns
    /// Ok if the positions are applied successfully. Otherwise, an error
    /// message.
    pub fn handle_position_mirror(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        rx: f64,
        ry: f64,
        rz: f64,
    ) -> Result<(), &'static str> {
        if self._mode != ClosedLoopControlMode::ClosedLoop {
            return Err("The control loop needs to be in closed-loop mode.");
        }

        let config = &self.config;

        // Get the hardpoint displacements
        let um_to_m = 1e-6;
        let arcsec_to_rad = 4.84814e-6;

        let displacments_xyz = rigid_body_to_actuator_displacement(
            &config.cell_geometry.loc_act_axial,
            &config.cell_geometry.loc_act_tangent,
            config.cell_geometry.radius_act_tangent,
            x * um_to_m,
            y * um_to_m,
            z * um_to_m,
            rx * arcsec_to_rad,
            ry * arcsec_to_rad,
            rz * arcsec_to_rad,
        );

        let hardpoints_displacement: Vec<f64> = config
            .hardpoints
            .iter()
            .enumerate()
            .map(|(idx, hp)| {
                displacments_xyz[*hp] + config.disp_hardpoint_home[idx]
                    - self._current_hardpoint_displacement[idx]
            })
            .collect();

        // Change the unit to be step
        let m_to_mm = 1e3;
        let hardpoints_steps: Vec<i32> = config
            .hardpoints
            .iter()
            .zip(hardpoints_displacement.iter())
            .map(|(idx, x)| self._open_loop.actuators[*idx].displacement_to_step(x * m_to_mm))
            .collect();

        // Update the active steps
        config.hardpoints.iter().enumerate().for_each(|(idx, hp)| {
            self.steps_position_mirror[*hp] = hardpoints_steps[idx];
        });

        Ok(())
    }

    /// Apply the actuator forces.
    ///
    /// # Arguments
    /// * `force` - 78 actuator force in Newton.
    ///
    /// # Returns
    /// Ok if the force is applied successfully. Otherwise, an error message.
    pub fn apply_force(&mut self, force: &[f64]) -> Result<(), &'static str> {
        if force.len() != NUM_ACTUATOR {
            return Err("The length of the force vector should be 78.");
        }

        if self._mode != ClosedLoopControlMode::ClosedLoop {
            return Err("The control loop needs to be in closed-loop mode.");
        }

        self.applied_force = force.to_vec();

        Ok(())
    }

    /// Reset the applied force.
    pub fn reset_force(&mut self) {
        self.applied_force = vec![0.0; NUM_ACTUATOR];
    }

    /// Move the actuators under the open-loop control.
    ///
    /// # Arguments
    /// * `command` - Start, stop, pause, or resume the actuator movement.
    /// * `actuators` - The actuator indices to move. Put an empty vector if the
    ///   command is not start.
    /// * `displacement` - The displacement in the unit. Put 0.0 if the command
    ///   is not start.
    /// * `unit` - The unit of the displacement. Put
    ///   `ActuatorDisplacementUnit::None` if the command is not start.
    ///
    /// # Returns
    /// Ok if the actuator movement is successful. Otherwise, an error message.
    pub fn move_actuators(
        &mut self,
        command: CommandActuator,
        actuators: &[usize],
        displacement: f64,
        unit: ActuatorDisplacementUnit,
    ) -> Result<(), &'static str> {
        if self._mode != ClosedLoopControlMode::OpenLoop {
            return Err("The control loop needs to be in open-loop mode.");
        }

        match command {
            CommandActuator::Start => self._open_loop.start(actuators, displacement, unit),
            CommandActuator::Stop => {
                self._open_loop.stop();
                Ok(())
            }
            CommandActuator::Pause => {
                self._open_loop.pause();
                Ok(())
            }
            CommandActuator::Resume => self._open_loop.resume(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use serde_json::json;

    use crate::constants::NUM_TEMPERATURE_RING;
    use crate::daq::data_acquisition::DataAcquisition;
    use crate::utility::assert_relative_eq_vector;

    const EPSILON: f64 = 1e-7;

    fn create_control_loop(is_simulation_mode: bool) -> ControlLoop {
        let config = Config::new(
            Path::new("config/parameters_control.yaml"),
            Path::new("config/lut/handling"),
        );

        ControlLoop::new(&config, false, is_simulation_mode)
    }

    fn create_data_acquisition(is_simulation_mode: bool) -> DataAcquisition {
        let mut data_acquisition = DataAcquisition::new(is_simulation_mode);

        if is_simulation_mode {
            data_acquisition.init_default_digital_output();
        }

        if let Some(plant) = &mut data_acquisition.plant {
            plant.power_system_communication.is_power_on = true;
        }

        data_acquisition
    }

    fn stabilize_control_loop(
        control_loop: &mut ControlLoop,
        data_acquisition: &mut DataAcquisition,
    ) -> TelemetryControlLoop {
        steps(
            control_loop,
            data_acquisition,
            10,
            ClosedLoopControlMode::TelemetryOnly,
        );

        loop {
            let telemetry = steps(
                control_loop,
                data_acquisition,
                1,
                ClosedLoopControlMode::ClosedLoop,
            );
            if telemetry.is_in_position {
                return telemetry;
            }
        }
    }

    fn steps(
        control_loop: &mut ControlLoop,
        data_acquisition: &mut DataAcquisition,
        cycle_times: i32,
        mode: ClosedLoopControlMode,
    ) -> TelemetryControlLoop {
        control_loop._mode = mode;

        let mut telemetry = TelemetryControlLoop::new();
        for _ in 0..cycle_times {
            telemetry = data_acquisition.get_telemetry_ilc();
            control_loop.process_telemetry_data(&mut telemetry);

            control_loop.step(&mut telemetry);
            if let Some(steps) = control_loop.take_steps_to_move_actuators() {
                data_acquisition
                    .plant
                    .as_mut()
                    .unwrap()
                    .move_actuator_steps(&steps);
            }
        }

        telemetry
    }

    #[test]
    fn test_new() {
        let control_loop = create_control_loop(true);

        let config = &control_loop.config;

        assert_eq!(config.control_frequency, 20.0);
        assert_eq!(config.hardpoints.len(), 6);
        assert_eq!(config.cell_geometry.loc_act_axial.len(), NUM_AXIAL_ACTUATOR);
        assert_eq!(config.step_limit["axial"], 40);
        assert_eq!(config.step_limit["tangent"], 40);

        assert_eq!(control_loop.external_elevation_angle, 0.0);
        assert_eq!(control_loop.applied_force, vec![0.0; NUM_ACTUATOR]);
        assert_eq!(
            control_loop._current_hardpoint_displacement,
            vec![0.0; NUM_HARDPOINTS]
        );

        assert!(control_loop._is_simulation_mode);
    }

    #[test]
    fn test_update_control_mode() {
        let mut control_loop = create_control_loop(true);

        control_loop.update_control_mode(ClosedLoopControlMode::ClosedLoop);

        assert_eq!(control_loop._mode, ClosedLoopControlMode::ClosedLoop);
        assert_eq!(
            control_loop.event_queue.get_events_and_clear(),
            vec![
                json!({
                    "id": "closedLoopControlMode",
                    "mode": 4,
                }),
                json!({
                    "id": "forceBalanceSystemStatus",
                    "status": true,
                })
            ]
        );
    }

    #[test]
    fn test_update_config() {
        let mut control_loop = create_control_loop(true);

        // Should succeed to update the config.
        let mut config = Config::new(
            Path::new("config/parameters_control.yaml"),
            Path::new("config/lut/optical"),
        );
        config.hardpoints = vec![4, 14, 24, 72, 74, 76];

        assert!(control_loop.update_config(config.clone()).is_ok());

        assert_eq!(control_loop.config.hardpoints, config.hardpoints);
        assert_eq!(control_loop.config.lut.dir_name, config.lut.dir_name);

        let lut_angle = 12.0;
        let ring_temperature = vec![1.0; NUM_TEMPERATURE_RING];
        let ref_temperature = vec![21.0; NUM_TEMPERATURE_RING];
        let enable_lut_temperature = true;

        assert_relative_eq_vector(
            &control_loop
                .config
                .lut
                .get_lut_forces(
                    lut_angle,
                    &ring_temperature,
                    &ref_temperature,
                    enable_lut_temperature,
                )
                .0,
            &config
                .lut
                .get_lut_forces(
                    lut_angle,
                    &ring_temperature,
                    &ref_temperature,
                    enable_lut_temperature,
                )
                .0,
            EPSILON,
        );

        // Should fail to update the config when the control loop is in
        // closed-loop mode.
        control_loop._mode = ClosedLoopControlMode::ClosedLoop;
        assert_eq!(
            control_loop.update_config(config),
            Err("The control loop can not be in closed-loop mode.")
        );
    }

    #[test]
    fn test_update_matrices_hardpoints() {
        let mut control_loop = create_control_loop(true);
        control_loop.config.hardpoints = vec![4, 14, 24, 72, 74, 76];

        let (kdc, hd_comp) = ControlLoop::calculate_matrices_hardpoints(
            control_loop.is_mirror,
            &control_loop.config.hardpoints,
            &control_loop.config.cell_geometry.loc_act_axial,
        );

        control_loop.update_matrices_hardpoints();

        assert_eq!(control_loop._closed_loop.kinfl, kdc);
        assert_eq!(control_loop._closed_loop.kdc, kdc);

        assert_eq!(control_loop._closed_loop.hd_comp, hd_comp);
    }

    #[test]
    fn test_step_closed_loop() {
        let mut control_loop = create_control_loop(true);
        control_loop.config.enable_lut_temperature = true;

        let mut data_acquisition = create_data_acquisition(true);

        let mut telemetry = steps(
            &mut control_loop,
            &mut data_acquisition,
            10,
            ClosedLoopControlMode::TelemetryOnly,
        );

        assert_relative_eq!(
            telemetry.forces["measured"][0],
            216.2038167,
            epsilon = EPSILON
        );

        // In the initial beginning, the actuators are not in position.
        steps(
            &mut control_loop,
            &mut data_acquisition,
            1,
            ClosedLoopControlMode::ClosedLoop,
        );
        telemetry = steps(
            &mut control_loop,
            &mut data_acquisition,
            1,
            ClosedLoopControlMode::TelemetryOnly,
        );

        assert_relative_eq!(
            telemetry.forces["measured"][0],
            216.4099693,
            epsilon = EPSILON
        );

        // After some running of closed-loop control, the actuators are in
        // position.
        telemetry = steps(
            &mut control_loop,
            &mut data_acquisition,
            30,
            ClosedLoopControlMode::ClosedLoop,
        );

        assert_relative_eq!(
            telemetry.forces["measured"][0],
            190.5481269,
            epsilon = EPSILON
        );
    }

    #[test]
    fn test_process_telemetry_data() {
        let mut control_loop = create_control_loop(true);
        control_loop.is_in_position = true;
        control_loop.external_elevation_angle = 10.0;
        control_loop.applied_force = vec![5.0; NUM_ACTUATOR];

        let mut data_acquisition = create_data_acquisition(true);

        // Let the hardpoints have the same positions as the home position.
        let mut telemetry = data_acquisition.get_telemetry_ilc();
        control_loop
            .config
            .hardpoints
            .iter()
            .enumerate()
            .for_each(|(idx, hardpoint)| {
                telemetry.actuator_positions[*hardpoint] =
                    control_loop.config.disp_hardpoint_home[idx] * 1e3;
            });

        control_loop.process_telemetry_data(&mut telemetry);

        assert_eq!(telemetry.inclinometer["processed"], 89.06);
        assert_relative_eq!(telemetry.inclinometer["zenith"], 0.94, epsilon = EPSILON);

        for key in telemetry.mirror_position.keys() {
            assert_relative_eq!(
                telemetry.mirror_position[key],
                telemetry.mirror_position_ims[key],
                epsilon = EPSILON
            );
        }

        assert!(telemetry.is_in_position);
        assert_eq!(telemetry.inclinometer["external"], 10.0);
        assert_eq!(telemetry.forces["applied"], vec![5.0; NUM_ACTUATOR]);
    }

    #[test]
    fn test_update_hardpoint_correction_to_telemetry() {
        let mut hardpoint_correction = vec![0.0; NUM_ACTUATOR];
        hardpoint_correction[1..4]
            .iter_mut()
            .zip((1..4).into_iter())
            .for_each(|(x, y)| {
                *x = y as f64;
            });
        hardpoint_correction[NUM_AXIAL_ACTUATOR..NUM_ACTUATOR]
            .iter_mut()
            .zip((1..7).into_iter())
            .for_each(|(x, y)| {
                *x = y as f64;
            });

        let control_loop = create_control_loop(true);

        let mut telemetry = TelemetryControlLoop::new();
        control_loop
            .update_hardpoint_correction_to_telemetry(&mut telemetry, &hardpoint_correction);

        assert_eq!(
            telemetry.forces["hardpointCorrection"],
            hardpoint_correction
        );

        ["fx", "fy", "fz", "mx", "my", "mz"]
            .iter()
            .for_each(|axis| {
                assert!(telemetry.force_balance[*axis] != 0.0);
            });
    }

    #[test]
    fn test_calculate_xyz_net_forces() {
        let mut forces = vec![0.0; NUM_ACTUATOR];
        forces[1..3]
            .iter_mut()
            .zip([1.0, 2.0].iter())
            .for_each(|(x, y)| {
                *x = *y;
            });
        forces[NUM_AXIAL_ACTUATOR..NUM_ACTUATOR]
            .iter_mut()
            .zip((1..7).into_iter())
            .for_each(|(x, y)| {
                *x = y as f64;
            });

        let control_loop = create_control_loop(true);
        let (fx, fy, fz) = control_loop.calculate_xyz_net_forces(&forces);

        assert_relative_eq!(fx, -3.0, epsilon = EPSILON);
        assert_relative_eq!(fy, -5.1961524, epsilon = EPSILON);
        assert_relative_eq!(fz, 3.0, epsilon = EPSILON);
    }

    #[test]
    fn test_calculate_xyz_net_moments() {
        let mut forces = vec![0.0; NUM_ACTUATOR];
        forces[1..4]
            .iter_mut()
            .zip((1..4).into_iter())
            .for_each(|(x, y)| {
                *x = y as f64;
            });
        forces[NUM_AXIAL_ACTUATOR..NUM_ACTUATOR]
            .iter_mut()
            .zip((1..7).into_iter())
            .for_each(|(x, y)| {
                *x = y as f64;
            });

        let control_loop = create_control_loop(true);
        let (mx, my, mz) = control_loop.calculate_xyz_net_moments(&forces);

        assert_relative_eq!(mx, 8.37691, epsilon = EPSILON);
        assert_relative_eq!(my, 4.45836999, epsilon = EPSILON);
        assert_relative_eq!(mz, 37.3839844, epsilon = EPSILON);
    }

    #[test]
    fn test_calculate_tangent_force_error() {
        let zenit_angle = 90.0 - correct_inclinometer_angle(89.853, 0.94);

        let control_loop = create_control_loop(true);
        let tangent_force_error = control_loop.calculate_tangent_force_error(
            &vec![-325.307, -447.377, 1128.37, -1249.98, 458.63, 267.627],
            zenit_angle,
        );

        assert_relative_eq_vector(
            &tangent_force_error,
            &vec![
                -325.307,
                -333.5718285,
                1031.0651034,
                -1249.98,
                343.3172124,
                177.9037622,
                -176.2723002,
                -168.037,
            ],
            EPSILON,
        );
    }

    #[test]
    fn test_set_current_hardpoint_displacement() {
        let mut control_loop = create_control_loop(true);
        let actuator_positions: Vec<f64> = (0..NUM_ACTUATOR).map(|x| x as f64).collect();

        control_loop.set_current_hardpoint_displacement(&actuator_positions);
        assert_eq!(
            control_loop._current_hardpoint_displacement,
            vec![0.005, 0.015, 0.025, 0.073, 0.075, 0.077]
        );
    }

    #[test]
    fn test_reset_steps() {
        let mut control_loop = create_control_loop(true);
        control_loop.steps_position_mirror = vec![1; NUM_ACTUATOR];

        control_loop.reset_steps();

        assert_eq!(control_loop.steps_position_mirror, vec![0; NUM_ACTUATOR]);
    }

    #[test]
    fn test_handle_position_mirror() {
        let mut control_loop = create_control_loop(true);

        // Should fail if not in closed-loop mode
        assert_eq!(
            control_loop.handle_position_mirror(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
            Err("The control loop needs to be in closed-loop mode.")
        );

        // Should succeed if in closed-loop mode
        control_loop._mode = ClosedLoopControlMode::ClosedLoop;

        assert!(control_loop
            .handle_position_mirror(1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
            .is_ok());

        let hardpoint_steps: Vec<i32> = control_loop
            .config
            .hardpoints
            .iter()
            .map(|hp| control_loop.steps_position_mirror[*hp])
            .collect();

        assert_eq!(hardpoint_steps, vec![92, 338, -580, -1477, -1564, -1186]);
    }

    #[test]
    fn test_handle_position_mirror_closed_loop() {
        // Stabilize the mirror first
        let mut control_loop = create_control_loop(true);
        let mut data_acquisition = create_data_acquisition(true);
        stabilize_control_loop(&mut control_loop, &mut data_acquisition);

        // Position the mirror
        let _ = control_loop.handle_position_mirror(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let telemetry = steps(
            &mut control_loop,
            &mut data_acquisition,
            220,
            ClosedLoopControlMode::ClosedLoop,
        );

        assert_eq!(control_loop.steps_position_mirror, vec![0; NUM_ACTUATOR]);

        let axes = ["x", "y", "z", "xRot", "yRot", "zRot"];
        axes.iter().enumerate().for_each(|(idx, axis)| {
            let position = idx as f64 + 1.0;
            assert_relative_eq!(telemetry.mirror_position[*axis], position, epsilon = 1e-1);
            assert_relative_eq!(
                telemetry.mirror_position_ims[*axis],
                position,
                epsilon = 1e-1
            );
        });
    }

    #[test]
    fn test_apply_force_fail() {
        let mut control_loop = create_control_loop(true);

        let force = vec![0.0; NUM_ACTUATOR - 1];
        assert_eq!(
            control_loop.apply_force(&force),
            Err("The length of the force vector should be 78.")
        );
    }

    #[test]
    fn test_apply_force_success() {
        let mut control_loop = create_control_loop(true);
        control_loop._mode = ClosedLoopControlMode::ClosedLoop;

        let force = vec![10.0; NUM_ACTUATOR];
        assert_eq!(control_loop.apply_force(&force), Ok(()));

        assert_relative_eq_vector(&control_loop.applied_force, &force, EPSILON);
    }

    #[test]
    fn test_apply_force_closed_loop() {
        // Stabilize the mirror first
        let mut control_loop = create_control_loop(true);
        let mut data_acquisition = create_data_acquisition(true);
        let mut telemetry = stabilize_control_loop(&mut control_loop, &mut data_acquisition);

        let force_original = telemetry.forces["measured"][3];

        // Apply the force
        let mut force = vec![0.0; NUM_ACTUATOR];
        force[3] = 5.0;
        let _ = control_loop.apply_force(&force);

        telemetry = stabilize_control_loop(&mut control_loop, &mut data_acquisition);

        let force_updated = telemetry.forces["measured"][3];

        assert_relative_eq!(
            force_updated - force_original,
            4.55580783,
            epsilon = EPSILON
        );
    }

    #[test]
    fn test_apply_force_closed_loop_and_check_calculation_and_stability() {
        // Stabilize the mirror first
        let mut control_loop = create_control_loop(true);
        let mut data_acquisition = create_data_acquisition(true);
        let mut telemetry = stabilize_control_loop(&mut control_loop, &mut data_acquisition);

        assert_relative_eq_vector(
            &telemetry.forces["hardpointCorrection"][0..5],
            &vec![
                -6.38991131,
                -6.34897354,
                -6.27481481,
                -6.17066679,
                -6.04108298,
            ],
            EPSILON,
        );

        // Apply the force
        let mut force = vec![0.0; NUM_ACTUATOR];
        force[3] = 100.0;
        let _ = control_loop.apply_force(&force);

        telemetry = stabilize_control_loop(&mut control_loop, &mut data_acquisition);

        let expected_hardpoint_correction_force = vec![
            -10.87706161,
            -11.19361034,
            -11.33033979,
            -11.28121744,
            -11.04842581,
        ];
        assert_relative_eq_vector(
            &telemetry.forces["hardpointCorrection"][0..5],
            &expected_hardpoint_correction_force,
            EPSILON,
        );

        // Check the stability by running more cycles
        for _ in 0..100 {
            telemetry = steps(
                &mut control_loop,
                &mut data_acquisition,
                1,
                ClosedLoopControlMode::ClosedLoop,
            );

            assert!(telemetry.is_in_position);
            assert_relative_eq_vector(
                &telemetry.forces["hardpointCorrection"][0..5],
                &expected_hardpoint_correction_force,
                EPSILON,
            );
        }

        // Reset the force
        let _ = control_loop.reset_force();

        telemetry = stabilize_control_loop(&mut control_loop, &mut data_acquisition);

        let expected_hardpoint_correction_reset = vec![
            -6.37788502,
            -6.32490499,
            -6.24109329,
            -6.13010428,
            -5.99678962,
        ];
        assert_relative_eq_vector(
            &telemetry.forces["hardpointCorrection"][0..5],
            &expected_hardpoint_correction_reset,
            EPSILON,
        );

        // Check the stability by running more cycles
        for _ in 0..100 {
            telemetry = steps(
                &mut control_loop,
                &mut data_acquisition,
                1,
                ClosedLoopControlMode::ClosedLoop,
            );

            assert!(telemetry.is_in_position);
            assert_relative_eq_vector(
                &telemetry.forces["hardpointCorrection"][0..5],
                &expected_hardpoint_correction_reset,
                EPSILON,
            );
        }
    }

    #[test]
    fn test_reset_force() {
        let mut control_loop = create_control_loop(true);
        let _ = control_loop.apply_force(&vec![10.0; NUM_ACTUATOR]);

        control_loop.reset_force();

        assert_eq!(control_loop.applied_force, vec![0.0; NUM_ACTUATOR]);
    }

    #[test]
    fn test_move_actuators() {
        let mut control_loop = create_control_loop(true);
        let mut data_acquisition = create_data_acquisition(true);
        let mut telemetry = stabilize_control_loop(&mut control_loop, &mut data_acquisition);

        // Should fail if not in open-loop mode
        assert_eq!(
            control_loop.move_actuators(
                CommandActuator::Start,
                &vec![1],
                60.0,
                ActuatorDisplacementUnit::Step
            ),
            Err("The control loop needs to be in open-loop mode.")
        );

        // Should succeed if in open-loop mode
        control_loop._mode = ClosedLoopControlMode::OpenLoop;

        // Get the initial actuator step
        let step_init = telemetry.actuator_steps[1];

        // Start the movement
        assert_eq!(
            control_loop.move_actuators(
                CommandActuator::Start,
                &vec![1],
                60.0,
                ActuatorDisplacementUnit::Step
            ),
            Ok(())
        );

        // Use the telemetry only mode to get the updated data of plant
        steps(
            &mut control_loop,
            &mut data_acquisition,
            1,
            ClosedLoopControlMode::OpenLoop,
        );
        telemetry = steps(
            &mut control_loop,
            &mut data_acquisition,
            1,
            ClosedLoopControlMode::TelemetryOnly,
        );
        let step_move = telemetry.actuator_steps[1];

        assert_eq!(
            step_move - step_init,
            control_loop.config.step_limit["axial"]
        );

        // Pause the movement
        control_loop._mode = ClosedLoopControlMode::OpenLoop;
        assert_eq!(
            control_loop.move_actuators(
                CommandActuator::Pause,
                &Vec::new(),
                0.0,
                ActuatorDisplacementUnit::None
            ),
            Ok(())
        );

        steps(
            &mut control_loop,
            &mut data_acquisition,
            1,
            ClosedLoopControlMode::OpenLoop,
        );
        telemetry = steps(
            &mut control_loop,
            &mut data_acquisition,
            1,
            ClosedLoopControlMode::TelemetryOnly,
        );
        let step_pause = telemetry.actuator_steps[1];

        assert_eq!(step_pause, step_move);

        // Resume the movement
        control_loop._mode = ClosedLoopControlMode::OpenLoop;
        assert_eq!(
            control_loop.move_actuators(
                CommandActuator::Resume,
                &Vec::new(),
                0.0,
                ActuatorDisplacementUnit::None
            ),
            Ok(())
        );

        steps(
            &mut control_loop,
            &mut data_acquisition,
            1,
            ClosedLoopControlMode::OpenLoop,
        );
        telemetry = steps(
            &mut control_loop,
            &mut data_acquisition,
            1,
            ClosedLoopControlMode::TelemetryOnly,
        );
        let step_resume = telemetry.actuator_steps[1];

        assert_eq!(step_resume - step_pause, 20);
        assert!(!control_loop._open_loop.is_running);

        // Stop the movement
        control_loop._mode = ClosedLoopControlMode::OpenLoop;
        assert_eq!(
            control_loop.move_actuators(
                CommandActuator::Stop,
                &Vec::new(),
                0.0,
                ActuatorDisplacementUnit::None
            ),
            Ok(())
        );

        steps(
            &mut control_loop,
            &mut data_acquisition,
            1,
            ClosedLoopControlMode::OpenLoop,
        );
        telemetry = steps(
            &mut control_loop,
            &mut data_acquisition,
            1,
            ClosedLoopControlMode::TelemetryOnly,
        );
        let step_stop = telemetry.actuator_steps[1];

        assert_eq!(step_stop, step_resume);
    }
}
