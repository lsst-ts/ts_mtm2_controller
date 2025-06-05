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
use crate::enums::{
    ActuatorDisplacementUnit, ClosedLoopControlMode, CommandActuator, InnerLoopControlMode,
};
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
    // Telemetry.
    pub telemetry: TelemetryControlLoop,
    // Mirror is in position or not.
    pub is_in_position: bool,
    // Steps to position the mirror.
    pub steps_position_mirror: Vec<i32>,
    // Events to publish
    pub event_queue: EventQueue,
    // Plant model
    pub plant: Option<MockPlant>,
}

impl ControlLoop {
    /// Create a new control loop.
    ///
    /// # Arguments
    /// * `config` - The configuration.
    /// * `is_mirror` - Is the mirror or the surrogate.
    /// * `is_simulation_mode` - Is the simulation mode or not.
    /// the loop.
    ///
    /// # Returns
    /// A new control loop.
    pub fn new(config: &Config, is_mirror: bool, is_simulation_mode: bool) -> Self {
        // Plant model
        let telemetry = TelemetryControlLoop::new();
        let plant = if is_simulation_mode {
            // Use the M2 stiffness matrix here intentionally to match the
            // simulation mode of ts_mtm2_cell.
            let mut mock_plant = MockPlant::new(
                &read_file_stiffness(Path::new("config/stiff_matrix_m2.yaml")),
                // Zenith angle by default
                90.0,
            );
            mock_plant.is_power_on_communication = true;
            mock_plant.is_power_on_motor = true;

            Option::Some(mock_plant)
        } else {
            Option::None
        };

        Self {
            is_mirror: is_mirror,

            _closed_loop: Self::create_closed_loop(
                config.control_frequency,
                is_mirror,
                Path::new(config.filename.as_str()),
                &config.hardpoints,
                &config.cell_geometry.loc_act_axial,
            ),
            _open_loop: OpenLoop::new(is_simulation_mode),

            _mode: ClosedLoopControlMode::Idle,

            config: config.clone(),

            telemetry: telemetry,

            is_in_position: false,

            steps_position_mirror: vec![0; NUM_ACTUATOR],

            event_queue: EventQueue::new(),

            plant: plant,
        }
    }

    /// Update the control mode.
    ///
    /// # Arguments
    /// * `mode` - The control mode to be set.
    pub fn update_control_mode(&mut self, mode: ClosedLoopControlMode) {
        self._mode = mode;

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
    /// file.
    /// * `hardpoints` - Six 0-based hardpoints. The order is from low to high.
    /// * `loc_act_axial` - Location of the axial actuators: (x, y). This should
    /// be a 72 x 2 matrix.
    ///
    /// # Returns
    /// A closed-loop.
    fn create_closed_loop(
        control_frequency: f64,
        is_mirror: bool,
        filepath_parameters_control: &Path,
        hardpoints: &[usize],
        loc_act_axial: &Vec<Vec<f64>>,
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
    /// be a 72 x 2 matrix.
    ///
    /// # Returns
    /// The kinetic decoupling matrix and the hardpoint compensation matrix.
    fn calculate_matrices_hardpoints(
        is_mirror: bool,
        hardpoints: &[usize],
        loc_act_axial: &Vec<Vec<f64>>,
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

        (kdc, hd_comp)
    }

    /// Update the matrices related to the hardpoints.
    pub fn update_matrices_hardpoints(&mut self) {
        let (kdc, hd_comp) = Self::calculate_matrices_hardpoints(
            self.is_mirror,
            &self.config.hardpoints,
            &self.config.cell_geometry.loc_act_axial,
        );

        self._closed_loop.kinfl = kdc.clone();
        self._closed_loop.kdc = kdc;

        self._closed_loop.hd_comp = hd_comp;
    }

    /// Is the simulation mode or not.
    ///
    /// # Returns
    /// True if the simulation mode is enabled. Otherwise, false.
    pub fn is_simulation_mode(&self) -> bool {
        self.plant.is_some()
    }

    /// Step the control loop.
    ///
    /// # Panics
    /// If not in simulation mode.
    pub fn step(&mut self) {
        // In idle mode, do nothing.
        if self._mode == ClosedLoopControlMode::Idle {
            return;
        }

        self.update_telemetry_data();
        self.process_telemetry_data();

        self.update_lut_forces();

        // Calculate the hardpoint correction and the is_in_position flag.
        let demanded_force = self.get_demanded_force(&self.telemetry.forces["applied"]);
        let config = &self.config;
        let (steps_closed_loop_control, hardpoint_correction, is_in_position) =
            self._closed_loop.calc_actuator_steps(
                &demanded_force,
                &self.telemetry.forces["measured"],
                &config.hardpoints,
                self.is_in_position,
            );

        self.is_in_position = is_in_position;

        // Update the telemetry data.
        self.telemetry
            .forces
            .insert(String::from("hardpointCorrection"), hardpoint_correction);

        // In telemetry only mode, do nothing after getting the telemetry.
        if self._mode == ClosedLoopControlMode::TelemetryOnly {
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
            for idx in 0..NUM_ACTUATOR {
                if idx < NUM_AXIAL_ACTUATOR {
                    steps[idx] = clip(
                        self.steps_position_mirror[idx],
                        -config.step_limit["axial"],
                        config.step_limit["axial"],
                    );
                } else {
                    steps[idx] = clip(
                        self.steps_position_mirror[idx],
                        -config.step_limit["tangent"],
                        config.step_limit["tangent"],
                    );
                }

                self.steps_position_mirror[idx] -= steps[idx];
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

        // Do the actuator movement.
        if self.is_simulation_mode() {
            if let Some(plant) = &mut self.plant {
                plant.move_actuator_steps(&steps);
            }
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Update the telemetry data related to the plant/hardware.
    ///
    /// # Panics
    /// If not in simulation mode.
    fn update_telemetry_data(&mut self) {
        self.telemetry.is_in_position = self.is_in_position;
        // Get the telemetry from the plant.
        if self.is_simulation_mode() {
            // Get the telemetry from the plant.
            if let Some(plant) = &mut self.plant {
                if plant.is_power_on_communication {
                    // Inclinometer
                    self.telemetry
                        .inclinometer
                        .insert(String::from("raw"), plant.inclinometer_angle);

                    // Get the actuator ILC data
                    let (ilc_status, ilc_encoders, forces) = plant.get_actuator_ilc_data();

                    // Get the actuator step and position based on the encoder
                    ilc_encoders.iter().enumerate().for_each(|(idx, encoder)| {
                        let (step, position) =
                            self._open_loop.actuators[idx].encoder_to_step_and_position(*encoder);
                        self.telemetry.actuator_steps[idx] = step;
                        self.telemetry.actuator_positions[idx] = position;
                    });

                    // Forces
                    self.telemetry
                        .forces
                        .insert(String::from("measured"), forces);

                    // ILC
                    self.telemetry.ilc_status = ilc_status;
                    self.telemetry.ilc_encoders = ilc_encoders;

                    // Temperatures
                    self.telemetry
                        .temperature
                        .insert(String::from("ring"), plant.temperature_ring.clone());
                    self.telemetry
                        .temperature
                        .insert(String::from("intake"), plant.temperature_intake.clone());
                    self.telemetry
                        .temperature
                        .insert(String::from("exhaust"), plant.temperature_exhaust.clone());

                    // Simulate IMS readings
                    let (theta_z, delta_z) = plant.calculate_ims_readings(
                        self.telemetry.mirror_position["x"],
                        self.telemetry.mirror_position["y"],
                        self.telemetry.mirror_position["z"],
                        self.telemetry.mirror_position["xRot"],
                        self.telemetry.mirror_position["yRot"],
                        self.telemetry.mirror_position["zRot"],
                    );

                    self.telemetry
                        .displacement_sensors
                        .insert(String::from("thetaZ"), theta_z);
                    self.telemetry
                        .displacement_sensors
                        .insert(String::from("deltaZ"), delta_z);
                }
            }
        } else {
            // Get the telemetry from the hardware.
            panic!("Not implemented yet.");
        }
    }

    /// Process the telemetry data.
    fn process_telemetry_data(&mut self) {
        let config = &self.config;

        // Process the inclinometer angle
        self.telemetry.inclinometer.insert(
            String::from("processed"),
            correct_inclinometer_angle(
                self.telemetry.inclinometer["raw"],
                config.inclinometer_offset,
            ),
        );
        self.telemetry.inclinometer.insert(
            String::from("zenith"),
            90.0 - self.telemetry.inclinometer["processed"],
        );

        // Calculate the net total forces.
        let (fx, fy, fz) = self.calculate_xyz_net_forces(&self.telemetry.forces["measured"]);
        self.telemetry
            .net_total_forces
            .insert(String::from("fx"), fx);
        self.telemetry
            .net_total_forces
            .insert(String::from("fy"), fy);
        self.telemetry
            .net_total_forces
            .insert(String::from("fz"), fz);

        // Calculate the net total moments.
        let (mx, my, mz) = self.calculate_xyz_net_moments(&self.telemetry.forces["measured"]);
        self.telemetry
            .net_total_moments
            .insert(String::from("mx"), mx);
        self.telemetry
            .net_total_moments
            .insert(String::from("my"), my);
        self.telemetry
            .net_total_moments
            .insert(String::from("mz"), mz);

        // Calculate the force balance.
        let (fx, fy, fz) =
            self.calculate_xyz_net_forces(&self.telemetry.forces["hardpointCorrection"]);
        let (mx, my, mz) =
            self.calculate_xyz_net_moments(&self.telemetry.forces["hardpointCorrection"]);
        self.telemetry.force_balance.insert(String::from("fx"), fx);
        self.telemetry.force_balance.insert(String::from("fy"), fy);
        self.telemetry.force_balance.insert(String::from("fz"), fz);
        self.telemetry.force_balance.insert(String::from("mx"), mx);
        self.telemetry.force_balance.insert(String::from("my"), my);
        self.telemetry.force_balance.insert(String::from("mz"), mz);

        // Calculate the tangent force error.
        self.telemetry.tangent_force_error = self.calculate_tangent_force_error(
            &(self.telemetry.forces["measured"][NUM_AXIAL_ACTUATOR..]),
        );

        // Calculate the rigid body position (hardpoints).
        if let Ok((x, y, z, rx, ry, rz)) = hardpoint_to_rigid_body(
            &config.cell_geometry.loc_act_axial,
            &config.cell_geometry.loc_act_tangent,
            config.cell_geometry.radius_act_tangent,
            &config.hardpoints,
            &self.get_current_hardpoint_displacement(),
            &config.disp_hardpoint_home,
        ) {
            // Need to update the unit from "m to um" and "rad to arcsec".
            let m_to_um = 1e6;

            // 1 rad × (180/pi) x 3600 arcsec ~ 206265
            let radian_to_arcsec = 206265.0;

            self.telemetry
                .mirror_position
                .insert(String::from("x"), x * m_to_um);
            self.telemetry
                .mirror_position
                .insert(String::from("y"), y * m_to_um);
            self.telemetry
                .mirror_position
                .insert(String::from("z"), z * m_to_um);
            self.telemetry
                .mirror_position
                .insert(String::from("xRot"), rx * radian_to_arcsec);
            self.telemetry
                .mirror_position
                .insert(String::from("yRot"), ry * radian_to_arcsec);
            self.telemetry
                .mirror_position
                .insert(String::from("zRot"), rz * radian_to_arcsec);
        }

        // Calculate the rigid body position (IMS).
        let (x, y, z, rx, ry, rz) = calculate_position_ims(
            &config.disp_matrix,
            &config.disp_offset,
            &self.telemetry.displacement_sensors["thetaZ"],
            &self.telemetry.displacement_sensors["deltaZ"],
        );

        self.telemetry
            .mirror_position_ims
            .insert(String::from("x"), x);
        self.telemetry
            .mirror_position_ims
            .insert(String::from("y"), y);
        self.telemetry
            .mirror_position_ims
            .insert(String::from("z"), z);
        self.telemetry
            .mirror_position_ims
            .insert(String::from("xRot"), rx);
        self.telemetry
            .mirror_position_ims
            .insert(String::from("yRot"), ry);
        self.telemetry
            .mirror_position_ims
            .insert(String::from("zRot"), rz);
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
        for idx in 0..NUM_AXIAL_ACTUATOR {
            mx += forces[idx] * cell_geometry.loc_act_axial[idx][1];
            my += forces[idx] * cell_geometry.loc_act_axial[idx][0];
        }

        let mz =
            forces[NUM_AXIAL_ACTUATOR..].iter().sum::<f64>() * cell_geometry.radius_act_tangent;

        (mx, my, mz)
    }

    /// Calculate the tangent force error.
    ///
    /// # Arguments
    /// * `tangent_forces` - Currant tangent force (A1-A6) in Newton.
    ///
    /// # Returns
    /// The tangent force error.
    fn calculate_tangent_force_error(&self, tangent_forces: &[f64]) -> Vec<f64> {
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
            * self.telemetry.inclinometer["zenith"].to_radians().sin();
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

    /// Get the current hardpoint displacements in meter.
    ///
    /// # Returns
    /// Current hardpoint displacements in meter.
    fn get_current_hardpoint_displacement(&self) -> Vec<f64> {
        // Change the unit from millimeter to meter.
        self.config
            .hardpoints
            .iter()
            .map(|&idx| self.telemetry.actuator_positions[idx] * 1e-3)
            .collect()
    }

    /// Update the look-up table (LUT) forces.
    fn update_lut_forces(&mut self) {
        let config = &self.config;

        // Calculate the LUT forces.
        let lut_angle = if config.use_external_elevation_angle {
            self.telemetry.inclinometer["external"]
        } else {
            self.telemetry.inclinometer["processed"]
        };

        let (lut_gravity, lut_temperature) = config.lut.get_lut_forces(
            lut_angle,
            &self.telemetry.temperature["ring"],
            &config.ref_temperature,
            config.enable_lut_temperature,
        );

        // Update the telemetry data.
        self.telemetry
            .forces
            .insert(String::from("lutGravity"), lut_gravity);
        self.telemetry
            .forces
            .insert(String::from("lutTemperature"), lut_temperature);
    }

    /// Get the demanded force.
    ///
    /// # Arguments
    /// * `applied_force` - The applied force in Newton.
    ///
    /// # Returns
    /// The demanded force in Newton.
    pub fn get_demanded_force(&self, applied_force: &[f64]) -> Vec<f64> {
        let forces = &self.telemetry.forces;
        (0..NUM_ACTUATOR)
            .into_iter()
            .map(|idx| {
                applied_force[idx] + forces["lutGravity"][idx] + forces["lutTemperature"][idx]
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
    /// `x` - The x position in micrometer.
    /// `y` - The y position in micrometer.
    /// `z` - The z position in micrometer.
    /// `rx` - The x rotation in arcsec.
    /// `ry` - The y rotation in arcsec.
    /// `rz` - The z rotation in arcsec.
    ///
    /// # Panics
    /// If not in simulation mode.
    pub fn handle_position_mirror(&mut self, x: f64, y: f64, z: f64, rx: f64, ry: f64, rz: f64) {
        // TODO: Need to implement the calculation of meter-to-step for 78
        // actuators.
        if !self.is_simulation_mode() {
            panic!("Not implemented yet.");
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

        let current_hardpoint_displacement = self.get_current_hardpoint_displacement();
        let hardpoints_displacement: Vec<f64> = config
            .hardpoints
            .iter()
            .enumerate()
            .map(|(idx, hp)| {
                displacments_xyz[*hp] + config.disp_hardpoint_home[idx]
                    - current_hardpoint_displacement[idx]
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

        self.telemetry
            .forces
            .insert(String::from("applied"), force.to_vec());

        Ok(())
    }

    /// Reset the applied force.
    pub fn reset_force(&mut self) {
        self.telemetry
            .forces
            .insert(String::from("applied"), vec![0.0; NUM_ACTUATOR]);
    }

    /// Move the actuators under the open-loop control.
    ///
    /// # Arguments
    /// * `command` - Start, stop, pause, or resume the actuator movement.
    /// * `actuators` - The actuator indices to move. Put an empty vector if the
    /// command is not start.
    /// * `displacement` - The displacement in the unit. Put 0.0 if the command
    /// is not start.
    /// * `unit` - The unit of the displacement. Put
    /// ActuatorDisplacementUnit::None if the command is not start.
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
        match command {
            CommandActuator::Start => return self._open_loop.start(actuators, displacement, unit),
            CommandActuator::Stop => {
                self._open_loop.stop();
                return Ok(());
            }
            CommandActuator::Pause => {
                self._open_loop.pause();
                return Ok(());
            }
            CommandActuator::Resume => return self._open_loop.resume(),
        }
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
        if self.is_simulation_mode() {
            if let Some(plant) = &mut self.plant {
                let new_mode = plant.set_ilc_mode(address, mode);
                self.event_queue
                    .add_event(Event::get_message_inner_loop_control_mode(
                        address, new_mode,
                    ));

                return new_mode;
            }
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }

        InnerLoopControlMode::Unknown
    }

    /// Get the mode of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The address of ILC.
    ///
    /// # Returns
    /// ILC mode.
    pub fn get_ilc_mode(&mut self, address: usize) -> InnerLoopControlMode {
        if self.is_simulation_mode() {
            if let Some(plant) = &self.plant {
                let mode = plant.get_ilc_mode(address);
                self.event_queue
                    .add_event(Event::get_message_inner_loop_control_mode(address, mode));

                return mode;
            }
        } else {
            // Update the hardware.
            panic!("Not implemented yet.");
        }

        InnerLoopControlMode::Unknown
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use serde_json::json;

    use crate::mock::mock_constants::{PLANT_TEMPERATURE_HIGH, PLANT_TEMPERATURE_LOW};
    use crate::utility::assert_relative_eq_vector;

    const EPSILON: f64 = 1e-7;

    fn create_control_loop(is_simulation_mode: bool) -> ControlLoop {
        let config = Config::new(
            Path::new("config/parameters_control.yaml"),
            Path::new("config/lut/handling"),
        );

        ControlLoop::new(&config, false, is_simulation_mode)
    }

    fn stabilize_control_loop(control_loop: &mut ControlLoop) {
        control_loop
            .plant
            .as_mut()
            .unwrap()
            .is_power_on_communication = true;
        control_loop.plant.as_mut().unwrap().is_power_on_motor = true;

        steps(control_loop, 10, ClosedLoopControlMode::TelemetryOnly);
        steps(control_loop, 30, ClosedLoopControlMode::ClosedLoop);
    }

    fn steps(control_loop: &mut ControlLoop, cycle_times: i32, mode: ClosedLoopControlMode) {
        control_loop._mode = mode;
        for _ in 0..cycle_times {
            control_loop.step();
        }
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

        assert_eq!(control_loop.telemetry.inclinometer["raw"], 0.0);

        assert!(control_loop.is_simulation_mode());
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
    fn test_is_simulation_mode_false() {
        assert!(!create_control_loop(false).is_simulation_mode());
    }

    #[test]
    fn test_step_closed_loop() {
        let mut control_loop = create_control_loop(true);
        control_loop.config.enable_lut_temperature = true;
        control_loop
            .plant
            .as_mut()
            .unwrap()
            .is_power_on_communication = true;
        control_loop.plant.as_mut().unwrap().is_power_on_motor = true;

        steps(&mut control_loop, 10, ClosedLoopControlMode::TelemetryOnly);

        assert_relative_eq!(
            control_loop.telemetry.forces["measured"][0],
            216.2038167,
            epsilon = EPSILON
        );

        // In the initial beginning, the actuators are not in position.
        steps(&mut control_loop, 1, ClosedLoopControlMode::ClosedLoop);
        steps(&mut control_loop, 1, ClosedLoopControlMode::TelemetryOnly);

        assert_relative_eq!(
            control_loop.telemetry.forces["measured"][0],
            216.4099693,
            epsilon = EPSILON
        );

        // After some running of closed-loop control, the actuators are in
        // position.
        steps(&mut control_loop, 30, ClosedLoopControlMode::ClosedLoop);

        assert_relative_eq!(
            control_loop.telemetry.forces["measured"][0],
            190.5481269,
            epsilon = EPSILON
        );
    }

    #[test]
    fn test_update_telemetry_data() {
        let mut control_loop = create_control_loop(true);
        control_loop.is_in_position = true;
        control_loop
            .plant
            .as_mut()
            .unwrap()
            .is_power_on_communication = true;

        control_loop.update_telemetry_data();

        assert!(control_loop.telemetry.is_in_position);
        assert_eq!(control_loop.telemetry.inclinometer["raw"], 90.0);

        assert_relative_eq!(
            control_loop.telemetry.forces["measured"][0],
            216.2038166,
            epsilon = EPSILON
        );

        assert_eq!(
            control_loop.telemetry.temperature["ring"][0],
            PLANT_TEMPERATURE_LOW
        );
        assert_eq!(
            control_loop.telemetry.temperature["intake"][0],
            PLANT_TEMPERATURE_LOW
        );
        assert_eq!(
            control_loop.telemetry.temperature["exhaust"][0],
            PLANT_TEMPERATURE_HIGH
        );
    }

    #[test]
    fn test_process_telemetry_data() {
        let mut control_loop = create_control_loop(true);
        control_loop
            .plant
            .as_mut()
            .unwrap()
            .is_power_on_communication = true;
        control_loop.update_telemetry_data();

        // Let the hardpoints have the same positions as the home position.
        control_loop
            .config
            .hardpoints
            .iter()
            .enumerate()
            .for_each(|(idx, hardpoint)| {
                control_loop.telemetry.actuator_positions[*hardpoint] =
                    control_loop.config.disp_hardpoint_home[idx] * 1e3;
            });

        control_loop.process_telemetry_data();

        assert_eq!(control_loop.telemetry.inclinometer["processed"], 89.06);
        assert_relative_eq!(
            control_loop.telemetry.inclinometer["zenith"],
            0.94,
            epsilon = EPSILON
        );

        for key in control_loop.telemetry.mirror_position.keys() {
            assert_relative_eq!(
                control_loop.telemetry.mirror_position[key],
                control_loop.telemetry.mirror_position_ims[key],
                epsilon = EPSILON
            );
        }
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

        let mut control_loop = create_control_loop(true);
        control_loop
            .telemetry
            .inclinometer
            .insert("zenith".to_string(), zenit_angle);
        let tangent_force_error = control_loop.calculate_tangent_force_error(&vec![
            -325.307, -447.377, 1128.37, -1249.98, 458.63, 267.627,
        ]);

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
    fn test_get_current_hardpoint_displacement() {
        let mut control_loop = create_control_loop(true);
        control_loop.telemetry.actuator_positions = (0..NUM_ACTUATOR).map(|x| x as f64).collect();

        let displacement = control_loop.get_current_hardpoint_displacement();
        assert_eq!(displacement, vec![0.005, 0.015, 0.025, 0.073, 0.075, 0.077]);
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

        control_loop.handle_position_mirror(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);

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
        stabilize_control_loop(&mut control_loop);

        // Position the mirror
        control_loop.handle_position_mirror(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        steps(&mut control_loop, 220, ClosedLoopControlMode::ClosedLoop);

        assert_eq!(control_loop.steps_position_mirror, vec![0; NUM_ACTUATOR]);

        let axes = ["x", "y", "z", "xRot", "yRot", "zRot"];
        axes.iter().enumerate().for_each(|(idx, axis)| {
            let position = idx as f64 + 1.0;
            assert_relative_eq!(
                control_loop.telemetry.mirror_position[*axis],
                position,
                epsilon = 1e-1
            );
            assert_relative_eq!(
                control_loop.telemetry.mirror_position_ims[*axis],
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

        let force = vec![10.0; NUM_ACTUATOR];
        assert_eq!(control_loop.apply_force(&force), Ok(()));

        assert_relative_eq_vector(&control_loop.telemetry.forces["applied"], &force, EPSILON);
    }

    #[test]
    fn test_apply_force_closed_loop() {
        // Stabilize the mirror first
        let mut control_loop = create_control_loop(true);
        stabilize_control_loop(&mut control_loop);

        let force_original = control_loop.telemetry.forces["measured"][3];

        // Apply the force
        let mut force = vec![0.0; NUM_ACTUATOR];
        force[3] = 5.0;
        let _ = control_loop.apply_force(&force);

        steps(&mut control_loop, 15, ClosedLoopControlMode::ClosedLoop);

        let force_updated = control_loop.telemetry.forces["measured"][3];

        assert_relative_eq!(
            force_updated - force_original,
            4.53263384,
            epsilon = EPSILON
        );
    }

    #[test]
    fn test_reset_force() {
        let mut control_loop = create_control_loop(true);
        let _ = control_loop.apply_force(&vec![10.0; NUM_ACTUATOR]);

        control_loop.reset_force();

        assert_eq!(
            control_loop.telemetry.forces["applied"],
            vec![0.0; NUM_ACTUATOR]
        );
    }

    #[test]
    fn test_move_actuators() {
        let mut control_loop = create_control_loop(true);
        stabilize_control_loop(&mut control_loop);

        // Get the initial actuator step
        let step_init = control_loop.telemetry.actuator_steps[1];

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
        control_loop._mode = ClosedLoopControlMode::OpenLoop;
        control_loop.step();

        control_loop._mode = ClosedLoopControlMode::TelemetryOnly;
        control_loop.step();
        let step_move = control_loop.telemetry.actuator_steps[1];

        assert_eq!(
            step_move - step_init,
            control_loop.config.step_limit["axial"]
        );

        // Pause the movement
        assert_eq!(
            control_loop.move_actuators(
                CommandActuator::Pause,
                &Vec::new(),
                0.0,
                ActuatorDisplacementUnit::None
            ),
            Ok(())
        );

        control_loop._mode = ClosedLoopControlMode::OpenLoop;
        control_loop.step();

        control_loop._mode = ClosedLoopControlMode::TelemetryOnly;
        control_loop.step();
        let step_pause = control_loop.telemetry.actuator_steps[1];

        assert_eq!(step_pause, step_move);

        // Resume the movement
        assert_eq!(
            control_loop.move_actuators(
                CommandActuator::Resume,
                &Vec::new(),
                0.0,
                ActuatorDisplacementUnit::None
            ),
            Ok(())
        );

        control_loop._mode = ClosedLoopControlMode::OpenLoop;
        control_loop.step();

        control_loop._mode = ClosedLoopControlMode::TelemetryOnly;
        control_loop.step();
        let step_resume = control_loop.telemetry.actuator_steps[1];

        assert_eq!(step_resume - step_pause, 20);
        assert!(!control_loop._open_loop.is_running);

        // Stop the movement
        assert_eq!(
            control_loop.move_actuators(
                CommandActuator::Stop,
                &Vec::new(),
                0.0,
                ActuatorDisplacementUnit::None
            ),
            Ok(())
        );

        control_loop._mode = ClosedLoopControlMode::OpenLoop;
        control_loop.step();

        control_loop._mode = ClosedLoopControlMode::TelemetryOnly;
        control_loop.step();
        let step_stop = control_loop.telemetry.actuator_steps[1];

        assert_eq!(step_stop, step_resume);
    }

    #[test]
    fn test_set_ilc_mode() {
        let mut control_loop = create_control_loop(true);

        let mode = control_loop.set_ilc_mode(10, InnerLoopControlMode::Disabled);

        assert_eq!(mode, InnerLoopControlMode::Disabled);
        assert_eq!(
            control_loop.event_queue.get_events_and_clear(),
            vec![json!({
                "id": "innerLoopControlMode",
                "address": 10,
                "mode": 2,
            })]
        );
    }

    #[test]
    fn test_get_ilc_mode() {
        let mut control_loop = create_control_loop(true);

        let mode = control_loop.get_ilc_mode(10);

        assert_eq!(mode, InnerLoopControlMode::Standby);
        assert_eq!(
            control_loop.event_queue.get_events_and_clear(),
            vec![json!({
                "id": "innerLoopControlMode",
                "address": 10,
                "mode": 1,
            })]
        );
    }
}
