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

use nalgebra::{SMatrix, SVector};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;

use crate::constants::{
    NUM_HARDPOINTS, NUM_IMS_READING, NUM_SPACE_DEGREE_OF_FREEDOM, NUM_TEMPERATURE_RING,
};
use crate::control::lut::Lut;
use crate::utility::{get_parameter, get_parameter_array, read_file_cell_geom, read_file_disp_ims};

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub struct CellGeometry {
    // Location (x, y) of the axial actuators in m.
    pub loc_act_axial: Vec<Vec<f64>>,
    // Location of the tangential actuators in degree.
    pub loc_act_tangent: Vec<f64>,
    // Radius of the tangential actuators in m.
    pub radius_act_tangent: f64,
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub struct Config {
    // Configuration filename.
    pub filename: String,
    // Look-up table (LUT).
    pub lut: Lut,
    // Control frequency in Hz.
    pub control_frequency: f64,
    // The actuator indices that are bypassed in the closed-loop control.
    pub bypassed_actuator_ilcs: Vec<usize>,
    // Six 0-based hardpoints. The order is from low to high.
    pub hardpoints: Vec<usize>,
    // Displacement of the hardpoints at the home position. The unit is meter.
    pub disp_hardpoint_home: Vec<f64>,
    // Cell geometry.
    pub cell_geometry: CellGeometry,
    // Actuator step limit in each control cycle in an active way.
    pub step_limit: HashMap<String, i32>,
    // Default inclinometer offset with the telescope mount assembly.
    pub inclinometer_offset: f64,
    // Enable the temperature LUT or not.
    pub enable_lut_temperature: bool,
    // Use the external elevation angle or not (e.g. TMA).
    pub use_external_elevation_angle: bool,
    // Enable the comparison between the internal and external elevation angles
    // or not.
    pub enable_angle_comparison: bool,
    // Maximum angle difference between the internal and external elevation
    // angles in degree.
    pub max_angle_difference: f64,
    // Maximum temperature difference between the cells (intakes and exhausts)
    // in degree Celsius.
    pub max_cell_temperature_difference: f64,
    // Reference ring temperature in degree Celsius.
    // The order is the same as the ring temperature.
    pub ref_temperature: Vec<f64>,
    // Mirror's weight in kg.
    pub mirror_weight_kg: f64,
    // Matrix and offset for the IMS displacement.
    pub disp_matrix: SMatrix<f64, NUM_SPACE_DEGREE_OF_FREEDOM, NUM_IMS_READING>,
    pub disp_offset: SVector<f64, NUM_IMS_READING>,
    // Force limits
    pub force_limit: HashMap<String, f64>,
    // The threshold of the tangent force error in Newton.
    pub tangent_force_error_threshold: HashMap<String, f64>,
    // Enable the open-loop maximum limit or not.
    pub enable_open_loop_max_limit: bool,
    // Enabled faults mask.
    pub enabled_faults_mask: u64,
    // Maximum count that the control loop can be out of time
    // (1/control_frequency).
    pub max_out_cycle_time: i32,
}

impl Config {
    /// Create a new config object.
    ///
    /// # Arguments
    /// * `filepath_parameters_control` - The path to the control parameters
    /// file.
    /// * `lut_dir` - The path to the directory that contains the look-up table.
    ///
    /// # Returns
    /// A new config object.
    pub fn new(filepath_parameters_control: &Path, lut_dir: &Path) -> Self {
        // Read the cell geometry from the configuration file.
        let (loc_act_axial, loc_act_tangent, radius_act_tangent) =
            read_file_cell_geom(Path::new("config/cell_geom.yaml"));

        // Read the step limit from the configuration file.
        let mut step_limit = HashMap::new();
        step_limit.insert(
            String::from("axial"),
            get_parameter(filepath_parameters_control, "step_limit_axial_active"),
        );
        step_limit.insert(
            String::from("tangent"),
            get_parameter(filepath_parameters_control, "step_limit_tangent_active"),
        );

        // Read the reference temperature from the configuration file.
        let ref_temperature = get_parameter_array(filepath_parameters_control, "ref_temperature");
        assert!(ref_temperature.len() == NUM_TEMPERATURE_RING);

        // Read the displacement matrix and offset from the configuration file.
        let (matrix, offset) = read_file_disp_ims(Path::new("config/disp_ims.yaml"));
        let disp_matrix: SMatrix<f64, NUM_SPACE_DEGREE_OF_FREEDOM, NUM_IMS_READING> =
            SMatrix::from_row_iterator(matrix.iter().flat_map(|row| row.iter().copied()));
        let disp_offset: SVector<f64, NUM_IMS_READING> =
            SVector::from_iterator(offset.iter().copied());

        // Read the force limits
        let mut force_limit = HashMap::new();
        for key in [
            "limit_force_axial_closed_loop",
            "limit_force_tangent_closed_loop",
            "limit_force_axial_open_loop",
            "limit_force_tangent_open_loop",
            "max_limit_force_axial_open_loop",
            "max_limit_force_tangent_open_loop",
        ] {
            force_limit.insert(
                String::from(key),
                get_parameter(filepath_parameters_control, key),
            );
        }

        // Read the tangent force error threshold
        let mut tangent_force_error_threshold = HashMap::new();
        for key in [
            "tangent_link_total_weight_error",
            "tangent_link_load_bearing_link",
            "tangent_link_theta_z_moment",
            "tangent_link_non_load_bearing_link",
        ] {
            tangent_force_error_threshold.insert(
                String::from(key),
                get_parameter(filepath_parameters_control, key),
            );
        }

        Self {
            filename: String::from(filepath_parameters_control.to_str().expect(&format!(
                "Should be able to convert {:?} to a string",
                filepath_parameters_control
            ))),
            lut: Lut::new(lut_dir),

            control_frequency: get_parameter(filepath_parameters_control, "control_frequency"),
            bypassed_actuator_ilcs: get_parameter_array(
                filepath_parameters_control,
                "bypassed_actuator_ilcs",
            ),
            hardpoints: Self::read_hardpoints(filepath_parameters_control),
            disp_hardpoint_home: vec![0.0; NUM_HARDPOINTS],
            cell_geometry: CellGeometry {
                loc_act_axial: loc_act_axial,
                loc_act_tangent: loc_act_tangent,
                radius_act_tangent: radius_act_tangent,
            },
            step_limit: step_limit,
            inclinometer_offset: get_parameter(filepath_parameters_control, "inclinometer_offset"),
            enable_lut_temperature: get_parameter(
                filepath_parameters_control,
                "enable_lut_temperature",
            ),
            use_external_elevation_angle: get_parameter(
                filepath_parameters_control,
                "use_external_elevation_angle",
            ),
            enable_angle_comparison: get_parameter(
                filepath_parameters_control,
                "enable_angle_comparison",
            ),
            max_angle_difference: get_parameter(
                filepath_parameters_control,
                "max_angle_difference",
            ),
            max_cell_temperature_difference: get_parameter(
                filepath_parameters_control,
                "max_cell_temperature_difference",
            ),
            ref_temperature: ref_temperature,
            mirror_weight_kg: get_parameter(filepath_parameters_control, "mirror_weight_kg"),

            disp_matrix: disp_matrix,
            disp_offset: disp_offset,

            force_limit: force_limit,

            tangent_force_error_threshold: tangent_force_error_threshold,

            enable_open_loop_max_limit: false,

            enabled_faults_mask: get_parameter(filepath_parameters_control, "enabled_faults_mask"),
            max_out_cycle_time: get_parameter(filepath_parameters_control, "max_out_cycle_time"),
        }
    }

    /// Read the hardpoints from the configuration file.
    ///
    /// # Arguments
    /// * `filepath_parameters_control` - The path to the control parameters
    /// file.
    ///
    /// # Returns
    /// Six 0-based hardpoints. The order is from low to high.
    fn read_hardpoints(filepath_parameters_control: &Path) -> Vec<usize> {
        let mut hardpoints = get_parameter_array(filepath_parameters_control, "hardpoints");
        hardpoints.sort();
        assert_eq!(hardpoints.len(), NUM_HARDPOINTS);
        hardpoints
    }
}
