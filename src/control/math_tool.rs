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

use kiddo::{KdTree, SquaredEuclidean};
use nalgebra::{vector, DMatrix, Rotation3, SMatrix, SVector};

use crate::constants::{
    NUM_ACTIVE_ACTUATOR, NUM_ACTIVE_ACTUATOR_AXIAL, NUM_ACTIVE_ACTUATOR_TANGENT, NUM_ACTUATOR,
    NUM_AXIAL_ACTUATOR, NUM_HARDPOINTS_AXIAL, NUM_HARDPOINTS_TANGENT, NUM_IMS_READING,
    NUM_SPACE_DEGREE_OF_FREEDOM,
};

/// Clip the value between the lower and upper bounds.
///
/// # Arguments
/// * `value` - The value to be clipped.
/// * `lower` - The lower bound.
/// * `upper` - The upper bound.
///
/// # Returns
/// The clipped value.
pub fn clip<T>(value: T, lower: T, upper: T) -> T
where
    T: PartialOrd,
{
    if value < lower {
        lower
    } else if value > upper {
        upper
    } else {
        value
    }
}

/// Correct the inclinometer's value and make sure to limit the
/// resulting value to the indicated range: (-270, 90).
///
/// # Arguments
/// * `angle` - Inclinometer angle in degree.
/// * `offset` - Offset in degree to correct the inclinometer angle.
///
/// # Returns
/// Corrected inclinometer angle in degree.
pub fn correct_inclinometer_angle(angle: f64, offset: f64) -> f64 {
    let angle_offset = angle + offset;
    let origin = if (angle_offset >= 0.0) && (angle_offset < 90.0) {
        360.0
    } else {
        0.0
    };

    let mut angle_correct = 180.0 - angle_offset - origin;

    // Make sure the calculated angle value is limited to this range
    if angle_correct > 90.0 {
        angle_correct = 90.0;
    } else if angle_correct < -270.0 {
        angle_correct = -270.0;
    }

    angle_correct
}

/// Calculate the hardpoint compensation matrix.
///
/// # Notes
/// <Axial actuators>
///
/// The axial hardpoint compensation matrix (M) fulfills:
/// M * (x_hp, y_hp, 1) = (x_nhp, y_nhp, 1)
///
/// x_hp: x-position of hardpoint
/// y_hp: y-position of hardpoint
/// x_nhp: x-position of non-hardpoint
/// y_nhp: y-position of non-hardpoint
///
/// The idea is to make the x-moment amd y-moment keep the same when distributes
/// the force of axial hardpoints to other axial actuators. It is the same idea
/// for tangential actuators with z-moment.
///
/// <Tangent links>
///
/// Assume the hardpoints are A1, A3, and A5. If they were active, the z-moment
/// would be: 3 * fm * mirror_radius, where fm = (f1 + f3 + f5) / 3.
///
/// For the active tagent links: A2, A4, and A5 to maintain this z-moment while
/// keeping the hardpoints to be passive, we have:
///
/// f2 = -(f5 - fm) + fm = 2 * fm - f5 = 2 / 3 * (f1 + f3 + f5) - f5
/// f4 = -(f1 - fm) + fm = 2 * fm - f1 = 2 / 3 * (f1 + f3 + f5) - f1
/// f6 = -(f3 - fm) + fm = 2 * fm - f3 = 2 / 3 * (f1 + f3 + f5) - f3
///
/// Then, we have:
///
/// f2 = 2 / 3 * f1 + 2 / 3 * f3 - 1 / 3 * f5
/// f4 = -1 / 3 * f1 + 2 / 3 * f3 + 2 / 3 * f5
/// f4 = -1 / 3 * f1 + 2 / 3 * f3 + 2 / 3 * f5
///
/// In the matrix form, it will be:
///
/// (f2, f4, f6).T = M * (f1, f3, f5).T
///
/// If the hardpoints were A2, A4, A6, we would have:
///
/// (f1, f3, f5).T = M ^ (-1) * (f2, f4, f6).T
///
/// # Arguments
/// * `loc_act_axial` - Location of the axial actuators: (x, y). This should be
/// a 72 x 2 matrix.
/// * `hardpoints_axial` - Three axial hardpoints. The order is from low to
/// high, e.g. [5, 15, 25].
/// * `hardpoints_tangent` - Three tangential hardpoints. This can only be [72,
/// 74, 76] or [73, 75, 77]. The order is from low to high.
///
/// # Returns
/// A tuple of two matrices: axial hardpoint compensation matrix and tangential
/// hardpoint compensation matrix.
pub fn calc_hp_comp_matrix(
    loc_act_axial: &Vec<Vec<f64>>,
    hardpoints_axial: &[usize],
    hardpoints_tangent: &[usize],
) -> (
    SMatrix<f64, NUM_ACTIVE_ACTUATOR_AXIAL, NUM_HARDPOINTS_AXIAL>,
    SMatrix<f64, NUM_ACTIVE_ACTUATOR_TANGENT, NUM_HARDPOINTS_TANGENT>,
) {
    // Axial hardpoints
    let mut matrix_loc_axial = DMatrix::from_row_iterator(
        loc_act_axial.len(),
        2,
        loc_act_axial.iter().flat_map(|row| row.iter().copied()),
    );
    matrix_loc_axial = matrix_loc_axial.insert_column(2, 1.0);

    let active_actuators_axial = get_active_actuators(hardpoints_axial, hardpoints_tangent).0;

    let hd_comp_axial_temp = matrix_loc_axial.select_rows(&active_actuators_axial)
        * matrix_loc_axial
            .select_rows(hardpoints_axial)
            .pseudo_inverse(f64::EPSILON)
            .expect(
                "Should be able to calculate the hardpoint compensation matrix of axial actuators.",
            );

    let hd_comp_axial: SMatrix<f64, NUM_ACTIVE_ACTUATOR_AXIAL, NUM_HARDPOINTS_AXIAL> =
        SMatrix::from_iterator(hd_comp_axial_temp.iter().copied());

    // Tangential hardpoints
    let mut hd_comp_tangent: SMatrix<f64, NUM_ACTIVE_ACTUATOR_TANGENT, NUM_HARDPOINTS_TANGENT> =
        SMatrix::repeat(2.0 / 3.0);
    if hardpoints_tangent == &vec![72, 74, 76] {
        hd_comp_tangent[(0, 2)] = -1.0 / 3.0;
        hd_comp_tangent[(1, 0)] = -1.0 / 3.0;
        hd_comp_tangent[(2, 1)] = -1.0 / 3.0;
    } else {
        hd_comp_tangent[(0, 1)] = -1.0 / 3.0;
        hd_comp_tangent[(1, 2)] = -1.0 / 3.0;
        hd_comp_tangent[(2, 0)] = -1.0 / 3.0;
    }

    (hd_comp_axial, hd_comp_tangent)
}

/// Get the active actuators.
///
/// # Arguments
/// * `hardpoints_axial` - Three axial hardpoints. The order is from low to
/// high, e.g. [5, 15, 25].
/// * `hardpoints_tangent` - Three tangential hardpoints. This can only be [72,
/// 74, 76] or [73, 75, 77]. The order is from low to high.
///
/// # Returns
/// A tuple of two vectors: active axial actuators and active tangent actuators.
///
/// # Panics
/// If the tangential hardpoints are wrong.
fn get_active_actuators(
    hardpoints_axial: &[usize],
    hardpoints_tangent: &[usize],
) -> (Vec<usize>, Vec<usize>) {
    // Axial hardpoints
    let active_actuators_axial: Vec<usize> = (0..NUM_AXIAL_ACTUATOR)
        .filter(|idx| !hardpoints_axial.contains(idx))
        .collect();

    // Tangent hardpoints
    let option_one = vec![72, 74, 76];
    let option_two = vec![73, 75, 77];

    let active_actuators_tangent: Vec<usize>;
    if hardpoints_tangent == &option_one {
        active_actuators_tangent = option_two;
    } else if hardpoints_tangent == &option_two {
        active_actuators_tangent = option_one;
    } else {
        panic!(
            "Tangential hardpoints can only be {:?} or {:?}.",
            option_one, option_two
        );
    }

    (active_actuators_axial, active_actuators_tangent)
}

/// Calculate the kinetic decoupling matrix.
///
/// # Notes
/// The unit of the element is the motor step (could be fraction, row) per
/// Newton (column). This matrix is used to multiply with the desired actuator's
/// force changes to get the related amounts of motor's step changes.
///
/// delta_force = M * delta_step => M^-1 = kdc to have:
/// delta_step = kdc * delta_force
///
/// # Arguments
/// * `loc_act_axial` - Location of the axial actuators: (x, y). This should be
/// a 72 x 2 matrix.
/// * `hardpoints_axial` - Three axial hardpoints. The order is from low to
/// high, e.g. [5, 15, 25].
/// * `hardpoints_tangent` - Three tangential hardpoints. This can only be [72,
/// 74, 76] or [73, 75, 77]. The order is from low to high.
/// * `stiffness` - Stiffness matrix of the actuators.
///
/// # Returns
/// Kinetic decoupling matrix.
pub fn calc_kinetic_decoupling_matrix(
    loc_act_axial: &Vec<Vec<f64>>,
    hardpoints_axial: &[usize],
    hardpoints_tangent: &[usize],
    stiffness: &Vec<Vec<f64>>,
) -> SMatrix<f64, NUM_ACTIVE_ACTUATOR, NUM_ACTIVE_ACTUATOR> {
    // Calculate the hardpoint force error
    let (active_actuators_axial, active_actuators_tangent) =
        get_active_actuators(hardpoints_axial, hardpoints_tangent);

    // There is a "minus" sign here because the hardpoints are the passive
    // actuators.
    let stiffness_matrix: SMatrix<f64, NUM_ACTUATOR, NUM_ACTUATOR> =
        SMatrix::from_row_iterator(stiffness.iter().flat_map(|row| row.iter().copied()));

    let hardpoint_error_axial = -stiffness_matrix
        .select_rows(&active_actuators_axial)
        .select_columns(hardpoints_axial);
    let hardpoint_error_tangent = -stiffness_matrix
        .select_rows(&active_actuators_tangent)
        .select_columns(hardpoints_tangent);

    // Force change of the active actuators
    let force_axial = stiffness_matrix
        .select_rows(&active_actuators_axial)
        .select_columns(&active_actuators_axial);
    let force_tangent = stiffness_matrix
        .select_rows(&active_actuators_tangent)
        .select_columns(&active_actuators_tangent);

    // Calculate the effective force feedback after hardpoint force compensation
    let (hd_comp_axial, hd_comp_tangent) =
        calc_hp_comp_matrix(loc_act_axial, hardpoints_axial, hardpoints_tangent);

    let force_feedback_axial = force_axial + hd_comp_axial * hardpoint_error_axial.transpose();
    let force_feedback_tangent =
        force_tangent + hd_comp_tangent * hardpoint_error_tangent.transpose();

    // Block diagnal matrix of force_feedback_axial and force_feedback_tangent
    let mut force_feedback: SMatrix<f64, NUM_ACTIVE_ACTUATOR, NUM_ACTIVE_ACTUATOR> =
        SMatrix::zeros();
    force_feedback
        .view_mut((0, 0), force_feedback_axial.shape())
        .copy_from(&force_feedback_axial);
    force_feedback
        .view_mut(force_feedback_axial.shape(), force_feedback_tangent.shape())
        .copy_from(&force_feedback_tangent);

    // Do the pseudo-inverse
    force_feedback
        .pseudo_inverse(f64::EPSILON)
        .expect("Should be able to calculate the kinetic decoupling matrix.")
}

/// Calculate the command delay filter parameters (or more clearly, the
/// numerator of the transfer function).
///
/// # Notes
/// For a transfer function, H(z), if the numerator is [1, 3, 3] and the
/// denominator is [1, 2, 1], it will be:
///
/// H(z) = (z^2 + 3 * z + 3) / (z^2 + 2 * z + 1)
///
/// Since the denominator of transfer function is the demanded force change
/// here, the linear transfer ([1.0]) is applied. Therefore, we only care about
/// the delay of command as the output.
///
/// # Arguments
/// * `is_mirror` - Is mirror or not.
/// * `control_frequency` - Frequency of the control loop in Hz.
/// * `bypass_delay` - Bypass the command delay or not.
/// * `num_degree` - Number of the degree in the transfer function.
///
/// # Returns
/// Numerator of the transfer function from the high degree to low degree.
pub fn calc_cmd_delay_filter_params(
    is_mirror: bool,
    control_frequency: f64,
    bypass_delay: bool,
    num_degree: usize,
) -> Vec<f64> {
    let time_sample = 1.0 / control_frequency;

    let mut time_delay = 0.0;
    if !bypass_delay {
        // Note the delay time of mirror is shorter than the surrogate
        time_delay = if is_mirror { 0.0993 } else { 0.1016 };
    }

    let num_sample_delay = (time_delay / time_sample).floor() as usize;
    let time_left = time_delay - (num_sample_delay as f64) * time_sample;

    let mut coeff_delay = vec![0.0; num_sample_delay + 2];
    coeff_delay[num_sample_delay] = 1.0 - time_left / time_sample;
    coeff_delay[num_sample_delay + 1] = time_left / time_sample;

    // 1 is for the constant part in the numerator coefficients
    for _ in 0..(num_degree + 1 - coeff_delay.len()) {
        coeff_delay.push(0.0);
    }

    coeff_delay
}

/// Check the selected hardpoints are good or not.
///
/// # Arguments
/// * `loc_act_axial` - Location of the axial actuators: (x, y). This should be
/// a 72 x 2 matrix.
/// * `hardpoints_axial` - Three axial hardpoints. The order is from low to
/// high, e.g. [5, 15, 25].
/// * `hardpoints_tangent` - Three tangential hardpoints. This can only be [72,
/// 74, 76] or [73, 75, 77]. The order is from low to high.
///
/// # Returns
/// Result of the check.
pub fn check_hardpoints(
    loc_act_axial: &Vec<Vec<f64>>,
    hardpoints_axial: &[usize],
    hardpoints_tangent: &[usize],
) -> Result<(), &'static str> {
    // Axial hardpoints

    // Check the hardpoints by comparing with the expectation
    if hardpoints_axial != &select_axial_hardpoints(loc_act_axial, hardpoints_axial[0]) {
        return Err("Bad selection of axial hardpoints.");
    }

    // Tangent hardpoints
    let option_one = vec![72, 74, 76];
    let option_two = vec![73, 75, 77];

    if (hardpoints_tangent != &option_one) && (hardpoints_tangent != &option_two) {
        return Err("Tangential hardpoints can only be [72, 74, 76] or [73, 75, 77].");
    }

    Ok(())
}

/// Select the axial hardpoints based on the specific axial hardpoint.
///
/// # Notes
/// The idea is to maximize the triangle constructed by 3 axial actuators, which
/// means it should be closed to the equilateral triangle.
///
/// # Arguments
/// * `loc_act_axial` - Location of the axial actuators: (x, y). This should be
/// a 72 x 2 matrix.
/// * `specific_axial_hardpoint` - Specific axial hardpoint.
///
/// # Returns
/// Selected 3 axial hardpoints that contains the specific axial hardpoint. The
/// order is from low to high.
fn select_axial_hardpoints(
    loc_act_axial: &Vec<Vec<f64>>,
    specific_axial_hardpoint: usize,
) -> Vec<usize> {
    // Get the polar coordinate of specific hardpoint
    let x = loc_act_axial[specific_axial_hardpoint][0];
    let y = loc_act_axial[specific_axial_hardpoint][1];

    let radius = (x.powi(2) + y.powi(2)).sqrt();
    let theta = y.atan2(x);

    // Get the angles of the other two hardpoints
    let theta_one = theta + 120.0_f64.to_radians();
    let theta_two = theta + 240.0_f64.to_radians();

    // Get the closest actuator index
    let mut tree: KdTree<f64, 2> = KdTree::new();
    for (idx, loc) in loc_act_axial.iter().enumerate() {
        tree.add(&[loc[0], loc[1]], idx as u64);
    }

    let nearest_one = tree
        .nearest_one::<SquaredEuclidean>(&[radius * theta_one.cos(), radius * theta_one.sin()])
        .item;
    let nearest_two = tree
        .nearest_one::<SquaredEuclidean>(&[radius * theta_two.cos(), radius * theta_two.sin()])
        .item;

    let mut hardpoints = vec![
        specific_axial_hardpoint,
        nearest_one as usize,
        nearest_two as usize,
    ];
    hardpoints.sort();

    hardpoints
}

/// Calculate the actuator displacements based on the rigid body position.
///
/// # Notes
/// The "radius" is an average of 5 of the 6 tangent location radius from the
/// center of the M2 cell which is to be used by the rigid body to displacement
/// calculation. Technically, it is not the B-ring radius and this should be
/// changed in the future.
///
/// # Arguments
/// * `loc_act_axial` - Location of the axial actuators: (x, y) in meter. This
/// should be a 72 x 2 matrix.
/// * `loc_tangent_link` - Location of the tangent links in degree.
/// * `radius` - Radius of the cell in meter.
/// * `dx` - Delta x position in meter.
/// * `dy` - Delta y position in meter.
/// * `dz` - Delta z position in meter.
/// * `drx` - Delta x rotator in radian.
/// * `dry` - Delta y rotator in radian.
/// * `drz` - Delta z rotator in radian.
///
/// # Returns
/// All actuator displacements in meter.
pub fn rigid_body_to_actuator_displacement(
    loc_act_axial: &Vec<Vec<f64>>,
    loc_act_tangent: &[f64],
    radius: f64,
    dx: f64,
    dy: f64,
    dz: f64,
    drx: f64,
    dry: f64,
    drz: f64,
) -> Vec<f64> {
    // Consider the displacement from (dx, dy, dz)

    // Transformation matrix. The raw is each actuator's displacement. The
    // column is the (x, y, z) movement.
    let mut matrix_trans: SMatrix<f64, NUM_ACTUATOR, 3> = SMatrix::zeros();

    // A positive axial displacement results in a negative change in piston.
    matrix_trans
        .column_part_mut(2, NUM_AXIAL_ACTUATOR)
        .fill(-1.0);

    loc_act_tangent.iter().enumerate().for_each(|(idx, angle)| {
        matrix_trans[(NUM_AXIAL_ACTUATOR + idx, 0)] = angle.to_radians().cos();
        matrix_trans[(NUM_AXIAL_ACTUATOR + idx, 1)] = -angle.to_radians().sin();
    });

    // Translate the (dx, dy, dz) motion to 78 actuator displacement.
    let disp_dxdydz = matrix_trans * vector![dx, dy, dz];

    // Consider the displacement from (drx, dry, drz)
    let mut location_axial: SMatrix<f64, NUM_AXIAL_ACTUATOR, 3> = SMatrix::zeros();
    loc_act_axial.iter().enumerate().for_each(|(idx, loc)| {
        location_axial[(idx, 0)] = loc[0];
        location_axial[(idx, 1)] = loc[1];
    });

    // Rotation matrix for the axial actuators
    // The primitive rotations are applied in order: 1 roll − 2 pitch − 3 yaw in
    // nalgebra. But we need to apply the inverse order here.
    let rot_xyz = Rotation3::from_euler_angles(drx, dry, drz).inverse();

    let disp_drxdrydrz_axial = rot_xyz * location_axial.transpose();
    let disp_drz_axial = disp_drxdrydrz_axial.row(2);

    // Displacement drz, note there is a "-1" here for the direction of
    // coordination system.
    let disp_drz_tangent = -radius * drz.sin();

    // Get the final displacement
    let mut disp: Vec<f64> = disp_dxdydz.iter().copied().collect();
    disp_drz_axial.iter().enumerate().for_each(|(idx, val)| {
        disp[idx] += val;
    });

    for idx in NUM_AXIAL_ACTUATOR..NUM_ACTUATOR {
        disp[idx] += disp_drz_tangent;
    }

    disp
}

/// Calculate the rigid body position based on the hardpoint displacements.
///
/// # Arguments
/// * `loc_act_axial` - Location of the axial actuators: (x, y) in meter. This
/// should be a 72 x 2 matrix.
/// * `loc_tangent_link` - Location of the tangent links in degree.
/// * `radius` - Radius of the cell in meter.
/// * `hardpoints` - Six hardpoints. The order is from low to high.
/// * `disp_hardpoint_current` - Six current hardpoint displacements. The unit
/// is meter.
/// * `disp_hardpoint_home` - Six hardpoint displacements at the home position.
/// The unit is meter.
///
/// # Returns
/// Result of the rigid body position (x, y, z, rx, ry, rz) in meter and radian.
pub fn hardpoint_to_rigid_body(
    loc_act_axial: &Vec<Vec<f64>>,
    loc_act_tangent: &[f64],
    radius: f64,
    hardpoints: &[usize],
    disp_hardpoint_current: &[f64],
    disp_hardpoint_home: &[f64],
) -> Result<(f64, f64, f64, f64, f64, f64), &'static str> {
    // Delta of the axial hardpoint displacements
    let disp_hardpoint_axial: SVector<f64, NUM_HARDPOINTS_AXIAL> = SVector::from_iterator(
        (0..NUM_HARDPOINTS_AXIAL)
            .into_iter()
            .map(|idx| disp_hardpoint_current[idx] - disp_hardpoint_home[idx]),
    );

    // Location of the axial hardpoints: (x_hp, y_hp, 1)
    let mut loc_axial_hardpoint: SMatrix<f64, NUM_HARDPOINTS_AXIAL, 3> = SMatrix::repeat(1.0);
    loc_axial_hardpoint
        .row_iter_mut()
        .enumerate()
        .for_each(|(idx, mut row)| {
            row[0] = loc_act_axial[hardpoints[idx]][0];
            row[1] = loc_act_axial[hardpoints[idx]][1];
        });

    // The displacement of axial actuators decides the (z, rx, ry) of rigid
    // body.
    let dof_axial = loc_axial_hardpoint
        .pseudo_inverse(f64::EPSILON)
        .expect("Should be able to do the pseudo-inverse of axial hardpoint locations")
        * disp_hardpoint_axial;

    // Note the negative piston means the +z
    let z = -dof_axial[2];

    // Calculate the (rx, ry) in radian
    let mat_axial_3_points: SMatrix<f64, NUM_HARDPOINTS_AXIAL, 3> =
        SMatrix::from_row_slice(&[0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 1.0]);

    let z_axial_3_points = mat_axial_3_points * dof_axial;

    let mut mat_axial_3_points_updated = mat_axial_3_points.clone();
    mat_axial_3_points_updated.set_column(2, &z_axial_3_points);

    let r1c = mat_axial_3_points_updated.row(1) - mat_axial_3_points_updated.row(0);
    let r2 = mat_axial_3_points_updated.row(2) - mat_axial_3_points_updated.row(0);

    let cross_r1c_r2_normalized = r1c.cross(&r2).normalize();

    let vector_rxryrz = vector![0.0, 0.0, 1.0]
        .transpose()
        .cross(&cross_r1c_r2_normalized);
    let norm_vector_rxryrz = vector_rxryrz.norm();

    let mut rxry = vector![0.0, 0.0];
    if norm_vector_rxryrz != 0.0 {
        rxry = -vector_rxryrz.fixed_view::<1, 2>(0, 0).transpose() / norm_vector_rxryrz
            * norm_vector_rxryrz.asin();
    }

    // The displacement of tangent links decides the (x, y, rz) of rigid body.
    let idx_tangent_hardpoint: Vec<usize> = hardpoints[NUM_HARDPOINTS_AXIAL..]
        .iter()
        .map(|hardpoint| hardpoint - NUM_AXIAL_ACTUATOR)
        .collect();

    let loc_tangent_hardpoint: Vec<f64> = idx_tangent_hardpoint
        .iter()
        .map(|idx| loc_act_tangent[*idx].to_radians())
        .collect();

    // Calculate the (x, y) in meter
    let (x_current, y_current, delta_xy_current) = calculate_rigid_body_xy(
        radius,
        &disp_hardpoint_current[NUM_HARDPOINTS_AXIAL..],
        &loc_tangent_hardpoint,
    );

    let (x_home, y_home, delta_xy_home) = calculate_rigid_body_xy(
        radius,
        &disp_hardpoint_home[NUM_HARDPOINTS_AXIAL..],
        &loc_tangent_hardpoint,
    );

    // The original developer comments the following in LabVIEW:
    // Need to compensate the X-Y displacements by scaling by 2 in order to
    // calculate the correct displacements as measured optically.
    let x = 2.0 * (x_current - x_home);
    let y = 2.0 * (y_current - y_home);

    // Calculate the rz in radian

    // The details can follow:
    // Least-Squares Rigid Motion Using SVD by Olga Sorkine-Hornung and
    // Michael Rabinovich, 2017
    // https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
    let svd = (delta_xy_home.transpose() * delta_xy_current).svd(true, true);
    let mat_u = match svd.u {
        Some(mat) => mat,
        None => return Err("Failed to get the U matrix in hardpoint_to_rigid_body()."),
    };
    let mat_v = match svd.v_t {
        Some(mat) => mat,
        None => return Err("Failed to get the V matrix in hardpoint_to_rigid_body()."),
    };

    // Consider the potential reflection when calculating the rotation matrix
    let mat_det_vut: SMatrix<f64, 2, 2> =
        SMatrix::from_row_slice(&[1.0, 0.0, 0.0, (mat_v * mat_u.transpose()).determinant()]);

    // For any version of SVD, the forms of U and V are not unique (this is
    // mainly due to different choices of orthogonal basis for each eigenspace).
    // Therefore, the following equation is not the same as the above reference.
    let mat_r = mat_v.transpose() * mat_det_vut * mat_u.transpose();

    // mat_r is the rotation matrix of rz, based on the sign of cos(rz) to
    // decide the direction of rotation.
    let rz = (mat_r[(0, 0)].signum() * mat_r[(1, 0)]).atan2(mat_r[(0, 0)].abs());

    Ok((x, y, z, rxry[0], rxry[1], rz))
}

/// Calculate the rigid body (x, y) position.
///
/// # Arguments
/// * `radius` - Radius of the cell in meter.
/// * `tangent_hardpoint_disp` - Displacement of the 3 tangent hardpoints in
/// meter.
/// * `tangent_hardpoint_loc` - Location of the 3 tangent hardpoints in radian.
///
/// # Returns
/// A tuple of 3 elements: x, y posintion in meter and the delta (x, y) position
/// compared with the mean (x, y) in meter.
fn calculate_rigid_body_xy(
    radius: f64,
    tangent_hardpoint_disp: &[f64],
    tangent_hardpoint_loc: &[f64],
) -> (f64, f64, SMatrix<f64, NUM_HARDPOINTS_TANGENT, 2>) {
    // d: actuator displacement
    // R: radius
    // theta, theta_0: angles of tangent links at current and home positions
    // d = R * tan(theta - theta_0)
    // => theta = theta_0 + tan^(-1)(d/R)
    let thetas: SVector<f64, NUM_HARDPOINTS_TANGENT> = SVector::from_iterator(
        tangent_hardpoint_disp
            .iter()
            .zip(tangent_hardpoint_loc.iter())
            .map(|(disp, loc)| loc + disp.atan2(radius)),
    );

    // x = R * cos(pi/2 - theta) = R * sin(theta)
    // y = R * sin(pi/2 - theta) = R * cos(theta)
    let tangent_hardpoint_x = radius * thetas.map(|theta| theta.sin());
    let tangent_hardpoint_y = radius * thetas.map(|theta| theta.cos());

    let mean_x = tangent_hardpoint_x.mean();
    let mean_y = tangent_hardpoint_y.mean();

    let delta_xy_tangent_hardpoint: SMatrix<f64, NUM_HARDPOINTS_TANGENT, 2> =
        SMatrix::from_columns(&[
            tangent_hardpoint_x.map(|x| x - mean_x),
            tangent_hardpoint_y.map(|y| y - mean_y),
        ]);

    (mean_x, mean_y, delta_xy_tangent_hardpoint)
}

/// Calculate the position based on the reading of independent measurement
/// system (IMS).
///
/// # Arguments
/// * `disp_matrix` - Displacement sensor matrix to calculate the rigid body
/// position.
/// * `disp_offset` - Displacement sensor offset to calculate the rigid body
/// position.
/// * `theta_z` - IMS theta z readings in micron.
/// * `delta_z` - IMS delta z readings in micron.
///
/// # Returns
/// A tuple of 6 elements: x, y, z, rx, ry, rz in micron and arcsec.
pub fn calculate_position_ims(
    disp_matrix: &SMatrix<f64, NUM_SPACE_DEGREE_OF_FREEDOM, NUM_IMS_READING>,
    disp_offset: &SVector<f64, NUM_IMS_READING>,
    theta_z: &[f64],
    delta_z: &[f64],
) -> (f64, f64, f64, f64, f64, f64) {
    // Combine the readings
    let mut readings = theta_z.to_vec();
    readings.extend(delta_z);

    let index_array = vec![8, 10, 4, 6, 0, 2, 9, 11, 5, 7, 1, 3];

    // Create a vector of tuples (index, value)
    let mut indexed_readings: Vec<(usize, f64)> = index_array
        .iter()
        .enumerate()
        .map(|(i, index)| (*index, readings[i]))
        .collect();

    // Sort the vector of tuples by the index
    indexed_readings.sort_by_key(|(index, _)| *index);

    // Extract the sorted values
    let sorted_readings: Vec<f64> = indexed_readings
        .into_iter()
        .map(|(_, value)| value)
        .collect();

    // The unit needs to change to mm from um
    let position = disp_matrix * (SVector::from_vec(sorted_readings) * 1e-3 - disp_offset);

    (
        position[0],
        position[1],
        position[2],
        position[3],
        position[4],
        position[5],
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::path::Path;

    use crate::utility::{assert_relative_eq_vector, read_file_cell_geom, read_file_stiffness};

    const EPSILON: f64 = 1e-7;

    #[test]
    fn test_clip() {
        assert_eq!(clip(10.0, -3.0, 5.0), 5.0);
        assert_eq!(clip(-10.0, -3.0, 5.0), -3.0);
        assert_eq!(clip(3.0, -3.0, 5.0), 3.0);
    }

    #[test]
    fn test_correct_inclinometer_angle() {
        let offset = 0.94;
        assert_eq!(correct_inclinometer_angle(-10.0, offset), 90.0);

        assert_eq!(correct_inclinometer_angle(0.0, offset), -180.94);
        assert_eq!(correct_inclinometer_angle(30.0, offset), -210.94);
        assert_eq!(correct_inclinometer_angle(89.06, offset), 90.0);
        assert_eq!(correct_inclinometer_angle(90.0, offset), 89.06);
        assert_eq!(correct_inclinometer_angle(120.0, offset), 59.06);
        assert_relative_eq!(correct_inclinometer_angle(200.0, offset), -20.94);

        assert_eq!(correct_inclinometer_angle(500.0, offset), -270.0);
    }

    #[test]
    fn test_calc_hp_comp_matrix() {
        let loc_act_axial = read_file_cell_geom(Path::new("config/cell_geom.yaml")).0;

        let (hd_comp_axial, hd_comp_tangent) =
            calc_hp_comp_matrix(&loc_act_axial, &vec![5, 15, 25], &vec![73, 75, 77]);

        assert_relative_eq!(hd_comp_axial[(0, 0)], 0.6666667, epsilon = EPSILON);
        assert_relative_eq!(hd_comp_axial[(0, 1)], -0.3333333, epsilon = EPSILON);
        assert_relative_eq!(hd_comp_axial[(0, 2)], 0.6666667, epsilon = EPSILON);
        assert_relative_eq!(hd_comp_axial[(68, 0)], 0.4057876, epsilon = EPSILON);
        assert_relative_eq!(hd_comp_axial[(68, 1)], -0.0587425, epsilon = EPSILON);
        assert_relative_eq!(hd_comp_axial[(68, 2)], 0.6529549, epsilon = EPSILON);

        assert_eq!(hd_comp_tangent[(0, 0)], 2.0 / 3.0);
        assert_eq!(hd_comp_tangent[(0, 1)], -1.0 / 3.0);
        assert_eq!(hd_comp_tangent[(0, 2)], 2.0 / 3.0);
        assert_eq!(hd_comp_tangent[(2, 0)], -1.0 / 3.0);
        assert_eq!(hd_comp_tangent[(2, 1)], 2.0 / 3.0);
        assert_eq!(hd_comp_tangent[(2, 2)], 2.0 / 3.0);
    }

    #[test]
    fn test_get_active_actuators() {
        let (active_actuators_axial, active_actuators_tangent) =
            get_active_actuators(&vec![0, 1, 2], &vec![73, 75, 77]);

        assert_eq!(active_actuators_axial.len(), NUM_ACTIVE_ACTUATOR_AXIAL);
        assert_eq!(active_actuators_axial[0], 3);

        assert_eq!(active_actuators_tangent, vec![72, 74, 76]);
    }

    #[test]
    #[should_panic(expected = "Tangential hardpoints can only be [72, 74, 76] or [73, 75, 77].")]
    fn test_get_active_actuators_panic() {
        get_active_actuators(&vec![0, 1, 2], &vec![72, 73, 74]);
    }

    #[test]
    fn test_calc_kinetic_decoupling_matrix() {
        let loc_act_axial = read_file_cell_geom(Path::new("config/cell_geom.yaml")).0;
        let stiffness = read_file_stiffness(Path::new("config/stiff_matrix_surrogate.yaml"));

        let kdc = calc_kinetic_decoupling_matrix(
            &loc_act_axial,
            &vec![5, 15, 25],
            &vec![73, 75, 77],
            &stiffness,
        );

        assert_relative_eq!(kdc[(0, 0)], -21.7403059, epsilon = EPSILON);
        assert_relative_eq!(kdc[(0, 1)], -5.03360965, epsilon = EPSILON);
        assert_relative_eq!(kdc[(0, 2)], -3.38452371, epsilon = EPSILON);
        assert_relative_eq!(kdc[(71, 69)], -0.06759257, epsilon = EPSILON);
        assert_relative_eq!(kdc[(71, 70)], -0.05124909, epsilon = EPSILON);
        assert_relative_eq!(kdc[(71, 71)], 6.2396548, epsilon = EPSILON);
    }

    #[test]
    fn test_calc_cmd_delay_filter_params() {
        // Surrogate
        assert_relative_eq_vector(
            &calc_cmd_delay_filter_params(false, 20.0, false, 5),
            &vec![0.0, 0.0, 0.968, 0.032, 0.0, 0.0],
            EPSILON,
        );
        assert_relative_eq_vector(
            &calc_cmd_delay_filter_params(false, 20.0, true, 5),
            &vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            EPSILON,
        );

        // Mirror
        assert_relative_eq_vector(
            &calc_cmd_delay_filter_params(true, 20.0, false, 5),
            &vec![0.0, 0.014, 0.986, 0.0, 0.0, 0.0],
            EPSILON,
        );
    }

    #[test]
    fn test_test_check_hardpoints() {
        let loc_act_axial = read_file_cell_geom(Path::new("config/cell_geom.yaml")).0;

        // Good hardpoints
        assert!(check_hardpoints(&loc_act_axial, &vec![5, 15, 25], &vec![72, 74, 76]).is_ok());

        // Bad hardpoints
        assert!(check_hardpoints(&loc_act_axial, &vec![5, 15, 24], &vec![72, 74, 76]).is_err());
        assert!(check_hardpoints(&loc_act_axial, &vec![5, 15, 25], &vec![72, 73, 74]).is_err());
    }

    #[test]
    fn test_select_axial_hardpoints() {
        let loc_act_axial = read_file_cell_geom(Path::new("config/cell_geom.yaml")).0;

        assert_eq!(select_axial_hardpoints(&loc_act_axial, 4), vec![4, 14, 24]);
        assert_eq!(select_axial_hardpoints(&loc_act_axial, 15), vec![5, 15, 25]);
        assert_eq!(select_axial_hardpoints(&loc_act_axial, 26), vec![6, 16, 26]);
        assert_eq!(select_axial_hardpoints(&loc_act_axial, 0), vec![0, 10, 20]);
    }

    #[test]
    fn test_rigid_body_to_actuator_displacement() {
        let (loc_act_axial, loc_act_tangent, radius_act_tangent) =
            read_file_cell_geom(Path::new("config/cell_geom.yaml"));

        // Test (x, y, z)
        let displacments_xyz = rigid_body_to_actuator_displacement(
            &loc_act_axial,
            &loc_act_tangent,
            radius_act_tangent,
            1.0,
            2.0,
            3.0,
            0.0,
            0.0,
            0.0,
        );

        assert_eq!(displacments_xyz[0], -3.0);
        assert_eq!(displacments_xyz[71], -3.0);

        assert_relative_eq!(displacments_xyz[72], 1.0, epsilon = EPSILON);
        assert_relative_eq!(displacments_xyz[73], -1.2320508, epsilon = EPSILON);
        assert_relative_eq!(displacments_xyz[74], -2.2320508, epsilon = EPSILON);
        assert_relative_eq!(displacments_xyz[75], -1.0, epsilon = EPSILON);
        assert_relative_eq!(displacments_xyz[76], 1.2320508, epsilon = EPSILON);
        assert_relative_eq!(displacments_xyz[77], 2.2320508, epsilon = EPSILON);

        // Test (rx, ry, rz)
        let displacments_rxryrz = rigid_body_to_actuator_displacement(
            &loc_act_axial,
            &loc_act_tangent,
            radius_act_tangent,
            0.0,
            0.0,
            0.0,
            0.1,
            0.2,
            0.3,
        );

        assert_relative_eq!(displacments_rxryrz[0], -0.0591682, epsilon = EPSILON);
        assert_relative_eq!(displacments_rxryrz[1], 0.0148073, epsilon = EPSILON);
        assert_relative_eq!(displacments_rxryrz[2], 0.0881348, epsilon = EPSILON);
        assert_relative_eq!(displacments_rxryrz[69], -0.2079914, epsilon = EPSILON);
        assert_relative_eq!(displacments_rxryrz[70], -0.1690006, epsilon = EPSILON);
        assert_relative_eq!(displacments_rxryrz[71], -0.1096264, epsilon = EPSILON);

        assert_relative_eq!(displacments_rxryrz[72], -0.5260820, epsilon = EPSILON);
        assert_relative_eq!(displacments_rxryrz[73], -0.5260820, epsilon = EPSILON);

        // Test (x, y, z, rx, ry, rz)
        let displacments_xyzrxryrz = rigid_body_to_actuator_displacement(
            &loc_act_axial,
            &loc_act_tangent,
            radius_act_tangent,
            2.0,
            -1.0,
            0.5,
            -0.3,
            0.2,
            -0.1,
        );

        assert_relative_eq!(displacments_xyzrxryrz[0], -0.0595715, epsilon = EPSILON);
        assert_relative_eq!(displacments_xyzrxryrz[1], 0.0034852, epsilon = EPSILON);
        assert_relative_eq!(displacments_xyzrxryrz[2], 0.0445402, epsilon = EPSILON);

        assert_relative_eq!(displacments_xyzrxryrz[72], 2.1777224, epsilon = EPSILON);
        assert_relative_eq!(displacments_xyzrxryrz[73], 2.0437478, epsilon = EPSILON);
        assert_relative_eq!(displacments_xyzrxryrz[74], 0.0437478, epsilon = EPSILON);
    }

    #[test]
    fn test_hardpoint_to_rigid_body() {
        let (loc_act_axial, loc_act_tangent, radius_act_tangent) =
            read_file_cell_geom(Path::new("config/cell_geom.yaml"));

        let hardpoints = vec![5, 15, 25, 73, 75, 77];

        // Test the axial hardpoint displacements
        let (x, y, z, rx, ry, rz) = hardpoint_to_rigid_body(
            &loc_act_axial,
            &loc_act_tangent,
            radius_act_tangent,
            &hardpoints,
            &vec![0.001, 0.002, 0.004, 0.0, 0.0, 0.0],
            &vec![0.0; 6],
        )
        .unwrap();

        assert_eq!(x, 0.0);
        assert_eq!(y, 0.0);
        assert_relative_eq!(rz, 0.0, epsilon = EPSILON);

        assert_relative_eq!(z, -0.0023333, epsilon = EPSILON);
        assert_relative_eq!(rx, -0.0002082, epsilon = EPSILON);
        assert_relative_eq!(ry, -0.0010819, epsilon = EPSILON);

        // Test the tangent hardpoint displacements
        let (x, y, z, rx, ry, rz) = hardpoint_to_rigid_body(
            &loc_act_axial,
            &loc_act_tangent,
            radius_act_tangent,
            &hardpoints,
            &vec![0.0, 0.0, 0.0, 0.005, 0.002, 0.003],
            &vec![0.0, 0.0, 0.0, 0.002, 0.003, 0.005],
        )
        .unwrap();

        assert_eq!(z, 0.0);
        assert_eq!(rx, 0.0);
        assert_eq!(ry, 0.0);

        // Use the mm and urad here for the comparison with LabVIEW calculation
        assert_relative_eq!(x * 1e3, 0.9939971, epsilon = EPSILON);
        assert_relative_eq!(y * 1e3, -2.8881361, epsilon = EPSILON);
        assert_relative_eq!(rz * 1e6, 0.2126332, epsilon = EPSILON);

        // Test all
        let (x, y, z, rx, ry, rz) = hardpoint_to_rigid_body(
            &loc_act_axial,
            &loc_act_tangent,
            radius_act_tangent,
            &hardpoints,
            &vec![0.001, 0.002, 0.003, 0.005, 0.002, 0.003],
            &vec![0.003, 0.001, 0.005, 0.002, 0.003, 0.005],
        )
        .unwrap();

        // Use the mm and urad here for the comparison with LabVIEW calculation
        assert_relative_eq!(x * 1e3, 0.9939971, epsilon = EPSILON);
        assert_relative_eq!(y * 1e3, -2.8881361, epsilon = EPSILON);
        assert_relative_eq!(z * 1e3, 1.0, epsilon = EPSILON);
        assert_relative_eq!(rx * 1e6, 1249.2185882, epsilon = EPSILON);
        assert_relative_eq!(ry * 1e6, 0.0, epsilon = EPSILON);
        assert_relative_eq!(rz * 1e6, 0.2126332, epsilon = EPSILON);
    }

    #[test]
    fn test_rigid_body_movement() {
        // x, y, z in mm
        // rx, ry, rz in urad
        let list_test_data = vec![
            vec![3.0, 4.0, 5.0, 15.0, -5.0, -8.0],
            vec![3.0, -4.0, 5.0, 15.0, -5.0, 0.0],
            vec![-3.0, 4.0, 5.0, 15.0, -5.0, 8.0],
        ];

        list_test_data.iter().for_each(|data| {
            _verify_rigid_body_movement(data[0], data[1], data[2], data[3], data[4], data[5], 1e-2);
        });
    }

    fn _verify_rigid_body_movement(
        target_x: f64,
        target_y: f64,
        target_z: f64,
        target_rx: f64,
        target_ry: f64,
        target_rz: f64,
        epsilon: f64,
    ) {
        let (loc_act_axial, loc_act_tangent, radius_act_tangent) =
            read_file_cell_geom(Path::new("config/cell_geom.yaml"));

        let displacments_xyz = rigid_body_to_actuator_displacement(
            &loc_act_axial,
            &loc_act_tangent,
            radius_act_tangent,
            target_x * 1e-3,
            target_y * 1e-3,
            target_z * 1e-3,
            target_rx * 1e-6,
            target_ry * 1e-6,
            target_rz * 1e-6,
        );

        let hardpoints = vec![5, 15, 25, 73, 75, 77];
        let disp_hardpoint_current = hardpoints
            .iter()
            .map(|hardpoint| displacments_xyz[*hardpoint])
            .collect::<Vec<f64>>();

        let (x, y, z, rx, ry, rz) = hardpoint_to_rigid_body(
            &loc_act_axial,
            &loc_act_tangent,
            radius_act_tangent,
            &hardpoints,
            &disp_hardpoint_current,
            &vec![0.0; 6],
        )
        .unwrap();

        assert_relative_eq!(x * 1e3, target_x, epsilon = epsilon);
        assert_relative_eq!(y * 1e3, target_y, epsilon = epsilon);
        assert_relative_eq!(z * 1e3, target_z, epsilon = epsilon);
        assert_relative_eq!(rx * 1e6, target_rx, epsilon = epsilon);
        assert_relative_eq!(ry * 1e6, target_ry, epsilon = epsilon);
        assert_relative_eq!(rz * 1e6, target_rz, epsilon = epsilon);
    }
}
