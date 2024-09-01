use nalgebra::{vector, DVector, SMatrix, SVector};
use serde::{Deserialize, Serialize};
use std::path::Path;

use crate::constants::{
    NUM_ACTUATOR, NUM_AXIAL_ACTUATOR, NUM_COLUMN_LUT_GRAVITY, NUM_LUT_TEMPERATURE,
    NUM_TANGENT_LINK, NUM_TEMPERATURE_RING,
};
use crate::utility::{read_file_lut_gravity, read_file_lut_temperature};

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub struct Lut {
    // Directory name.
    pub dir_name: String,
    // Reference angles of the gravity look-up table (LUT) in degrees.
    _ref_angles: Vec<f64>,
    // Lookup table for the gravity.
    _gravity: SMatrix<f64, NUM_ACTUATOR, NUM_COLUMN_LUT_GRAVITY>,
    // Lookup table for the temperature.
    _temperature: SMatrix<f64, NUM_AXIAL_ACTUATOR, NUM_LUT_TEMPERATURE>,
    // Temperature inversion matrix.
    _temp_inv: SMatrix<f64, NUM_LUT_TEMPERATURE, NUM_TEMPERATURE_RING>,
}

impl Lut {
    /// Get the look-up table (LUT).
    ///
    /// # Arguments
    /// * `dir_path` - LUT directory path.
    ///
    /// # Returns
    /// LUT.
    pub fn new(dir_path: &Path) -> Self {
        let (ref_angles, gravity) = Self::get_lut_gravity(dir_path);
        let temperature = Self::get_lut_temperature(dir_path);

        Self {
            dir_name: String::from(dir_path.to_str().expect(&format!(
                "Should be able to convert {:?} to a string",
                dir_path
            ))),
            _ref_angles: ref_angles,
            _gravity: gravity,
            _temperature: temperature,
            _temp_inv: Self::calc_temp_inv_matrix(),
        }
    }

    /// Get the lookup table for the gravity.
    ///
    /// # Arguments
    /// * `dir_path` - LUT directory path.
    ///
    /// # Returns
    /// Reference angles of the gravity look-up table (LUT) in degrees and the
    /// lookup table for the gravity.
    fn get_lut_gravity(
        dir_path: &Path,
    ) -> (Vec<f64>, SMatrix<f64, NUM_ACTUATOR, NUM_COLUMN_LUT_GRAVITY>) {
        let mut ref_angles = vec![0.0; NUM_COLUMN_LUT_GRAVITY];
        let mut matrix_lut: SMatrix<f64, NUM_ACTUATOR, NUM_COLUMN_LUT_GRAVITY> = SMatrix::zeros();

        let tables = ["F_E.csv", "F_0.csv", "F_A.csv", "F_F.csv"];
        tables.iter().for_each(|table: &&str| {
            let matrix = read_file_lut_gravity(dir_path.join(table).as_path());

            // Reference angles of the elevation LUT
            if table == &"F_E.csv" {
                ref_angles = matrix[0].clone();
            }

            // Skip the header
            for row in 1..matrix.len() {
                for col in 0..NUM_COLUMN_LUT_GRAVITY {
                    matrix_lut[(row - 1, col)] += matrix[row][col];
                }
            }
        });

        (ref_angles, matrix_lut)
    }

    /// Get the lookup table for the temperature.
    ///
    /// # Arguments
    /// * `dir_path` - LUT directory path.
    ///
    /// # Returns
    /// Lookup table for the temperature.
    fn get_lut_temperature(
        dir_path: &Path,
    ) -> SMatrix<f64, NUM_AXIAL_ACTUATOR, NUM_LUT_TEMPERATURE> {
        let tables = ["Tr.csv", "Tx.csv", "Ty.csv", "Tu.csv"];

        let mut lut: SMatrix<f64, NUM_AXIAL_ACTUATOR, NUM_LUT_TEMPERATURE> = SMatrix::zeros();
        for (idx, table) in tables.iter().enumerate() {
            let values = read_file_lut_temperature(dir_path.join(table).as_path());
            lut.set_column(idx, &SVector::from_iterator(values.iter().copied()));
        }

        lut
    }

    /// Calculate the temperature inversion matrix.
    ///
    /// # Notes
    /// Radial gradient: T(r) = Tr * r / R
    /// X gradient: T(x) = Tx * x / R = Tx * r * cos(theta) / R
    /// Y gradient: T(y) = Ty * y / R = Ty * r * sin(theta) / R
    /// Uniform bias: T(u) = Tu
    ///
    /// The matrix is: [r/R, r/R * cos(theta), r/R * sin(theta), 1], and we
    /// have: matrix * [Tr, Tx, Ty, Tu].T = [T1, T2, ..., T12].T
    ///
    /// # Returns
    /// Temperature inversion matrix.
    fn calc_temp_inv_matrix() -> SMatrix<f64, NUM_LUT_TEMPERATURE, NUM_TEMPERATURE_RING> {
        // Positions of 12 temperature sensors
        // Radial is in millimeter, theta is in degree
        let r_sensors = vector![937.514, 1634.744, 1734.82];
        let theta_sensors = vector![98.5, 278.5, 0.0, 180.0];

        let len_sensors = r_sensors.len();
        let len_theta_sensors = theta_sensors.len();
        let mut r_normalized = DVector::from_element(len_sensors * len_theta_sensors, 0.0);
        for idx in 0..len_theta_sensors {
            r_normalized
                .rows_mut(idx * len_sensors, len_sensors)
                .copy_from(&r_sensors);
        }
        r_normalized = r_normalized / r_sensors.max();

        let theta = theta_sensors.kronecker(&DVector::from_element(len_sensors, 1.0));

        let r_normalized_cos =
            r_normalized.component_mul(&theta.map(|x: f64| x.to_radians().cos()));
        let r_normalized_sin =
            r_normalized.component_mul(&theta.map(|x: f64| x.to_radians().sin()));

        let len_r_normalized = r_normalized.len();
        let mut matrix: SMatrix<f64, NUM_TEMPERATURE_RING, NUM_LUT_TEMPERATURE> = SMatrix::zeros();
        matrix.set_column(0, &r_normalized);
        matrix.set_column(1, &r_normalized_cos);
        matrix.set_column(2, &r_normalized_sin);
        matrix.set_column(3, &DVector::from_element(len_r_normalized, 1.0));

        // Do the pseudo-inverse
        matrix
            .pseudo_inverse(f64::EPSILON)
            .expect("Should be able to calculate the temperature inversion matrix.")
    }

    /// Get the look-up table (LUT) forces.
    ///
    /// # Arguments
    /// * `lut_angle` - Angle used to calculate the LUT forces in degree.
    /// * `ring_temperature` - Ring temperature in degree Celsius. The order is:
    /// [LG2-1, LG2-2, LG2-3, LG2-4, LG3-1, LG3-2, LG3-3, LG3-4, LG4-1, LG4-2,
    /// LG4-3, LG4-4].
    /// * `ref_temperature` - Reference temperature in degree Celsius. The order
    /// is the same as the ring temperature.
    /// * `enable_lut_temperature` - Enable the temperature LUT or not. If it is
    /// false, the temperature-related forces will be zero.
    ///
    /// # Returns
    /// A tuple of two vectors: gravity correction forces and temperature
    /// correction forces in Newton.
    pub fn get_lut_forces(
        &self,
        lut_angle: f64,
        ring_temperature: &Vec<f64>,
        ref_temperature: &Vec<f64>,
        enable_lut_temperature: bool,
    ) -> (Vec<f64>, Vec<f64>) {
        let (forces_gravity, mut forces_temperature) = Self::calc_look_up_forces(
            lut_angle,
            &self._ref_angles,
            &self._gravity,
            ring_temperature,
            ref_temperature,
            &self._temp_inv,
            &self._temperature,
            enable_lut_temperature,
        );

        // There is no temperature force for the tangent links
        forces_temperature.extend(vec![0.0; NUM_TANGENT_LINK]);

        (forces_gravity, forces_temperature)
    }

    /// Calculate look-up table (LUT) forces using current system state (position
    /// and temperature).
    ///
    /// # Arguments
    /// * `lut_angle` - Angle used to calculate the LUT forces in degree.
    /// * `ref_angles` - Reference angles of the elevation LUT.
    /// * `lut_gravity` - Gravity LUT.
    /// * `ring_temperature` - Ring temperature in degree Celsius. The order is:
    /// [LG2-1, LG2-2, LG2-3, LG2-4, LG3-1, LG3-2, LG3-3, LG3-4, LG4-1, LG4-2,
    /// LG4-3, LG4-4].
    /// * `ref_temperature` - Reference temperature in degree Celsius. The order is
    /// the same as the ring temperature.
    /// * `temp_inv` - Temperature inversion matrix.
    /// * `lut_temperature` - Temperature LUT.
    /// * `enable_lut_temperature` - Enable the temperature LUT or not. If it is
    /// false, the temperature-related forces will be zero.
    ///
    /// # Returns
    /// A tuple of two vectors: gravity correction forces and temperature correction
    /// forces in Newton.
    fn calc_look_up_forces(
        lut_angle: f64,
        ref_angles: &Vec<f64>,
        lut_gravity: &SMatrix<f64, NUM_ACTUATOR, NUM_COLUMN_LUT_GRAVITY>,
        ring_temperature: &Vec<f64>,
        ref_temperature: &Vec<f64>,
        temp_inv: &SMatrix<f64, NUM_LUT_TEMPERATURE, NUM_TEMPERATURE_RING>,
        lut_temperature: &SMatrix<f64, NUM_AXIAL_ACTUATOR, NUM_LUT_TEMPERATURE>,
        enable_lut_temperature: bool,
    ) -> (Vec<f64>, Vec<f64>) {
        let forces_gravity = Self::calc_look_up_forces_gravity(lut_angle, ref_angles, lut_gravity);
        if !enable_lut_temperature {
            return (forces_gravity, vec![0.0; NUM_AXIAL_ACTUATOR]);
        }

        // Rearrange the order of ring temperatures to the order of LUT calculation
        let order = [1, 2, 3, 4, 9, 8, 5, 6, 7, 11, 10, 0];
        let (reordered_temperatures, reordered_ref_temperature) = order
            .iter()
            .map(|idx| (ring_temperature[*idx], ref_temperature[*idx]))
            .unzip();

        let forces_temperature: Vec<f64> = Self::calc_look_up_forces_temperature(
            &reordered_temperatures,
            &reordered_ref_temperature,
            temp_inv,
            lut_temperature,
        );

        (forces_gravity, forces_temperature)
    }

    /// Calculate the temperature-related forces based on the look-up table
    /// (LUT) in Newton.
    ///
    /// # Notes
    /// Only the axial actuators have this correction.
    ///
    /// # Arguments
    /// * `temperature` - Ring temperature in degree Celsius. The order is:
    /// [LG2-2, LG2-3, LG2-4, LG3-1, LG4-2, LG4-1, LG3-2, LG3-3, LG3-4, LG4-4,
    /// LG4-3, LG2-1].
    /// * `ref_temperature` - Reference temperature in degree Celsius. The order is
    /// the same as the temperature.
    /// * `temp_inv` - Temperature inversion matrix.
    /// * `lut` - Temperature LUT: [Tr, Tx, Ty, Tu].
    ///
    /// # Returns
    /// Temperature correction force in Newton.
    fn calc_look_up_forces_temperature(
        temperature: &Vec<f64>,
        ref_temperature: &Vec<f64>,
        temp_inv: &SMatrix<f64, NUM_LUT_TEMPERATURE, NUM_TEMPERATURE_RING>,
        lut: &SMatrix<f64, NUM_AXIAL_ACTUATOR, NUM_LUT_TEMPERATURE>,
    ) -> Vec<f64> {
        let delta_temperature = SVector::from_iterator(
            temperature
                .iter()
                .zip(ref_temperature.iter())
                .map(|(x, ref_x)| x - ref_x),
        );
        let tcoef = temp_inv * delta_temperature;
        let correction = lut * tcoef;

        correction.iter().copied().collect()
    }

    /// Calculate the gravity-related forces based on the look-up table (LUT) in
    /// Newton.
    ///
    /// # Arguments
    /// * `lut_angle` - Angle used to calculate the LUT forces of gravity component
    /// in degree.
    /// * `ref_angles` - Reference angles of the elevation LUT.
    /// * `lut` - Gravity LUT. The row is the actuator index and the column is the
    /// correction under each reference angle.
    ///
    /// # Returns
    /// Gravity correction forces in Newton.
    fn calc_look_up_forces_gravity(
        lut_angle: f64,
        ref_angles: &Vec<f64>,
        lut: &SMatrix<f64, NUM_ACTUATOR, NUM_COLUMN_LUT_GRAVITY>,
    ) -> Vec<f64> {
        // Search the nearest angles in the LUT
        let mut left = 0;
        let mut right = 0;
        for idx in 0..(NUM_COLUMN_LUT_GRAVITY - 1) {
            if lut_angle >= ref_angles[idx] && lut_angle <= ref_angles[idx + 1] {
                left = idx;
                right = idx + 1;
                break;
            }
        }

        assert!(left != right, "The angle is out of the LUT range.");

        // Do the interpolation
        let forces = lut.column(left).lerp(
            &lut.column(right),
            (lut_angle - ref_angles[left]) / (ref_angles[right] - ref_angles[left]),
        );

        forces.iter().copied().collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    use crate::mock::mock_constants::{PLANT_TEMPERATURE_HIGH, PLANT_TEMPERATURE_LOW};
    use crate::utility::assert_relative_eq_vector;

    const EPSILON: f64 = 1e-7;

    #[test]
    fn test_calc_temp_inv_matrix() {
        let matrix = Lut::calc_temp_inv_matrix();

        assert_relative_eq!(matrix[(0, 0)], -0.5726841, epsilon = EPSILON);
        assert_relative_eq!(matrix[(0, 1)], 0.2288206, epsilon = EPSILON);
        assert_relative_eq!(matrix[(0, 2)], 0.3438635, epsilon = EPSILON);
        assert_relative_eq!(matrix[(0, 3)], -0.5726841, epsilon = EPSILON);
    }

    #[test]
    fn test_calc_look_up_forces_temperature() {
        let correction = Lut::calc_look_up_forces_temperature(
            &get_temperature(),
            &vec![21.0; NUM_TEMPERATURE_RING],
            &Lut::calc_temp_inv_matrix(),
            &Lut::get_lut_temperature(Path::new("config/lut/handling")),
        );

        assert_relative_eq_vector(
            &correction[0..3],
            &[3.5022743, -4.3345335, -0.3062321],
            EPSILON,
        );
    }

    fn get_temperature() -> Vec<f64> {
        let mut temperature = vec![PLANT_TEMPERATURE_LOW; NUM_TEMPERATURE_RING];

        // Indexes of the ring temperatures that have a higher temperature.
        let indexes: [usize; 4] = [2, 7, 8, 9];
        indexes.iter().for_each(|&idx| {
            temperature[idx] = PLANT_TEMPERATURE_HIGH;
        });

        temperature
    }

    #[test]
    fn test_calc_look_up_forces_gravity() {
        let (ref_angles, lut) = Lut::get_lut_gravity(Path::new("config/lut/handling"));

        // Random angle
        let mut forces = Lut::calc_look_up_forces_gravity(59.06, &ref_angles, &lut);

        assert_eq!(forces.len(), NUM_ACTUATOR);
        assert_relative_eq_vector(
            &forces[0..3],
            &[126.8558538, 170.3816964, 161.0958779],
            EPSILON,
        );

        // The first angle
        let limits = [193.86808, 190.83808, 195.02876];
        forces = Lut::calc_look_up_forces_gravity(ref_angles[0], &ref_angles, &lut);
        assert_relative_eq_vector(&forces[0..3], &limits, EPSILON);

        assert_eq!(forces[NUM_ACTUATOR - 1], -28.964);

        // The last angle
        forces = Lut::calc_look_up_forces_gravity(
            ref_angles[NUM_COLUMN_LUT_GRAVITY - 1],
            &ref_angles,
            &lut,
        );
        assert_relative_eq_vector(&forces[0..3], &limits, EPSILON);

        assert_eq!(forces[NUM_ACTUATOR - 2], -9.0112);
    }

    #[test]
    #[should_panic(expected = "The angle is out of the LUT range.")]
    fn test_calc_look_up_forces_gravity_panic_1() {
        let (ref_angles, lut) = Lut::get_lut_gravity(Path::new("config/lut/handling"));

        Lut::calc_look_up_forces_gravity(ref_angles[0] - 1.0, &ref_angles, &lut);
    }

    #[test]
    #[should_panic(expected = "The angle is out of the LUT range.")]
    fn test_calc_look_up_forces_gravity_panic_2() {
        let (ref_angles, lut) = Lut::get_lut_gravity(Path::new("config/lut/handling"));

        Lut::calc_look_up_forces_gravity(
            ref_angles[NUM_COLUMN_LUT_GRAVITY - 1] + 1.0,
            &ref_angles,
            &lut,
        );
    }

    #[test]
    fn test_calc_look_up_forces() {
        let (ref_angles, lut_gravity) = Lut::get_lut_gravity(Path::new("config/lut/handling"));
        let lut_temperature = Lut::get_lut_temperature(Path::new("config/lut/handling"));

        let (forces_gravity, forces_temperature) = Lut::calc_look_up_forces(
            59.06,
            &ref_angles,
            &lut_gravity,
            &get_ring_temperature(),
            &vec![21.0; NUM_TEMPERATURE_RING],
            &Lut::calc_temp_inv_matrix(),
            &lut_temperature,
            true,
        );

        assert_eq!(forces_gravity.len(), NUM_ACTUATOR);
        assert_eq!(forces_temperature.len(), NUM_AXIAL_ACTUATOR);

        assert_relative_eq!(forces_gravity[0], 126.8558538, epsilon = EPSILON);
        assert_relative_eq!(forces_temperature[0], 3.5022743, epsilon = EPSILON);

        // Tangent link A1
        assert_eq!(forces_gravity[NUM_AXIAL_ACTUATOR], 16.113);

        // Tangent link A2
        assert_eq!(forces_gravity[NUM_AXIAL_ACTUATOR + 1], -131.72);
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
    fn test_calc_look_up_forces_disable_lut_temperature() {
        let (ref_angles, lut_gravity) = Lut::get_lut_gravity(Path::new("config/lut/handling"));
        let lut_temperature = Lut::get_lut_temperature(Path::new("config/lut/handling"));

        let forces_temperature = Lut::calc_look_up_forces(
            59.06,
            &ref_angles,
            &lut_gravity,
            &get_ring_temperature(),
            &vec![21.0; NUM_TEMPERATURE_RING],
            &Lut::calc_temp_inv_matrix(),
            &lut_temperature,
            false,
        )
        .1;

        assert_eq!(forces_temperature, vec![0.0; NUM_AXIAL_ACTUATOR]);
    }
}
