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

use crate::utility::get_parameter;

pub struct Actuator {
    // Actuator model
    pub model: String,
    pub actuator_serial_number: String,
    // Load cell
    pub cell_serial_number: String,
    // Sensitivity (mV/V)
    _cell_sensitivity: f64,
    // Encoder limit
    _encoder_limit_extend: i32,
    _encoder_limit_retract: i32,
    // Displacement data (mm)
    _displacement_limit_extend: f64,
    _displacement_limit_retract: f64,
    _displacement_range: f64,
    // Displacement gain
    // Step (mm/step)
    pub gain_step_to_mm: f64,
    // Encoder (count/step)
    pub gain_step_to_count: f64,
}

impl Actuator {
    /// Create a new actuator instance.
    ///
    /// # Arguments
    /// * `filepath` - Path to the actuator configuration file.
    ///
    /// # Returns
    /// A new actuator instance.
    pub fn new(filepath: &Path) -> Self {
        let m_to_mm = 1000.0;

        Self {
            model: get_parameter(filepath, "model"),
            actuator_serial_number: get_parameter(filepath, "actuator_serial_number"),

            cell_serial_number: get_parameter(filepath, "cell_serial_number"),
            _cell_sensitivity: get_parameter(filepath, "cell_sensitivity"),

            _encoder_limit_extend: get_parameter(filepath, "encoder_limit_extend"),
            _encoder_limit_retract: get_parameter(filepath, "encoder_limit_retract"),

            _displacement_limit_extend: m_to_mm
                * get_parameter::<f64>(filepath, "displacement_limit_extend"),
            _displacement_limit_retract: m_to_mm
                * get_parameter::<f64>(filepath, "displacement_limit_retract"),
            _displacement_range: m_to_mm * get_parameter::<f64>(filepath, "displacement_range"),

            gain_step_to_mm: get_parameter(filepath, "gain_step_to_mm"),
            gain_step_to_count: get_parameter(filepath, "gain_step_to_count"),
        }
    }

    /// The encoder value is out of limit or not.
    ///
    /// # Arguments
    /// * `value` - Encoder value to check.
    ///
    /// # Returns
    /// True if the encoder value is out of limit. Otherwise, False.
    pub fn is_encoder_out_limit(&self, value: i32) -> bool {
        (value < self._encoder_limit_retract) || (value > self._encoder_limit_extend)
    }

    /// Convert encoder value to step and position.
    ///
    /// # Arguments
    /// * `value` - Encoder value to convert.
    ///
    /// # Returns
    /// A tuple containing the step and position in millimeter.
    pub fn encoder_to_step_and_position(&self, value: i32) -> (i32, f64) {
        let step = (value as f64) / self.gain_step_to_count;
        let position = step * self.gain_step_to_mm;

        (step as i32, position)
    }

    /// Convert the displacement in millimeter to step.
    ///
    /// # Arguments
    /// * `displacement` - Displacement in millimeter to convert.
    ///
    /// # Returns
    /// The corresponding step value.
    pub fn displacement_to_step(&self, displacement: f64) -> i32 {
        (displacement / self.gain_step_to_mm) as i32
    }

    /// Create a vector of actuators from a cell mapping file. This function
    /// expects the actuator files to be located in a subdirectory named
    /// "actuator" from the directory of the mapping file.
    ///
    /// # Arguments
    /// * `filepath` - Path to the cell mapping file.
    ///
    /// # Returns
    /// A vector of `Actuator` instances.
    pub fn from_cell_mapping_file(filepath: &Path) -> Vec<Self> {
        let rings = ["B", "C", "D", "A"];
        let nums = [30, 24, 18, 6];

        let dir_actuator = filepath
            .parent()
            .expect("Should get the parent directory of mapping file")
            .join("actuator");

        let mut actuators = Vec::new();
        for (ring, num) in rings.iter().zip(nums.iter()) {
            for idx in 0..num.to_owned() {
                let name = format!("{}{}", ring, idx + 1);
                let actuator_file: String = get_parameter(&filepath, &name);
                actuators.push(Self::new(&dir_actuator.join(actuator_file)));
            }
        }

        actuators
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use approx::assert_relative_eq;

    use crate::constants::NUM_ACTUATOR;

    const EPSILON: f64 = 1e-7;

    fn create_actuator() -> Actuator {
        let filepath = Path::new("config/cell/actuator/A_Model_14902-1000_SN_001.yaml");
        Actuator::new(filepath)
    }

    #[test]
    fn test_new() {
        let actuator = create_actuator();

        assert_eq!(actuator.model, "14902-1000");
        assert_eq!(actuator.actuator_serial_number, "001");

        assert_eq!(actuator.cell_serial_number, "748512");
        assert_eq!(actuator._cell_sensitivity, 1.60926);

        assert_eq!(actuator._encoder_limit_extend, 1719494);
        assert_eq!(actuator._encoder_limit_retract, -1719494);

        assert_relative_eq!(
            actuator._displacement_limit_extend,
            10.3604,
            epsilon = EPSILON
        );
        assert_relative_eq!(
            actuator._displacement_limit_retract,
            1.9786,
            epsilon = EPSILON
        );
        assert_relative_eq!(actuator._displacement_range, 8.3818, epsilon = EPSILON);

        assert_eq!(actuator.gain_step_to_mm, 1.9967536601e-05);
        assert_eq!(actuator.gain_step_to_count, 8.19562539783576);
    }

    #[test]
    fn test_is_encoder_out_limit() {
        let actuator = create_actuator();

        // Out of limit
        assert!(actuator.is_encoder_out_limit(actuator._encoder_limit_extend + 1));
        assert!(actuator.is_encoder_out_limit(actuator._encoder_limit_retract - 1));

        // In limit
        assert!(!actuator.is_encoder_out_limit(0));
        assert!(!actuator.is_encoder_out_limit(actuator._encoder_limit_extend));
        assert!(!actuator.is_encoder_out_limit(actuator._encoder_limit_retract));
    }

    #[test]
    fn test_encoder_to_step_and_position() {
        let actuator = create_actuator();

        let (step, position) = actuator.encoder_to_step_and_position(0);
        assert_eq!(step, 0);
        assert_relative_eq!(position, 0.0, epsilon = EPSILON);

        let (step, position) = actuator.encoder_to_step_and_position(100);
        assert_eq!(step, 12);
        assert_relative_eq!(position, 0.0002436, epsilon = EPSILON);

        let (step, position) = actuator.encoder_to_step_and_position(-100);
        assert_eq!(step, -12);
        assert_relative_eq!(position, -0.0002436, epsilon = EPSILON);
    }

    #[test]
    fn test_displacement_to_step() {
        let actuator = create_actuator();

        assert_eq!(actuator.displacement_to_step(0.0), 0);
        assert_eq!(actuator.displacement_to_step(actuator.gain_step_to_mm), 1);
        assert_eq!(actuator.displacement_to_step(-actuator.gain_step_to_mm), -1);
    }

    #[test]
    fn test_from_cell_mapping_file() {
        let actuators =
            Actuator::from_cell_mapping_file(Path::new("config/cell/cell_actuator_mapping.yaml"));

        assert_eq!(actuators.len(), NUM_ACTUATOR);
        assert_eq!(actuators[NUM_ACTUATOR - 1].model, "14903-1000");
        assert_eq!(actuators[NUM_ACTUATOR - 1].actuator_serial_number, "004");
        assert_eq!(actuators[NUM_ACTUATOR - 1].cell_serial_number, "62434881");
    }
}
