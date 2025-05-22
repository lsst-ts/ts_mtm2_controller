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

use crate::constants::NUM_HARDPOINTS_AXIAL;

pub struct DeadbandControl {
    _threshold_lower: f64,
    _threshold_upper: f64,
    _keep_tracking: bool,
    _hardpoint_error_track: Vec<f64>,
}

impl DeadbandControl {
    /// Deadband control to select the force error of hardpoints.
    ///
    /// # Notes
    /// Force error values are latched when the largest force error falls below
    /// the low threshold and the output remains constant until the largest
    /// force error exceeds the high threshold.
    ///
    /// # Arguments
    /// * `threshold_lower` - Lower threshold in Newton.
    /// * `threshold_upper` - Upper threshold in Newton.
    ///
    /// # Returns
    /// A new DeadbandControl object.
    pub fn new(threshold_lower: f64, threshold_upper: f64) -> Self {
        Self {
            _threshold_lower: threshold_lower,
            _threshold_upper: threshold_upper,
            _keep_tracking: false,
            _hardpoint_error_track: vec![0.0; NUM_HARDPOINTS_AXIAL],
        }
    }

    /// Reset the deadband.
    ///
    /// # Arguments
    /// * `reset_all` - Reset all of the internal data or not.
    pub fn reset(&mut self, reset_all: bool) {
        self._keep_tracking = false;

        if reset_all {
            self._hardpoint_error_track = vec![0.0; NUM_HARDPOINTS_AXIAL];
        }
    }

    /// Select the hardpoint error.
    ///
    /// # Arguments
    /// * `hardpoint_error` - Force error of the hardpoints in Newton.
    /// * `is_enabled` - Enable the deadzone or not.
    ///
    /// # Returns
    /// Selected hardpoint error.
    ///
    /// # Panics
    /// If the size of hardpoint error is not 3.
    pub fn select(&mut self, hardpoint_error: &[f64], is_enabled: bool) -> Vec<f64> {
        // Check the size of hardpoint error
        assert!(
            hardpoint_error.len() == NUM_HARDPOINTS_AXIAL,
            "Size of hardpoint error should be {NUM_HARDPOINTS_AXIAL}."
        );

        let threshold = if self._keep_tracking {
            self._threshold_upper
        } else {
            self._threshold_lower
        };
        let keep_tracking = is_enabled
            && (hardpoint_error
                .iter()
                .fold(0.0_f64, |max: f64, &val| max.max(val.abs()))
                < threshold);

        if !(keep_tracking && self._keep_tracking) {
            self._hardpoint_error_track = hardpoint_error.to_vec();
        }

        self._keep_tracking = keep_tracking;

        self._hardpoint_error_track.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_deadband_control() -> DeadbandControl {
        DeadbandControl::new(1.0, 2.0)
    }

    #[test]
    fn test_new() {
        let deadband_control = create_deadband_control();

        assert_eq!(deadband_control._threshold_lower, 1.0);
        assert_eq!(deadband_control._threshold_upper, 2.0);
        assert_eq!(deadband_control._keep_tracking, false);
        assert_eq!(
            deadband_control._hardpoint_error_track,
            vec![0.0; NUM_HARDPOINTS_AXIAL]
        );
    }

    #[test]
    #[should_panic(expected = "Size of hardpoint error should be 3.")]
    fn test_select_panic() {
        let mut deadband_control = create_deadband_control();
        deadband_control.select(&vec![0.0; 2], false);
    }

    #[test]
    fn test_select_disabled() {
        let mut deadband_control = create_deadband_control();

        let hardpoint_error = vec![0.1, 0.2, 0.0];
        let hardpoint_error_selected = deadband_control.select(&hardpoint_error, false);

        assert_eq!(hardpoint_error_selected, hardpoint_error);
        assert_eq!(deadband_control._keep_tracking, false);
    }

    #[test]
    fn test_select_enabled() {
        let mut deadband_control = create_deadband_control();

        // Latch the hardpoint error
        let hardpoint_error_1 = vec![0.1, 0.2, 0.0];
        let hardpoint_error_selected_1 = deadband_control.select(&hardpoint_error_1, true);

        assert_eq!(hardpoint_error_selected_1, hardpoint_error_1);
        assert_eq!(deadband_control._keep_tracking, true);

        // Use the latched hardpoint error
        let hardpoint_error_2 = vec![0.3, 0.4, 0.0];
        let hardpoint_error_selected_2 = deadband_control.select(&hardpoint_error_2, true);

        assert_eq!(hardpoint_error_selected_2, hardpoint_error_1);
        assert_eq!(deadband_control._keep_tracking, true);

        // Still use the latched hardpoint error because the maximum value is
        // still lower than the upper limit
        let hardpoint_error_3 = vec![-1.3, 0.4, 0.0];
        let hardpoint_error_selected_3 = deadband_control.select(&hardpoint_error_3, true);

        assert_eq!(hardpoint_error_selected_3, hardpoint_error_1);
        assert_eq!(deadband_control._keep_tracking, true);

        // The latch is broken because the maximum error is higher than the
        // upper threshold
        let hardpoint_error_4 = vec![-2.1, 0.4, 0.0];
        let hardpoint_error_selected_4 = deadband_control.select(&hardpoint_error_4, true);

        assert_eq!(hardpoint_error_selected_4, hardpoint_error_4);
        assert_eq!(deadband_control._keep_tracking, false);

        // After the latch is broken, we need to wait until the maximum error
        // is lower than the lower threshold
        let hardpoint_error_5 = vec![-1.3, 0.4, 0.0];
        let hardpoint_error_selected_5 = deadband_control.select(&hardpoint_error_5, true);

        assert_eq!(hardpoint_error_selected_5, hardpoint_error_5);
        assert_eq!(deadband_control._keep_tracking, false);

        // Latch the hardpoint error again
        let hardpoint_error_6 = vec![-0.3, -0.4, 0.0];
        let hardpoint_error_selected_6 = deadband_control.select(&hardpoint_error_6, true);

        assert_eq!(hardpoint_error_selected_6, hardpoint_error_6);
        assert_eq!(deadband_control._keep_tracking, true);
    }

    #[test]
    fn test_select_zero_threshold() {
        let mut deadband_control = DeadbandControl::new(0.0, 0.0);

        // Disabled
        let hardpoint_error_1 = vec![0.1, 0.2, 0.0];
        let hardpoint_error_selected_1 = deadband_control.select(&hardpoint_error_1, false);

        assert_eq!(hardpoint_error_selected_1, hardpoint_error_1);
        assert_eq!(deadband_control._keep_tracking, false);

        // Enabled
        let hardpoint_error_2 = vec![0.2, 0.3, 0.0];
        let hardpoint_error_selected_2 = deadband_control.select(&hardpoint_error_2, true);

        assert_eq!(hardpoint_error_selected_2, hardpoint_error_2);
        assert_eq!(deadband_control._keep_tracking, false);
    }

    #[test]
    fn test_reset() {
        let mut deadband_control = create_deadband_control();

        let hardpoint_error = vec![0.1, 0.2, 0.0];
        deadband_control.select(&hardpoint_error, true);

        // Do not reset all
        deadband_control.reset(false);

        assert_eq!(deadband_control._keep_tracking, false);
        assert_eq!(deadband_control._hardpoint_error_track, hardpoint_error);

        // Reset all
        deadband_control.reset(true);
        assert_eq!(
            deadband_control._hardpoint_error_track,
            vec![0.0; NUM_HARDPOINTS_AXIAL]
        );
    }
}
