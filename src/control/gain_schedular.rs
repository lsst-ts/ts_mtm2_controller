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

pub struct GainSchedular {
    // Rates of the ramping up process
    _rate_up_axial: f64,
    _rate_up_tangent: f64,
    // Rates of the ramping down process
    _rate_down_axial: f64,
    _rate_down_tangent: f64,
    // Gains
    _gain_axial: f64,
    _gain_tangent: f64,
    _min_gain_axial: f64,
    _min_gain_tangent: f64,
    // Sample settle time
    _max_sample_settle: i32,
    _sample_settle: i32,
    pub max_gain: f64,
}

impl GainSchedular {
    /// Gain schedular to tune the gains for the slewing, settling, and
    /// imaging processes.
    ///
    /// # Arguments
    /// * `min_gain_axial` - Minimum gain for the axial actuators. The value
    /// should be in (0.0, 1.0).
    /// * `min_gain_tangent` - Minimum gain for the tangent actuators. The value
    /// should be in (0.0, 1.0).
    /// * `num_sample_ramp_up` - Number of samples in the ramping up process.
    /// This value can not be 0.
    /// * `num_sample_ramp_down` - Number of samples in the ramping down
    /// process. This value can not be 0.
    /// * `max_sample_settle` - Maximum number of samples in the settling
    /// process. This value can not be 0.
    ///
    /// # Returns
    /// A new GainSchedular object.
    pub fn new(
        min_gain_axial: f64,
        min_gain_tangent: f64,
        num_sample_ramp_up: i32,
        num_sample_ramp_down: i32,
        max_sample_settle: i32,
    ) -> Self {
        let max_gain = 1.0;
        Self {
            _rate_up_axial: (max_gain - min_gain_axial) / (num_sample_ramp_up as f64),
            _rate_up_tangent: (max_gain - min_gain_tangent) / (num_sample_ramp_up as f64),

            _rate_down_axial: (min_gain_axial - max_gain) / (num_sample_ramp_down as f64),
            _rate_down_tangent: (min_gain_tangent - max_gain) / (num_sample_ramp_down as f64),

            _gain_axial: min_gain_axial,
            _gain_tangent: min_gain_tangent,

            _min_gain_axial: min_gain_axial,
            _min_gain_tangent: min_gain_tangent,

            _max_sample_settle: max_sample_settle,
            _sample_settle: max_sample_settle,

            max_gain: max_gain,
        }
    }

    /// Reset the internal data.
    pub fn reset(&mut self) {
        self._gain_axial = self._min_gain_axial;
        self._gain_tangent = self._min_gain_tangent;

        self._sample_settle = self._max_sample_settle;
    }

    /// Get the gain values.
    ///
    /// # Arguments
    /// * `is_in_position` - Mirror is in position or not.
    ///
    /// # Returns
    /// A tuple containing two `f64` values:
    /// - The first value is the gain of the axial actuators.
    /// - The second value is the gain of the tangent actuators.
    pub fn get_gain(&mut self, is_in_position: bool) -> (f64, f64) {
        // Return the current gains in the settling process at the settling
        // period
        if is_in_position && (self._sample_settle > 0) {
            self._sample_settle -= 1;
            return (self._gain_axial, self._gain_tangent);
        }

        // Ramp down the gains after the settling process.
        // We want to ramp down the gains to the minimum to filter out the high
        // frequency noise in the imaging process.
        // If the mirror is not in position, we will ramp up the gains to the
        // maximum in the slewing process.
        let is_ramp_down = is_in_position && (self._sample_settle == 0);

        // Because we will begin to ramp up the gains, reset the settling period
        if !is_in_position {
            self._sample_settle = self._max_sample_settle;
        }

        // Calculate the new gains
        let rate_axial = if is_ramp_down {
            self._rate_down_axial
        } else {
            self._rate_up_axial
        };
        let rate_tangent = if is_ramp_down {
            self._rate_down_tangent
        } else {
            self._rate_up_tangent
        };

        self._gain_axial = self.calc_next_gain(self._gain_axial, rate_axial, self._min_gain_axial);
        self._gain_tangent =
            self.calc_next_gain(self._gain_tangent, rate_tangent, self._min_gain_tangent);

        (self._gain_axial, self._gain_tangent)
    }

    /// Calculate the next gain. The output will be in [min, max].
    ///
    /// # Arguments
    /// * `gain_current` - Current gain.
    /// * `rate` - Rate of gain.
    /// * `min_gain` - Minimum gain.
    ///
    /// # Returns
    /// Next gain.
    fn calc_next_gain(&self, gain_current: f64, rate: f64, min_gain: f64) -> f64 {
        let gain_new = gain_current + rate;
        if gain_new >= self.max_gain {
            return self.max_gain;
        } else if gain_new <= min_gain {
            return min_gain;
        } else {
            return gain_new;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const MIN_GAIN_AXIAL: f64 = 0.1;
    const MIN_GAIN_TANGENT: f64 = 0.2;
    const NUM_SAMPLE_RAMP_UP: i32 = 5;
    const NUM_SAMPLE_RAMP_DOWN: i32 = 10;
    const MAX_SAMPLE_SETTLE: i32 = 5;

    fn create_gain_schedular() -> GainSchedular {
        GainSchedular::new(
            MIN_GAIN_AXIAL,
            MIN_GAIN_TANGENT,
            NUM_SAMPLE_RAMP_UP,
            NUM_SAMPLE_RAMP_DOWN,
            MAX_SAMPLE_SETTLE,
        )
    }

    #[test]
    fn test_calc_next_gain() {
        let schedular = create_gain_schedular();

        let gain_normal = schedular.calc_next_gain(0.1, 0.2, 0.05);
        assert_relative_eq!(gain_normal, 0.3);

        let gain_max = schedular.calc_next_gain(0.1, 1.2, 0.05);
        assert_eq!(gain_max, schedular.max_gain);

        let gain_min = schedular.calc_next_gain(0.1, -1.2, 0.05);
        assert_eq!(gain_min, 0.05);
    }

    #[test]
    fn test_reset() {
        let mut schedular = create_gain_schedular();

        // In position
        schedular.get_gain(true);

        schedular.reset();

        assert_eq!(schedular._sample_settle, MAX_SAMPLE_SETTLE);

        // Not in position
        for _ in 0..2 {
            schedular.get_gain(false);
        }

        schedular.reset();

        assert_eq!(schedular._gain_axial, MIN_GAIN_AXIAL);
        assert_eq!(schedular._gain_tangent, MIN_GAIN_TANGENT);
    }

    #[test]
    fn test_get_gain() {
        let mut schedular = create_gain_schedular();

        // In position in the initial beginning
        assert_eq!(schedular._sample_settle, MAX_SAMPLE_SETTLE);

        for _ in 0..(MAX_SAMPLE_SETTLE + 1) {
            let (gain_axial_0, gain_tangent_0) = schedular.get_gain(true);

            assert_eq!(gain_axial_0, MIN_GAIN_AXIAL);
            assert_eq!(gain_tangent_0, MIN_GAIN_TANGENT);
        }

        assert_eq!(schedular._sample_settle, 0);

        // Mirror is not in the position, ramp up the gains to the maximum
        let mut gain_axial_up = 0.0;
        let mut gain_tangent_up = 0.0;
        for _ in 0..NUM_SAMPLE_RAMP_UP {
            (gain_axial_up, gain_tangent_up) = schedular.get_gain(false);
        }

        assert_eq!(gain_axial_up, schedular.max_gain);
        assert_eq!(gain_tangent_up, schedular.max_gain);

        assert_eq!(schedular._sample_settle, MAX_SAMPLE_SETTLE);

        // Settling process
        let mut gain_axial_settle = 0.0;
        let mut gain_tangent_settle = 0.0;
        for _ in 0..MAX_SAMPLE_SETTLE {
            (gain_axial_settle, gain_tangent_settle) = schedular.get_gain(true);
        }

        assert_eq!(gain_axial_settle, schedular.max_gain);
        assert_eq!(gain_tangent_settle, schedular.max_gain);

        assert_eq!(schedular._sample_settle, 0);

        // Ramp down to the minimum
        let mut gain_axial_down = 0.0;
        let mut gain_tangent_down = 0.0;
        for _ in 0..NUM_SAMPLE_RAMP_DOWN {
            (gain_axial_down, gain_tangent_down) = schedular.get_gain(true);
        }

        const EPSILON: f64 = 1e-7;
        assert_relative_eq!(gain_axial_down, MIN_GAIN_AXIAL, epsilon = EPSILON);
        assert_relative_eq!(gain_tangent_down, MIN_GAIN_TANGENT, epsilon = EPSILON);

        assert_eq!(schedular._sample_settle, 0);
    }
}
