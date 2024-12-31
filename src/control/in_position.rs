use std::collections::VecDeque;

use crate::constants::{NUM_ACTIVE_ACTUATOR, NUM_ACTIVE_ACTUATOR_AXIAL};

pub struct InPosition {
    _queue: VecDeque<Vec<f64>>,
    _running_sum: Vec<f64>,
    // Squared threshold of the force error of axial actuator in square Newton.
    pub threshold_squared_axial: f64,
    // Squared threshold of the force error of tangent actuator in square
    // Newton.
    pub threshold_squared_tangent: f64,
}

impl InPosition {
    /// InPosition class to determine if the mirror is in position.
    ///
    /// # Arguments
    /// * `window_size` - Window size in second.
    /// * `control_frequency` - Control frequency in Hz.
    /// * `threshold_axial` - Threshold of the force error of axial actuator in
    /// Newton.
    /// * `threshold_tangent` - Threshold of the force error of tangent actuator
    /// in Newton.
    ///
    /// # Returns
    /// A new InPosition object.
    pub fn new(
        window_size: f64,
        control_frequency: f64,
        threshold_axial: f64,
        threshold_tangent: f64,
    ) -> Self {
        let num_row = (window_size * control_frequency) as usize;

        let mut queue: VecDeque<Vec<f64>> = VecDeque::with_capacity(num_row);
        // Initialize each row with a VecDeque containing initial_cols_per_row
        // elements
        for _ in 0..num_row {
            queue.push_back(Vec::with_capacity(NUM_ACTIVE_ACTUATOR));
        }

        Self {
            _queue: queue,
            _running_sum: vec![0.0; NUM_ACTIVE_ACTUATOR],
            threshold_squared_axial: threshold_axial.powi(2),
            threshold_squared_tangent: threshold_tangent.powi(2),
        }
    }

    /// Mirror is in position or not based on the thresholds of actuator force
    /// error.
    ///
    /// # Arguments
    /// * force_error - Force error of the 72 active actuators in Newton.
    ///
    /// # Returns
    /// Return true if the mirror is in position. Otherwise, false.
    ///
    /// # Panics
    /// If the size of force error is not 72.
    pub fn is_in_position(&mut self, force_error: &Vec<f64>) -> bool {
        // Check the size of force error
        assert!(
            force_error.len() == NUM_ACTIVE_ACTUATOR,
            "Size of force error should be {NUM_ACTIVE_ACTUATOR}."
        );

        // Always pop out the earliest value. Since it might be empty at the
        // beginning, we need to check if it is empty. If not, minus the
        // earliest value in the running sum.
        if let Some(earliest) = self._queue.pop_front() {
            if !earliest.is_empty() {
                // Minus the earliest value in the running sum
                self._running_sum
                    .iter_mut()
                    .zip(earliest.iter())
                    .for_each(|(sum, val)| *sum -= val);
            }
        }

        // Square the force error
        let force_error_square: Vec<f64> = force_error.iter().map(|val| val.powi(2)).collect();

        // Put the new value into queue and update the running sum
        self._queue.push_back(force_error_square.clone());
        self._running_sum
            .iter_mut()
            .zip(force_error_square.iter())
            .for_each(|(sum, val)| *sum += val);

        // Check if the running sum is within the threshold
        let num_queue = self._queue.len();

        let in_position_axial = self._running_sum[..NUM_ACTIVE_ACTUATOR_AXIAL]
            .iter()
            .all(|&val| val < self.threshold_squared_axial * (num_queue as f64));
        let in_position_tangent = self._running_sum[NUM_ACTIVE_ACTUATOR_AXIAL..]
            .iter()
            .all(|&val| val < self.threshold_squared_tangent * (num_queue as f64));

        in_position_axial && in_position_tangent
    }

    /// Reset the internal data.
    pub fn reset(&mut self) {
        self._queue.iter_mut().for_each(|vector| vector.clear());
        self._running_sum = vec![0.0; NUM_ACTIVE_ACTUATOR];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn create_in_position() -> InPosition {
        InPosition::new(1.0, 2.0, 2.0, 3.0)
    }

    #[test]
    fn test_new() {
        let in_position = create_in_position();

        assert_eq!(in_position.threshold_squared_axial, 4.0);
        assert_eq!(in_position.threshold_squared_tangent, 9.0);
        check_queue_size(&in_position);
    }

    fn check_queue_size(in_position: &InPosition) {
        assert_eq!(in_position._queue.len(), 2);
        assert_eq!(in_position._queue.capacity(), 2);
    }

    #[test]
    #[should_panic(expected = "Size of force error should be 72.")]
    fn test_is_in_position_panic() {
        let mut in_pointion = create_in_position();
        in_pointion.is_in_position(&vec![1.0; 10]);
    }

    #[test]
    fn test_is_in_position() {
        let mut in_position = create_in_position();

        // First time
        assert!(is_in_position(&mut in_position, 0.0, 0.0));
        check_queue_size(&in_position);

        // Second time
        assert!(is_in_position(&mut in_position, 1.0, 1.0));
        assert_eq!(in_position._running_sum[0], 1.0);
        check_queue_size(&in_position);

        // Third time
        assert!(is_in_position(&mut in_position, 0.5, 1.0));
        assert_eq!(in_position._running_sum[0], 1.25);
        check_queue_size(&in_position);

        // Fourth time
        assert!(is_in_position(&mut in_position, 1.3, 1.0));
        assert_relative_eq!(in_position._running_sum[0], 1.94);
        check_queue_size(&in_position);

        // Fifth time
        assert!(is_in_position(&mut in_position, 1.4, 1.0));
        assert_eq!(in_position._running_sum[0], 3.65);
        check_queue_size(&in_position);

        // Sixth time
        assert!(is_in_position(&mut in_position, 0.1, 1.0));
        assert_relative_eq!(in_position._running_sum[0], 1.97);
        check_queue_size(&in_position);

        // Seventh time
        assert!(!is_in_position(&mut in_position, 0.1, 10.0));
        assert_eq!(*in_position._running_sum.last().unwrap(), 101.0);
        check_queue_size(&in_position);
    }

    fn is_in_position(in_posiiton: &mut InPosition, value_axial: f64, value_tangent: f64) -> bool {
        let mut force_error = vec![0.0; NUM_ACTIVE_ACTUATOR];
        force_error[0] = value_axial;
        force_error[NUM_ACTIVE_ACTUATOR - 1] = value_tangent;

        in_posiiton.is_in_position(&force_error)
    }

    #[test]
    fn test_reset() {
        let mut in_position = create_in_position();
        is_in_position(&mut in_position, 1.0, 1.0);

        in_position.reset();

        check_queue_size(&in_position);
        assert_eq!(in_position._running_sum, vec![0.0; NUM_ACTIVE_ACTUATOR]);
    }
}
