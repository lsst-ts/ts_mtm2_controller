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

use std::collections::VecDeque;

pub struct SimpleDelayFilter {
    _coeffs: Vec<f64>,
    _delay_history: VecDeque<Vec<f64>>,
}

impl SimpleDelayFilter {
    /// Simple Delay Filter class to filter the input signal with a delay.
    ///
    /// # Notes
    /// This is a simplified implementation that considers the denominator of
    /// transfer function is 1:
    ///
    /// H(z) = b0 + b1 * z ^ (-1) + b2 * z ^ (-2) + ... + bN * z ^ (-N)
    ///      = Y(z) / X(z)
    ///
    /// => y[n] = b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + ... + bN * x[n-N]
    ///
    /// # Arguments
    /// * `coefficients` - Filter coefficients: [b0, b1, b2, ..., bN].
    /// * `num_element` - Number of the elements when calling the "filter()".
    ///
    /// # Returns
    /// A new SimpleDelayFilter object.
    pub fn new(coefficients: &[f64], num_element: usize) -> Self {
        let mut delay_history: VecDeque<Vec<f64>> = VecDeque::with_capacity(coefficients.len());
        for _ in 0..coefficients.len() {
            delay_history.push_back(vec![0.0; num_element]);
        }

        Self {
            _coeffs: coefficients.to_vec(),
            _delay_history: delay_history,
        }
    }

    /// Reset the filter.
    pub fn reset(&mut self) {
        self._delay_history
            .iter_mut()
            .for_each(|vector| vector.iter_mut().for_each(|val| *val = 0.0));
    }

    /// Filter the input value.
    ///
    /// # Arguments
    /// * `value` - Input value with the same number of elements (aka.
    /// "num_element") when initializing this class.
    ///
    /// # Returns
    /// Filtered value.
    ///
    /// # Panics
    /// * If the size of the input value is not the same as the number of
    /// elements.
    pub fn filter(&mut self, value: &[f64]) -> Vec<f64> {
        // Check the size of the input value
        assert!(
            value.len() == self._delay_history[0].len(),
            "The size of the input value should be {}.",
            self._delay_history[0].len()
        );

        // Filtered value
        let mut value_filter: Vec<f64> = value.iter().map(|val| self._coeffs[0] * val).collect();
        for idx in 1..self._coeffs.len() {
            value_filter
                .iter_mut()
                .zip(self._delay_history[idx - 1].iter())
                .for_each(|(val, delay)| *val += self._coeffs[idx] * delay);
        }

        // Update the history
        self._delay_history.pop_back();
        self._delay_history.push_front(value.to_vec());

        // return value_filter
        value_filter
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::utility::assert_relative_eq_vector;

    const EPSILON: f64 = 1e-7;

    fn create_simple_delay_filter() -> SimpleDelayFilter {
        SimpleDelayFilter::new(&vec![0.1, 0.2], 2)
    }

    #[test]
    fn test_new() {
        let simple_delay_filter = create_simple_delay_filter();

        assert_eq!(simple_delay_filter._coeffs, vec![0.1, 0.2]);
        check_delay_history_size(&simple_delay_filter);
    }

    fn check_delay_history_size(simple_delay_filter: &SimpleDelayFilter) {
        assert_eq!(simple_delay_filter._delay_history.len(), 2);
        assert_eq!(simple_delay_filter._delay_history.capacity(), 2);
    }

    #[test]
    fn test_filter() {
        let mut simple_delay_filter = create_simple_delay_filter();

        // First time
        let value_filter_1 = simple_delay_filter.filter(&vec![1.0, 3.0]);

        assert_relative_eq_vector(&value_filter_1, &vec![0.1, 0.3], EPSILON);
        check_delay_history(&simple_delay_filter, &vec![1.0, 3.0], &vec![0.0, 0.0]);

        // Second time
        let value_filter_2 = simple_delay_filter.filter(&value_filter_1);

        assert_relative_eq_vector(&value_filter_2, &vec![0.21, 0.63], EPSILON);
        check_delay_history(&simple_delay_filter, &vec![0.1, 0.3], &vec![1.0, 3.0]);

        // Third time
        let value_filter_3: Vec<f64> = simple_delay_filter.filter(&value_filter_2);

        assert_relative_eq_vector(&value_filter_3, &vec![0.041, 0.123], EPSILON);
        check_delay_history(&simple_delay_filter, &vec![0.21, 0.63], &vec![0.1, 0.3]);
    }

    fn check_delay_history(
        simple_delay_filter: &SimpleDelayFilter,
        value_1: &Vec<f64>,
        value_2: &Vec<f64>,
    ) {
        check_delay_history_size(simple_delay_filter);

        assert_relative_eq_vector(&simple_delay_filter._delay_history[0], value_1, EPSILON);

        assert_relative_eq_vector(&simple_delay_filter._delay_history[1], value_2, EPSILON);
    }

    #[test]
    #[should_panic(expected = "The size of the input value should be 2.")]
    fn test_filter_panic() {
        let mut simple_delay_filter = create_simple_delay_filter();
        simple_delay_filter.filter(&vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_reset() {
        let mut simple_delay_filter = create_simple_delay_filter();
        simple_delay_filter.filter(&vec![0.1, 0.2]);

        simple_delay_filter.reset();

        check_delay_history(&simple_delay_filter, &vec![0.0, 0.0], &vec![0.0, 0.0]);
    }
}
