struct SingleBiquadraticFilter {
    _a11: f64,
    _a21: f64,
    _b11: f64,
    _b21: f64,
    _g_m1: Vec<f64>,
    _g_m2: Vec<f64>,
}

pub struct BiquadraticFilter {
    _gain: f64,
    _bqd_filters: Vec<SingleBiquadraticFilter>,
}

impl SingleBiquadraticFilter {
    /// Single biquadratic filter.
    ///
    /// # Notes
    /// The equation is:
    ///        1 + b11 z^-1 + b21 z^-2    Y(z)
    /// H(z) = ----------------------- = -----
    ///        1 + a11 z^-1 + a21 z^-2.   X(z)
    ///
    /// This can be considered as the following process:
    ///
    /// g[n-2] = b21 * x[n] - a21 * y[n]
    /// g[n-1] = g[n-2] + b11 * x[n] - a11 * y[n]
    ///
    /// Therefore, we will have:
    ///
    /// y[n] = x[n] + g[n-1]
    ///
    /// # Arguments
    /// * `a11` - Coefficient a11 in the equation..
    /// * `a21` - Coefficient a21 in the equation..
    /// * `b11` - Coefficient b11 in the equation..
    /// * `b21` - Coefficient b21 in the equation..
    /// * `num_element` - Number of the elements when calling the "filter()".
    ///
    /// # Returns
    /// A new SingleBiquadraticFilter object.
    fn new(a11: f64, a21: f64, b11: f64, b21: f64, num_element: usize) -> Self {
        Self {
            _a11: a11,
            _a21: a21,
            _b11: b11,
            _b21: b21,
            _g_m1: vec![0.0; num_element],
            _g_m2: vec![0.0; num_element],
        }
    }

    /// Reset the filter.
    fn reset(&mut self) {
        self._g_m1 = vec![0.0; self._g_m1.len()];
        self._g_m2 = vec![0.0; self._g_m2.len()];
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
    fn filter(&mut self, value: &[f64]) -> Vec<f64> {
        // Check the size of the input value
        assert!(
            value.len() == self._g_m1.len(),
            "Size of the input value should be {}.",
            self._g_m1.len()
        );

        // Filtered value
        // value_filter = value + self._g_m1
        let value_filter: Vec<f64> = self
            ._g_m1
            .iter()
            .zip(value.iter())
            .map(|(g_m1, val)| g_m1 + val)
            .collect();

        // Calculate and update the updated internal values
        // g_m2 = self._b21 * value - self._a21 * value_filter
        // g_m1 = self._g_m2 + self._b11 * value - self._a11 * value_filter
        let g_m2 = value
            .iter()
            .zip(value_filter.iter())
            .map(|(val, val_filter)| self._b21 * val - self._a21 * val_filter)
            .collect();
        let g_m1 = self
            ._g_m2
            .iter()
            .zip(value.iter())
            .zip(value_filter.iter())
            .map(|((g_m2, val), val_filter)| g_m2 + self._b11 * val - self._a11 * val_filter)
            .collect();

        self._g_m1 = g_m1;
        self._g_m2 = g_m2;

        value_filter
    }
}

impl BiquadraticFilter {
    /// Biquadratic filter.
    ///
    /// # Notes
    /// Compact biquadratic format:
    /*
                   1 + b11 z^-1 + b21 z^-2   1 + b12 z^-1 + b22 z^-2
    H(z) = gain * (-----------------------) (-----------------------) ...
                   1 + a11 z^-1 + a21 z^-2   1 + a12 z^-1 + a22 z^-2

            1 + b1N z^-1 + b2N z^-2
           (-----------------------)
            1 + a1N z^-1 + a2N z^-2
    */
    /// The reference is:
    /// https://en.wikipedia.org/wiki/Digital_biquad_filter
    ///
    /// # Arguments
    /// * `gain` - Gain of the filter.
    /// * `coefficients` - Coefficients of the biquadratic filters. This should
    /// be a 2D array. Each row is the [a1n, a2n, b1n, b2n] for the nth
    /// biquadratic filter.
    /// * `num_element` - Number of the elements when calling the "filter()".
    ///
    /// # Returns
    /// A new BiquadraticFilter object.
    pub fn new(gain: f64, coefficients: &Vec<Vec<f64>>, num_element: usize) -> Self {
        let mut bqd_filters = Vec::new();
        coefficients.iter().for_each(|coefficient| {
            let bqd_filter = SingleBiquadraticFilter::new(
                coefficient[0],
                coefficient[1],
                coefficient[2],
                coefficient[3],
                num_element,
            );
            bqd_filters.push(bqd_filter);
        });

        Self {
            _gain: gain,
            _bqd_filters: bqd_filters,
        }
    }

    /// Reset the filter.
    pub fn reset(&mut self) {
        self._bqd_filters
            .iter_mut()
            .for_each(|bqd_filter| bqd_filter.reset());
    }

    /// Filter the input value.
    ///
    /// # Arguments
    /// * `value` - Input value with the same number of elements (aka.
    /// "num_element") when initializing this class.
    ///
    /// # Returns
    /// Filtered value.
    pub fn filter(&mut self, value: &[f64]) -> Vec<f64> {
        let mut value_filter = value.to_vec();
        self._bqd_filters.iter_mut().for_each(|bqd_filter| {
            value_filter = bqd_filter.filter(&value_filter);
        });

        value_filter.iter_mut().for_each(|val| *val *= self._gain);

        value_filter
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::utility::assert_relative_eq_vector;

    const EPSILON: f64 = 1e-7;
    struct SingleBiquadraticFilterVerification {
        _a11: f64,
        _a21: f64,
        _b11: f64,
        _b21: f64,
        _x_m1: Vec<f64>,
        _x_m2: Vec<f64>,
        _y_m1: Vec<f64>,
        _y_m2: Vec<f64>,
    }

    impl SingleBiquadraticFilterVerification {
        fn new(a11: f64, a21: f64, b11: f64, b21: f64, num_element: usize) -> Self {
            Self {
                _a11: a11,
                _a21: a21,
                _b11: b11,
                _b21: b21,
                _x_m1: vec![0.0; num_element],
                _x_m2: vec![0.0; num_element],
                _y_m1: vec![0.0; num_element],
                _y_m2: vec![0.0; num_element],
            }
        }

        fn filter(&mut self, value: &[f64]) -> Vec<f64> {
            // Filtered value
            let y: Vec<f64> = value
                .iter()
                .zip(self._x_m1.iter())
                .zip(self._x_m2.iter())
                .zip(self._y_m1.iter())
                .zip(self._y_m2.iter())
                .map(|((((val, x_m1), x_m2), y_m1), y_m2)| {
                    val + self._b11 * x_m1 + self._b21 * x_m2 - self._a11 * y_m1 - self._a21 * y_m2
                })
                .collect();

            // Update the internal data
            self._x_m2 = self._x_m1.clone();
            self._x_m1 = value.to_vec();

            self._y_m2 = self._y_m1.clone();
            self._y_m1 = y.clone();

            y
        }
    }

    fn create_single_biquadratic_filter() -> SingleBiquadraticFilter {
        SingleBiquadraticFilter::new(2.0, 3.0, 4.0, 5.0, 2)
    }

    fn create_single_biquadratic_filter_verification() -> SingleBiquadraticFilterVerification {
        SingleBiquadraticFilterVerification::new(2.0, 3.0, 4.0, 5.0, 2)
    }

    fn create_biquadratic_filter() -> BiquadraticFilter {
        let gain = 3.0;
        let coefficients = vec![vec![2.0, 3.0, 4.0, 5.0], vec![2.0, 3.0, 4.0, 5.0]];

        BiquadraticFilter::new(gain, &coefficients, 2)
    }

    #[test]
    fn test_filter_single_biquadratic_filter() {
        let mut bqd_filter = create_single_biquadratic_filter();
        let mut bqd_filter_verification = create_single_biquadratic_filter_verification();

        let mut value_bqd = vec![0.1, 0.3];
        let mut value_verification = vec![0.1, 0.3];

        for _ in 0..20 {
            value_bqd = bqd_filter.filter(&value_bqd);
            value_verification = bqd_filter_verification.filter(&value_verification);

            assert_relative_eq_vector(&value_bqd, &value_verification, EPSILON);
        }
    }

    #[test]
    #[should_panic(expected = "Size of the input value should be 2.")]
    fn test_filter_single_biquadratic_filter_panic() {
        let mut bqd_filter = create_single_biquadratic_filter();

        bqd_filter.filter(&vec![0.1, 0.3, 0.5]);
    }

    #[test]
    fn test_reset_single_biquadratic_filter() {
        let mut bqd_filter = create_single_biquadratic_filter();
        bqd_filter.filter(&vec![0.1, 0.3]);

        bqd_filter.reset();

        assert_eq!(bqd_filter._g_m1, vec![0.0, 0.0]);
        assert_eq!(bqd_filter._g_m2, vec![0.0, 0.0]);
    }

    #[test]
    fn test_filter_biquadratic_filter() {
        let mut biquadratic_filter = create_biquadratic_filter();

        // First time
        let value_filter_1 = biquadratic_filter.filter(&vec![0.1, 0.3]);

        assert_relative_eq_vector(&value_filter_1, &vec![0.3, 0.9], EPSILON);

        assert_relative_eq_vector(
            &biquadratic_filter._bqd_filters[1]._g_m1,
            &vec![0.2, 0.6],
            EPSILON,
        );
        assert_relative_eq_vector(
            &biquadratic_filter._bqd_filters[1]._g_m2,
            &vec![0.2, 0.6],
            EPSILON,
        );

        // Second time
        let new_value: Vec<f64> = value_filter_1
            .iter()
            .map(|val| val / biquadratic_filter._gain)
            .collect();

        let value_filter_2 = biquadratic_filter.filter(&new_value);

        assert_relative_eq_vector(&value_filter_2, &vec![1.5, 4.5], EPSILON);

        assert_relative_eq_vector(
            &biquadratic_filter._bqd_filters[0]._g_m1,
            &vec![0.0, 0.0],
            EPSILON,
        );
        assert_relative_eq_vector(
            &biquadratic_filter._bqd_filters[0]._g_m2,
            &vec![-0.4, -1.2],
            EPSILON,
        );
    }

    #[test]
    fn test_reset_biquadratic_filter() {
        let mut biquadratic_filter = create_biquadratic_filter();
        biquadratic_filter.filter(&vec![0.1, 0.3]);

        biquadratic_filter.reset();

        assert_eq!(biquadratic_filter._bqd_filters[0]._g_m1, vec![0.0, 0.0]);
        assert_eq!(biquadratic_filter._bqd_filters[0]._g_m2, vec![0.0, 0.0]);
    }
}
