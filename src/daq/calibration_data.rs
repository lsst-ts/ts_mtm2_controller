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

use crate::constants::NUM_ILC_CHANNEL;
use crate::utility::get_f32_values_from_u8_array;

#[derive(Debug, PartialEq)]
pub struct CalibrationData {
    // Gains of the channels.
    pub gains: [f32; NUM_ILC_CHANNEL],
    // Offsets of the channels.
    pub offsets: [f32; NUM_ILC_CHANNEL],
    // Sensitivities of the channels.
    pub sensitivities: [f32; NUM_ILC_CHANNEL],
}

impl CalibrationData {
    /// Create a new instance of CalibrationData from a byte array.
    ///
    /// # Arguments
    /// * `frame` - A byte array containing the calibration data.
    ///
    /// # Returns
    /// * `Option<CalibrationData>` - Some(CalibrationData) if the frame
    ///   contains valid data, None otherwise.
    pub fn from_frame(frame: &[u8]) -> Option<CalibrationData> {
        // f32 is 4 bytes, and we have 3 arrays of NUM_ILC_CHANNEL elements
        // each.
        const BYTES_PER_F32: usize = 4;
        if frame.len() != (BYTES_PER_F32 * NUM_ILC_CHANNEL * 3) {
            return None;
        }

        Some(Self {
            gains: get_f32_values_from_u8_array::<NUM_ILC_CHANNEL>(
                &frame[0..(BYTES_PER_F32 * NUM_ILC_CHANNEL)],
            )?,
            offsets: get_f32_values_from_u8_array::<NUM_ILC_CHANNEL>(
                &frame[(BYTES_PER_F32 * NUM_ILC_CHANNEL)..(BYTES_PER_F32 * NUM_ILC_CHANNEL * 2)],
            )?,
            sensitivities: get_f32_values_from_u8_array::<NUM_ILC_CHANNEL>(
                &frame
                    [(BYTES_PER_F32 * NUM_ILC_CHANNEL * 2)..(BYTES_PER_F32 * NUM_ILC_CHANNEL * 3)],
            )?,
        })
    }

    /// Convert the CalibrationData instance into a byte array.
    ///
    /// Returns
    /// A byte array representing the calibration data.
    pub fn to_frame(&self) -> Vec<u8> {
        const BYTES_PER_F32: usize = 4;
        let mut frame = vec![0; BYTES_PER_F32 * NUM_ILC_CHANNEL * 3];

        for (idx, &gain) in self.gains.iter().enumerate() {
            frame[idx * BYTES_PER_F32..(idx + 1) * BYTES_PER_F32]
                .copy_from_slice(&gain.to_be_bytes());
        }

        for (idx, &offset) in self.offsets.iter().enumerate() {
            frame[BYTES_PER_F32 * NUM_ILC_CHANNEL + idx * BYTES_PER_F32
                ..BYTES_PER_F32 * NUM_ILC_CHANNEL + (idx + 1) * BYTES_PER_F32]
                .copy_from_slice(&offset.to_be_bytes());
        }

        for (idx, &sensitivity) in self.sensitivities.iter().enumerate() {
            frame[BYTES_PER_F32 * 2 * NUM_ILC_CHANNEL + idx * BYTES_PER_F32
                ..BYTES_PER_F32 * 2 * NUM_ILC_CHANNEL + (idx + 1) * BYTES_PER_F32]
                .copy_from_slice(&sensitivity.to_be_bytes());
        }

        frame
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_frame() {
        // Create a valid frame with calibration data
        let mut frame = [0; 48];
        for idx in 0..12 {
            let value = idx as f32;
            frame[idx * 4..(idx + 1) * 4].copy_from_slice(&value.to_be_bytes());
        }

        let calibration_data = CalibrationData::from_frame(&frame).unwrap();

        assert_eq!(calibration_data.gains, [0.0, 1.0, 2.0, 3.0]);
        assert_eq!(calibration_data.offsets, [4.0, 5.0, 6.0, 7.0]);
        assert_eq!(calibration_data.sensitivities, [8.0, 9.0, 10.0, 11.0]);
    }

    #[test]
    fn test_to_frame() {
        let calibration_data = CalibrationData {
            gains: [0.0, 1.0, 2.0, 3.0],
            offsets: [4.0, 5.0, 6.0, 7.0],
            sensitivities: [8.0, 9.0, 10.0, 11.0],
        };

        let frame = calibration_data.to_frame();

        assert_eq!(
            CalibrationData::from_frame(&frame).unwrap(),
            calibration_data
        );
    }
}
