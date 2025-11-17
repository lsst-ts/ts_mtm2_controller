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

pub struct ConfigDataAcquisition {
    // The frequency of data acquisition (DAQ) loop in Hz
    pub frequency_loop: f64,
    // The frequency to send the telemetry in Hz
    pub frequency_send_telemetry: f64,
    // The frequency to toggle the closed-loop control bit in Hz
    pub frequency_toggle_bit: f64,
}

impl ConfigDataAcquisition {
    /// Create a new ConfigDataAcquisition object.
    ///
    /// # Returns
    /// A new ConfigDataAcquisition object.
    pub fn new() -> Self {
        let filepath = Path::new("config/parameters_daq.yaml");

        Self {
            frequency_loop: get_parameter(filepath, "frequency_loop"),
            frequency_send_telemetry: get_parameter(filepath, "frequency_send_telemetry"),
            frequency_toggle_bit: get_parameter(filepath, "frequency_toggle_bit"),
        }
    }
}
