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

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use ts_control_utils::utility::{get_parameter, get_parameter_array};

#[derive(Default)]
pub struct ConfigDataAcquisition {
    // The frequency of data acquisition (DAQ) loop in Hz.
    pub frequency_loop: f64,
    // The frequency to toggle the closed-loop control bit in Hz.
    pub frequency_toggle_bit: f64,
    // The limit of the inner-loop controller (ILC) stale data.
    pub ilc_stale_data_limit: i32,
    // Bypassed actuator ILC list (0-based) to check the error reported by ILC
    // directly.
    pub bypassed_actuator_ilcs: Vec<usize>,
    // Bypass the check of the stale data for inclinometer.
    pub bypass_check_stale_inclinometer: bool,
    // Sleep time in microseconds to wait for the ILC action after broadcasting
    // the global command to all ILCs.
    pub sleep_time_broadcast_ilc: u64,
    // Sleep time in microseconds to wait for the ILC to be ready for the next
    // command.
    pub sleep_time_ilc_reading: u64,
    // NI FPGA bitfile path.
    pub path_bitfile: PathBuf,
    // NI FPGA header file path.
    pub path_header: PathBuf,
    // Resource of the NI FPGA.
    pub fpga_resource: String,
    // The number of depth requested in the DAQ FIFO (power and digital input
    // data).
    pub requested_depth_in_fifo_daq: usize,
    // The number of depth requested in the inbound and outbound FIFOs for the
    // ILC communication.
    pub requested_depth_in_fifo_inbound_outbound: usize,
    // Buffer time to clear the DAQ FIFO in milliseconds.
    pub buffer_time_to_clear_fifo_daq: u64,
    // Pace to write the data to ILC in ticks.
    pub write_fifo_pace_ticks: u16,
    // Timeout for the interrupt request (IRQ) in milliseconds.
    pub timeout_irq: u32,
    // Timeout to get the next character from the ILC in microseconds.
    pub timeout_get_next_character: u32,
    // Payload byte for each type of data acquisition.
    pub payload_byte: HashMap<String, i32>,
    // Latency for each type of data acquisition in microseconds.
    pub latency: HashMap<String, u32>,
}

impl ConfigDataAcquisition {
    /// Create a new ConfigDataAcquisition object.
    ///
    /// # Returns
    /// A new ConfigDataAcquisition object.
    pub fn new() -> Self {
        let filepath: &Path = Path::new("config/parameters_daq.yaml");
        let fpga_directory = Path::new("fpga");

        Self {
            frequency_loop: get_parameter(filepath, "frequency_loop"),
            frequency_toggle_bit: get_parameter(filepath, "frequency_toggle_bit"),

            ilc_stale_data_limit: get_parameter(filepath, "ilc_stale_data_limit"),
            bypassed_actuator_ilcs: get_parameter_array::<usize>(
                filepath,
                "bypassed_actuator_ilcs",
            ),
            bypass_check_stale_inclinometer: get_parameter(
                filepath,
                "bypass_check_stale_inclinometer",
            ),

            sleep_time_broadcast_ilc: get_parameter(filepath, "sleep_time_broadcast_ilc"),
            sleep_time_ilc_reading: get_parameter(filepath, "sleep_time_ilc_reading"),

            path_bitfile: fpga_directory.join(get_parameter::<String>(filepath, "name_bitfile")),
            path_header: fpga_directory.join(get_parameter::<String>(filepath, "name_header")),
            fpga_resource: get_parameter(filepath, "fpga_resource"),

            requested_depth_in_fifo_daq: get_parameter(filepath, "requested_depth_in_fifo_daq"),
            requested_depth_in_fifo_inbound_outbound: get_parameter(
                filepath,
                "requested_depth_in_fifo_inbound_outbound",
            ),

            buffer_time_to_clear_fifo_daq: get_parameter(filepath, "buffer_time_to_clear_fifo_daq"),

            write_fifo_pace_ticks: get_parameter(filepath, "write_fifo_pace_ticks"),

            timeout_irq: get_parameter(filepath, "timeout_irq"),
            timeout_get_next_character: get_parameter(filepath, "timeout_get_next_character"),

            payload_byte: Self::create_dict(
                &[
                    "server_id",
                    "server_status",
                    "ilc_mode",
                    "force_and_status",
                    "offset_and_sensitivity",
                    "temperature",
                    "displacement",
                    "inclinometer",
                    "reset",
                    "scan_rate",
                    "calibration_data",
                ],
                &[
                    get_parameter(filepath, "payload_byte_report_server_id"),
                    get_parameter(filepath, "payload_byte_report_server_status"),
                    get_parameter(filepath, "payload_byte_ilc_mode"),
                    get_parameter(filepath, "payload_byte_get_force_and_status"),
                    get_parameter(filepath, "payload_byte_set_offset_and_sensitivity"),
                    get_parameter(filepath, "payload_byte_get_temperature"),
                    get_parameter(filepath, "payload_byte_get_displacement"),
                    get_parameter(filepath, "payload_byte_get_inclinometer"),
                    get_parameter(filepath, "payload_byte_reset"),
                    get_parameter(filepath, "payload_byte_scan_rate"),
                    get_parameter(filepath, "payload_byte_read_calibration_data"),
                ],
            ),
            latency: Self::create_dict(
                &[
                    "server_id",
                    "server_status",
                    "ilc_mode",
                    "force_and_status",
                    "offset_and_sensitivity",
                    "temperature",
                    "displacement",
                    "inclinometer",
                    "reset",
                    "scan_rate",
                    "calibration_data",
                ],
                &[
                    get_parameter(filepath, "latency_report_server_id"),
                    get_parameter(filepath, "latency_report_server_status"),
                    get_parameter(filepath, "latency_ilc_mode"),
                    get_parameter(filepath, "latency_get_force_and_status"),
                    get_parameter(filepath, "latency_set_offset_and_sensitivity"),
                    get_parameter(filepath, "latency_get_temperature"),
                    get_parameter(filepath, "latency_get_displacement"),
                    get_parameter(filepath, "latency_get_inclinometer"),
                    get_parameter(filepath, "latency_reset"),
                    get_parameter(filepath, "latency_scan_rate"),
                    get_parameter(filepath, "latency_read_calibration_data"),
                ],
            ),
        }
    }

    /// Create a dictionary from the given keys and values.
    ///
    /// # Arguments
    /// * `keys` - The keys of the dictionary.
    /// * `values` - The values of the dictionary.
    ///
    /// # Returns
    /// A dictionary created from the given keys and values.
    fn create_dict<T: Clone>(keys: &[&str], values: &[T]) -> HashMap<String, T> {
        let mut dict = HashMap::new();
        for (idx, key) in keys.iter().enumerate() {
            dict.insert(String::from(*key), values[idx].clone());
        }
        dict
    }
}
