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

use flexi_logger::LoggerHandle;
use log::{info, warn};
use signal_hook::{
    consts::{SIGINT, SIGTERM},
    flag::register,
};
use std::fs::read_to_string;
use std::path::Path;
use std::sync::atomic::Ordering;
use std::thread::sleep;
use std::time::Duration;

use crate::model::Model;
use ts_control_utils::{constants::ALL_HOST, utility::get_parameter};

/// Run the application.
///
/// # Arguments
/// * `port_command_gui` - Command port for the GUI.
/// * `port_telemetry_gui` - Telemetry port for the GUI.
/// * `port_command_csc` - Command port for the CSC.
/// * `port_telemetry_csc` - Telemetry port for the CSC.
/// * `is_simulation_mode` - Is the simulation mode or not.
/// * `logger_handle` - Logger handle.
pub fn run(
    port_command_gui: i32,
    port_telemetry_gui: i32,
    port_command_csc: i32,
    port_telemetry_csc: i32,
    is_simulation_mode: bool,
    logger_handle: Option<LoggerHandle>,
) {
    // Log the running mode
    let mode = if is_simulation_mode {
        "simulation mode"
    } else {
        "hardware mode"
    };
    info!("Run the M2 control system in {mode}.");

    // Read the FPGA bitfile status and safety module status.
    let config_file = Path::new("config/parameters_app.yaml");

    if !is_simulation_mode {
        let sleep_time_wait_for_labview = get_parameter(config_file, "sleep_time_wait_for_labview");
        info!("Wait for {sleep_time_wait_for_labview} seconds for the LabVIEW startup execution to finish.");
        sleep(Duration::from_secs(sleep_time_wait_for_labview));

        let labview_errlog_path: String = get_parameter(config_file, "labview_errlog_path");
        read_labview_errlog(Path::new(&labview_errlog_path), 4);
    }

    // Decide the ports to the TCP/IP servers
    let (final_port_command_gui, final_port_telemetry_gui) =
        get_final_ports(config_file, port_command_gui, port_telemetry_gui, true);
    let (final_port_command_csc, final_port_telemetry_csc) =
        get_final_ports(config_file, port_command_csc, port_telemetry_csc, false);

    // Create the model
    let mut model = Model::new(
        is_simulation_mode,
        ALL_HOST,
        final_port_command_gui,
        final_port_telemetry_gui,
        final_port_command_csc,
        final_port_telemetry_csc,
        logger_handle,
    );

    // Register the signals that stop the application
    for signal in [SIGTERM, SIGINT].iter() {
        let _ = register(*signal, model.stop.clone());
    }

    // Run the processes
    model.run_processes();

    // Run the main loop
    while !model.stop.load(Ordering::Relaxed) {
        model.step();
    }

    info!("Stopping the M2 control system...");

    // Stop the servers
    model.stop();

    // Wait for all the threads to stop and log the messages
    sleep(Duration::from_millis(1000));
    info!("M2 control system should be stopped.");
}

/// Read the LabVIEW errlog.txt to get the statuses of FPGA bitfile and safety
/// module.
///
/// # Arguments
/// * `filepath` - Path to the LabVIEW errlog.txt file.
/// * `last_lines_to_read` - Number of last lines to read from the errlog file.
///
/// # Returns
/// A vector of strings containing the messages read from the errlog file, or
/// None if the file does not exist.
fn read_labview_errlog(filepath: &Path, last_lines_to_read: usize) -> Option<Vec<String>> {
    if filepath.exists() {
        info!(
            "Read the statuses of FPGA bitfile and safety module in LabVIEW: {:?}.",
            filepath
        );

        let mut messages = Vec::new();
        if let Ok(lines) = read_to_string(filepath) {
            let content: Vec<&str> = lines.lines().collect();
            let last_lines = content
                .iter()
                .rev()
                .take(last_lines_to_read)
                .collect::<Vec<&&str>>();
            for line in last_lines.iter().rev() {
                let parts: Vec<&str> = line.split('\t').collect();
                if parts.len() < 3 {
                    warn!("Invalid line in LabVIEW errlog.txt: {line}");
                    continue;
                }
                let message = format!("{} ({})", parts[2], reformat_timestamp(parts[0], parts[1]));

                info!("{}", message);

                messages.push(message);
            }
        }

        info!("Finished the reading of LabVIEW errlog.txt.");

        Some(messages)
    } else {
        warn!(
            "LabVIEW errlog.txt does not exist. Please check the path: {:?}.",
            filepath
        );

        None
    }
}

/// Reformat the timestamp from the LabVIEW errlog.txt.
///
/// # Arguments
/// * `date` - Date string in the format "MM/DD/YYYY".
/// * `time` - Time string in the format "HH:MM:SS".
///
/// # Returns
/// * Reformatted timestamp string in the format "YYYY/MM/DD HH:MM:SS".
fn reformat_timestamp(date: &str, time: &str) -> String {
    let date_parts: Vec<&str> = date.split('/').collect();
    format!(
        "{}/{}/{} {}",
        date_parts[2], date_parts[0], date_parts[1], time
    )
}

/// Get the final ports.
///
/// # Arguments
/// * `config_file` - Configuration file.
/// * `port_command` - Command port. If the value is 0, the port is read from
///   the configuration file.
/// * `port_telemetry` - Telemetry port. If the value is 0, the port is read
///   from the configuration file.
/// * `is_gui` - Is the GUI or not. If not, it is the CSC.
///
/// # Returns
/// * `final_port_command` - Final command port. If the value is 0, the OS will
///   assign a port.
/// * `final_port_telemetry` - Final telemetry port. If the value is 0, the OS
///   will assign a port.
fn get_final_ports(
    config_file: &Path,
    port_command: i32,
    port_telemetry: i32,
    is_gui: bool,
) -> (i32, i32) {
    let key_command = if is_gui {
        "port_command_gui"
    } else {
        "port_command_csc"
    };
    let key_telemetry = if is_gui {
        "port_telemetry_gui"
    } else {
        "port_telemetry_csc"
    };

    let final_port_command = if port_command == 0 {
        get_parameter(config_file, key_command)
    } else {
        port_command
    };
    let final_port_telemetry = if port_telemetry == 0 {
        get_parameter(config_file, key_telemetry)
    } else {
        port_telemetry
    };

    (final_port_command, final_port_telemetry)
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::fs::write;
    use tempfile::NamedTempFile;

    #[test]
    fn test_read_labview_errlog() {
        let temp_file = NamedTempFile::new().unwrap();
        let errlog_content = "07/13/2026\t21:40:46\tstartup.rtexe: Begin to load the FPGA bitfile.\n07/13/2026\t21:40:47\tstartup.rtexe: Load the FPGA bitfile: true.\n";
        write(&temp_file, errlog_content).unwrap();

        let messages = read_labview_errlog(temp_file.path(), 2).unwrap();

        assert_eq!(messages.len(), 2);
        assert_eq!(
            messages[0],
            "startup.rtexe: Begin to load the FPGA bitfile. (2026/07/13 21:40:46)"
        );
        assert_eq!(
            messages[1],
            "startup.rtexe: Load the FPGA bitfile: true. (2026/07/13 21:40:47)"
        );

        temp_file.close().unwrap();
    }

    #[test]
    fn test_reformat_timestamp() {
        let reformatted_timestamp = reformat_timestamp("07/13/2026", "21:40:46");

        assert_eq!(reformatted_timestamp, "2026/07/13 21:40:46");
    }

    #[test]
    fn test_get_final_ports() {
        let config_file = Path::new("config/parameters_app.yaml");

        let (final_port_command, final_port_telemetry) = get_final_ports(config_file, 0, 0, true);

        assert_eq!(final_port_command, 50010);
        assert_eq!(final_port_telemetry, 50011);

        let (final_port_command, final_port_telemetry) = get_final_ports(config_file, 10, 11, true);

        assert_eq!(final_port_command, 10);
        assert_eq!(final_port_telemetry, 11);
    }
}
