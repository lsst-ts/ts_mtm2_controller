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

use log::info;
use signal_hook::{
    consts::{SIGINT, SIGTERM},
    flag::register,
};
use std::path::Path;
use std::sync::atomic::Ordering;
use std::thread::sleep;
use std::time::Duration;

use crate::constants::ALL_HOST;
use crate::model::Model;
use crate::utility::get_parameter;

/// Run the application.
///
/// # Arguments
/// * `port_command_gui` - Command port for the GUI.
/// * `port_telemetry_gui` - Telemetry port for the GUI.
/// * `port_command_csc` - Command port for the CSC.
/// * `port_telemetry_csc` - Telemetry port for the CSC.
/// * `is_simulation_mode` - Is the simulation mode or not.
pub fn run(
    port_command_gui: i32,
    port_telemetry_gui: i32,
    port_command_csc: i32,
    port_telemetry_csc: i32,
    is_simulation_mode: bool,
) {
    // Log the running mode
    let mode = if is_simulation_mode {
        "simulation mode"
    } else {
        "hardware mode"
    };
    info!("Run the M2 control system in {mode}.");

    // Decide the ports to the TCP/IP servers
    let config_file = Path::new("config/parameters_app.yaml");
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

/// Get the final ports.
///
/// # Arguments
/// * `config_file` - Configuration file.
/// * `port_command` - Command port. If the value is 0, the port is read from
/// the configuration file.
/// * `port_telemetry` - Telemetry port. If the value is 0, the port is read
/// from the configuration file.
/// * `is_gui` - Is the GUI or not. If not, it is the CSC.
///
/// # Returns
/// * `final_port_command` - Final command port. If the value is 0, the OS will
/// assign a port.
/// * `final_port_telemetry` - Final telemetry port. If the value is 0, the OS
/// will assign a port.
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
