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
use simplelog::{ColorChoice, Config, LevelFilter, TermLogger, TerminalMode};

use run_m2::daq::{config_data_acquisition::ConfigDataAcquisition, fpga_wrapper::FpgaWrapper};

fn main() {
    // Set up the logger
    if let Err(error) = TermLogger::init(
        LevelFilter::Info,
        Config::default(),
        TerminalMode::Mixed,
        ColorChoice::Auto,
    ) {
        eprintln!("Failed to initialize logger: {error}");
    }

    info!("Begin to test the FPGA.");

    let config = ConfigDataAcquisition::new();
    let mut fpga_wrapper = FpgaWrapper::new(config.path_header.as_path());
    fpga_wrapper.open(&config.path_bitfile, &config.fpga_resource);

    info!("Session opened.");

    // Read the NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On
    let name = "controlIlcCommPowerOn";
    let mut control_bool_ilc_comm_power_on = fpga_wrapper.read_control_value(name).unwrap();

    info!(
        "NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On (init): {}",
        control_bool_ilc_comm_power_on
    );

    // Set the value to true
    let _ = fpga_wrapper.write_control_value(name, true);

    // Read the value back
    control_bool_ilc_comm_power_on = fpga_wrapper.read_control_value(name).unwrap();
    info!(
        "NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On (after write): {}",
        control_bool_ilc_comm_power_on
    );

    // Set the value back to false
    let _ = fpga_wrapper.write_control_value(name, false);

    // Read the value back
    control_bool_ilc_comm_power_on = fpga_wrapper.read_control_value(name).unwrap();
    info!(
        "NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On (after reset): {}",
        control_bool_ilc_comm_power_on
    );

    // Log the serial configuration
    fpga_wrapper.log_serial_config();

    info!("End of FPGA test.");
}
