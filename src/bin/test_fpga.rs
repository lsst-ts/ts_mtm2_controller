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

use flexi_logger::Logger;
use log::info;

use run_m2::constants::CODE_FORCE_REQUEST;
use run_m2::daq::{config_data_acquisition::ConfigDataAcquisition, fpga_wrapper::FpgaWrapper};
use run_m2::enums::{DigitalOutput, DigitalOutputStatus};

fn main() {
    // Set up the logger
    if let Err(error) =
        Logger::try_with_str("info").and_then(|logger| logger.log_to_stdout().start())
    {
        eprintln!("Failed to initialize logger: {error}");
    }

    info!("Begin to test the FPGA.");

    let config = ConfigDataAcquisition::new();
    let mut fpga_wrapper = FpgaWrapper::new(config.path_header.as_path());

    fpga_wrapper.init_hardware(
        &config.path_bitfile,
        &config.fpga_resource,
        200,
        config.write_fifo_pace_ticks,
        config.timeout_get_next_character,
        config.timeout_irq,
        config.requested_depth_in_fifo_daq,
        config.requested_depth_in_fifo_inbound_outbound,
        1000,
    );

    info!("Hardware initialized.");

    // Read the NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On
    let name = "controlIlcCommPowerOn";
    let mut control_bool_ilc_comm_power_on = fpga_wrapper.read_control_value_bool(name).unwrap();

    info!(
        "NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On (init): {}",
        control_bool_ilc_comm_power_on
    );

    // Set the value to true
    fpga_wrapper.switch_digital_output(
        DigitalOutput::CommunicationPower,
        DigitalOutputStatus::BinaryHighLevel,
    );

    // Read the value back
    control_bool_ilc_comm_power_on = fpga_wrapper.read_control_value_bool(name).unwrap();
    info!(
        "NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On (after write): {}",
        control_bool_ilc_comm_power_on
    );

    // Set the value back to false
    fpga_wrapper.switch_digital_output(
        DigitalOutput::CommunicationPower,
        DigitalOutputStatus::BinaryLowLevel,
    );

    // Read the value back
    control_bool_ilc_comm_power_on = fpga_wrapper.read_control_value_bool(name).unwrap();
    info!(
        "NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On (after reset): {}",
        control_bool_ilc_comm_power_on
    );

    // Check the capture of the DAQ FIFO
    let mut control_enable_capture = fpga_wrapper
        .read_control_value_bool("controlEnableCapture")
        .unwrap();
    info!(
        "NiFpga_portSerialMasterSlave_ControlBool_Enable_Capture (init): {}",
        control_enable_capture
    );

    // Turn off the capture
    fpga_wrapper.write_control_value_bool("controlEnableCapture", false);

    control_enable_capture = fpga_wrapper
        .read_control_value_bool("controlEnableCapture")
        .unwrap();
    info!(
        "NiFpga_portSerialMasterSlave_ControlBool_Enable_Capture (after write): {}",
        control_enable_capture
    );

    // Enable the capture again
    fpga_wrapper.write_control_value_bool("controlEnableCapture", true);

    // Sleep for 1000 millisecond to allow some data to be captured
    std::thread::sleep(std::time::Duration::from_millis(1000));

    // Disable the capture again
    fpga_wrapper.write_control_value_bool("controlEnableCapture", false);

    // Sleep for 1000 millisecond to make sure all the data is captured in the
    // loop
    std::thread::sleep(std::time::Duration::from_millis(1000));

    // Read the power data from the FIFO
    let power_data = fpga_wrapper.read_power_and_digital_input().unwrap();
    info!("Power data read from the DAQ FIFO: {:?}", power_data);

    // Read the ILC frame (should fail)
    let ilc_frame = fpga_wrapper.request_ilc(
        &[0x02, CODE_FORCE_REQUEST, 0x41, 0x21],
        1,
        1,
        1000,
        config.timeout_irq,
    );
    info!(
        "ILC frame read: {:?}. The None is expected due to no response from the ILC.",
        ilc_frame
    );

    info!("End of FPGA test.");
}
