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

use fixed::types::{I21F11, U21F11};
use log::{error, info};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;
use std::thread::sleep;
use std::time::Duration;

use crate::enums::{DigitalOutput, DigitalOutputStatus};
use ts_control_utils::enums::BitEnum;

#[cfg(feature = "fpga")]
use nifpga_dll::{ReadFifo, Session, WriteFifo};

#[cfg(feature = "fpga")]
#[link(name = "nifpga_c_api", kind = "static")]
unsafe extern "C" {}

#[cfg(feature = "fpga")]
#[link(name = "dl")]
unsafe extern "C" {}

#[derive(Default)]
pub struct FpgaWrapper {
    // Registers in the FPGA
    pub registers: HashMap<String, u32>,
    // Digital output.
    pub digital_output: u8,
    // Session to communicate with the FPGA
    #[cfg(feature = "fpga")]
    _session: Option<Session>,
    // FIFO for the data acquisition (power and digital input)
    #[cfg(feature = "fpga")]
    _fifo_daq: Option<ReadFifo<u64>>,
    #[cfg(feature = "fpga")]
    // Inbound FIFO to receive the frame from the inner-loop controller (ILC)
    _fifo_inbound: Option<ReadFifo<u8>>,
    #[cfg(feature = "fpga")]
    // Outbound FIFO to send the frame to the ILC
    _fifo_outbound: Option<WriteFifo<u8>>,
}

impl FpgaWrapper {
    /// Create a new FPGA wrapper object.
    ///
    /// # Arguments
    /// * `filepath` - The path to the FPGA header file.
    ///
    /// # Returns
    /// A new FPGA wrapper object.
    pub fn new(filepath: &Path) -> Self {
        // Registers to read/write in the FPGA.
        let keys = [
            "controlIlcCommPowerOn",
            "controlIlcMotorPowerOn",
            "controlMod4Ch6",
            "controlMod4Ch7",
            "controlResetCommPowerBreakers",
            "controlResetMotorPowerBreakers",
            "controlCrioInterlockEnable",
            "controlClosedLoopControlOn",
            "controlEnableCapture",
            "controlSerialConfigResource",
            "controlDataLoopRateInUs",
            "indicatorDaqFifoFull",
            "fifoDaq",
            "fifoInbound",
            "fifoOutbound",
        ];
        let names = [
            "NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On",
            "NiFpga_portSerialMasterSlave_ControlBool_ILC_Motor_Power_On",
            "NiFpga_portSerialMasterSlave_ControlBool_Mod4CH6",
            "NiFpga_portSerialMasterSlave_ControlBool_Mod4CH7",
            "NiFpga_portSerialMasterSlave_ControlBool_Reset_Comm_Power_Breakers",
            "NiFpga_portSerialMasterSlave_ControlBool_Reset_Motor_Power_Breakers",
            "NiFpga_portSerialMasterSlave_ControlBool_cRIO_Interlock_Enable",
            "NiFpga_portSerialMasterSlave_ControlBool_closedLoopControlOn",
            "NiFpga_portSerialMasterSlave_ControlBool_EnableCapture",
            "NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_Resource",
            "NiFpga_portSerialMasterSlave_ControlU32_LoopRateuS",
            "NiFpga_portSerialMasterSlave_IndicatorBool_DataFIFOFull",
            "NiFpga_portSerialMasterSlave_TargetToHostFifoFxp_DAQ_FIFO_Resource",
            "NiFpga_portSerialMasterSlave_TargetToHostFifoU8_Inbound_FIFO",
            "NiFpga_portSerialMasterSlave_HostToTargetFifoU8_Outbound_FIFO",
        ];

        Self {
            registers: Self::create_register_map(filepath, &keys, &names),
            digital_output: 0,

            #[cfg(feature = "fpga")]
            _session: None,

            #[cfg(feature = "fpga")]
            _fifo_daq: None,

            #[cfg(feature = "fpga")]
            _fifo_inbound: None,

            #[cfg(feature = "fpga")]
            _fifo_outbound: None,
        }
    }

    /// Create a register map from the FPGA header file.
    ///
    /// # Arguments
    /// * `filepath` - The path to the FPGA header file.
    /// * `keys` - The keys to use in the register map.
    /// * `names` - The names of the registers in the FPGA header file to get
    ///   the addresses of.
    ///
    /// # Returns
    /// A register map from the FPGA header file.
    ///
    /// # Panics
    /// Panics if any of the registers in the FPGA header file is not found.
    fn create_register_map(filepath: &Path, keys: &[&str], names: &[&str]) -> HashMap<String, u32> {
        let mut registers = HashMap::new();
        for (key, name) in keys.iter().zip(names.iter()) {
            if let Some(register) = Self::get_register(filepath, name) {
                registers.insert(key.to_string(), register);
            }
        }

        // Panic if any register is not found
        if registers.len() != keys.len() {
            let missing_keys: Vec<String> = keys
                .iter()
                .filter(|key| !registers.contains_key(**key))
                .map(|key| key.to_string())
                .collect();

            Self::log_error_and_panic(&format!(
                "Failed to get the register addresses of the following registers: {}.",
                missing_keys.join(", ")
            ));
        }

        registers
    }

    /// Log an error message and panic.
    ///
    /// # Arguments
    /// * `message` - The error message to log and panic with.
    ///
    /// # Panics
    /// Panics with the given error message.
    fn log_error_and_panic(message: &str) {
        error!("{}", message);
        panic!("{}", message);
    }

    /// Get the register address of a register in the FPGA header file.
    ///
    /// # Arguments
    /// * `filepath` - The path to the FPGA header file.
    /// * `register_name` - The name of the register to get the address of.
    ///
    /// # Returns
    /// The register address of the register, or None if the register is not
    /// found.
    fn get_register(filepath: &Path, register_name: &str) -> Option<u32> {
        let file = File::open(filepath).expect("Should open the FPGA header file.");
        let reader = BufReader::new(file);
        for line in reader.lines() {
            let line = line.expect("Should read the FPGA header file");
            if line.contains(register_name) {
                let parts: Vec<&str> = line.split('=').collect();
                if parts.len() == 2 {
                    let register_str = parts[1].trim().trim_end_matches([',', ';']);
                    if let Ok(register) =
                        u32::from_str_radix(register_str.trim_start_matches("0x"), 16)
                    {
                        return Some(register);
                    }
                }
            }
        }

        None
    }

    /// Open a session to communicate with the FPGA.
    ///
    /// # Arguments
    /// * `filepath` - The path to the FPGA bitfile.
    /// * `_resource` - The resource name of the FPGA.
    ///
    /// # Panics
    /// Panics if the session cannot be opened.
    pub fn open(&mut self, filepath: &Path, _resource: &str) {
        // Get the signature of the bitfile
        let signature = self
            .get_signature(filepath)
            .expect("Should get the signature of the bitfile.");

        info!("FPGA bitfile signature is: {}.", signature);

        #[cfg(feature = "fpga")]
        {
            // Get the absolute path of the bitfile
            let bitfile_absolute_path = filepath
                .canonicalize()
                .expect("Should get the absolute path of the bitfile.")
                .to_str()
                .expect("Should convert the absolute path of the bitfile to a string.")
                .to_string();

            // Open the session to communicate with the FPGA
            match Session::open(&bitfile_absolute_path, &signature, _resource, true, false) {
                Ok(session) => {
                    info!("Successfully opened the FPGA session.");
                    self._session = Some(session);
                }
                Err(error) => {
                    Self::log_error_and_panic(&format!(
                        "Failed to open the FPGA session: {}.",
                        error
                    ));
                }
            }
        }
    }

    /// Get the signature of the FPGA bitfile.
    ///
    /// # Arguments
    /// * `filepath` - The path to the FPGA bitfile.
    ///
    /// # Returns
    /// The signature of the FPGA bitfile, or None if the signature is not
    /// found.
    fn get_signature(&self, filepath: &Path) -> Option<String> {
        let file = File::open(filepath).expect("Should open the FPGA bitfile.");
        let reader = BufReader::new(file);
        for line in reader.lines() {
            let line = line.expect("Should read the FPGA bitfile");
            if line.contains("<SignatureRegister>") {
                let start = line.find("<SignatureRegister>")? + "<SignatureRegister>".len();
                let end = line.find("</SignatureRegister>")?;
                return Some(line[start..end].to_string());
            }
        }

        None
    }

    /// Log the value of the serial config resource in the FPGA.
    ///
    /// # Returns
    /// The value of the serial config resource, or None if the control
    /// register is not found or the value cannot be read.
    pub fn log_serial_config(&self) -> Option<[u8; 6]> {
        let _register = self.registers.get("controlSerialConfigResource")?;
        #[cfg(feature = "fpga")]
        {
            if let Some(session) = self._session.as_ref() {
                let mut config = [0; 6];
                match session.read_array::<u8>(*_register, &mut config) {
                    Ok(()) => {
                        info!(
                            "{}",
                            self._format_serial_config(
                                u16::from_be_bytes([config[0], config[1]]),
                                config[2],
                                config[3],
                                config[4],
                                config[5],
                            )
                        );
                        return Some(config);
                    }
                    Err(error) => {
                        error!("Failed to read the value of the control controlSerialConfigResource: {}",
                            error
                        );
                        return None;
                    }
                }
            }
        }

        None
    }

    /// Format the value of the serial config resource in the FPGA into a
    /// human-readable string.
    ///
    /// # Note
    /// See the "Serial Config" cluster in the portSerialMasterSlave.vi in
    /// ts_mtm2_cell.
    ///
    /// # Arguments
    /// * `value_baud_rate` - The value of the baud rate field.
    /// * `value_parity` - The value of the parity field.
    /// * `value_flow_control` - The value of the flow control field.
    /// * `value_data_bits` - The value of the data bits field.
    /// * `value_stop_bits` - The value of the stop bits field.
    ///
    /// # Returns
    /// A human-readable string representing the serial config resource.
    fn _format_serial_config(
        &self,
        value_baud_rate: u16,
        value_parity: u8,
        value_flow_control: u8,
        value_data_bits: u8,
        value_stop_bits: u8,
    ) -> String {
        let baud_rate = match value_baud_rate {
            0 => "75 bps",
            1 => "110 bps",
            2 => "134 bps",
            3 => "150 bps",
            4 => "300 bps",
            5 => "600 bps",
            6 => "1200 bps",
            7 => "1800 bps",
            8 => "2400 bps",
            9 => "4800 bps",
            10 => "7200 bps",
            11 => "9600 bps",
            12 => "14.4 kbps",
            13 => "19.2 kbps",
            14 => "28.8 kbps",
            15 => "38.4 kbps",
            16 => "57.6 kbps",
            17 => "115.2 kbps",
            18 => "230.4 kbps",
            19 => "460.8 kbps",
            20 => "921.6 kbps",
            _ => "Unknown",
        };

        let parity = match value_parity {
            0 => "None",
            1 => "Odd",
            2 => "Even",
            3 => "Mark",
            4 => "Space",
            _ => "Unknown",
        };

        let flow_control = match value_flow_control {
            0 => "None",
            1 => "Software (XON/XOFF)",
            2 => "Hardware (RTS/CTS)",
            _ => "Unknown",
        };

        let data_bits = match value_data_bits {
            0 => "5",
            1 => "6",
            2 => "7",
            3 => "8",
            _ => "Unknown",
        };

        let stop_bits = match value_stop_bits {
            0 => "1",
            1 => "1.5",
            2 => "2",
            _ => "Unknown",
        };

        format!(
            "Serial config - Baud rate: {}, Parity: {}, Flow control: {}, Data bits: {}, Stop bits: {}.",
            baud_rate, parity, flow_control, data_bits, stop_bits
        )
    }

    /// Open the FIFO in the FPGA.
    ///
    /// # Arguments
    /// * `_depth_daq` - The depth of the DAQ FIFO to open. Note that the
    ///   actual depth of the FIFO may be different from the requested depth.
    /// * `_depth_inbound_outbound` - The depth of the inbound and outbound
    ///   FIFOs. Note that the actual depth of the FIFOs may be different from
    ///   the requested depth.
    ///
    /// # Panics
    /// Panics if the FIFO cannot be opened.
    pub fn open_fifo(&mut self, _depth_daq: usize, _depth_inbound_outbound: usize) {
        #[cfg(feature = "fpga")]
        {
            if let Some(session) = self._session.as_ref() {
                match session.open_read_fifo::<u64>(self.registers["fifoDaq"], _depth_daq) {
                    Ok((reader, actual_depth)) => {
                        info!(
                            "Successfully opened the DAQ FIFO with actual depth {}.",
                            actual_depth
                        );
                        self._fifo_daq = Some(reader);
                    }
                    Err(error) => {
                        Self::log_error_and_panic(&format!(
                            "Failed to open the DAQ FIFO: {}.",
                            error
                        ));
                    }
                }

                match session
                    .open_read_fifo::<u8>(self.registers["fifoInbound"], _depth_inbound_outbound)
                {
                    Ok((reader, actual_depth)) => {
                        info!(
                            "Successfully opened the Inbound FIFO with actual depth {}.",
                            actual_depth
                        );
                        self._fifo_inbound = Some(reader);
                    }
                    Err(error) => {
                        Self::log_error_and_panic(&format!(
                            "Failed to open the Inbound FIFO: {}.",
                            error
                        ));
                    }
                }

                match session
                    .open_write_fifo::<u8>(self.registers["fifoOutbound"], _depth_inbound_outbound)
                {
                    Ok((writer, actual_depth)) => {
                        info!(
                            "Successfully opened the Outbound FIFO with actual depth {}.",
                            actual_depth
                        );
                        self._fifo_outbound = Some(writer);
                    }
                    Err(error) => {
                        Self::log_error_and_panic(&format!(
                            "Failed to open the Outbound FIFO: {}.",
                            error
                        ));
                    }
                }
            }
        }
    }

    /// Read the value of a control register in the FPGA.
    ///
    /// # Arguments
    /// * `name` - The name of the control register to read the value of.
    ///
    /// # Returns
    /// The value of the control register, or None if the control register is
    /// not found or the value cannot be read.
    pub fn read_control_value(&self, name: &str) -> Option<bool> {
        let _register = self.registers.get(name)?;
        #[cfg(feature = "fpga")]
        {
            if let Some(session) = self._session.as_ref() {
                match session.read::<bool>(*_register) {
                    Ok(value) => return Some(value),
                    Err(error) => {
                        error!(
                            "Failed to read the value of the control {}: {}",
                            name, error
                        );
                        return None;
                    }
                }
            }
        }

        None
    }

    /// Write the value of a control register in the FPGA.
    ///
    /// # Arguments
    /// * `name` - The name of the control register to write the value of.
    /// * `_value` - The value to write to the control register.
    ///
    /// # Returns
    /// Some(()) if the value is successfully written to the control register,
    /// or None if the control register is not found or the value cannot be
    /// written.
    pub fn write_control_value(&self, name: &str, _value: bool) -> Option<()> {
        let _register = self.registers.get(name)?;
        #[cfg(feature = "fpga")]
        {
            if let Some(session) = self._session.as_ref() {
                match session.write::<bool>(*_register, _value) {
                    Ok(_) => return Some(()),
                    Err(error) => {
                        error!(
                            "Failed to write the value of the control {}: {}",
                            name, error
                        );
                        return None;
                    }
                }
            }
        }

        None
    }

    /// Switch the value of a digital output in the FPGA.
    ///
    /// # Arguments
    /// * `digital_output` - The digital output to switch.
    /// * `status` - Digital output status.
    ///
    /// # Returns
    /// Some(()) if the value is successfully switched, or None if the control
    /// register for the digital output is not found or the value cannot be
    /// written.
    pub fn switch_digital_output(
        &mut self,
        digital_output: DigitalOutput,
        status: DigitalOutputStatus,
    ) -> Option<()> {
        let name = match digital_output {
            DigitalOutput::MotorPower => "controlIlcMotorPowerOn",
            DigitalOutput::CommunicationPower => "controlIlcCommPowerOn",
            DigitalOutput::InterlockEnable => "controlCrioInterlockEnable",
            DigitalOutput::ResetMotorBreakers => "controlResetMotorPowerBreakers",
            DigitalOutput::ResetCommunicationBreakers => "controlResetCommPowerBreakers",
            DigitalOutput::ClosedLoopControl => "controlClosedLoopControlOn",
            DigitalOutput::Spare6 => "controlMod4Ch6",
            DigitalOutput::Spare7 => "controlMod4Ch7",
        };
        let value = match status {
            DigitalOutputStatus::BinaryLowLevel => false,
            DigitalOutputStatus::BinaryHighLevel => true,
            DigitalOutputStatus::ToggleBit => {
                let current_value = (self.digital_output & digital_output.bit_value()) != 0;
                !current_value
            }
        };

        match self.write_control_value(name, value) {
            Some(()) => {
                match value {
                    true => self.digital_output |= digital_output.bit_value(),
                    false => self.digital_output &= !digital_output.bit_value(),
                }

                Some(())
            }
            None => {
                error!(
                    "Failed to switch the digital output {:?} to be {:?}",
                    digital_output, status
                );

                None
            }
        }
    }

    /// Write the data loop rate in the FPGA.
    ///
    /// # Notes
    /// This loop rate is defined in the portSerialMasterSlave.vi in
    /// ts_mtm2_cell. See the control: Loop Rate (uS). Instead of the name of
    /// rate, it is the period of the loop in microseconds actually. Keep this
    /// name to be consistent with the name of the control in the FPGA, but it
    /// may be confusing. Be careful when using this function.
    ///
    /// # Arguments
    /// * `_rate` - The data loop rate in microseconds to write to the control
    ///   register.
    ///
    /// # Returns
    /// Some(()) if the value is successfully written to the control register,
    /// or None if the control register is not found or the value cannot be
    /// written.
    pub fn write_data_loop_rate(&self, _rate: u32) -> Option<()> {
        let _register = self.registers.get("controlDataLoopRateInUs")?;
        #[cfg(feature = "fpga")]
        {
            if let Some(session) = self._session.as_ref() {
                match session.write::<u32>(*_register, _rate) {
                    Ok(_) => {
                        info!(
                            "Successfully wrote the data loop rate {} us to the control register.",
                            _rate
                        );
                        return Some(());
                    }
                    Err(error) => {
                        error!("Failed to write the data loop rate: {}", error);
                        return None;
                    }
                }
            }
        }

        None
    }

    /// Clear the DAQ FIFO in the FPGA by reading all the remaining elements in
    /// the FIFO.
    ///
    /// # Notes
    /// This function will disable the capture before reading the elements to
    /// make sure no new element is added to the FIFO during the clearing
    /// process.
    ///
    /// # Arguments
    /// * `delay_time` - The time in milliseconds to wait after disabling the
    ///   capture before reading the elements.
    pub fn clear_fifo_daq(&self, delay_time: u64) {
        // After disabling the capture, wait for the FPGA to complete one cycle
        // (plus some timing margin).
        self.write_control_value("controlEnableCapture", false);
        sleep(Duration::from_millis(delay_time));

        // Read the elements in the FIFO until there is no element remaining.
        let mut number = 0;
        loop {
            match self.read_fifo_elements_daq(number, 0) {
                Some((_, elements_remaining)) => {
                    number = elements_remaining;
                    if number == 0 {
                        // Give the DMA (Direct Memory Access) controller some
                        // time to move the data to FIFO if any.
                        sleep(Duration::from_millis(delay_time));
                        if let Some((_, final_elements_remaining)) =
                            self.read_fifo_elements_daq(0, 0)
                        {
                            if final_elements_remaining == 0 {
                                break;
                            } else {
                                number = final_elements_remaining;
                            }
                        }
                    }
                }
                None => {
                    error!("Failed to read the remaining elements in the DAQ FIFO.");

                    break;
                }
            }
        }
    }

    /// Read the elements in the DAQ FIFO in the FPGA.
    ///
    /// # Arguments
    /// * `_number` - The number of elements to read from the FIFO.
    /// * `_timeout` - The timeout in milliseconds to wait for the elements to
    ///   be read from the FIFO.
    ///
    /// # Returns
    /// A tuple containing a vector of the elements read from the FIFO and the
    /// number of elements remaining, or None if the FIFO is not found or the
    /// elements cannot be read.
    fn read_fifo_elements_daq(&self, _number: usize, _timeout: u32) -> Option<(Vec<u64>, usize)> {
        #[cfg(feature = "fpga")]
        {
            if let Some(reader) = self._fifo_daq.as_ref() {
                let mut buffer = vec![0; _number];
                match reader.read(&mut buffer, _timeout) {
                    Ok(elements_remaining) => return Some((buffer, elements_remaining)),
                    Err(error) => {
                        error!("Failed to read from the DAQ FIFO: {}", error);

                        return None;
                    }
                }
            }
        }

        None
    }

    /// Check if the DAQ FIFO in the FPGA is full.
    ///
    /// # Returns
    /// Some(true) if the FIFO is full, Some(false) if the FIFO is not full,
    /// or None if the FIFO is not found or the value cannot be read.
    pub fn is_daq_fifo_full(&self) -> Option<bool> {
        self.read_control_value("indicatorDaqFifoFull")
    }

    /// Read the power and digital input data from the DAQ FIFO in the FPGA.
    ///
    /// # Returns
    /// A tuple containing the motor power bus current (A), the communication
    /// power bus current (A), the motor power bus voltage (V), the
    /// communication power bus voltage (V), and digital input data. Or None if
    /// the FIFO is not found or the data cannot be read.
    pub fn read_power_and_digital_input(&self) -> Option<(f64, f64, f64, f64, u32)> {
        let elements_remaining = self.read_fifo_elements_daq(0, 0)?.1;
        // FPGA code writes 9 elements for each data point of power and digital
        // input.
        let number_elements_to_read = 9;
        if elements_remaining >= number_elements_to_read {
            let elements = self.read_fifo_elements_daq(number_elements_to_read, 0)?.0;
            return Some(self.process_power_data(&elements));
        }

        None
    }

    /// Process the raw data read from the DAQ FIFO in the FPGA.
    ///
    /// # Arguments
    /// * `data` - The raw data read from the DAQ FIFO, which should contain 9
    ///   elements.
    ///
    /// # Returns
    /// A tuple containing the motor power bus current (A), the communication
    /// power bus current (A), the motor power bus voltage (V), the
    /// communication power bus voltage (V), and digital input data.
    fn process_power_data(&self, data: &[u64]) -> (f64, f64, f64, f64, u32) {
        // For the data in the DAQ FIFO in the FPGA code, the FXP setting is
        // (1, 27, 16), which means the signed value with 27 bits (word length)
        // and 16 bits (integer word length).

        let digital_input = self.combine_digital_input_bits(
            self.truncate_u64_fixed_point_to_u8(data[5]),
            self.truncate_u64_fixed_point_to_u8(data[6]),
            self.truncate_u64_fixed_point_to_u8(data[7]),
            self.truncate_u64_fixed_point_to_u8(data[8]),
        );

        (
            self.truncate_u64_fixed_point_to_f64(data[0]),
            self.truncate_u64_fixed_point_to_f64(data[1]),
            self.truncate_u64_fixed_point_to_f64(data[2]),
            self.truncate_u64_fixed_point_to_f64(data[3]),
            digital_input,
        )
    }

    /// Combine the bits of the digital input into a single U32 value.
    ///
    /// # Arguments
    /// * `bits_0_to_7` - The bits 0 to 7 of the digital input.
    /// * `bits_8_to_15` - The bits 8 to 15 of the digital input.
    /// * `bits_16_to_23` - The bits 16 to 23 of the digital input.
    /// * `bits_24_to_31` - The bits 24 to 31 of the digital input.
    ///
    /// # Returns
    /// A U32 value representing the combined bits of the digital input.
    fn combine_digital_input_bits(
        &self,
        bits_0_to_7: u8,
        bits_8_to_15: u8,
        bits_16_to_23: u8,
        bits_24_to_31: u8,
    ) -> u32 {
        ((bits_24_to_31 as u32) << 24)
            | ((bits_16_to_23 as u32) << 16)
            | ((bits_8_to_15 as u32) << 8)
            | (bits_0_to_7 as u32)
    }

    /// Truncate the raw U64 fixed-point value read from the DAQ FIFO in the
    /// FPGA to a U8 value.
    ///
    /// # Notes
    /// There is a truncation here because only 8 bits are used for the
    /// fixed-point value for a digital input element in the FPGA code.
    ///
    /// # Arguments
    /// * `fixed_point_u64` - The raw U64 fixed-point value read from the DAQ
    ///   FIFO.
    ///
    /// # Returns
    /// The truncated U8 value.
    fn truncate_u64_fixed_point_to_u8(&self, fixed_point_u64: u64) -> u8 {
        // Truncate to 32 bits first. The upper 37 (= 64 - 27) bits are just
        // zeros from the FIFO padding.
        let fixed_point_u32 = fixed_point_u64 as u32;

        // Use the unsigned fixed-point data type here because for the digital
        // input element, it is a U8 value actually in the FPGA code.
        let fixed_point = U21F11::from_bits(fixed_point_u32);

        fixed_point.to_num::<u8>()
    }

    /// Truncate the raw U64 fixed-point value read from the DAQ FIFO in the
    /// FPGA to a F64 value.
    ///
    /// # Notes
    /// There is a truncation here because only 27 bits are used for the
    /// fixed-point value for a floating value in the FPGA code.
    ///
    /// # Arguments
    /// * `fixed_point_u64` - The raw U64 fixed-point value read from the DAQ
    ///   FIFO.
    ///
    /// # Returns
    /// The truncated F64 value.
    fn truncate_u64_fixed_point_to_f64(&self, fixed_point_u64: u64) -> f64 {
        // Truncate to 32 bits first. The upper 37 (= 64 - 27) bits are just
        // zeros from the FIFO padding.
        let mut fixed_point_i32 = fixed_point_u64 as i32;

        // Check the sign bit (the 27th bit) and extend the sign if it is negative.
        let sign_bit = 1 << 26;
        if (fixed_point_i32 & sign_bit) != 0 {
            fixed_point_i32 |= !0 << 27;
        }

        let fixed_point = I21F11::from_bits(fixed_point_i32);

        fixed_point.to_num::<f64>()
    }
}

impl Drop for FpgaWrapper {
    fn drop(&mut self) {
        #[cfg(feature = "fpga")]
        {
            if self._fifo_daq.is_some() {
                self._fifo_daq = None;
            }

            if self._fifo_inbound.is_some() {
                self._fifo_inbound = None;
            }

            if self._fifo_outbound.is_some() {
                self._fifo_outbound = None;
            }

            info!("Closed the FIFOs.");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use approx::assert_relative_eq;

    use crate::daq::config_data_acquisition::ConfigDataAcquisition;

    const EPSILON_FIXED_POINT: f64 = 1e-3;

    fn create_fpga_wrapper() -> FpgaWrapper {
        let config = ConfigDataAcquisition::new();
        FpgaWrapper::new(config.path_header.as_path())
    }

    #[test]
    fn test_new() {
        let fpga_wrapper = create_fpga_wrapper();

        assert_eq!(fpga_wrapper.registers.len(), 15);
    }

    #[test]
    fn test_create_register_map() {
        let config = ConfigDataAcquisition::new();
        let keys = ["controlIlcCommPowerOn", "controlIlcMotorPowerOn"];
        let names = [
            "NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On",
            "NiFpga_portSerialMasterSlave_ControlBool_ILC_Motor_Power_On",
        ];

        let register_map =
            FpgaWrapper::create_register_map(config.path_header.as_path(), &keys, &names);

        assert_eq!(register_map.len(), 2);
        assert_eq!(register_map["controlIlcCommPowerOn"], 0x18056);
        assert_eq!(register_map["controlIlcMotorPowerOn"], 0x1805A);
    }

    #[test]
    fn test_get_register() {
        let config = ConfigDataAcquisition::new();
        let header_path = config.path_header.as_path();

        assert_eq!(
            FpgaWrapper::get_register(
                header_path,
                "NiFpga_portSerialMasterSlave_ControlBool_EnableCapture"
            ),
            Some(0x1803E)
        );
        assert_eq!(
            FpgaWrapper::get_register(header_path, "NiFpga_portSerialMasterSlave_ControlBool_stop"),
            Some(0x18046)
        );
        assert_eq!(
            FpgaWrapper::get_register(
                header_path,
                "NiFpga_portSerialMasterSlave_TargetToHostFifoU8_Inbound_FIFO"
            ),
            Some(3)
        );

        assert_eq!(
            FpgaWrapper::get_register(
                header_path,
                "NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_Resource"
            ),
            Some(0x18004)
        );

        assert!(FpgaWrapper::get_register(header_path, "NonExistentRegister").is_none());
    }

    #[test]
    fn test_get_signature() {
        let fpga_wrapper = create_fpga_wrapper();

        let bitfile = Path::new("fpga/NiFpga_portSerialMasterSlave.lvbitx");
        let signature = fpga_wrapper.get_signature(bitfile);

        assert_eq!(
            signature,
            Some("BCA48EBD4CE9E2D0FA6A1B6BC2636EC6".to_string())
        );
    }

    #[test]
    fn test_format_serial_config() {
        let fpga_wrapper = create_fpga_wrapper();
        let message = fpga_wrapper._format_serial_config(20, 0, 0, 3, 0);
        assert_eq!(
            message,
            "Serial config - Baud rate: 921.6 kbps, Parity: None, Flow control: None, Data bits: 8, Stop bits: 1."
        );
    }

    #[test]
    fn test_switch_digital_output_fail() {
        let mut fpga_wrapper = create_fpga_wrapper();

        assert!(fpga_wrapper
            .switch_digital_output(
                DigitalOutput::MotorPower,
                DigitalOutputStatus::BinaryHighLevel
            )
            .is_none());
    }

    #[test]
    fn test_process_power_data() {
        let fpga_wrapper = create_fpga_wrapper();

        // Test data
        let data_test = [
            0b000000000000_0000_00000000000,
            0b111111111111_1111_00000000000,
            0b000000000000_0001_00000000000,
            0b111111111111_1110_00000000000,
            0,
            0b11111101_00000000000,
            0b11111011_00000000000,
            0b11110111_00000000000,
            0b11101111_00000000000,
        ];

        let result_test = fpga_wrapper.process_power_data(&data_test);

        assert_relative_eq!(result_test.0, 0.0, epsilon = EPSILON_FIXED_POINT);
        assert_relative_eq!(result_test.1, -1.0, epsilon = EPSILON_FIXED_POINT);
        assert_relative_eq!(result_test.2, 1.0, epsilon = EPSILON_FIXED_POINT);
        assert_relative_eq!(result_test.3, -2.0, epsilon = EPSILON_FIXED_POINT);
        assert_eq!(result_test.4, 0b11101111_11110111_11111011_11111101);

        // Data from the cRIO simulator
        let data_crio = [
            0b1111111111111111_11111110110,
            0b0000000000000000_00000010110,
            0b0000000000000000_00001011000,
            0b0000000000000000_00000011100,
            0,
            0b11001111_00000000000,
            0b11111111_00000000000,
            0b00000000_00000000000,
            0b10011111_00000000000,
        ];

        let result_crio = fpga_wrapper.process_power_data(&data_crio);

        assert_relative_eq!(result_crio.0, -0.0048828125, epsilon = EPSILON_FIXED_POINT);
        assert_relative_eq!(result_crio.1, 0.0107421875, epsilon = EPSILON_FIXED_POINT);
        assert_relative_eq!(result_crio.2, 0.04296875, epsilon = EPSILON_FIXED_POINT);
        assert_relative_eq!(result_crio.3, 0.013671875, epsilon = EPSILON_FIXED_POINT);
        assert_eq!(result_crio.4, 0b10011111_00000000_11111111_11001111);
    }

    #[test]
    fn test_combine_digital_input_bits() {
        let fpga_wrapper = create_fpga_wrapper();

        let combined = fpga_wrapper.combine_digital_input_bits(0x12, 0x34, 0x56, 0x78);

        assert_eq!(combined, 0x78563412);
    }

    #[test]
    fn test_truncate_u64_fixed_point_to_u8() {
        let fpga_wrapper = create_fpga_wrapper();

        let fraction = 11;
        let fixed_point_u64 = 0b11011101 << fraction;
        let truncated = fpga_wrapper.truncate_u64_fixed_point_to_u8(fixed_point_u64);

        assert_eq!(truncated, 0b11011101);
    }

    #[test]
    fn test_truncate_u64_fixed_point_to_f64() {
        let fpga_wrapper = create_fpga_wrapper();

        // Test with a positive value.
        let positive_value_f64 = 13.7;
        let positive_value_fixed_point = I21F11::from_num(positive_value_f64);
        let positive_value_fixed_point_u64 = positive_value_fixed_point.to_bits() as u64;

        assert_eq!(positive_value_fixed_point_u64, 0b1101_10110011010);

        let positive_value_f64_truncated =
            fpga_wrapper.truncate_u64_fixed_point_to_f64(positive_value_fixed_point_u64);

        assert_relative_eq!(
            positive_value_f64,
            positive_value_f64_truncated,
            epsilon = EPSILON_FIXED_POINT
        );

        // Test with a negative value. Only 27 bits (0x07FFFFFF) are used in
        // the FPGA.
        let negative_value_f64 = -13.7;
        let negative_value_fixed_point = I21F11::from_num(negative_value_f64);
        let negative_value_fixed_point_u64 =
            (negative_value_fixed_point.to_bits() as u64) & 0x07FFFFFF;

        assert_eq!(
            negative_value_fixed_point_u64,
            0b111111111111_0010_01001100110
        );

        let negative_value_f64_truncated =
            fpga_wrapper.truncate_u64_fixed_point_to_f64(negative_value_fixed_point_u64);

        assert_relative_eq!(
            negative_value_f64,
            negative_value_f64_truncated,
            epsilon = EPSILON_FIXED_POINT
        );
    }

    #[cfg(feature = "fpga")]
    mod fpga_hardware {
        use super::*;

        fn create_fpga_wrapper_and_open(depth: usize) -> FpgaWrapper {
            let mut fpga_wrapper = create_fpga_wrapper();

            let config = ConfigDataAcquisition::new();
            fpga_wrapper.open(&config.path_bitfile, &config.fpga_resource);
            fpga_wrapper.open_fifo(depth, depth);

            fpga_wrapper
        }

        fn read_data_loop_rate(fpga_wrapper: &FpgaWrapper) -> Option<u32> {
            let register = fpga_wrapper.registers.get("controlDataLoopRateInUs")?;
            if let Some(session) = fpga_wrapper._session.as_ref() {
                match session.read::<u32>(*register) {
                    Ok(value) => return Some(value),
                    Err(_) => return None,
                }
            }

            None
        }

        fn capture_data_in_fifo_daq(fpga_wrapper: &FpgaWrapper) {
            fpga_wrapper.write_data_loop_rate(200);

            fpga_wrapper.write_control_value("controlEnableCapture", true);
            sleep(Duration::from_millis(100));
            fpga_wrapper.write_control_value("controlEnableCapture", false);
        }

        #[test]
        fn test_open() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            assert!(fpga_wrapper._session.is_some());
        }

        #[test]
        fn test_log_serial_config() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            assert_eq!(fpga_wrapper.log_serial_config(), Some([0, 20, 0, 0, 3, 0]));
        }

        #[test]
        fn test_read_control_value() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            assert_eq!(
                fpga_wrapper.read_control_value("controlMod4Ch7"),
                Some(false)
            );
            assert!(fpga_wrapper.read_control_value("noThisControl").is_none());
        }

        #[test]
        fn test_write_control_value() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            assert_eq!(
                fpga_wrapper.read_control_value("controlMod4Ch7"),
                Some(false)
            );

            assert!(fpga_wrapper
                .write_control_value("controlMod4Ch7", true)
                .is_some());
            assert_eq!(
                fpga_wrapper.read_control_value("controlMod4Ch7"),
                Some(true)
            );

            assert!(fpga_wrapper
                .write_control_value("controlMod4Ch7", false)
                .is_some());
            assert_eq!(
                fpga_wrapper.read_control_value("controlMod4Ch7"),
                Some(false)
            );

            assert!(fpga_wrapper
                .write_control_value("noThisControl", false)
                .is_none());
        }

        #[test]
        fn test_switch_digital_output() {
            let mut fpga_wrapper = create_fpga_wrapper_and_open(90);

            assert_eq!(fpga_wrapper.digital_output, 0);

            assert!(fpga_wrapper
                .switch_digital_output(
                    DigitalOutput::MotorPower,
                    DigitalOutputStatus::BinaryHighLevel
                )
                .is_some());
            assert_eq!(fpga_wrapper.digital_output, 1);

            assert!(fpga_wrapper
                .switch_digital_output(
                    DigitalOutput::CommunicationPower,
                    DigitalOutputStatus::BinaryHighLevel
                )
                .is_some());
            assert_eq!(fpga_wrapper.digital_output, 3);

            assert!(fpga_wrapper
                .switch_digital_output(
                    DigitalOutput::MotorPower,
                    DigitalOutputStatus::BinaryLowLevel
                )
                .is_some());
            assert_eq!(fpga_wrapper.digital_output, 2);

            assert!(fpga_wrapper
                .switch_digital_output(
                    DigitalOutput::CommunicationPower,
                    DigitalOutputStatus::BinaryLowLevel
                )
                .is_some());
            assert_eq!(fpga_wrapper.digital_output, 0);

            assert!(fpga_wrapper
                .switch_digital_output(
                    DigitalOutput::InterlockEnable,
                    DigitalOutputStatus::BinaryHighLevel
                )
                .is_some());
            assert_eq!(fpga_wrapper.digital_output, 4);

            assert!(fpga_wrapper
                .switch_digital_output(
                    DigitalOutput::InterlockEnable,
                    DigitalOutputStatus::BinaryLowLevel
                )
                .is_some());
            assert_eq!(fpga_wrapper.digital_output, 0);

            assert!(fpga_wrapper
                .switch_digital_output(
                    DigitalOutput::InterlockEnable,
                    DigitalOutputStatus::ToggleBit
                )
                .is_some());
            assert_eq!(fpga_wrapper.digital_output, 4);

            assert!(fpga_wrapper
                .switch_digital_output(
                    DigitalOutput::InterlockEnable,
                    DigitalOutputStatus::ToggleBit
                )
                .is_some());
            assert_eq!(fpga_wrapper.digital_output, 0);
        }

        #[test]
        fn test_write_data_loop_rate() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            let current_loop_rate = read_data_loop_rate(&fpga_wrapper).unwrap();

            assert!(fpga_wrapper
                .write_data_loop_rate(2 * current_loop_rate)
                .is_some());
            assert_eq!(
                read_data_loop_rate(&fpga_wrapper).unwrap(),
                2 * current_loop_rate
            );

            assert!(fpga_wrapper
                .write_data_loop_rate(current_loop_rate)
                .is_some());
            assert_eq!(
                read_data_loop_rate(&fpga_wrapper).unwrap(),
                current_loop_rate
            );
        }

        #[test]
        fn test_clear_fifo_daq() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            capture_data_in_fifo_daq(&fpga_wrapper);
            fpga_wrapper.clear_fifo_daq(1000);

            assert_eq!(fpga_wrapper.read_fifo_elements_daq(0, 0).unwrap().1, 0);
        }

        #[test]
        fn test_read_fifo_elements_daq() {
            let fpga_wrapper = create_fpga_wrapper_and_open(4500);

            capture_data_in_fifo_daq(&fpga_wrapper);

            assert!(fpga_wrapper.read_fifo_elements_daq(0, 0).unwrap().1 >= 4500);
        }

        #[test]
        fn test_is_daq_fifo_full() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            capture_data_in_fifo_daq(&fpga_wrapper);

            assert_eq!(fpga_wrapper.is_daq_fifo_full(), Some(true));
        }

        #[test]
        fn test_read_power_and_digital_input() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            capture_data_in_fifo_daq(&fpga_wrapper);

            assert!(fpga_wrapper.read_power_and_digital_input().is_some());
        }
    }
}
