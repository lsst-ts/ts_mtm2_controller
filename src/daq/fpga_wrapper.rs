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

use crate::enums::{
    CustomFpgaModbusError, DigitalOutput, DigitalOutputStatus, IlcCommand, ModbusMode,
};
use ts_control_utils::enums::BitEnum;

#[cfg(feature = "fpga")]
use crate::constants::{IRQ_NUMBER_ILC, NUMBER_ILC_PORT};

#[cfg(feature = "fpga")]
use nifpga_dll::{Context, ReadFifo, Session, Type, WriteFifo};

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
    #[cfg(feature = "fpga")]
    // Interrupt request (IRQ) context for waiting on the IRQ from the FPGA
    _irq_context: Option<Context>,
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
            "controlPort",
            "controlCommandForFpga",
            "controlWriteFifoPaceTicks",
            "indicatorDaqFifoFull",
            "indicatorErrorOut",
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
            "NiFpga_portSerialMasterSlave_ControlU8_Port",
            "NiFpga_portSerialMasterSlave_ControlU16_CmdforFPGA",
            "NiFpga_portSerialMasterSlave_ControlU16_WriteFIFOpaceticks",
            "NiFpga_portSerialMasterSlave_IndicatorBool_DataFIFOFull",
            "NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_Resource",
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

            #[cfg(feature = "fpga")]
            _irq_context: None,
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
    /// Some(()) if the value is successfully logged, or None if the control
    /// register for the serial config resource is not found or the value
    /// cannot be read.
    pub fn log_serial_config(&self) -> Option<()> {
        let config = self.get_serial_config()?;
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

        Some(())
    }

    /// Get the value of the serial config resource in the FPGA.
    ///
    /// # Returns
    /// The value of the serial config resource, or None if the control
    /// register is not found or the value cannot be read.
    fn get_serial_config(&self) -> Option<[u8; 6]> {
        let _register = self.registers.get("controlSerialConfigResource")?;
        #[cfg(feature = "fpga")]
        {
            if let Some(session) = self._session.as_ref() {
                let mut config = [0; 6];
                match session.read_array::<u8>(*_register, &mut config) {
                    Ok(()) => {
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
    /// ts_mtm2_cell. And see the fpga/NiFpga_portSerialMasterSlave.c for the
    /// function:
    /// NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_UnpackCluster()
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

    /// Configure the serial config resource in the FPGA according to the given
    /// Modbus mode.
    ///
    /// # Arguments
    /// * `mode` - The Modbus mode to configure the serial config resource
    ///   for.
    /// * `_timeout` - The timeout in milliseconds to wait for the interrupt
    ///   request (IRQ).
    ///
    /// # Returns
    /// Some(()) if the serial config resource is successfully configured, or
    /// None if the control register is not found or the value cannot be read
    /// or written.
    pub fn configure_serial_config(&self, mode: ModbusMode, _timeout: u32) -> Option<()> {
        // See the self._format_serial_config() for the mapping between the
        // value of each field in the serial config resource and the actual
        // data bits.
        let mut _config = self.get_serial_config()?;
        match mode {
            ModbusMode::Ascii => {
                _config[4] = 2; // 7 data bits
            }
            ModbusMode::Rtu => {
                _config[4] = 3; // 8 data bits
            }
        }

        let _register = self.registers.get("controlSerialConfigResource")?;
        #[cfg(feature = "fpga")]
        {
            if let Some(session) = self._session.as_ref() {
                match session.write_array::<u8>(*_register, &_config) {
                    Ok(()) => {
                        info!("Successfully wrote the value of the control controlSerialConfigResource.");
                    }
                    Err(error) => {
                        error!("Failed to write the value of the control controlSerialConfigResource: {}",
                            error
                        );
                        return None;
                    }
                }
            }

            for idx in 1..(NUMBER_ILC_PORT + 1) {
                self.write_control_value_u8("controlPort", idx)?;
                self.command_ilc(IlcCommand::Config, IRQ_NUMBER_ILC, _timeout)?;
            }
        }

        Some(())
    }

    /// Send a command to the inner-loop controller (ILC) and wait for the
    /// interrupt request (IRQ) from the FPGA indicating the command is
    /// executed.
    ///
    /// # Arguments
    /// * `command` - The command to send to the ILC.
    /// * `_irq_number` - The IRQ number to wait for from the FPGA.
    /// * `_timeout` - The timeout in milliseconds to wait for the IRQ.
    ///
    /// # Returns
    /// Some(CustomFpgaModbusError) if the command is successfully sent and the
    /// expected IRQ is received, or None if the control register is not found,
    /// the value cannot be written, or the expected IRQ is not received within
    /// the timeout. The CustomFpgaModbusError value indicates the error code
    /// from the ILC if an error occurs.
    pub fn command_ilc(
        &self,
        command: IlcCommand,
        _irq_number: u32,
        _timeout: u32,
    ) -> Option<CustomFpgaModbusError> {
        self.write_control_value_u16("controlCommandForFpga", command as u16)?;
        #[cfg(feature = "fpga")]
        {
            if let Some(session) = self._session.as_ref() {
                if let Err(error) = session.acknowledge_irqs(_irq_number) {
                    error!(
                        "Failed to acknowledge IRQ number {}: {}",
                        _irq_number, error
                    );
                    return None;
                }

                if let Some(context) = self._irq_context.as_ref() {
                    match context.wait_on_irqs(_irq_number, _timeout) {
                        Ok((irqs_asserted, is_timed_out)) => {
                            if irqs_asserted != _irq_number {
                                error!("Received an IRQ, but it is not the expected IRQ number {}. Actual IRQs asserted: {}.",
                                    _irq_number, irqs_asserted
                                );
                                return None;
                            }

                            if is_timed_out {
                                error!("Timed out while waiting for IRQ number {}.", _irq_number);
                                return None;
                            }
                        }
                        Err(error) => {
                            error!("Failed to wait for IRQ number {}: {}", _irq_number, error);
                            return None;
                        }
                    }
                } else {
                    error!("Failed to get the IRQ context.");
                    return None;
                }

                return self.read_ilc_error_code();
            }
        }

        Some(CustomFpgaModbusError::None)
    }

    /// Read the inner-loop controller (ILC) error code from the FPGA after
    /// sending a command to the ILC.
    ///
    /// # Returns
    /// Some(CustomFpgaModbusError) if the ILC error code is successfully read
    /// from the FPGA, or None if the control register for the ILC error code
    /// is not found or the value cannot be read. The CustomFpgaModbusError
    /// value indicates the error code from the ILC.
    pub fn read_ilc_error_code(&self) -> Option<CustomFpgaModbusError> {
        let _register = self.registers.get("indicatorErrorOut")?;
        #[cfg(feature = "fpga")]
        {
            if let Some(session) = self._session.as_ref() {
                let mut ilc_error = [0; 5];
                match session.read_array::<u8>(*_register, &mut ilc_error) {
                    Ok(()) => {
                        return Some(self.map_ilc_error_code(&ilc_error));
                    }
                    Err(error) => {
                        error!(
                            "Failed to read the value of the indicator indicatorErrorOut: {}",
                            error
                        );
                        return None;
                    }
                }
            }
        }

        Some(CustomFpgaModbusError::None)
    }

    /// Map the raw inner-loop controller (ILC) error code read from the FPGA
    /// into a CustomFpgaModbusError.
    ///
    /// # Arguments
    /// * `ilc_error` - The raw ILC error code read from the FPGA, represented
    ///   as a byte array of length 5.
    ///
    /// Returns
    /// The mapped CustomFpgaModbusError based on the raw ILC error code. If
    /// the raw ILC error code indicates no error, returns
    /// CustomFpgaModbusError::None. If the raw ILC error code indicates an
    /// error, returns the corresponding CustomFpgaModbusError variant. If the
    /// raw ILC error code indicates an error but does not match any known
    /// error code, returns CustomFpgaModbusError::Unknown.
    pub fn map_ilc_error_code(&self, ilc_error: &[u8; 5]) -> CustomFpgaModbusError {
        let (has_error, code) = self.map_error_out(ilc_error);
        if has_error {
            let error_code =
                CustomFpgaModbusError::from_repr(code).unwrap_or(CustomFpgaModbusError::Unknown);

            error!(
                "Received the code value: {} as the ILC error: {:?}",
                code, error_code
            );
            error_code
        } else {
            CustomFpgaModbusError::None
        }
    }

    /// Map the raw error output read from the FPGA into a tuple of
    /// (has_error, code).
    ///
    /// # Notes
    /// See the fpga/NiFpga_portSerialMasterSlave.c for the function:
    /// NiFpga_portSerialMasterSlave_IndicatorCluster_FPGAErrorOut_UnpackCluster()
    ///
    /// # Arguments
    /// * `error_out` - The raw error output read from the FPGA, represented
    ///   as a byte array of length 5.
    ///
    /// # Returns
    /// A tuple of (has_error, code), where has_error is a boolean
    /// indicating whether an error is present, and code is the extracted
    /// error code as a 32-bit integer if has_error is true, or 0 if has_error
    /// is false.
    fn map_error_out(&self, error_out: &[u8; 5]) -> (bool, i32) {
        // Extract the status (1 bit)
        let has_error = ((error_out[0] >> 7) & 0x1) != 0;

        // Extract the code (32 bits)
        let mut code: u32 = 0;
        if has_error {
            // We must cast to u32 before shifting, otherwise Rust will panic
            // for trying to shift a u8 by more than 7 bits.
            code |= ((error_out[0] & 0x7F) as u32) << 25;
            code |= (error_out[1] as u32) << 17;
            code |= (error_out[2] as u32) << 9;
            code |= (error_out[3] as u32) << 1;
            code |= ((error_out[4] >> 7) & 0x1) as u32;
        };

        (has_error, code as i32)
    }

    /// Reserve an interrupt request (IRQ) context for waiting on IRQs from the
    /// FPGA.
    ///
    /// # Panics
    /// Panics if the IRQ context cannot be reserved.
    pub fn reserve_irq_context(&mut self) {
        #[cfg(feature = "fpga")]
        {
            if let Some(session) = self._session.as_ref() {
                match session.reserve_irq_context() {
                    Ok(context) => {
                        info!("Successfully reserved the IRQ context.");
                        self._irq_context = Some(context);
                    }
                    Err(error) => {
                        Self::log_error_and_panic(&format!(
                            "Failed to reserve the IRQ context: {}.",
                            error
                        ));
                    }
                }
            }
        }
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

    /// Read the boolean value of a control register in the FPGA.
    ///
    /// # Notes
    /// This is a thin typed wrapper around a private generic helper. Keeping
    /// the public API concrete avoids exposing nifpga-dll trait bounds.
    ///
    /// # Arguments
    /// * `name` - The name of the control register to read the value of.
    ///
    /// # Returns
    /// The value of the control register, or None if the control register is
    /// not found or the value cannot be read.
    pub fn read_control_value_bool(&self, name: &str) -> Option<bool> {
        self.read_control_value::<bool>(name)
    }

    /// Read the 8-bit unsigned integer value of a control register in the
    /// FPGA.
    ///
    /// # Notes
    /// This is a thin typed wrapper around a private generic helper. Keeping
    /// the public API concrete avoids exposing nifpga-dll trait bounds.
    ///
    /// # Arguments
    /// * `name` - The name of the control register to read the value of.
    ///
    /// # Returns
    /// The value of the control register, or None if the control register is
    /// not found or the value cannot be read.
    pub fn read_control_value_u8(&self, name: &str) -> Option<u8> {
        self.read_control_value::<u8>(name)
    }

    /// Read the 16-bit unsigned integer value of a control register in the
    /// FPGA.
    ///
    /// # Notes
    /// This is a thin typed wrapper around a private generic helper. Keeping
    /// the public API concrete avoids exposing nifpga-dll trait bounds.
    ///
    /// # Arguments
    /// * `name` - The name of the control register to read the value of.
    ///
    /// # Returns
    /// The value of the control register, or None if the control register is
    /// not found or the value cannot be read.
    pub fn read_control_value_u16(&self, name: &str) -> Option<u16> {
        self.read_control_value::<u16>(name)
    }

    /// Read the 32-bit unsigned integer value of a control register in the
    /// FPGA.
    ///
    /// # Notes
    /// This is a thin typed wrapper around a private generic helper. Keeping
    /// the public API concrete avoids exposing nifpga-dll trait bounds.
    ///
    /// # Arguments
    /// * `name` - The name of the control register to read the value of.
    ///
    /// # Returns
    /// The value of the control register, or None if the control register is
    /// not found or the value cannot be read.
    pub fn read_control_value_u32(&self, name: &str) -> Option<u32> {
        self.read_control_value::<u32>(name)
    }

    /// Read the value of a control register in the FPGA.
    ///
    /// # Notes
    /// Keep this function private so that the public API does not expose the
    /// generic bound from nifpga-dll.
    ///
    /// # Arguments
    /// * `name` - The name of the control register to read the value of.
    ///
    /// # Returns
    /// The value of the control register, or None if the control register is
    /// not found or the value cannot be read.
    #[cfg(feature = "fpga")]
    fn read_control_value<T: Type>(&self, name: &str) -> Option<T> {
        let register = self.registers.get(name)?;
        if let Some(session) = self._session.as_ref() {
            match session.read::<T>(*register) {
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

        None
    }

    /// Read the value of a control register in the FPGA.
    ///
    /// # Notes
    /// This stub keeps non-fpga builds independent from nifpga-dll.
    ///
    /// # Arguments
    /// * `_name` - The name of the control register to read the value of.
    ///
    /// # Returns
    /// The value of the control register, or None if the control register is
    /// not found or the value cannot be read.
    #[cfg(not(feature = "fpga"))]
    fn read_control_value<T>(&self, _name: &str) -> Option<T> {
        None
    }

    /// Write the boolean value of a control register in the FPGA.
    ///
    /// # Notes
    /// This is a thin typed wrapper around a private generic helper. Keeping
    /// the public API concrete avoids exposing nifpga-dll trait bounds.
    ///
    /// # Arguments
    /// * `name` - The name of the control register to write the value of.
    /// * `value` - The value to write to the control register.
    ///
    /// # Returns
    /// Some(()) if the value is successfully written to the control register,
    /// or None if the control register is not found or the value cannot be
    /// written.
    pub fn write_control_value_bool(&self, name: &str, value: bool) -> Option<()> {
        self.write_control_value::<bool>(name, value)
    }

    /// Write the 8-bit unsigned integer value of a control register in the
    /// FPGA.
    ///
    /// # Notes
    /// This is a thin typed wrapper around a private generic helper. Keeping
    /// the public API concrete avoids exposing nifpga-dll trait bounds.
    ///
    /// # Arguments
    /// * `name` - The name of the control register to write the value of.
    /// * `value` - The value to write to the control register.
    ///
    /// # Returns
    /// Some(()) if the value is successfully written to the control register,
    /// or None if the control register is not found or the value cannot be
    /// written.
    pub fn write_control_value_u8(&self, name: &str, value: u8) -> Option<()> {
        self.write_control_value::<u8>(name, value)
    }

    /// Write the 16-bit unsigned integer value of a control register in the
    /// FPGA.
    ///
    /// # Notes
    /// This is a thin typed wrapper around a private generic helper. Keeping
    /// the public API concrete avoids exposing nifpga-dll trait bounds.
    ///
    /// # Arguments
    /// * `name` - The name of the control register to write the value of.
    /// * `value` - The value to write to the control register.
    ///
    /// # Returns
    /// Some(()) if the value is successfully written to the control register,
    /// or None if the control register is not found or the value cannot be
    /// written.
    pub fn write_control_value_u16(&self, name: &str, value: u16) -> Option<()> {
        self.write_control_value::<u16>(name, value)
    }

    /// Write the 32-bit unsigned integer value of a control register in the
    /// FPGA.
    ///
    /// # Notes
    /// This is a thin typed wrapper around a private generic helper. Keeping
    /// the public API concrete avoids exposing nifpga-dll trait bounds.
    ///
    /// # Arguments
    /// * `name` - The name of the control register to write the value of.
    /// * `value` - The value to write to the control register.
    ///
    /// # Returns
    /// Some(()) if the value is successfully written to the control register,
    /// or None if the control register is not found or the value cannot be
    /// written.
    pub fn write_control_value_u32(&self, name: &str, value: u32) -> Option<()> {
        self.write_control_value::<u32>(name, value)
    }

    /// Write the value of a control register in the FPGA.
    ///
    /// # Notes
    /// Keep this function private so that the public API does not expose the
    /// generic bound from nifpga-dll.
    ///
    /// # Arguments
    /// * `name` - The name of the control register to write the value of.
    /// * `value` - The value to write to the control register.
    ///
    /// # Returns
    /// Some(()) if the value is successfully written to the control register,
    /// or None if the control register is not found or the value cannot be
    /// written.
    #[cfg(feature = "fpga")]
    fn write_control_value<T: Type>(&self, name: &str, value: T) -> Option<()> {
        let register = self.registers.get(name)?;
        if let Some(session) = self._session.as_ref() {
            match session.write::<T>(*register, value) {
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

        None
    }

    /// Write the value of a control register in the FPGA.
    ///
    /// # Notes
    /// This stub keeps non-fpga builds independent from nifpga-dll.
    ///
    /// # Arguments
    /// * `_name` - The name of the control register to write the value of.
    /// * `_value` - The value to write to the control register.
    ///
    /// # Returns
    /// Some(()) if the value is successfully written to the control register,
    /// or None if the control register is not found or the value cannot be
    /// written.
    #[cfg(not(feature = "fpga"))]
    fn write_control_value<T>(&self, _name: &str, _value: T) -> Option<()> {
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

        match self.write_control_value_bool(name, value) {
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
        self.write_control_value_bool("controlEnableCapture", false);
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
        self.read_control_value_bool("indicatorDaqFifoFull")
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

            if self._irq_context.is_some() {
                self._irq_context = None;
            }

            info!("Closed the IRQ context.");
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

        assert_eq!(fpga_wrapper.registers.len(), 19);
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
            )
            .unwrap(),
            0x1803E
        );
        assert_eq!(
            FpgaWrapper::get_register(header_path, "NiFpga_portSerialMasterSlave_ControlBool_stop")
                .unwrap(),
            0x18046
        );
        assert_eq!(
            FpgaWrapper::get_register(
                header_path,
                "NiFpga_portSerialMasterSlave_TargetToHostFifoU8_Inbound_FIFO"
            )
            .unwrap(),
            3
        );

        assert_eq!(
            FpgaWrapper::get_register(
                header_path,
                "NiFpga_portSerialMasterSlave_ControlCluster_SerialConfig_Resource"
            )
            .unwrap(),
            0x18004
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
    fn test_map_ilc_error_code() {
        let fpga_wrapper = create_fpga_wrapper();

        // Test with no error.
        let ilc_error_no_error = [0b00000000, 0, 0, 0, 0];
        assert_eq!(
            fpga_wrapper.map_ilc_error_code(&ilc_error_no_error),
            CustomFpgaModbusError::None
        );

        // Test with a known error code.
        let ilc_error_known = [0b10000000, 0, 0b00001001, 0b11001001, 0b00000000];
        assert_eq!(
            fpga_wrapper.map_ilc_error_code(&ilc_error_known),
            CustomFpgaModbusError::NoRespond
        );

        // Test with an unknown error code.
        let ilc_error_unknown = [0b11111111, 0, 0, 0, 0];
        assert_eq!(
            fpga_wrapper.map_ilc_error_code(&ilc_error_unknown),
            CustomFpgaModbusError::Unknown
        );
    }

    #[test]
    fn test_map_error_out() {
        let fpga_wrapper = create_fpga_wrapper();

        assert_eq!(fpga_wrapper.map_error_out(&[0; 5]), (false, 0));
        assert_eq!(
            fpga_wrapper
                .map_error_out(&[0b10110000, 0b01110000, 0b01111000, 0b01111100, 0b10000000]),
            (true, 0b01100000_11100000_11110000_11111001)
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

        use crate::constants::IRQ_NUMBER_ILC;

        fn create_fpga_wrapper_and_open(depth: usize) -> FpgaWrapper {
            let mut fpga_wrapper = create_fpga_wrapper();

            let config = ConfigDataAcquisition::new();
            fpga_wrapper.open(&config.path_bitfile, &config.fpga_resource);
            fpga_wrapper.reserve_irq_context();
            fpga_wrapper.open_fifo(depth, depth);

            fpga_wrapper
        }

        fn capture_data_in_fifo_daq(fpga_wrapper: &FpgaWrapper) {
            fpga_wrapper.write_control_value_u32("controlDataLoopRateInUs", 200);

            fpga_wrapper.write_control_value_bool("controlEnableCapture", true);
            sleep(Duration::from_millis(100));
            fpga_wrapper.write_control_value_bool("controlEnableCapture", false);
        }

        #[test]
        fn test_open() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            assert!(fpga_wrapper._session.is_some());
        }

        #[test]
        fn test_get_serial_config() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            assert_eq!(
                fpga_wrapper.get_serial_config().unwrap(),
                [0, 20, 0, 0, 3, 0]
            );
        }

        #[test]
        fn test_configure_serial_config() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            assert!(fpga_wrapper
                .configure_serial_config(ModbusMode::Ascii, 50)
                .is_some());
            assert_eq!(
                fpga_wrapper.get_serial_config().unwrap(),
                [0, 20, 0, 0, 2, 0]
            );

            assert!(fpga_wrapper
                .configure_serial_config(ModbusMode::Rtu, 50)
                .is_some());
            assert_eq!(
                fpga_wrapper.get_serial_config().unwrap(),
                [0, 20, 0, 0, 3, 0]
            );
        }

        #[test]
        fn test_command_ilc() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            assert_eq!(
                fpga_wrapper
                    .command_ilc(IlcCommand::DoNothing, IRQ_NUMBER_ILC, 1000)
                    .unwrap(),
                CustomFpgaModbusError::None
            );
            assert!(fpga_wrapper
                .command_ilc(IlcCommand::DoNothing, 1, 1000)
                .is_none());
        }

        #[test]
        fn test_read_ilc_error_code() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            assert_eq!(
                fpga_wrapper.read_ilc_error_code().unwrap(),
                CustomFpgaModbusError::None
            );
        }

        #[test]
        fn test_read_control_value_bool() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            assert_eq!(
                fpga_wrapper.read_control_value_bool("controlMod4Ch7"),
                Some(false)
            );
            assert!(fpga_wrapper
                .read_control_value_bool("noThisControl")
                .is_none());
        }

        #[test]
        fn test_write_control_value_bool() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            assert!(!fpga_wrapper
                .read_control_value_bool("controlMod4Ch7")
                .unwrap());

            assert!(fpga_wrapper
                .write_control_value_bool("controlMod4Ch7", true)
                .is_some());
            assert!(fpga_wrapper
                .read_control_value_bool("controlMod4Ch7")
                .unwrap());

            assert!(fpga_wrapper
                .write_control_value_bool("controlMod4Ch7", false)
                .is_some());
            assert!(!fpga_wrapper
                .read_control_value_bool("controlMod4Ch7")
                .unwrap());

            assert!(fpga_wrapper
                .write_control_value_bool("noThisControl", false)
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

            assert!(fpga_wrapper.is_daq_fifo_full().unwrap());
        }

        #[test]
        fn test_read_power_and_digital_input() {
            let fpga_wrapper = create_fpga_wrapper_and_open(90);

            capture_data_in_fifo_daq(&fpga_wrapper);

            assert!(fpga_wrapper.read_power_and_digital_input().is_some());
        }
    }
}
