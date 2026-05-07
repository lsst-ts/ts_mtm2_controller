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

use log::{error, info};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

#[cfg(feature = "fpga")]
use nifpga_dll::Session;

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
    // Session to communicate with the FPGA
    #[cfg(feature = "fpga")]
    _session: Option<Session>,
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
        Self {
            registers: Self::create_register_map(filepath),
            #[cfg(feature = "fpga")]
            _session: None,
        }
    }

    /// Create a register map from the FPGA header file.
    ///
    /// # Arguments
    /// * `filepath` - The path to the FPGA header file.
    ///
    /// # Returns
    /// A register map from the FPGA header file.
    ///
    /// # Panics
    /// Panics if any of the registers in the FPGA header file is not found.
    fn create_register_map(filepath: &Path) -> HashMap<String, u32> {
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
        ];

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
                            self.format_serial_config(
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
    pub fn format_serial_config(
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
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::daq::config_data_acquisition::ConfigDataAcquisition;

    fn create_fpga_wrapper() -> FpgaWrapper {
        let config = ConfigDataAcquisition::new();
        FpgaWrapper::new(config.path_header.as_path())
    }

    #[test]
    fn test_new() {
        let fpga_wrapper = create_fpga_wrapper();

        assert_eq!(fpga_wrapper.registers.len(), 10);
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
        let message = fpga_wrapper.format_serial_config(20, 0, 0, 3, 0);
        assert_eq!(
            message,
            "Serial config - Baud rate: 921.6 kbps, Parity: None, Flow control: None, Data bits: 8, Stop bits: 1."
        );
    }

    #[cfg(feature = "fpga")]
    mod fpga_hardware {
        use super::*;

        fn create_fpga_wrapper_and_open() -> FpgaWrapper {
            let mut fpga_wrapper = create_fpga_wrapper();

            let config = ConfigDataAcquisition::new();
            fpga_wrapper.open(&config.path_bitfile, &config.fpga_resource);

            fpga_wrapper
        }

        #[test]
        fn test_open() {
            let fpga_wrapper = create_fpga_wrapper_and_open();

            assert!(fpga_wrapper._session.is_some());
        }

        #[test]
        fn test_log_serial_config() {
            let fpga_wrapper = create_fpga_wrapper_and_open();

            assert_eq!(fpga_wrapper.log_serial_config(), Some([0, 20, 0, 0, 3, 0]));
        }

        #[test]
        fn test_read_control_value() {
            let fpga_wrapper = create_fpga_wrapper_and_open();

            assert_eq!(
                fpga_wrapper.read_control_value("controlMod4Ch7"),
                Some(false)
            );
            assert!(fpga_wrapper.read_control_value("noThisControl").is_none());
        }

        #[test]
        fn test_write_control_value() {
            let fpga_wrapper = create_fpga_wrapper_and_open();

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
    }
}
