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

use crc::{Crc, CRC_16_MODBUS};

use crate::constants::{
    BROADCAST_ADDRESS, CODE_FORCE_REQUEST, CODE_ILC_MODE, CODE_MONITOR_SENSOR,
    CODE_READ_CALIBRATION_DATA, CODE_REPORT_SERVER_ID, CODE_REPORT_SERVER_STATUS, CODE_RESET,
    CODE_SCAN_RATE, CODE_SET_OFFSET_AND_SENSITIVITY, CODE_STEP_MOTOR_BROADCAST, NUM_ACTUATOR,
    NUM_ILC_TEMPERATURE_MONITOR_SENSOR,
};
use crate::daq::calibration_data::CalibrationData;
use crate::enums::InnerLoopControlMode;
use crate::utility::get_f32_values_from_u8_array;

pub struct InnerLoopController {
    // Cyclic redundancy check (CRC) calculator for the inner-loop controller
    // (ILC) communication.
    _crc: Crc<u16>,
    // Frames to get the force and status from ILC for all actuators.
    _frames_get_force_and_status: [[u8; 4]; NUM_ACTUATOR],
    // Frames to get the temperature monitor sensor data from ILC.
    _frames_temperature: [[u8; 4]; 4],
    // Frames to get the displacement monitor sensor data from ILC.
    _frame_displacement: [u8; 4],
    // Frames to get the inclinometer monitor sensor data from ILC.
    _frame_inclinometer: [u8; 4],
    // Communication counter 0-15 with the ILC.
    pub communication_counter: u8,
}

impl Default for InnerLoopController {
    fn default() -> Self {
        Self::new()
    }
}

impl InnerLoopController {
    /// Create a new inner-loop controller (ILC) by following the document:
    /// 1. LSST-ILC Firmware: MODBUS Protocol Interface Control Document for M2
    ///    Support System
    /// 2. LTS-346, ILC Communications Protocol For M2 Support System
    ///
    /// # Returns
    /// A new ILC instance.
    pub fn new() -> Self {
        // Use the const here because the lookup table for the CRC algorithm
        // is precomputed and stored in the constant.
        const CRC_ALGORITHM: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);

        let mut frames_get_force_and_status = [[0; 4]; NUM_ACTUATOR];
        for (index, frame) in frames_get_force_and_status.iter_mut().enumerate() {
            *frame = Self::create_frame_get_force_and_status(&CRC_ALGORITHM, index as u8);
        }

        let mut frames_temperature = [[0; 4]; NUM_ILC_TEMPERATURE_MONITOR_SENSOR];
        for (index, frame) in frames_temperature.iter_mut().enumerate() {
            *frame = Self::create_frame_monitor(&CRC_ALGORITHM, (index + NUM_ACTUATOR) as u8);
        }

        let frame_displacement = Self::create_frame_monitor(
            &CRC_ALGORITHM,
            (NUM_ACTUATOR + NUM_ILC_TEMPERATURE_MONITOR_SENSOR) as u8,
        );
        let frame_inclinometer = Self::create_frame_monitor(
            &CRC_ALGORITHM,
            (NUM_ACTUATOR + NUM_ILC_TEMPERATURE_MONITOR_SENSOR + 1) as u8,
        );

        Self {
            _crc: CRC_ALGORITHM,

            _frames_get_force_and_status: frames_get_force_and_status,
            _frames_temperature: frames_temperature,
            _frame_displacement: frame_displacement,
            _frame_inclinometer: frame_inclinometer,

            communication_counter: 0x0F,
        }
    }

    /// Create a frame to get the force and status from the inner-loop
    /// controller (ILC) of actuators.
    ///
    /// # Arguments
    /// * `crc` - The CRC calculator to compute the checksum.
    /// * `address` - The 0-based address of the ILC to get the force and
    ///   status.
    ///
    /// # Returns
    /// A byte array representing the frame to get the force and status.
    fn create_frame_get_force_and_status(crc: &Crc<u16>, address: u8) -> [u8; 4] {
        let mut frame = [0; 4];
        frame[0] = Self::to_one_based_address(address);
        frame[1] = CODE_FORCE_REQUEST;
        Self::calculate_crc_and_update_frame(crc, &mut frame);

        frame
    }

    /// Convert a 0-based address to a 1-based address for the inner-loop
    /// controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The 0-based address to be converted.
    ///
    /// # Returns
    /// The 1-based address corresponding to the given 0-based address.
    fn to_one_based_address(address: u8) -> u8 {
        address + 1
    }

    /// Calculate the cyclic redundancy check (CRC) checksum for the given
    /// frame and update it.
    ///
    /// # Arguments
    /// * `crc` - The CRC calculator to compute the checksum.
    /// * `frame` - The frame to be updated with the CRC checksum (final two
    ///   bytes).
    ///
    /// # Panics
    /// Panics if the frame length is less than 3 bytes, as at least 1 byte of
    /// data and 2 bytes for CRC are required.
    pub fn calculate_crc_and_update_frame(crc: &Crc<u16>, frame: &mut [u8]) {
        let length = frame.len();
        if length < 3 {
            panic!("Modbus frame must have at least 3 bytes to accommodate data and CRC.");
        }

        let checksum = crc.checksum(&frame[..length - 2]);

        // Convert to Little-Endian (Low byte first)
        let crc_bytes = checksum.to_le_bytes();

        frame[length - 2] = crc_bytes[0];
        frame[length - 1] = crc_bytes[1];
    }

    /// Create a frame to get the monitor sensor data from the inner-loop
    /// controller (ILC) for a given address.
    ///
    /// # Arguments
    /// * `crc` - The CRC calculator to compute the checksum.
    /// * `address` - The 0-based address of the ILC to get the monitor sensor
    ///   data from.
    ///
    /// # Returns
    /// A byte array representing the frame to get the monitor sensor data.
    fn create_frame_monitor(crc: &Crc<u16>, address: u8) -> [u8; 4] {
        let mut frame = [0; 4];
        frame[0] = Self::to_one_based_address(address);
        frame[1] = CODE_MONITOR_SENSOR;
        Self::calculate_crc_and_update_frame(crc, &mut frame);

        frame
    }

    /// Get the precomputed frame to get the force and status from the
    /// inner-loop controller (ILC) for a given address.
    ///
    /// # Arguments
    /// * `address` - The 0-based address of the ILC to get the frame for.
    ///
    /// # Returns
    /// An optional reference to the byte array representing the frame to get
    /// the force and status of the ILC. Returns `None` if the address is out
    /// of bounds.
    pub fn get_frame_get_force_and_status(&self, address: usize) -> Option<&[u8; 4]> {
        self._frames_get_force_and_status.get(address)
    }

    /// Get the precomputed frame to get the temperature monitor sensor data from
    /// the inner-loop controller (ILC) for a given address.
    ///
    /// # Arguments
    /// * `address` - The 0-based address defined in the followings:
    ///   0 - Mirror (LG2)
    ///   1 - Cell (intake & exhaust)
    ///   2 - Mirror (LG4)
    ///   3 - Mirror (LG3)
    ///
    /// # Returns
    /// An optional reference to the byte array representing the frame to get
    /// the temperature monitor sensor data of the ILC. Returns `None` if the address is out
    /// of bounds.
    pub fn get_frame_temperature(&self, address: usize) -> Option<&[u8; 4]> {
        self._frames_temperature.get(address)
    }

    /// Get the precomputed frame to get the displacement data from the
    /// inner-loop controller (ILC).
    ///
    /// # Returns
    /// A reference to the byte array representing the frame to get the
    /// displacement data of the ILC.
    pub fn get_frame_displacement(&self) -> &[u8; 4] {
        &self._frame_displacement
    }

    /// Get the precomputed frame to get the inclinometer data from the
    /// inner-loop controller (ILC).
    ///
    /// # Returns
    /// A reference to the byte array representing the frame to get the
    /// inclinometer data of the ILC.
    pub fn get_frame_inclinometer(&self) -> &[u8; 4] {
        &self._frame_inclinometer
    }

    /// Create a frame to get the mode of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The 0-based address of the ILC to get the mode from.
    ///
    /// # Returns
    /// A byte array representing the frame to get the mode of the ILC.
    pub fn create_frame_get_mode(&self, address: u8) -> [u8; 6] {
        self.create_frame_set_mode(address, InnerLoopControlMode::NoChange)
    }

    /// Create a frame to set the mode of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The 0-based address of the ILC to set the mode.
    /// * `mode` - The mode to be set for the ILC.
    ///
    /// # Returns
    /// A byte array representing the frame to set the mode of the ILC.
    pub fn create_frame_set_mode(&self, address: u8, mode: InnerLoopControlMode) -> [u8; 6] {
        let mut frame = [0; 6];
        frame[0] = Self::to_one_based_address(address);
        frame[1] = CODE_ILC_MODE;
        frame[2..4].copy_from_slice(&Self::get_mode_value(mode).to_be_bytes());

        Self::calculate_crc_and_update_frame(&self._crc, &mut frame);

        frame
    }

    /// Get the inner-loop controller (ILC) mode value.
    ///
    /// # Notes
    /// This function should be consistent with
    /// `Self::get_mode_from_mode_value()`.
    ///
    /// # Arguments
    /// * `mode` - The mode to be set for the ILC.
    ///
    /// # Returns
    /// The mode value corresponding to the given ILC mode.
    pub fn get_mode_value(mode: InnerLoopControlMode) -> u16 {
        match mode {
            InnerLoopControlMode::Standby => 0x0000,
            InnerLoopControlMode::Disabled => 0x0001,
            InnerLoopControlMode::Enabled => 0x0002,
            InnerLoopControlMode::FirmwareUpdate => 0x0003,
            InnerLoopControlMode::Fault => 0x0004,
            InnerLoopControlMode::ClearFaults => 0x0005,
            _ => 0xFFFF, // No change - respond with the current mode
        }
    }

    /// Get the inner-loop controller (ILC) mode from a mode value.
    ///
    /// # Notes
    /// This function should be consistent with `Self::get_mode_value()`.
    ///
    /// # Arguments
    /// * `value` - The mode value to be translated to ILC mode.
    ///
    /// # Returns
    /// The ILC mode corresponding to the given mode value. If the mode value
    /// is not recognized, it returns `InnerLoopControlMode::Unknown`.
    pub fn get_mode_from_value(value: u16) -> InnerLoopControlMode {
        match value {
            0x0000 => InnerLoopControlMode::Standby,
            0x0001 => InnerLoopControlMode::Disabled,
            0x0002 => InnerLoopControlMode::Enabled,
            0x0003 => InnerLoopControlMode::FirmwareUpdate,
            0x0004 => InnerLoopControlMode::Fault,
            0x0005 => InnerLoopControlMode::ClearFaults,
            0xFFFF => InnerLoopControlMode::NoChange,
            _ => InnerLoopControlMode::Unknown,
        }
    }

    /// Create a frame to broadcast step motor command to all stepper
    /// controlled actuator ILCs (tangent & axial only).
    ///
    /// # Arguments
    /// * `steps` - A slice of 8-bit signed integers representing the step
    ///   motor commands for each actuator. The length of the slice should
    ///   match the number of stepper controlled actuators.
    ///
    /// # Returns
    /// A byte array representing the frame to broadcast the step motor
    /// commands to all actuator ILCs.
    pub fn create_frame_move_steps(&mut self, steps: &[i8]) -> [u8; 83] {
        let mut frame = [0; 83];
        frame[0] = BROADCAST_ADDRESS;
        frame[1] = CODE_STEP_MOTOR_BROADCAST;
        frame[2] = self.get_next_communication_counter();

        steps
            .iter()
            .enumerate()
            .for_each(|(index, step)| frame[index + 3] = *step as u8);

        Self::calculate_crc_and_update_frame(&self._crc, &mut frame);

        frame
    }

    /// Get the next communication counter.
    ///
    /// # Returns
    /// Communication counter.
    fn get_next_communication_counter(&mut self) -> u8 {
        self.communication_counter = (self.communication_counter + 1) & 0x0F;

        self.communication_counter
    }

    /// Create a frame to report the server ID that contains the server
    /// identifier information.
    ///
    /// # Arguments
    /// * `address` - The 0-based address of the inner-loop controller (ILC) to
    ///   report the server ID.
    ///
    /// # Returns
    /// A byte array representing the frame to report the server ID.
    pub fn create_frame_report_server_id(&self, address: u8) -> [u8; 4] {
        self.create_frame_code_only(address, CODE_REPORT_SERVER_ID)
    }

    /// Create a frame with only the address and code for the inner-loop
    /// controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The 0-based address of ILC.
    /// * `code` - The code representing the specific command or request.
    ///
    /// # Returns
    /// A byte array representing the frame with only the 1-based address,
    /// code, and cyclic redundancy check (CRC).
    fn create_frame_code_only(&self, address: u8, code: u8) -> [u8; 4] {
        let mut frame = [0; 4];
        frame[0] = Self::to_one_based_address(address);
        frame[1] = code;

        Self::calculate_crc_and_update_frame(&self._crc, &mut frame);

        frame
    }

    /// Create a frame to report the server status that reports the mode,
    /// status, and faults.
    ///
    /// # Arguments
    /// * `address` - The 0-based address of the inner-loop controller (ILC) to
    ///   report the server status.
    ///
    /// # Returns
    /// A byte array representing the frame to report the server status.
    pub fn create_frame_report_server_status(&self, address: u8) -> [u8; 4] {
        self.create_frame_code_only(address, CODE_REPORT_SERVER_STATUS)
    }

    /// Create a frame to get the scan rate of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The 0-based address of ILC.
    ///
    /// # Returns
    /// A byte array representing the frame to get the scan rate.
    pub fn create_frame_get_scan_rate(&self, address: u8) -> [u8; 5] {
        // 0xFF = No Change - Respond with the current scan rate
        self.create_frame_set_scan_rate(address, 0xFF)
    }

    /// Create a frame to set the scan rate of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The 0-based address of ILC.
    /// * `scan_rate` - The scan rate value.
    ///   0xFF = No Change - Respond with the current scan rate
    ///   0 = 50 (scan rate in samples per second)
    ///   1 = 60
    ///   2 = 100
    ///   3 = 120
    ///   4 = 200
    ///   5 = 240
    ///   6 = 300
    ///   7 = 400
    ///   8 = 480
    ///   9 = 600
    ///   10 = 1200
    ///   11 = 2400
    ///   12 = 4800
    ///
    /// # Returns
    /// A byte array representing the frame to set the scan rate.
    pub fn create_frame_set_scan_rate(&self, address: u8, scan_rate: u8) -> [u8; 5] {
        let mut frame = [0; 5];
        frame[0] = Self::to_one_based_address(address);
        frame[1] = CODE_SCAN_RATE;
        frame[2] = scan_rate;

        Self::calculate_crc_and_update_frame(&self._crc, &mut frame);

        frame
    }

    /// Create a frame to set the offset and sensitivity for a specific channel.
    ///
    /// # Arguments
    /// * `address` - The 0-based address of the inner-loop controller (ILC).
    /// * `channel` - The 0-based channel number (0-3) to set the offset and
    ///   sensitivity.
    /// * `offset` - The offset value.
    /// * `sensitivity` - The sensitivity value.
    ///
    /// # Returns
    /// A byte array representing the frame to set the offset and sensitivity.
    pub fn create_frame_set_offset_and_sensitivity(
        &self,
        address: u8,
        channel: u8,
        offset: f32,
        sensitivity: f32,
    ) -> [u8; 13] {
        let mut frame = [0; 13];
        frame[0] = Self::to_one_based_address(address);
        frame[1] = CODE_SET_OFFSET_AND_SENSITIVITY;
        frame[2] = Self::to_one_based_address(channel);
        frame[3..7].copy_from_slice(&offset.to_be_bytes());
        frame[7..11].copy_from_slice(&sensitivity.to_be_bytes());

        Self::calculate_crc_and_update_frame(&self._crc, &mut frame);

        frame
    }

    /// Create a frame to reset the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `address` - The 0-based address of ILC to reset.
    ///
    /// # Returns
    /// A byte array representing the frame to reset the ILC.
    pub fn create_frame_reset(&self, address: u8) -> [u8; 4] {
        self.create_frame_code_only(address, CODE_RESET)
    }

    /// Create a frame to read calibration data from the inner-loop controller
    /// (ILC) EEPROM.
    ///
    /// # Arguments
    /// * `address` - The 0-based address of ILC to read the calibration data.
    ///
    /// # Returns
    /// A byte array representing the frame to read calibration data from the
    /// ILC EEPROM.
    pub fn create_frame_read_calibration_data(&self, address: u8) -> [u8; 4] {
        self.create_frame_code_only(address, CODE_READ_CALIBRATION_DATA)
    }

    /// Get the inner-loop control mode from a received frame.
    ///
    /// # Arguments
    /// * `frame` - The received frame.
    ///
    /// # Returns
    /// The inner-loop control mode translated from the frame. If the value
    /// is not recognized, it returns `InnerLoopControlMode::Unknown`. If the
    /// frame is too short to contain the mode value, it returns `None`.
    pub fn get_ilc_mode_from_frame(&self, frame: &[u8]) -> Option<InnerLoopControlMode> {
        if frame.len() != 2 {
            return None;
        }

        Some(Self::get_mode_from_value(u16::from_be_bytes([
            frame[0], frame[1],
        ])))
    }

    /// Get the force and status from a received frame.
    ///
    /// # Arguments
    /// * `frame` - The received frame containing the force and status data.
    ///
    /// # Returns
    /// A tuple containing the status, encoder count, and force (in Newtons)
    /// extracted from the frame. If the frame is too short to contain the
    /// required data, it returns `None`. For the details of status, see
    /// `Self::check_actuator_ilc_status()` and
    /// `Self::is_expected_communication_counter()`.
    pub fn get_force_and_status_from_frame(&self, frame: &[u8]) -> Option<(u8, i32, f32)> {
        if frame.len() != 9 {
            return None;
        }

        let status = frame[0];
        let encoder_count = i32::from_be_bytes([frame[1], frame[2], frame[3], frame[4]]);
        let force = f32::from_be_bytes([frame[5], frame[6], frame[7], frame[8]]);

        // Need to negate the force in order to keep in line with the
        // convention that tension is a positive force.
        Some((status, encoder_count, -force))
    }

    /// Check the actuator inner-loop control (ILC) status from a received
    /// status byte.
    ///
    /// # Arguments
    /// * `status` - The received status byte.
    ///
    /// # Returns
    /// A tuple containing the fault status, communication error status, and
    /// closed limit switch status (clockwise and counter-clockwise).
    pub fn check_actuator_ilc_status(status: u8) -> (bool, bool, bool, bool) {
        // Bit 0
        let is_fault = (status & 0x01) != 0;
        // Bit 1
        let is_communication_error = (status & 0x02) != 0;
        // Bit 2
        let is_closed_limit_switch_cw = (status & 0x04) != 0;
        // Bit 3
        let is_closed_limit_switch_ccw = (status & 0x08) != 0;

        (
            is_fault,
            is_communication_error,
            is_closed_limit_switch_cw,
            is_closed_limit_switch_ccw,
        )
    }

    /// Check if the broadcast communication counter from the actuator
    /// inner-loop control (ILC) status matches the current communication
    /// counter.
    ///
    /// # Arguments
    /// * `status` - The received status byte.
    ///
    /// # Returns
    /// `true` if the broadcast communication counter matches the current
    /// communication counter, `false` otherwise.
    pub fn is_expected_communication_counter(&self, status: u8) -> bool {
        // Bit 4-7 is the broadcast communication counter
        let broadcast_communication_counter = (status >> 4) & 0x0F;

        broadcast_communication_counter == self.communication_counter
    }

    /// Get the temperature values from a received frame.
    ///
    /// # Notes
    /// See get_frame_temperature() for the address details. The followings are
    /// the order of the sensors for each address:
    /// 0 - LG2-1, LG2-2, LG2-3, LG2-4
    /// 1 - Intake#1, Exhaust#1, Exhaust#2, Intake#2
    /// 2 - LG4-1, LG4-2, LG4-3, LG4-4
    /// 3 - LG3-1, LG3-2, LG3-3, LG3-4
    ///
    /// # Arguments
    /// * `frame` - The received frame containing the temperature data.
    ///
    /// # Returns
    /// An array of 4 floating-point numbers representing the temperature
    /// values (in degrees Celsius) extracted from the frame. If the frame
    /// length is not exactly 16 bytes, it returns `None`.
    pub fn get_temperature_from_frame(&self, frame: &[u8]) -> Option<[f32; 4]> {
        if frame.len() != 16 {
            return None;
        }

        get_f32_values_from_u8_array::<4>(frame)
    }

    /// Get the displacement values from a received frame.
    ///
    /// # Notes
    /// Order of the sensors:
    /// [A5TZ, A5DZ, A6TZ, A6DZ, A3TZ, A3DZ, A4TZ, A4DZ, A1TZ, A1DZ, A2TZ,
    /// A2DZ].
    ///
    /// # Arguments
    /// * `frame` - The received frame containing the displacement data.
    ///
    /// # Returns
    /// An array of 12 floating-point numbers representing the displacement
    /// values (in millimeters) extracted from the frame. If the frame length
    /// is not exactly 48 bytes, it returns `None`.
    pub fn get_displacement_from_frame(&self, frame: &[u8]) -> Option<[f32; 12]> {
        if frame.len() != 48 {
            return None;
        }

        get_f32_values_from_u8_array::<12>(frame)
    }

    /// Get the inclinometer value from a received frame.
    ///
    /// # Arguments
    /// * `frame` - The received frame containing the inclinometer data.
    ///
    /// # Returns
    /// A floating-point number representing the inclinometer value (in
    /// degrees) extracted from the frame. If the frame length is not exactly 4
    /// bytes, it returns `None`.
    pub fn get_inclinometer_from_frame(&self, frame: &[u8]) -> Option<f32> {
        if frame.len() != 4 {
            return None;
        }

        Some(f32::from_be_bytes([frame[0], frame[1], frame[2], frame[3]]))
    }

    /// Get the server status from a received frame.
    ///
    /// # Notes
    /// Status:
    /// bit 0: Major Fault - 0 = None, 1 = Major System Fault
    /// bit 1: Minor Fault - 0 = None, 1 = Minor System Fault
    /// bit 2: Reserved - 0
    /// bit 3: Fault Override - 1 = One or more faults are overridden
    /// bit 4: Cal Main - 1 = Main Calibration Error
    /// bit 5: Cal Back - 1 = Backup Calibration Error
    /// bit 6..7: Reserved - 0
    /// bit 8: Limit Switch 1 - 1 = Limit 1 activated
    /// bit 9: Limit Switch 2 - 1 = Limit 2 activated
    /// bit 10..11: Reserved - 0
    /// bit 12: Monitor Instrument - 1 = Communications timeout with instrument
    ///                              monitor
    /// bit 13..15: Reserved - 0
    ///
    /// Faults:
    /// bit 0: Unique ID - 0 = Unique ID verified, 1 = Unique ID CRC error
    /// bit 1: App Type - 0 = App Type and Network Node Type match,
    ///                   1 = App Type and Network Node Type do not match
    /// bit 2: No App - 0 = ILC App present, 1 = No ILC App programmed
    /// bit 3: App CRC - 0 = ILC-App verified, 1 = ILC-App CRC error
    /// bit 4: No 1-Wire - 0 = 1-Wire present, 1 = No 1-Wire found
    /// bit 5: 1-Wire 1 - 0 = 1-Wire copy 1 verified,
    ///                   1 = 1-Wire copy 1 error
    /// bit 6: 1-Wire 2 - 0 = 1-Wire copy 2 verified,
    ///                   1 = 1-Wire copy 2 error
    /// bit 7: Reserved - 0
    /// bit 8: Watchdog Reset - 0 = Watchdog timer OK,  
    ///                         1 = A reset occurred due to Watchdog timeout
    /// bit 9: Brown-Out - 0 = OK, 1 = Power brown-out occurred
    /// bit 10: Event Trap - 0 = OK,
    ///                      1 = A reset occurred due to controller event trap
    /// bit 11: Motor Driver - 0 = OK, 1 = Motor driver fail
    /// bit 12: SSR Power - 0 = OK, 1 = SSR power fail
    /// bit 13: Aux Power - 0 = OK, 1 = Aux power fail
    /// bit 14: SMC Power - 0 = OK, 1 = Motor controller power fail
    /// bit 15: Reserved - 0
    ///
    /// # Arguments
    /// * `frame` - The received frame containing the server status data.
    ///
    /// # Returns
    /// An optional tuple containing the inner-loop control mode, status, and
    /// faults extracted from the frame. Returns `None` if the frame length is
    /// not exactly 5 bytes.
    pub fn get_server_status_from_frame(
        &self,
        frame: &[u8],
    ) -> Option<(InnerLoopControlMode, u16, u16)> {
        if frame.len() != 5 {
            return None;
        }

        let mode_value = u16::from_be_bytes([0, frame[0]]);
        let mode = Self::get_mode_from_value(mode_value);

        let status = u16::from_be_bytes([frame[1], frame[2]]);
        let faults = u16::from_be_bytes([frame[3], frame[4]]);

        Some((mode, status, faults))
    }

    /// Get the calibration data from a received frame.
    ///
    /// # Arguments
    /// * `frame` - The received frame containing the calibration data.
    ///
    /// # Returns
    /// An optional tuple containing two `CalibrationData` structs, one for the
    /// main calibration data and one for the backup calibration data. Returns
    /// `None` if the frame length is not exactly 96 bytes.
    pub fn get_calibration_data_from_frame(
        &self,
        frame: &[u8],
    ) -> Option<(CalibrationData, CalibrationData)> {
        if frame.len() != 96 {
            return None;
        }

        Some((
            CalibrationData::from_frame(&frame[0..48])?,
            CalibrationData::from_frame(&frame[48..96])?,
        ))
    }

    /// Verify the cyclic redundancy check (CRC) of a received frame.
    ///
    /// # Arguments
    /// * `frame` - The received frame to be verified.
    ///
    /// # Returns
    /// `true` if the CRC is valid, `false` otherwise.
    pub fn verify_crc(&self, frame: &[u8]) -> bool {
        // Not enough data for CRC-16 (2 bytes) + at least 1 byte of data
        let frame_length = frame.len();
        if frame_length < 3 {
            return false;
        }

        let data = &frame[..frame_length - 2];
        let crc_received = u16::from_le_bytes([frame[frame_length - 2], frame[frame_length - 1]]);
        let crc_calculated = self._crc.checksum(data);

        crc_received == crc_calculated
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let ilc = InnerLoopController::new();

        assert_eq!(ilc._frames_get_force_and_status.len(), NUM_ACTUATOR);
        assert_eq!(
            ilc._frames_get_force_and_status[1],
            [0x02, CODE_FORCE_REQUEST, 0x41, 0x21]
        );

        assert_eq!(
            ilc._frames_temperature[0],
            [0x4F, CODE_MONITOR_SENSOR, 0x34, 0x7F]
        );
        assert_eq!(
            ilc._frames_temperature[3],
            [0x52, CODE_MONITOR_SENSOR, 0x3D, 0x2F]
        );
        assert_eq!(
            ilc._frame_displacement,
            [0x53, CODE_MONITOR_SENSOR, 0x3C, 0xBF]
        );
        assert_eq!(
            ilc._frame_inclinometer,
            [0x54, CODE_MONITOR_SENSOR, 0x3E, 0x8F]
        );
    }

    #[test]
    fn test_calculate_crc_and_update_frame() {
        let ilc = InnerLoopController::new();

        // Check the CRC calculation matches the
        // "MB FPGA Serial Modbus Data Unit to String.vi" in ts_mtm2_cell.
        let mut frame_1 = [1, 2, 3, 4, 0, 0];
        InnerLoopController::calculate_crc_and_update_frame(&ilc._crc, &mut frame_1);

        assert_eq!(frame_1, [1, 2, 3, 4, 0xA1, 0x2B]);

        let mut frame_2 = [1, 2, 3, 4, 0, 0, 0];
        InnerLoopController::calculate_crc_and_update_frame(&ilc._crc, &mut frame_2);

        assert_eq!(frame_2, [1, 2, 3, 4, 0, 0xEA, 0xB8]);
    }

    #[should_panic(
        expected = "Modbus frame must have at least 3 bytes to accommodate data and CRC."
    )]
    #[test]
    fn test_calculate_crc_and_update_frame_panic() {
        let ilc = InnerLoopController::new();

        InnerLoopController::calculate_crc_and_update_frame(&ilc._crc, &mut [0; 2]);
    }

    #[test]
    fn test_get_frame_get_force_and_status() {
        let ilc = InnerLoopController::new();

        // Valid address
        assert_eq!(
            ilc.get_frame_get_force_and_status(0x01),
            Some(&[0x02, CODE_FORCE_REQUEST, 0x41, 0x21])
        );

        // Invalid address
        assert!(ilc.get_frame_get_force_and_status(0xFF).is_none());
    }

    #[test]
    fn test_get_frame_temperature() {
        let ilc = InnerLoopController::new();

        // Valid address
        assert_eq!(
            ilc.get_frame_temperature(0),
            Some(&[0x4F, CODE_MONITOR_SENSOR, 0x34, 0x7F])
        );

        // Invalid address
        assert!(ilc.get_frame_temperature(5).is_none());
    }

    #[test]
    fn test_create_frame_get_mode() {
        let ilc = InnerLoopController::new();

        let address = 0x01;
        let frame = ilc.create_frame_get_mode(address);

        assert_eq!(frame, [address + 1, CODE_ILC_MODE, 0xFF, 0xFF, 0x50, 0x38]);
    }

    #[test]
    fn test_create_frame_set_mode() {
        let ilc = InnerLoopController::new();

        let address = 0x03;

        // Enabled state
        let frame_enabled = ilc.create_frame_set_mode(address, InnerLoopControlMode::Enabled);

        assert_eq!(
            frame_enabled,
            [address + 1, CODE_ILC_MODE, 0x0, 0x2, 0xD0, 0xC1]
        );

        // Unknown state
        let frame_unknown = ilc.create_frame_set_mode(address, InnerLoopControlMode::Unknown);
        assert_eq!(
            frame_unknown,
            [address + 1, CODE_ILC_MODE, 0xFF, 0xFF, 0x50, 0xB0]
        );
    }

    #[test]
    fn test_get_mode_value() {
        assert_eq!(
            InnerLoopController::get_mode_value(InnerLoopControlMode::Standby),
            0x0000
        );
        assert_eq!(
            InnerLoopController::get_mode_value(InnerLoopControlMode::Disabled),
            0x0001
        );
        assert_eq!(
            InnerLoopController::get_mode_value(InnerLoopControlMode::Enabled),
            0x0002
        );
        assert_eq!(
            InnerLoopController::get_mode_value(InnerLoopControlMode::FirmwareUpdate),
            0x0003
        );
        assert_eq!(
            InnerLoopController::get_mode_value(InnerLoopControlMode::Fault),
            0x0004
        );
        assert_eq!(
            InnerLoopController::get_mode_value(InnerLoopControlMode::ClearFaults),
            0x0005
        );
        assert_eq!(
            InnerLoopController::get_mode_value(InnerLoopControlMode::NoChange),
            0xFFFF
        );
        assert_eq!(
            InnerLoopController::get_mode_value(InnerLoopControlMode::Unknown),
            0xFFFF
        );
    }

    #[test]
    fn test_get_mode_from_value() {
        assert_eq!(
            InnerLoopController::get_mode_from_value(0x0000),
            InnerLoopControlMode::Standby
        );
        assert_eq!(
            InnerLoopController::get_mode_from_value(0x0001),
            InnerLoopControlMode::Disabled
        );
        assert_eq!(
            InnerLoopController::get_mode_from_value(0x0002),
            InnerLoopControlMode::Enabled
        );
        assert_eq!(
            InnerLoopController::get_mode_from_value(0x0003),
            InnerLoopControlMode::FirmwareUpdate
        );
        assert_eq!(
            InnerLoopController::get_mode_from_value(0x0004),
            InnerLoopControlMode::Fault
        );
        assert_eq!(
            InnerLoopController::get_mode_from_value(0x0005),
            InnerLoopControlMode::ClearFaults
        );
        assert_eq!(
            InnerLoopController::get_mode_from_value(0xFFFF),
            InnerLoopControlMode::NoChange
        );
        assert_eq!(
            InnerLoopController::get_mode_from_value(0x1234),
            InnerLoopControlMode::Unknown
        );
    }

    #[test]
    fn test_create_frame_move_steps() {
        let mut ilc = InnerLoopController::new();

        let mut steps = [0; NUM_ACTUATOR];
        steps[0] = 1;
        steps[1] = -1;
        steps[2] = 127;
        steps[3] = -128;
        steps[4] = -127;
        steps[NUM_ACTUATOR - 1] = 1;

        // Check the frame content for the first 5 actuators. The rest should
        // be 0. Be careful with the signed to unsigned conversion for negative
        // values.
        let frame_1 = ilc.create_frame_move_steps(&steps);
        assert_eq!(
            frame_1[0..9],
            [
                BROADCAST_ADDRESS,
                CODE_STEP_MOTOR_BROADCAST,
                0x00, // Communication counter starts at 0
                0x01, // Step for actuator 1
                0xFF, // Step for actuator 2 (-1 in two's complement)
                0x7F, // Step for actuator 3 (127)
                0x80, // Step for actuator 4 (-128 in two's complement)
                0x81, // Step for actuator 5 (-127 in two's complement)
                0x0,
            ]
        );

        let length = frame_1.len();
        assert_eq!(frame_1[length - 3], 0x01);

        // Check the CRC at the end of the frame.
        assert_eq!(frame_1[length - 2], 0x55);
        assert_eq!(frame_1[length - 1], 0x9B);

        // Check the communication counter is incremented by 1.
        let frame_2 = ilc.create_frame_move_steps(&steps);
        assert_eq!(frame_2[2], 0x01);
    }

    #[test]
    fn test_get_next_communication_counter() {
        let mut ilc = InnerLoopController::new();

        for counter in 0..20 {
            assert_eq!(ilc.get_next_communication_counter(), counter % 16);
        }
    }

    #[test]
    fn test_create_frame_report_server_id() {
        let ilc = InnerLoopController::new();

        let address = 0x02;
        let frame = ilc.create_frame_report_server_id(address);

        assert_eq!(frame, [address + 1, CODE_REPORT_SERVER_ID, 0xC1, 0x4C]);
    }

    #[test]
    fn test_create_frame_code_only() {
        let ilc = InnerLoopController::new();

        let address = 0x01;
        let code = 0x10;
        let frame = ilc.create_frame_code_only(address, code);

        assert_eq!(frame, [address + 1, code, 0x1, 0x1C]);
    }

    #[test]
    fn test_create_frame_report_server_status() {
        let ilc = InnerLoopController::new();

        let address = 0x03;
        let frame = ilc.create_frame_report_server_status(address);

        assert_eq!(frame, [address + 1, CODE_REPORT_SERVER_STATUS, 0x83, 0x7D]);
    }

    #[test]
    fn test_create_frame_get_scan_rate() {
        let ilc = InnerLoopController::new();

        let address = 0x01;
        let frame = ilc.create_frame_get_scan_rate(address);

        assert_eq!(frame, [address + 1, CODE_SCAN_RATE, 0xFF, 0xAC, 0x40]);
    }

    #[test]
    fn test_create_frame_set_scan_rate() {
        let ilc = InnerLoopController::new();

        let address = 0x02;
        let scan_rate = 0x05;
        let frame = ilc.create_frame_set_scan_rate(address, scan_rate);

        assert_eq!(frame, [address + 1, CODE_SCAN_RATE, scan_rate, 0x7D, 0xC3]);
    }

    #[test]
    fn test_create_frame_set_offset_and_sensitivity() {
        let ilc = InnerLoopController::new();

        let address = 0x01;
        let channel = 0x02;
        let offset = 1.23;
        let sensitivity = 4.56;
        let frame =
            ilc.create_frame_set_offset_and_sensitivity(address, channel, offset, sensitivity);

        assert_eq!(frame[0], address + 1);
        assert_eq!(frame[1], CODE_SET_OFFSET_AND_SENSITIVITY);
        assert_eq!(frame[2], channel + 1);
        assert_eq!(&frame[3..7], &offset.to_be_bytes());
        assert_eq!(&frame[7..11], &sensitivity.to_be_bytes());
    }

    #[test]
    fn test_create_frame_reset() {
        let ilc = InnerLoopController::new();

        let address = 0x02;
        let frame = ilc.create_frame_reset(address);

        assert_eq!(frame, [address + 1, CODE_RESET, 0x40, 0xAF]);
    }

    #[test]
    fn test_create_frame_read_calibration_data() {
        let ilc = InnerLoopController::new();

        let address = 0x03;
        let frame = ilc.create_frame_read_calibration_data(address);

        assert_eq!(frame, [address + 1, CODE_READ_CALIBRATION_DATA, 0x82, 0x9C]);
    }

    #[test]
    fn test_get_ilc_mode_from_frame() {
        let ilc = InnerLoopController::new();

        assert_eq!(
            ilc.get_ilc_mode_from_frame(&[0x00, 0x00]),
            Some(InnerLoopControlMode::Standby)
        );
        assert_eq!(
            ilc.get_ilc_mode_from_frame(&[0x00, 0x01]),
            Some(InnerLoopControlMode::Disabled)
        );
        assert_eq!(
            ilc.get_ilc_mode_from_frame(&[0x00, 0x02]),
            Some(InnerLoopControlMode::Enabled)
        );
        assert_eq!(
            ilc.get_ilc_mode_from_frame(&[0x00, 0x03]),
            Some(InnerLoopControlMode::FirmwareUpdate)
        );
        assert_eq!(
            ilc.get_ilc_mode_from_frame(&[0x00, 0x04]),
            Some(InnerLoopControlMode::Fault)
        );
        assert_eq!(
            ilc.get_ilc_mode_from_frame(&[0x00, 0x05]),
            Some(InnerLoopControlMode::ClearFaults)
        );
        assert_eq!(
            ilc.get_ilc_mode_from_frame(&[0xFF, 0xFF]),
            Some(InnerLoopControlMode::NoChange)
        );

        // Unrecognized value should return Unknown
        assert_eq!(
            ilc.get_ilc_mode_from_frame(&[0x12, 0x34]),
            Some(InnerLoopControlMode::Unknown)
        );

        assert!(ilc.get_ilc_mode_from_frame(&[0x00]).is_none());
    }

    #[test]
    fn test_get_force_and_status_from_frame() {
        let ilc = InnerLoopController::new();

        let (status, encoder_count, force) = ilc
            .get_force_and_status_from_frame(&[
                0x01, 0x00, 0x00, 0x00, 0x10, 0x41, 0x20, 0x00, 0x00,
            ])
            .unwrap();

        assert_eq!(status, 0x01);
        assert_eq!(encoder_count, 16);
        assert_eq!(force, -10.0);

        assert!(ilc.get_force_and_status_from_frame(&[0x01, 0x00]).is_none());
    }

    #[test]
    fn test_check_actuator_ilc_status() {
        let (
            is_fault,
            is_communication_error,
            is_closed_limit_switch_cw,
            is_closed_limit_switch_ccw,
        ) = InnerLoopController::check_actuator_ilc_status(0b11111101);

        assert!(is_fault);
        assert!(!is_communication_error);
        assert!(is_closed_limit_switch_cw);
        assert!(is_closed_limit_switch_ccw);
    }

    #[test]
    fn test_is_expected_communication_counter() {
        let ilc = InnerLoopController::new();

        assert!(ilc.is_expected_communication_counter(0xF0));
        assert!(!ilc.is_expected_communication_counter(0x0));
    }

    #[test]
    fn test_get_temperature_from_frame() {
        let ilc = InnerLoopController::new();

        let temperatures = ilc
            .get_temperature_from_frame(&[
                0x41, 0x20, 0x00, 0x00, 0xC1, 0x20, 0x00, 0x00, 0x42, 0x48, 0x00, 0x00, 0xC2, 0x48,
                0x00, 0x00,
            ])
            .unwrap();
        assert_eq!(temperatures, [10.0, -10.0, 50.0, -50.0]);

        assert!(ilc.get_temperature_from_frame(&[0x41, 0x20]).is_none());
    }

    #[test]
    fn test_get_displacement_from_frame() {
        let ilc = InnerLoopController::new();

        let mut frame = [0; 48];
        for idx in 0..12 {
            let value = (idx as f32) * 10.0;
            frame[idx * 4..(idx + 1) * 4].copy_from_slice(&value.to_be_bytes());
        }

        let displacements = ilc.get_displacement_from_frame(&frame).unwrap();
        assert_eq!(
            displacements,
            [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0]
        );

        assert!(ilc.get_displacement_from_frame(&[0x41, 0x20]).is_none());
    }

    #[test]
    fn test_get_inclinometer_from_frame() {
        let ilc = InnerLoopController::new();

        let inclinometer = ilc
            .get_inclinometer_from_frame(&[0x41, 0x20, 0x00, 0x00])
            .unwrap();

        assert_eq!(inclinometer, 10.0);

        assert!(ilc.get_inclinometer_from_frame(&[0x41, 0x20]).is_none());
    }

    #[test]
    fn test_get_server_status_from_frame() {
        let ilc = InnerLoopController::new();

        // Valid frame with server status data
        let frame = [0x01, 0x01, 0x02, 0x03, 0x04];

        let (mode, status, faults) = ilc.get_server_status_from_frame(&frame).unwrap();

        assert_eq!(mode, InnerLoopControlMode::Disabled);
        assert_eq!(status, 0x0102);
        assert_eq!(faults, 0x0304);

        // Invalid frame length
        assert!(ilc.get_server_status_from_frame(&[0x00]).is_none());
    }

    #[test]
    fn test_get_calibration_data_from_frame() {
        let ilc = InnerLoopController::new();

        // Create a valid frame with calibration data
        let mut frame = [0; 96];
        for idx in 0..24 {
            let value = idx as f32;
            frame[idx * 4..(idx + 1) * 4].copy_from_slice(&value.to_be_bytes());
        }

        let (main_calibration, backup_calibration) =
            ilc.get_calibration_data_from_frame(&frame).unwrap();

        assert_eq!(main_calibration.gains, [0.0, 1.0, 2.0, 3.0]);
        assert_eq!(main_calibration.offsets, [4.0, 5.0, 6.0, 7.0]);
        assert_eq!(main_calibration.sensitivities, [8.0, 9.0, 10.0, 11.0]);

        assert_eq!(backup_calibration.gains, [12.0, 13.0, 14.0, 15.0]);
        assert_eq!(backup_calibration.offsets, [16.0, 17.0, 18.0, 19.0]);
        assert_eq!(backup_calibration.sensitivities, [20.0, 21.0, 22.0, 23.0]);

        // Invalid frame length
        assert!(ilc.get_calibration_data_from_frame(&[0x00]).is_none());
    }

    #[test]
    fn test_verify_crc() {
        // Success
        let ilc = InnerLoopController::new();
        let frame = ilc.create_frame_get_mode(0x01);

        assert!(ilc.verify_crc(&frame));

        // Fail, corrupt the address
        let mut invalid_frame = frame.clone();
        invalid_frame[0] = 0x03;
        assert!(!ilc.verify_crc(&invalid_frame));

        // Fail, no enough data
        assert!(!ilc.verify_crc(&[0x01, 0x01]));
    }
}
