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
    CODE_STEP_MOTOR_BROADCAST, NUM_ACTUATOR,
};
use crate::enums::InnerLoopControlMode;

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
    /// LSST-ILC Firmware: MODBUS Protocol Interface Control Document for M2
    /// Support System
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

        const NUM_ILC_TEMPERATURE_MONITOR_SENSOR: usize = 4;
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
    /// required data, it returns `None`.
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

        self.get_f32_values_from_u8_array::<4>(frame)
    }

    /// Get the floating-point values from a byte array. The byte array is
    /// expected to contain `N` consecutive 4-byte big-endian floating-point
    /// values.
    ///
    /// # Arguments
    /// * `frame` - The byte array containing the floating-point values.
    ///
    /// # Returns
    /// An array of `N` floating-point numbers extracted from the byte array.
    /// Returns `None` if the length of the byte array does not match the
    /// expected length of `N * 4` bytes.
    fn get_f32_values_from_u8_array<const N: usize>(&self, frame: &[u8]) -> Option<[f32; N]> {
        if frame.len() != N * 4 {
            return None;
        }

        let mut values = [0.0; N];
        for index in 0..N {
            values[index] = f32::from_be_bytes([
                frame[index * 4],
                frame[index * 4 + 1],
                frame[index * 4 + 2],
                frame[index * 4 + 3],
            ]);
        }

        Some(values)
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

        self.get_f32_values_from_u8_array::<12>(frame)
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
