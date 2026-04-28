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
    CODE_STEP_MOTOR_BROADCAST,
};
use crate::daq::inner_loop_controller::InnerLoopController;
use crate::enums::InnerLoopControlMode;
use crate::mock::mock_constants::{MOCK_CODE_ILC_ERROR, MOCK_CODE_ILC_EXCEPTION};

// Mock data structure to hold the actuator status.
#[derive(Clone)]
pub struct MockData {
    // Current status.
    pub status: u8,
    // Encoder count.
    pub encoder_count: i32,
    // Measured force in newtons.
    pub force: f32,
}

/// Mock Inner-Loop Controller (ILC) to simulate the behavior of hardware.
#[derive(Clone)]
pub struct MockInnerLoopController {
    // Cyclic redundancy check (CRC) calculator for the inner-loop controller
    // (ILC) communication.
    _crc: Crc<u16>,
    // Controller's mode.
    pub mode: InnerLoopControlMode,
    // Actuator ILC data.
    pub data: MockData,
    // Monitor values (temperature, displacement, and inclinometer).
    pub monitor_values: Vec<f32>,
}

impl Default for MockInnerLoopController {
    fn default() -> Self {
        Self::new()
    }
}

impl MockInnerLoopController {
    /// Create a new inner-loop controller (ILC).
    ///
    /// # Returns
    /// A new model with a random number.
    pub fn new() -> Self {
        Self {
            _crc: Crc::<u16>::new(&CRC_16_MODBUS),

            mode: InnerLoopControlMode::default(),
            data: MockData {
                status: 0,
                encoder_count: 0,
                force: 0.0,
            },
            monitor_values: Vec::new(),
        }
    }

    /// Request the inner-loop controller (ILC) to process a request frame and
    /// return a response frame.
    ///
    /// # Arguments
    /// * `frame_request` - Request frame containing the command and
    ///   parameters.
    ///
    /// # Returns
    /// A response frame containing the result of the command and CRC checksum.
    /// This might be an error frame if the request is invalid.
    pub fn request(&mut self, frame_request: &[u8]) -> Vec<u8> {
        // Prepare the error response frame in case of invalid address or
        // function code.
        let address = frame_request[0];
        let mut frame_error = vec![address, MOCK_CODE_ILC_ERROR, MOCK_CODE_ILC_EXCEPTION, 0, 0];
        InnerLoopController::calculate_crc_and_update_frame(&self._crc, &mut frame_error);

        // Check the address. Valid individual address is 1-247. The 248 is
        // reserved for the broadcast.
        if (address == 0) || (address > BROADCAST_ADDRESS) {
            return frame_error;
        }

        match frame_request[1] {
            CODE_ILC_MODE => {
                let mode_command = InnerLoopController::get_mode_from_value(u16::from_be_bytes([
                    frame_request[2],
                    frame_request[3],
                ]));
                match mode_command {
                    InnerLoopControlMode::Unknown => frame_error,
                    InnerLoopControlMode::NoChange => self.get_mode(frame_request),
                    _ => self.set_mode(frame_request),
                }
            }
            CODE_FORCE_REQUEST => self.get_ilc_data(frame_request),
            CODE_MONITOR_SENSOR => self.get_monitor_values(frame_request),
            CODE_STEP_MOTOR_BROADCAST => {
                self.update_communication_counter(frame_request[2]);
                Vec::new()
            }
            _ => frame_error,
        }
    }

    /// Get the mode of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `frame_request` - Request frame.
    ///
    /// # Returns
    /// A response frame containing the current mode and CRC checksum.
    fn get_mode(&self, frame_request: &[u8]) -> Vec<u8> {
        self.create_frame_mode(frame_request[0], frame_request[1])
    }

    /// Create a response frame of the inner-loop controller (ILC) mode.
    ///
    /// # Arguments
    /// * `address` - The 1-based address of the ILC.
    /// * `function_code` - The function code of the command.
    ///
    /// # Returns
    /// A response frame containing the ILC mode and CRC checksum.
    fn create_frame_mode(&self, address: u8, function_code: u8) -> Vec<u8> {
        let mut frame = vec![0; 6];

        frame[0] = address;
        frame[1] = function_code;
        frame[2..4].copy_from_slice(&InnerLoopController::get_mode_value(self.mode).to_be_bytes());

        InnerLoopController::calculate_crc_and_update_frame(&self._crc, &mut frame);

        frame
    }

    /// Set the mode of the controller.
    ///
    /// # Arguments
    /// * `frame_request` - Request frame containing the mode to be set.
    ///
    /// # Returns
    /// A response frame containing the current mode and CRC checksum.
    fn set_mode(&mut self, frame_request: &[u8]) -> Vec<u8> {
        let mode_command = InnerLoopController::get_mode_from_value(u16::from_be_bytes([
            frame_request[2],
            frame_request[3],
        ]));

        let mode_new;
        match mode_command {
            InnerLoopControlMode::ClearFaults => {
                if self.mode == InnerLoopControlMode::Fault {
                    mode_new = InnerLoopControlMode::Standby;
                } else {
                    mode_new = self.mode;
                }
            }
            InnerLoopControlMode::NoChange | InnerLoopControlMode::Unknown => {
                mode_new = self.mode;
            }
            _ => {
                mode_new = mode_command;
            }
        }

        self.mode = mode_new;
        self.create_frame_mode(frame_request[0], frame_request[1])
    }

    /// Get the data of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `frame_request` - Request frame.
    ///
    /// # Returns
    /// A response frame containing the ILC data and CRC checksum.
    fn get_ilc_data(&self, frame_request: &[u8]) -> Vec<u8> {
        let mut frame_response = vec![0; 13];
        frame_response[0] = frame_request[0];
        frame_response[1] = frame_request[1];

        frame_response[2] = self.data.status;
        frame_response[3..7].copy_from_slice(&self.data.encoder_count.to_be_bytes());
        frame_response[7..11].copy_from_slice(&self.data.force.to_be_bytes());

        InnerLoopController::calculate_crc_and_update_frame(&self._crc, &mut frame_response);

        frame_response
    }

    /// Get the monitor values of the inner-loop controller (ILC).
    ///
    /// # Arguments
    /// * `frame_request` - Request frame.
    ///
    /// # Returns
    /// A response frame containing the monitor values and CRC checksum.
    fn get_monitor_values(&self, frame_request: &[u8]) -> Vec<u8> {
        // 1 f32 value = 4 u8 size
        let mut frame_response = vec![0; 4 + self.monitor_values.len() * 4];
        frame_response[0] = frame_request[0];
        frame_response[1] = frame_request[1];

        for (idx, value) in self.monitor_values.iter().enumerate() {
            frame_response[(2 + idx * 4)..(2 + (idx + 1) * 4)]
                .copy_from_slice(&value.to_be_bytes());
        }

        InnerLoopController::calculate_crc_and_update_frame(&self._crc, &mut frame_response);

        frame_response
    }

    /// Update the communication counter in the inner-loop controller (ILC)
    /// status.
    ///
    /// # Arguments
    /// * `counter` - The communication counter value (0-15) from the client.
    pub fn update_communication_counter(&mut self, counter: u8) {
        // Bit 4-7 of the status byte is used as the communication counter.
        self.data.status = counter << 4;
    }

    /// Update the encoder count and force values in the inner-loop controller
    /// (ILC) data.
    ///
    /// # Arguments
    /// * `encoder_count` - The new encoder count.
    /// * `force` - The new force value in Newtons.
    pub fn update_encoder_and_force(&mut self, encoder_count: i32, force: f64) {
        self.data.encoder_count = encoder_count;
        // Negate the force here.
        // See the InnerLoopController.get_force_and_status_from_frame() for
        // more details.
        self.data.force = -force as f32;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::constants::{CODE_FORCE_REQUEST, CODE_ILC_MODE, CODE_MONITOR_SENSOR};

    #[test]
    fn test_request_error() {
        let mut ilc = MockInnerLoopController::new();

        // Address 0 is invalid.
        let mut frame_request = [0, CODE_ILC_MODE, 0, 0, 0, 0];
        let frame_response = ilc.request(&frame_request);
        assert_eq!(frame_response[0], frame_request[0]);
        assert_eq!(frame_response[1], MOCK_CODE_ILC_ERROR);
        assert_eq!(frame_response[2], MOCK_CODE_ILC_EXCEPTION);

        // Address 249 is invalid.
        frame_request[0] = 249;
        assert_eq!(ilc.request(&frame_request)[1], MOCK_CODE_ILC_ERROR);

        // Invalid function code.
        frame_request[0] = 1;
        frame_request[1] = 0xFF;
        assert_eq!(ilc.request(&frame_request)[1], MOCK_CODE_ILC_ERROR);
    }

    #[test]
    fn test_get_mode() {
        let mut ilc_mock = MockInnerLoopController::new();
        let ilc = InnerLoopController::new();

        let address = 0;
        assert_eq!(
            ilc_mock.get_mode(&ilc.create_frame_get_mode(address)),
            [
                address + 1,
                CODE_ILC_MODE,
                0,
                InnerLoopController::get_mode_value(InnerLoopControlMode::Standby) as u8,
                81,
                204
            ]
        );

        let mode = InnerLoopControlMode::Disabled;
        ilc_mock.set_mode(&ilc.create_frame_set_mode(address, mode));
        assert_eq!(
            ilc_mock.get_mode(&ilc.create_frame_get_mode(address)),
            [
                address + 1,
                CODE_ILC_MODE,
                0,
                InnerLoopController::get_mode_value(mode) as u8,
                144,
                12
            ]
        );
    }

    #[test]
    fn test_set_mode() {
        let mut ilc_mock = MockInnerLoopController::new();
        let ilc = InnerLoopController::new();

        // To Disabled state.
        let address = 0;
        let mode_disabled = InnerLoopControlMode::Disabled;
        assert_eq!(
            ilc_mock.set_mode(&ilc.create_frame_set_mode(address, mode_disabled)),
            [
                address + 1,
                CODE_ILC_MODE,
                0,
                InnerLoopController::get_mode_value(mode_disabled) as u8,
                144,
                12
            ]
        );

        // Request ClearFaults when no fault.
        assert_eq!(
            ilc_mock
                .set_mode(&ilc.create_frame_set_mode(address, InnerLoopControlMode::ClearFaults)),
            [
                address + 1,
                CODE_ILC_MODE,
                0,
                InnerLoopController::get_mode_value(mode_disabled) as u8,
                144,
                12
            ]
        );

        // To Fault state.
        let mode_fault = InnerLoopControlMode::Fault;
        assert_eq!(
            ilc_mock.set_mode(&ilc.create_frame_set_mode(address, mode_fault)),
            [
                address + 1,
                CODE_ILC_MODE,
                0,
                InnerLoopController::get_mode_value(mode_fault) as u8,
                80,
                15
            ]
        );

        // Request ClearFaults to transition to Standby state.
        let mode_standby = InnerLoopControlMode::Standby;
        assert_eq!(
            ilc_mock
                .set_mode(&ilc.create_frame_set_mode(address, InnerLoopControlMode::ClearFaults)),
            [
                address + 1,
                CODE_ILC_MODE,
                0,
                InnerLoopController::get_mode_value(mode_standby) as u8,
                81,
                204
            ]
        );

        // No change
        assert_eq!(
            ilc_mock.set_mode(&ilc.create_frame_set_mode(address, InnerLoopControlMode::NoChange)),
            [
                address + 1,
                CODE_ILC_MODE,
                0,
                InnerLoopController::get_mode_value(mode_standby) as u8,
                81,
                204
            ]
        );
    }

    #[test]
    fn test_get_ilc_data() {
        let mut ilc = MockInnerLoopController::new();
        ilc.data.status = 0b1010_0000;
        ilc.data.encoder_count = 123456789;
        ilc.data.force = 123.456;

        let frame_request = [0x01, CODE_FORCE_REQUEST, 0, 0];
        let frame_response = ilc.get_ilc_data(&frame_request);
        assert_eq!(frame_response[0], frame_request[0]);
        assert_eq!(frame_response[1], frame_request[1]);
        assert_eq!(frame_response[2], ilc.data.status);
        assert_eq!(&frame_response[3..7], &ilc.data.encoder_count.to_be_bytes());
        assert_eq!(&frame_response[7..11], &ilc.data.force.to_be_bytes());
    }

    #[test]
    fn test_get_monitor_values() {
        let mut ilc = MockInnerLoopController::new();
        ilc.monitor_values = vec![1.23, 4.56, 7.89];

        let frame_request = [0x01, CODE_MONITOR_SENSOR, 0, 0];
        let frame_response = ilc.get_monitor_values(&frame_request);

        assert_eq!(frame_response.len(), 4 + ilc.monitor_values.len() * 4);

        assert_eq!(frame_response[0], frame_request[0]);
        assert_eq!(frame_response[1], frame_request[1]);

        for (idx, value) in ilc.monitor_values.iter().enumerate() {
            assert_eq!(
                &frame_response[(2 + idx * 4)..(2 + (idx + 1) * 4)],
                &value.to_be_bytes()
            );
        }
    }

    #[test]
    fn test_update_communication_counter() {
        let mut ilc = MockInnerLoopController::new();

        ilc.update_communication_counter(1);
        assert_eq!(ilc.data.status, 0b0001_0000);

        ilc.update_communication_counter(15);
        assert_eq!(ilc.data.status, 0b1111_0000);
    }

    #[test]
    fn test_update_encoder_and_force() {
        let mut ilc = MockInnerLoopController::new();

        let encoder_count = 123456789;
        let force = 123.456;
        ilc.update_encoder_and_force(encoder_count, force);

        assert_eq!(ilc.data.encoder_count, encoder_count);
        assert_eq!(ilc.data.force, -force as f32);
    }
}
