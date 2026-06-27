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

pub const NUM_AXIAL_ACTUATOR: usize = 72;
pub const NUM_TANGENT_LINK: usize = 6;
pub const NUM_ACTUATOR: usize = NUM_AXIAL_ACTUATOR + NUM_TANGENT_LINK;

pub const NUM_HARDPOINTS_AXIAL: usize = 3;
pub const NUM_HARDPOINTS_TANGENT: usize = 3;
pub const NUM_HARDPOINTS: usize = NUM_HARDPOINTS_AXIAL + NUM_HARDPOINTS_TANGENT;

pub const NUM_ACTIVE_ACTUATOR_AXIAL: usize = NUM_AXIAL_ACTUATOR - NUM_HARDPOINTS_AXIAL;
pub const NUM_ACTIVE_ACTUATOR_TANGENT: usize = NUM_TANGENT_LINK - NUM_HARDPOINTS_TANGENT;

pub const NUM_ACTIVE_ACTUATOR: usize = NUM_ACTIVE_ACTUATOR_AXIAL + NUM_ACTIVE_ACTUATOR_TANGENT;

pub const NUM_INNER_LOOP_CONTROLLER: usize = 84;
pub const NUM_ILC_TEMPERATURE_MONITOR_SENSOR: usize = 4;
pub const NUM_ILC_CHANNEL: usize = 4;

pub const NUM_TEMPERATURE_RING: usize = 12;
pub const NUM_TEMPERATURE_INTAKE: usize = 2;
pub const NUM_TEMPERATURE_EXHAUST: usize = 2;

pub const NUM_LUT_TEMPERATURE: usize = 4;

// Independent measurement system (IMS)
pub const NUM_IMS: usize = 6;
pub const NUM_IMS_READING: usize = 2 * NUM_IMS;

pub const NUM_SPACE_DEGREE_OF_FREEDOM: usize = 6;

// Each column has 5 degree difference from 0 to 360 degree.
// Therefore, we have (360 / 5) + 1 = 73 columns.
pub const NUM_COLUMN_LUT_GRAVITY: usize = 73;

pub const BOUND_SYNC_CHANNEL: usize = 100;

pub const DEFAULT_POSITION_FILENAME: &str = "position.yaml";

// Code to report the server ID that contains the server identifier information
// relating to the ILC revisions and operating state.
pub const CODE_REPORT_SERVER_ID: u8 = 0x11;

// Code to report the server status that reports the mode, status, and faults.
pub const CODE_REPORT_SERVER_STATUS: u8 = 0x12;

// Code to read or change inner-loop controller (ILC) mode. This is used for
// all ILCs.
pub const CODE_ILC_MODE: u8 = 0x41;

// Code used with BROADCAST_ADDRESS to broadcast step motor command to all
// stepper controlled actuator ILCs (tangent & axial only).
pub const CODE_STEP_MOTOR_BROADCAST: u8 = 0x42;

// Code to read load cell force and ILC status data from the individual
// actuator ILCs (tangent & axial only).
pub const CODE_FORCE_REQUEST: u8 = 0x43;

// Code to read or set the scan rate of the ADC converter of the actuator ILCs.
pub const CODE_SCAN_RATE: u8 = 0x50;

// Code to set the ADC channel offset and sensitivity of step motor driven
// actuator ILCs. This sets the offset and load cell sensitivity values of a
// sensor attached to one of the four ILC analog input channels. These values
// are programmed to the ILC EEPROM calibration memory and need only be set
// once.
pub const CODE_SET_OFFSET_AND_SENSITIVITY: u8 = 0x51;

// Code to read the monitor sensor: temperature, displacement, and
// inclinometer.
pub const CODE_MONITOR_SENSOR: u8 = 0x54;

// Code to reset the ILC. Following the request, the ILC performs a reset and
// then responds with the response frame.
pub const CODE_RESET: u8 = 0x6B;

// Code to respond to the client with calibration data read from the ILC EEPROM
// of actuator ILCs.
pub const CODE_READ_CALIBRATION_DATA: u8 = 0x6E;

// Broadcast address for all ILCs. This is used to send the step command to all
// actuator ILCs at once.
pub const BROADCAST_ADDRESS: u8 = 248;

// Number of the ILC ports.
pub const NUMBER_ILC_PORT: u8 = 4;

// Interrupt request (IRQ) number for the ILC communication.
// The portSerialMasterSlave.vi in ts_mtm2_cell uses the bit 1 in Interrupt.
// Therefore, the IRQ number is 2^1 = 2.
pub const IRQ_NUMBER_ILC: u32 = 2;
