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

// 1 step equals 1.9967536601e-5 millimeter
// In the simulation, we just use a single value. In the real system, each
// actuator has its own calibrated value.
pub const PLANT_STEP_TO_MM: f64 = 1.9967536601e-5;

// Temperature in Celsius
pub const PLANT_TEMPERATURE_HIGH: f64 = 26.53;
pub const PLANT_TEMPERATURE_LOW: f64 = 24.49;

// Voltage in Volt
pub const PLANT_VOLTAGE: f64 = 24.0;

// Current in Ampere
pub const PLANT_CURRENT_COMMUNICATION: f64 = 6.5;
pub const PLANT_CURRENT_MOTOR: f64 = 1.7;

// Test values
pub const TEST_DIGITAL_OUTPUT_NO_POWER: u8 = 0x1C;
pub const TEST_DIGITAL_OUTPUT_POWER_COMM: u8 = 0x1E;
pub const TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR: u8 = 0x1F;
pub const TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR_CLOSED_LOOP: u8 = 0x3F;

pub const TEST_DIGITAL_INPUT_NO_POWER: u32 = 0x9F00FFFF;
pub const TEST_DIGITAL_INPUT_POWER_COMM: u32 = 0x80007FFF;
pub const TEST_DIGITAL_INPUT_POWER_COMM_MOTOR: u32 = 0x3F;
