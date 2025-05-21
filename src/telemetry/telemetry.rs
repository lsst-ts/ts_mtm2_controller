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

use serde_json::Value;

use crate::telemetry::telemetry_control_loop::TelemetryControlLoop;
use crate::telemetry::telemetry_power::TelemetryPower;

pub struct Telemetry {
    // Telemetry of the power system.
    pub power: Option<TelemetryPower>,
    // Telemetry of the control loop.
    pub control_loop: Option<TelemetryControlLoop>,
    // Command execution result.
    pub command_result: Option<Value>,
    // Events to publish.
    pub events: Option<Vec<Value>>,
}

impl Telemetry {
    /// Create a new telemetry object.
    ///
    /// # Arguments
    /// * `power` - Telemetry of the power system.
    /// * `control_loop` - Telemetry of the control loop.
    /// * `command_result` - Command execution result.
    /// * `events` - Events to publish.
    ///
    /// # Returns
    /// A new telemetry object.
    pub fn new(
        power: Option<TelemetryPower>,
        control_loop: Option<TelemetryControlLoop>,
        command_result: Option<Value>,
        events: Option<Vec<Value>>,
    ) -> Self {
        Self {
            power: power,
            control_loop: control_loop,
            command_result: command_result,
            events: events,
        }
    }
}
