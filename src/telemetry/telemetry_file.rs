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

use rmp_serde::{encode::Error, to_vec};
use serde::{Deserialize, Serialize};

use crate::telemetry::{
    telemetry_control_loop::TelemetryControlLoop, telemetry_power::TelemetryPower,
};

#[derive(Clone, Default, PartialEq, Debug, Serialize, Deserialize)]
pub struct TelemetryFile {
    // This struct should be consistent with telemetry_file.py to ensure the
    // reading and writing of telemetry data between Rust and Python are
    // compatible.

    // Elapsed timestamp of the telemetry data in milliseconds from the
    // beginning of the telemetry file process.
    pub timestamp: u128,
    // Processed communication voltage in volt.
    _processed_voltage_communication: f64,
    // Processed motor voltage in volt.
    _processed_voltage_motor: f64,
    // Processed communication current in ampere.
    _processed_current_communication: f64,
    // Processed motor current in ampere.
    _processed_current_motor: f64,
    // Mirror is in position or not.
    _is_in_position: bool,
    // Actuator steps.
    _actuator_steps: Vec<i32>,
    // Actuator positions in millimeter.
    _actuator_positions: Vec<f64>,
    // Gravity look-up table (LUT) forces in Newton.
    _forces_lut_gravity: Vec<f64>,
    // Temperature LUT forces in Newton.
    _forces_lut_temperature: Vec<f64>,
    // Applied actuator forces in Newton.
    _forces_applied: Vec<f64>,
    // Measured actuator forces in Newton.
    _forces_measured: Vec<f64>,
    // Hardpoint correction forces in Newton.
    _forces_hardpoint_correction: Vec<f64>,
    // Inner loop controller (ILC) status.
    _ilc_status: Vec<u8>,
    // ILC encoder values.
    _ilc_encoders: Vec<i32>,
    // Ring temperature in degree C.
    _temperature_ring: Vec<f64>,
    // Intake temperature in degree C.
    _temperature_intake: Vec<f64>,
    // Exhaust temperature in degree C.
    _temperature_exhaust: Vec<f64>,
    // Theta-Z displacement sensors in micron.
    _displacement_sensors_theta_z: Vec<f64>,
    // Delta-Z displacement sensors in micron.
    _displacement_sensors_delta_z: Vec<f64>,
    // Mirror position x based on the hardpoints in um.
    _mirror_position_x: f64,
    // Mirror position y based on the hardpoints in um.
    _mirror_position_y: f64,
    // Mirror position z based on the hardpoints in um.
    _mirror_position_z: f64,
    // Mirror rotation x based on the hardpoints in arcsec.
    _mirror_position_rx: f64,
    // Mirror rotation y based on the hardpoints in arcsec.
    _mirror_position_ry: f64,
    // Mirror rotation z based on the hardpoints in arcsec.
    _mirror_position_rz: f64,
    // Raw inclinometer angle in degree.
    _inclinometer_raw: f64,
    // Processed inclinometer angle in degree.
    _inclinometer_processed: f64,
    // Cycle time in milliseconds.
    _cycle_time: u64,
}

impl TelemetryFile {
    /// Create a new TelemetryFile instance and fill it with telemetry data
    /// from the provided TelemetryPower and TelemetryControlLoop structs.
    ///
    /// # Arguments
    /// * `telemetry_power` - TelemetryPower struct to fill the telemetry data.
    /// * `telemetry_control_loop` - TelemetryControlLoop struct to fill the
    ///   telemetry data.
    ///
    /// # Returns
    /// A new TelemetryFile instance filled with telemetry data.
    pub fn new(
        telemetry_power: &TelemetryPower,
        telemetry_control_loop: &TelemetryControlLoop,
    ) -> Self {
        Self {
            timestamp: 0,

            // Power telemetry data
            _processed_voltage_communication: telemetry_power.power_processed["commVoltage"],
            _processed_voltage_motor: telemetry_power.power_processed["motorVoltage"],
            _processed_current_communication: telemetry_power.power_processed["commCurrent"],
            _processed_current_motor: telemetry_power.power_processed["motorCurrent"],

            // Control loop telemetry data
            _is_in_position: telemetry_control_loop.is_in_position,

            _actuator_steps: telemetry_control_loop.actuator_steps.clone(),
            _actuator_positions: telemetry_control_loop.actuator_positions.clone(),

            _forces_lut_gravity: telemetry_control_loop.forces["lutGravity"].clone(),
            _forces_lut_temperature: telemetry_control_loop.forces["lutTemperature"].clone(),
            _forces_applied: telemetry_control_loop.forces["applied"].clone(),
            _forces_measured: telemetry_control_loop.forces["measured"].clone(),
            _forces_hardpoint_correction: telemetry_control_loop.forces["hardpointCorrection"]
                .clone(),

            _ilc_status: telemetry_control_loop.ilc_status.clone(),
            _ilc_encoders: telemetry_control_loop.ilc_encoders.clone(),

            _temperature_ring: telemetry_control_loop.temperature["ring"].clone(),
            _temperature_intake: telemetry_control_loop.temperature["intake"].clone(),
            _temperature_exhaust: telemetry_control_loop.temperature["exhaust"].clone(),

            _displacement_sensors_theta_z: telemetry_control_loop.displacement_sensors["thetaZ"]
                .clone(),
            _displacement_sensors_delta_z: telemetry_control_loop.displacement_sensors["deltaZ"]
                .clone(),

            _mirror_position_x: telemetry_control_loop.mirror_position["x"],
            _mirror_position_y: telemetry_control_loop.mirror_position["y"],
            _mirror_position_z: telemetry_control_loop.mirror_position["z"],
            _mirror_position_rx: telemetry_control_loop.mirror_position["xRot"],
            _mirror_position_ry: telemetry_control_loop.mirror_position["yRot"],
            _mirror_position_rz: telemetry_control_loop.mirror_position["zRot"],

            _inclinometer_raw: telemetry_control_loop.inclinometer["raw"],
            _inclinometer_processed: telemetry_control_loop.inclinometer["processed"],

            _cycle_time: telemetry_control_loop.cycle_time,
        }
    }

    /// Serialize the telemetry data to a vector of bytes.
    ///
    /// # Returns
    /// Result containing the serialized telemetry data as a vector of bytes,
    /// or an error if serialization fails.
    pub fn serialize(&self) -> Result<Vec<u8>, Error> {
        to_vec(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let mut telemetry_power = TelemetryPower::new();
        telemetry_power
            .power_processed
            .insert("commVoltage".to_string(), 12.0);
        telemetry_power
            .power_processed
            .insert("motorVoltage".to_string(), 24.0);
        telemetry_power
            .power_processed
            .insert("commCurrent".to_string(), 1.5);
        telemetry_power
            .power_processed
            .insert("motorCurrent".to_string(), 3.0);

        let mut telemetry_control_loop = TelemetryControlLoop::new();
        telemetry_control_loop.is_in_position = true;
        telemetry_control_loop
            .mirror_position
            .insert("x".to_string(), 1.0);
        telemetry_control_loop
            .mirror_position
            .insert("y".to_string(), 2.0);
        telemetry_control_loop
            .mirror_position
            .insert("z".to_string(), 3.0);
        telemetry_control_loop
            .mirror_position
            .insert("xRot".to_string(), 0.1);
        telemetry_control_loop
            .mirror_position
            .insert("yRot".to_string(), 0.2);
        telemetry_control_loop
            .mirror_position
            .insert("zRot".to_string(), 0.3);
        telemetry_control_loop
            .inclinometer
            .insert("raw".to_string(), 0.5);
        telemetry_control_loop
            .inclinometer
            .insert("processed".to_string(), 0.6);
        telemetry_control_loop.cycle_time = 10;

        let telemetry = TelemetryFile::new(&telemetry_power, &telemetry_control_loop);

        assert!(telemetry._is_in_position);

        assert_eq!(
            telemetry._actuator_steps,
            telemetry_control_loop.actuator_steps
        );
        assert_eq!(
            telemetry._actuator_positions,
            telemetry_control_loop.actuator_positions
        );

        assert_eq!(
            telemetry._forces_lut_gravity,
            telemetry_control_loop.forces["lutGravity"]
        );
        assert_eq!(
            telemetry._forces_lut_temperature,
            telemetry_control_loop.forces["lutTemperature"]
        );
        assert_eq!(
            telemetry._forces_applied,
            telemetry_control_loop.forces["applied"]
        );
        assert_eq!(
            telemetry._forces_measured,
            telemetry_control_loop.forces["measured"]
        );
        assert_eq!(
            telemetry._forces_hardpoint_correction,
            telemetry_control_loop.forces["hardpointCorrection"]
        );

        assert_eq!(telemetry._ilc_status, telemetry_control_loop.ilc_status);
        assert_eq!(telemetry._ilc_encoders, telemetry_control_loop.ilc_encoders);

        assert_eq!(telemetry._processed_voltage_communication, 12.0);
        assert_eq!(telemetry._processed_voltage_motor, 24.0);
        assert_eq!(telemetry._processed_current_communication, 1.5);
        assert_eq!(telemetry._processed_current_motor, 3.0);

        assert_eq!(
            telemetry._temperature_ring,
            telemetry_control_loop.temperature["ring"]
        );
        assert_eq!(
            telemetry._temperature_intake,
            telemetry_control_loop.temperature["intake"]
        );
        assert_eq!(
            telemetry._temperature_exhaust,
            telemetry_control_loop.temperature["exhaust"]
        );

        assert_eq!(
            telemetry._displacement_sensors_theta_z,
            telemetry_control_loop.displacement_sensors["thetaZ"]
        );
        assert_eq!(
            telemetry._displacement_sensors_delta_z,
            telemetry_control_loop.displacement_sensors["deltaZ"]
        );

        assert_eq!(telemetry._mirror_position_x, 1.0);
        assert_eq!(telemetry._mirror_position_y, 2.0);
        assert_eq!(telemetry._mirror_position_z, 3.0);
        assert_eq!(telemetry._mirror_position_rx, 0.1);
        assert_eq!(telemetry._mirror_position_ry, 0.2);
        assert_eq!(telemetry._mirror_position_rz, 0.3);

        assert_eq!(telemetry._inclinometer_raw, 0.5);
        assert_eq!(telemetry._inclinometer_processed, 0.6);

        assert_eq!(telemetry._cycle_time, 10);
    }

    #[test]
    fn test_serialize() {
        let telemetry = TelemetryFile::default();

        let serialized = telemetry.serialize();

        assert!(serialized.is_ok());
        assert!(!serialized.unwrap().is_empty());
    }
}
