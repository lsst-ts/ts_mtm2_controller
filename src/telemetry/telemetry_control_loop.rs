use serde_json::{json, Value};
use std::collections::HashMap;

use crate::constants::{
    NUM_ACTUATOR, NUM_AXIAL_ACTUATOR, NUM_IMS, NUM_TANGENT_LINK, NUM_TEMPERATURE_EXHAUST,
    NUM_TEMPERATURE_INTAKE, NUM_TEMPERATURE_RING,
};
use crate::telemetry::telemetry_default::TelemetryDefault;

#[derive(Clone)]
pub struct TelemetryControlLoop {
    // Mirror is in position or not.
    pub is_in_position: bool,
    // Inclinometer angle in degree.
    pub inclinometer: HashMap<String, f64>,
    // Temperature in degree C.
    pub temperature: HashMap<String, Vec<f64>>,
    // Actuator steps.
    pub actuator_steps: Vec<i32>,
    // Actuator positions in millimeter.
    pub actuator_positions: Vec<f64>,
    // Actuator forces in Newton.
    pub forces: HashMap<String, Vec<f64>>,
    // Inner loop controller (ILC) data.
    pub ilc_data: Vec<u8>,
    // Data of the displacement sensors in micron.
    pub displacement_sensors: HashMap<String, Vec<f64>>,
    // Mirror position based on the hardpoints in um and arcsrc.
    pub mirror_position: HashMap<String, f64>,
    // Mirror position based on the independent measurement system (IMS) in um
    // and arcsrc.
    pub mirror_position_ims: HashMap<String, f64>,
    // Net total force in Newton.
    pub net_total_forces: HashMap<String, f64>,
    // Net total moment in Newton * meter.
    pub net_total_moments: HashMap<String, f64>,
    // Tangent force error. The first 6 elements are the "force". The last 2 elements are the "weight" and "sum". The unit is Newton.
    pub tangent_force_error: Vec<f64>,
    // Force balance based on the hardpoint correction. The unit are Newton for
    // the forces and Newton * meter for the moments.
    pub force_balance: HashMap<String, f64>,
    // Cycle time in second.
    pub cycle_time: f64,
}

impl TelemetryDefault for TelemetryControlLoop {
    fn get_messages(&self, digit: i32) -> Vec<Value> {
        let mut messages = Vec::new();
        messages.push(self.get_message_position(digit));
        messages.push(self.get_message_position_ims(digit));
        messages.push(self.get_message_axial_force(digit));
        messages.push(self.get_message_tangent_force(digit));
        messages.push(self.get_message_force_error_tangent(digit));
        messages.push(self.get_message_temperature(digit));
        messages.push(self.get_message_zenith_angle(digit));
        messages.push(self.get_message_inclinometer_angle_tma(digit));
        messages.push(self.get_message_axial_actuator_steps());
        messages.push(self.get_message_tangent_actuator_steps());
        messages.push(self.get_message_axial_encoder_positions(digit));
        messages.push(self.get_message_tangent_encoder_positions(digit));
        messages.push(self.get_message_ilc_data());
        messages.push(self.get_message_displacement_sensors(digit));
        messages.push(self.get_message_force_balance(digit));
        messages.push(self.get_message_net_forces_total(digit));
        messages.push(self.get_message_net_moments_total(digit));

        messages
    }
}

impl TelemetryControlLoop {
    /// Create a new control-loop telemetry object.
    pub fn new() -> Self {
        let mut temperature = HashMap::new();
        temperature.insert(String::from("ring"), vec![0.0; NUM_TEMPERATURE_RING]);
        temperature.insert(String::from("intake"), vec![0.0; NUM_TEMPERATURE_INTAKE]);
        temperature.insert(String::from("exhaust"), vec![0.0; NUM_TEMPERATURE_EXHAUST]);

        Self {
            is_in_position: false,
            inclinometer: Self::initialize_dict_value(
                &["raw", "processed", "zenith", "external"],
                0.0,
            ),
            temperature: temperature,
            actuator_steps: vec![0; NUM_ACTUATOR],
            actuator_positions: vec![0.0; NUM_ACTUATOR],
            forces: Self::initialize_dict_vector(
                &[
                    "lutGravity",
                    "lutTemperature",
                    "applied",
                    "measured",
                    "hardpointCorrection",
                ],
                0.0,
                NUM_ACTUATOR,
            ),
            ilc_data: vec![0; NUM_ACTUATOR],
            displacement_sensors: Self::initialize_dict_vector(&["thetaZ", "deltaZ"], 0.0, NUM_IMS),
            mirror_position: Self::initialize_dict_value(
                &["x", "y", "z", "xRot", "yRot", "zRot"],
                0.0,
            ),
            mirror_position_ims: Self::initialize_dict_value(
                &["x", "y", "z", "xRot", "yRot", "zRot"],
                0.0,
            ),
            net_total_forces: Self::initialize_dict_value(&["fx", "fy", "fz"], 0.0),
            net_total_moments: Self::initialize_dict_value(&["mx", "my", "mz"], 0.0),
            tangent_force_error: vec![0.0; NUM_TANGENT_LINK + 2],
            force_balance: Self::initialize_dict_value(&["fx", "fy", "fz", "mx", "my", "mz"], 0.0),

            cycle_time: 0.0,
        }
    }

    /// Get the message of the position.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the position.
    fn get_message_position(&self, digit: i32) -> Value {
        json!({
            "id": "position",
            "x": self.round(self.mirror_position["x"], digit),
            "y": self.round(self.mirror_position["y"], digit),
            "z": self.round(self.mirror_position["z"], digit),
            "xRot": self.round(self.mirror_position["xRot"], digit),
            "yRot": self.round(self.mirror_position["yRot"], digit),
            "zRot": self.round(self.mirror_position["zRot"], digit),
        })
    }

    /// Get the message of the position based on the independent measurement
    /// system (IMS).
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the position based on the IMS.
    fn get_message_position_ims(&self, digit: i32) -> Value {
        json!({
            "id": "positionIMS",
            "x": self.round(self.mirror_position_ims["x"], digit),
            "y": self.round(self.mirror_position_ims["y"], digit),
            "z": self.round(self.mirror_position_ims["z"], digit),
            "xRot": self.round(self.mirror_position_ims["xRot"], digit),
            "yRot": self.round(self.mirror_position_ims["yRot"], digit),
            "zRot": self.round(self.mirror_position_ims["zRot"], digit),
        })
    }

    /// Get the message of the axial force.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the axial force.
    fn get_message_axial_force(&self, digit: i32) -> Value {
        json!({
            "id": "axialForce",
            "lutGravity":self.round_vector(&self.forces["lutGravity"][..NUM_AXIAL_ACTUATOR], digit),
            "lutTemperature": self.round_vector(&self.forces["lutTemperature"][..NUM_AXIAL_ACTUATOR], digit),
            "applied": self.round_vector(&self.forces["applied"][..NUM_AXIAL_ACTUATOR], digit),
            "measured": self.round_vector(&self.forces["measured"][..NUM_AXIAL_ACTUATOR], digit),
            "hardpointCorrection": self.round_vector(&self.forces["hardpointCorrection"][..NUM_AXIAL_ACTUATOR], digit),
        })
    }

    /// Get the message of the tangent force.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the tangent force.
    fn get_message_tangent_force(&self, digit: i32) -> Value {
        json!({
            "id": "tangentForce",
            "lutGravity": self.round_vector(&self.forces["lutGravity"][NUM_AXIAL_ACTUATOR..NUM_ACTUATOR], digit),
            "lutTemperature": self.round_vector(&self.forces["lutTemperature"][NUM_AXIAL_ACTUATOR..NUM_ACTUATOR], digit),
            "applied": self.round_vector(&self.forces["applied"][NUM_AXIAL_ACTUATOR..NUM_ACTUATOR], digit),
            "measured": self.round_vector(&self.forces["measured"][NUM_AXIAL_ACTUATOR..NUM_ACTUATOR], digit),
            "hardpointCorrection": self.round_vector(&self.forces["hardpointCorrection"][NUM_AXIAL_ACTUATOR..NUM_ACTUATOR], digit),
        })
    }

    /// Get the message of the tangent force error.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the tangent force error.
    fn get_message_force_error_tangent(&self, digit: i32) -> Value {
        json!({
            "id": "forceErrorTangent",
            "force": self.round_vector(&self.tangent_force_error[..NUM_TANGENT_LINK], digit),
            "weight": self.round(self.tangent_force_error[NUM_TANGENT_LINK], digit),
            "sum": self.round(self.tangent_force_error[NUM_TANGENT_LINK + 1], digit),
        })
    }

    /// Get the message of the temperature.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the temperature.
    fn get_message_temperature(&self, digit: i32) -> Value {
        json!({
            "id": "temperature",
            "ring": self.round_vector(&self.temperature["ring"], digit),
            "intake": self.round_vector(&self.temperature["intake"], digit),
            "exhaust": self.round_vector(&self.temperature["exhaust"], digit),
        })
    }

    /// Get the message of the zenith angle.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the zenith angle.
    fn get_message_zenith_angle(&self, digit: i32) -> Value {
        json!({
            "id": "zenithAngle",
            "measured": self.round(self.inclinometer["zenith"], digit),
            "inclinometerRaw": self.round(self.inclinometer["raw"], digit),
            "inclinometerProcessed": self.round(self.inclinometer["processed"], digit),
        })
    }

    /// Get the message of the telescope mount assembly (TMA) inclinometer
    /// angle.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the TMA inclinometer angle.
    fn get_message_inclinometer_angle_tma(&self, digit: i32) -> Value {
        json!({
            "id": "inclinometerAngleTma",
            "inclinometer": self.round(self.inclinometer["external"], digit),
        })
    }

    /// Get the message of the axial actuator steps.
    ///
    /// # Returns
    /// The message of the axial actuator steps.
    fn get_message_axial_actuator_steps(&self) -> Value {
        json!({
            "id": "axialActuatorSteps",
            "steps": &self.actuator_steps[..NUM_AXIAL_ACTUATOR],
        })
    }

    /// Get the message of the tangent actuator steps.
    ///
    /// # Returns
    /// The message of the tangent actuator steps.
    fn get_message_tangent_actuator_steps(&self) -> Value {
        json!({
            "id": "tangentActuatorSteps",
            "steps": &self.actuator_steps[NUM_AXIAL_ACTUATOR..NUM_ACTUATOR],
        })
    }

    /// Get the message of the axial encoder positions in micron.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the axial encoder positions.
    fn get_message_axial_encoder_positions(&self, digit: i32) -> Value {
        // Change the unit from mm to um
        let actuator_positions: Vec<f64> = self.actuator_positions[..NUM_AXIAL_ACTUATOR]
            .iter()
            .map(|x| self.round(x * 1000.0, digit))
            .collect();

        json!({
            "id": "axialEncoderPositions",
            "position": actuator_positions,
        })
    }

    /// Get the message of the tangent encoder positions in micron.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the tangent encoder positions.
    fn get_message_tangent_encoder_positions(&self, digit: i32) -> Value {
        // Change the unit from mm to um
        let actuator_positions: Vec<f64> = self.actuator_positions
            [NUM_AXIAL_ACTUATOR..NUM_ACTUATOR]
            .iter()
            .map(|x| self.round(x * 1000.0, digit))
            .collect();

        json!({
            "id": "tangentEncoderPositions",
            "position": actuator_positions,
        })
    }

    /// Get the message of the inner loop controller (ILC) data.
    ///
    /// # Returns
    /// The message of the ILC data.
    fn get_message_ilc_data(&self) -> Value {
        json!({
            "id": "ilcData",
            "status": self.ilc_data,
        })
    }

    /// Get the message of the displacement sensors.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the displacement sensors.
    fn get_message_displacement_sensors(&self, digit: i32) -> Value {
        json!({
            "id": "displacementSensors",
            "thetaZ": self.round_vector(&self.displacement_sensors["thetaZ"], digit),
            "deltaZ": self.round_vector(&self.displacement_sensors["deltaZ"], digit),
        })
    }

    /// Get the message of the net forces and moments as commanded by the force
    /// balance system.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the force balance system.
    fn get_message_force_balance(&self, digit: i32) -> Value {
        json!({
            "id": "forceBalance",
            "fx": self.round(self.force_balance["fx"], digit),
            "fy": self.round(self.force_balance["fy"], digit),
            "fz": self.round(self.force_balance["fz"], digit),
            "mx": self.round(self.force_balance["mx"], digit),
            "my": self.round(self.force_balance["my"], digit),
            "mz": self.round(self.force_balance["mz"], digit),
        })
    }

    /// Get the message of the net total forces.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the net total forces.
    fn get_message_net_forces_total(&self, digit: i32) -> Value {
        json!({
            "id": "netForcesTotal",
            "fx": self.round(self.net_total_forces["fx"], digit),
            "fy": self.round(self.net_total_forces["fy"], digit),
            "fz": self.round(self.net_total_forces["fz"], digit),
        })
    }

    /// Get the message of the net total moments.
    ///
    /// # Arguments
    /// * `digit` - The number of digits after the decimal point.
    ///
    /// # Returns
    /// The message of the net total moments.
    fn get_message_net_moments_total(&self, digit: i32) -> Value {
        json!({
            "id": "netMomentsTotal",
            "mx": self.round(self.net_total_moments["mx"], digit),
            "my": self.round(self.net_total_moments["my"], digit),
            "mz": self.round(self.net_total_moments["mz"], digit),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::from_value;

    #[test]
    fn test_get_messages() {
        let telemetry = TelemetryControlLoop::new();

        let messages_on = telemetry.get_messages(2);

        assert_eq!(messages_on.len(), 17);
    }

    #[test]
    fn test_get_message_position() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_position(2);

        assert_eq!(message["id"], "position");
        for key in ["x", "y", "z", "xRot", "yRot", "zRot"].iter() {
            assert_eq!(
                message[*key],
                telemetry.round(telemetry.mirror_position[*key], 2)
            );
        }
    }

    #[test]
    fn test_get_message_position_ims() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_position_ims(2);

        assert_eq!(message["id"], "positionIMS");
        for key in ["x", "y", "z", "xRot", "yRot", "zRot"].iter() {
            assert_eq!(
                message[*key],
                telemetry.round(telemetry.mirror_position_ims[*key], 2)
            );
        }
    }

    #[test]
    fn test_get_message_axial_force() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_axial_force(2);

        assert_eq!(message["id"], "axialForce");
        for key in [
            "lutGravity",
            "lutTemperature",
            "applied",
            "measured",
            "hardpointCorrection",
        ]
        .iter()
        {
            assert_eq!(
                from_value::<Vec<f64>>(message[*key].clone()).unwrap(),
                telemetry.round_vector(&telemetry.forces[*key][..NUM_AXIAL_ACTUATOR], 2)
            );
        }
    }

    #[test]
    fn test_get_message_tangent_force() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_tangent_force(2);

        assert_eq!(message["id"], "tangentForce");
        for key in [
            "lutGravity",
            "lutTemperature",
            "applied",
            "measured",
            "hardpointCorrection",
        ]
        .iter()
        {
            assert_eq!(
                from_value::<Vec<f64>>(message[*key].clone()).unwrap(),
                telemetry
                    .round_vector(&telemetry.forces[*key][NUM_AXIAL_ACTUATOR..NUM_ACTUATOR], 2)
            );
        }
    }

    #[test]
    fn test_get_message_force_error_tangent() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_force_error_tangent(2);

        assert_eq!(message["id"], "forceErrorTangent");
        assert_eq!(
            from_value::<Vec<f64>>(message["force"].clone()).unwrap(),
            telemetry.round_vector(&telemetry.tangent_force_error[..NUM_TANGENT_LINK], 2)
        );
        assert_eq!(
            message["weight"],
            telemetry.round(telemetry.tangent_force_error[NUM_TANGENT_LINK], 2)
        );
        assert_eq!(
            message["sum"],
            telemetry.round(telemetry.tangent_force_error[NUM_TANGENT_LINK + 1], 2)
        );
    }

    #[test]
    fn test_get_message_temperature() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_temperature(2);

        assert_eq!(message["id"], "temperature");
        for key in ["ring", "intake", "exhaust"].iter() {
            assert_eq!(
                from_value::<Vec<f64>>(message[*key].clone()).unwrap(),
                telemetry.round_vector(&telemetry.temperature[*key], 2)
            );
        }
    }

    #[test]
    fn test_get_message_zenith_angle() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_zenith_angle(2);

        assert_eq!(message["id"], "zenithAngle");
        for (key1, key2) in std::iter::zip(
            ["measured", "inclinometerRaw", "inclinometerProcessed"],
            ["zenith", "raw", "processed"],
        ) {
            assert_eq!(
                message[key1],
                telemetry.round(telemetry.inclinometer[key2], 2)
            );
        }
    }

    #[test]
    fn test_get_message_inclinometer_angle_tma() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_inclinometer_angle_tma(2);

        assert_eq!(message["id"], "inclinometerAngleTma");
        assert_eq!(
            message["inclinometer"],
            telemetry.round(telemetry.inclinometer["external"], 2)
        );
    }

    #[test]
    fn test_get_message_axial_actuator_steps() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_axial_actuator_steps();

        assert_eq!(message["id"], "axialActuatorSteps");
        assert_eq!(
            from_value::<Vec<i32>>(message["steps"].clone()).unwrap(),
            telemetry.actuator_steps[..NUM_AXIAL_ACTUATOR]
        );
    }

    #[test]
    fn test_get_message_tangent_actuator_steps() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_tangent_actuator_steps();

        assert_eq!(message["id"], "tangentActuatorSteps");
        assert_eq!(
            from_value::<Vec<i32>>(message["steps"].clone()).unwrap(),
            telemetry.actuator_steps[NUM_AXIAL_ACTUATOR..NUM_ACTUATOR]
        );
    }

    #[test]
    fn test_get_message_axial_encoder_positions() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_axial_encoder_positions(2);

        let actuator_positions: Vec<f64> = telemetry.actuator_positions[..NUM_AXIAL_ACTUATOR]
            .iter()
            .map(|x| telemetry.round(x * 1000.0, 2))
            .collect();

        assert_eq!(message["id"], "axialEncoderPositions");
        assert_eq!(
            from_value::<Vec<f64>>(message["position"].clone()).unwrap(),
            actuator_positions
        );
    }

    #[test]
    fn test_get_message_tangent_encoder_positions() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_tangent_encoder_positions(2);

        let actuator_positions: Vec<f64> = telemetry.actuator_positions
            [NUM_AXIAL_ACTUATOR..NUM_ACTUATOR]
            .iter()
            .map(|x| telemetry.round(x * 1000.0, 2))
            .collect();

        assert_eq!(message["id"], "tangentEncoderPositions");
        assert_eq!(
            from_value::<Vec<f64>>(message["position"].clone()).unwrap(),
            actuator_positions
        );
    }

    #[test]
    fn test_get_message_ilc_data() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_ilc_data();

        assert_eq!(message["id"], "ilcData");
        assert_eq!(
            from_value::<Vec<u8>>(message["status"].clone()).unwrap(),
            telemetry.ilc_data
        );
    }

    #[test]
    fn test_get_message_displacement_sensors() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_displacement_sensors(2);

        assert_eq!(message["id"], "displacementSensors");
        for key in ["thetaZ", "deltaZ"].iter() {
            assert_eq!(
                from_value::<Vec<f64>>(message[*key].clone()).unwrap(),
                telemetry.round_vector(&telemetry.displacement_sensors[*key], 2)
            );
        }
    }

    #[test]
    fn test_get_message_force_balance() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_force_balance(2);

        assert_eq!(message["id"], "forceBalance");
        for key in ["fx", "fy", "fz", "mx", "my", "mz"].iter() {
            assert_eq!(
                message[*key],
                telemetry.round(telemetry.force_balance[*key], 2)
            );
        }
    }

    #[test]
    fn test_get_message_net_forces_total() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_net_forces_total(2);

        assert_eq!(message["id"], "netForcesTotal");
        for key in ["fx", "fy", "fz"].iter() {
            assert_eq!(
                message[*key],
                telemetry.round(telemetry.net_total_forces[*key], 2)
            );
        }
    }

    #[test]
    fn test_get_message_net_moments_total() {
        let telemetry = TelemetryControlLoop::new();

        let message = telemetry.get_message_net_moments_total(2);

        assert_eq!(message["id"], "netMomentsTotal");
        for key in ["mx", "my", "mz"].iter() {
            assert_eq!(
                message[*key],
                telemetry.round(telemetry.net_total_moments[*key], 2)
            );
        }
    }
}
