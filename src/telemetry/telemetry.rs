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
