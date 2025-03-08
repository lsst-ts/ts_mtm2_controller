use std::collections::{HashMap, HashSet};

use crate::constants::NUM_INNER_LOOP_CONTROLLER;
use crate::enums::{
    ClosedLoopControlMode, Commander, InnerLoopControlMode, PowerSystemState, PowerType,
};
use crate::power::sub_power_system::SubPowerSystem;

struct ConnectionStatus {
    // Is connected or not.
    is_connected: bool,
    // Sent welcome message or not.
    sent_welcome_message: bool,
}

impl ConnectionStatus {
    /// Create a new connection status.
    ///
    /// # Returns
    /// A new connection status.
    fn new() -> Self {
        Self {
            is_connected: false,
            sent_welcome_message: false,
        }
    }
}

pub struct Status {
    // Connection status.
    _connections: HashMap<Commander, ConnectionStatus>,
    // Mirror is in position or not.
    pub is_in_position: bool,
    // Cell temperature is high or not
    pub is_cell_temperature_high: bool,
    // Interlock is on or not.
    pub is_interlock_on: bool,
    // Digital output status.
    pub digital_output: u8,
    // Digital input status.
    pub digital_input: u32,
    // Power system status.
    pub power_system: HashMap<PowerType, SubPowerSystem>,
    // Closed loop control mode.
    pub mode: ClosedLoopControlMode,
    // Inner loop controller (ILC) modes.
    pub ilc_modes: Vec<InnerLoopControlMode>,
    // Triggered Limit switches.
    pub limit_switch: HashMap<String, HashSet<i32>>,
    // Summary of the faults status.
    pub summary_faults_status: u64,
}

impl Status {
    /// Create a new status.
    ///
    /// # Returns
    /// A new status.
    pub fn new() -> Self {
        let mut connections = HashMap::new();
        [Commander::CSC, Commander::GUI].iter().for_each(|key| {
            connections.insert(*key, ConnectionStatus::new());
        });

        let mut power_system = HashMap::new();
        [PowerType::Motor, PowerType::Communication]
            .iter()
            .for_each(|key| {
                power_system.insert(*key, SubPowerSystem::new());
            });

        let mut limit_switch = HashMap::new();
        ["retract", "extend"].iter().for_each(|key| {
            limit_switch.insert(String::from(*key), HashSet::new());
        });

        Self {
            _connections: connections,

            is_in_position: false,
            is_cell_temperature_high: false,
            is_interlock_on: true,

            digital_output: 0,
            digital_input: 0,

            power_system: power_system,

            mode: ClosedLoopControlMode::Idle,

            ilc_modes: vec![InnerLoopControlMode::Unknown; NUM_INNER_LOOP_CONTROLLER],
            limit_switch: limit_switch,

            summary_faults_status: 0,
        }
    }

    /// Is the source connected?
    ///
    /// # Arguments
    /// `source` - Source of the connection.
    ///
    /// # Returns
    /// True if the source was connected. Otherwise, false.
    pub fn is_connected(&self, source: Commander) -> bool {
        self._connections[&source].is_connected
    }

    /// Update the connection status.
    ///
    /// # Arguments
    /// `source` - Source of the connection.
    /// `is_connected` - Is connected or not.
    pub fn update_connection_status(&mut self, source: Commander, is_connected: bool) {
        if let Some(connection_status) = self._connections.get_mut(&source) {
            connection_status.is_connected = is_connected;
        }

        if !is_connected {
            self.update_welcome_message_status(source, false);
        }
    }

    /// Update the welcome message status.
    ///
    /// # Arguments
    /// `source` - Source of the connection.
    /// `is_sent` - Welcome message is sent or not.
    pub fn update_welcome_message_status(&mut self, source: Commander, is_sent: bool) {
        if let Some(connection_status) = self._connections.get_mut(&source) {
            connection_status.sent_welcome_message = is_sent;
        }
    }

    /// Has the welcome message been sent?
    ///
    /// # Arguments
    /// `source` - Source of the connection.
    ///
    /// # Returns
    /// True if the welcome message has been sent. Otherwise, false.
    pub fn has_sent_welcome_message(&self, source: Commander) -> bool {
        self._connections[&source].sent_welcome_message
    }

    /// Check if all the inner loop controllers (ILCs) are enabled.
    ///
    /// # Arguments
    /// * `bypassed_ilcs` - Bypassed ILCs.
    ///
    /// # Returns
    /// True if all the ILCs are enabled. Otherwise, false.
    pub fn are_ilc_enabled(&self, bypassed_ilcs: &Vec<usize>) -> bool {
        for (idx, ilc) in self.ilc_modes.iter().enumerate() {
            if !bypassed_ilcs.contains(&idx) && *ilc != InnerLoopControlMode::Enabled {
                return false;
            }
        }

        true
    }

    /// Update the power system.
    ///
    /// # Arguments
    /// * `power_type` - Power type.
    /// * `is_power_on` - Is power on or not.
    /// * `state` - Power system state.
    ///
    /// # Returns
    /// Some(()) if the power system is updated successfully. Otherwise, None.
    pub fn update_power_system(
        &mut self,
        power_type: PowerType,
        is_power_on: bool,
        state: PowerSystemState,
    ) -> Option<()> {
        if let Some(power_system) = self.power_system.get_mut(&power_type) {
            power_system.is_power_on = is_power_on;
            power_system.state = state;

            return Some(());
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_status() -> Status {
        Status::new()
    }

    #[test]
    fn test_update_connection_status() {
        let mut status = create_status();

        // Connected.
        status.update_connection_status(Commander::CSC, true);
        assert!(status.is_connected(Commander::CSC));

        status.update_connection_status(Commander::GUI, true);
        assert!(status.is_connected(Commander::GUI));

        // Disconnected.
        status.update_welcome_message_status(Commander::CSC, true);

        status.update_connection_status(Commander::CSC, false);

        assert!(!status.is_connected(Commander::CSC));
        assert!(!status.has_sent_welcome_message(Commander::CSC));
    }

    #[test]
    fn test_update_welcome_message_status() {
        let mut status = create_status();

        // Sent.
        status.update_welcome_message_status(Commander::CSC, true);

        assert!(status.has_sent_welcome_message(Commander::CSC));

        status.update_welcome_message_status(Commander::GUI, true);

        assert!(status.has_sent_welcome_message(Commander::GUI));

        // Not sent.
        status.update_welcome_message_status(Commander::CSC, false);

        assert!(!status.has_sent_welcome_message(Commander::CSC));

        status.update_welcome_message_status(Commander::GUI, false);

        assert!(!status.has_sent_welcome_message(Commander::GUI));
    }

    #[test]
    fn test_are_ilc_enabled() {
        let mut status = create_status();

        assert_eq!(status.are_ilc_enabled(&vec![]), false);

        // No bypassed ILCs.
        for idx in 0..NUM_INNER_LOOP_CONTROLLER {
            status.ilc_modes[idx] = InnerLoopControlMode::Enabled;
        }

        assert_eq!(status.are_ilc_enabled(&vec![]), true);

        // Bypassed ILCs.
        status.ilc_modes[3] = InnerLoopControlMode::Unknown;

        assert_eq!(status.are_ilc_enabled(&vec![]), false);
        assert_eq!(status.are_ilc_enabled(&vec![3]), true);
    }

    #[test]
    fn test_update_power_system() {
        let mut status = create_status();

        status.update_power_system(PowerType::Motor, true, PowerSystemState::PoweredOn);

        assert_eq!(status.power_system[&PowerType::Motor].is_power_on(), true);
        assert_eq!(
            status.power_system[&PowerType::Motor].state,
            PowerSystemState::PoweredOn
        );
    }
}
