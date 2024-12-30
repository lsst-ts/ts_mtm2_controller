use crate::enums::PowerSystemState;

#[derive(Copy, Clone)]
pub struct SubPowerSystem {
    // Power is on or not
    pub is_power_on: bool,
    // System state
    pub state: PowerSystemState,
}

impl SubPowerSystem {
    /// Create a new instance of the sub power system.
    ///
    /// # Returns
    /// New instance of the sub power system.
    pub fn new() -> Self {
        Self {
            is_power_on: false,
            state: PowerSystemState::Init,
        }
    }

    /// Is the power on or not.
    ///
    /// # Returns
    /// True if the power is on. Otherwise, false.
    pub fn is_power_on(&self) -> bool {
        self.is_power_on
            && (self.state == PowerSystemState::PoweringOn
                || self.state == PowerSystemState::PoweredOn
                || self.state == PowerSystemState::ResettingBreakers)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_sub_power_system() -> SubPowerSystem {
        SubPowerSystem::new()
    }

    #[test]
    fn test_is_power_on() {
        let mut sub_power_system = create_sub_power_system();

        assert!(!sub_power_system.is_power_on());

        sub_power_system.is_power_on = true;
        sub_power_system.state = PowerSystemState::PoweringOn;
        assert!(sub_power_system.is_power_on());

        sub_power_system.state = PowerSystemState::PoweredOn;
        assert!(sub_power_system.is_power_on());

        sub_power_system.state = PowerSystemState::ResettingBreakers;
        assert!(sub_power_system.is_power_on());
    }
}
