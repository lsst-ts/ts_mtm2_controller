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
