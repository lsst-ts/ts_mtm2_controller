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

use crate::enums::InnerLoopControlMode;

/// Mock Inner-Loop Controller (ILC) to simulate the behavior of hardware.
#[derive(Clone)]
pub struct MockInnerLoopController {
    // Controller's mode.
    pub mode: InnerLoopControlMode,
    _status: u8,
}

impl MockInnerLoopController {
    /// Create a new inner-loop controller (ILC).
    ///
    /// # Returns
    /// A new model with a random number.
    pub fn new() -> Self {
        Self {
            mode: InnerLoopControlMode::Standby,
            _status: 0b0000_0000,
        }
    }

    /// Set the mode of the controller.
    ///
    /// # Arguments
    /// * `mode` - The mode to be set.
    ///
    /// # Returns
    /// Current mode.
    pub fn set_mode(&mut self, mode: InnerLoopControlMode) -> InnerLoopControlMode {
        match mode {
            InnerLoopControlMode::NoChange => return self.mode,
            InnerLoopControlMode::Unknown => return self.mode,
            InnerLoopControlMode::ClearFaults => {
                self.mode = InnerLoopControlMode::Standby;
            }
            other => {
                self.mode = other;
            }
        }

        self.mode
    }

    /// Get the status.
    ///
    /// # Returns
    /// Status.
    pub fn get_status(&mut self) -> u8 {
        let current_status = self._status;

        // Update the communication bits 4-7.
        let mask = 0b1111_0000;
        if current_status & mask != mask {
            self._status += 0b0001_0000;
        } else {
            self._status -= mask;
        }

        current_status
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_set_mode() {
        let mut ilc = MockInnerLoopController::new();

        assert_eq!(
            ilc.set_mode(InnerLoopControlMode::Fault),
            InnerLoopControlMode::Fault
        );

        ilc.set_mode(InnerLoopControlMode::ClearFaults);
        assert_eq!(
            ilc.set_mode(InnerLoopControlMode::ClearFaults),
            InnerLoopControlMode::Standby
        );

        ilc.set_mode(InnerLoopControlMode::NoChange);
        assert_eq!(
            ilc.set_mode(InnerLoopControlMode::NoChange),
            InnerLoopControlMode::Standby
        );
    }

    #[test]
    fn test_get_status() {
        let mut ilc = MockInnerLoopController::new();

        (0..16).into_iter().for_each(|idx| {
            assert_eq!(ilc.get_status(), idx << 4);
        });

        assert_eq!(ilc.get_status(), 0b0000_0000);
    }
}
