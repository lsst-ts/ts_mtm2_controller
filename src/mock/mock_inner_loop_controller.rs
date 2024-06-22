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
