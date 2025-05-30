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

use std::path::Path;

use crate::constants::{NUM_ACTUATOR, NUM_AXIAL_ACTUATOR};
use crate::control::actuator::Actuator;
use crate::control::math_tool::clip;
use crate::enums::ActuatorDisplacementUnit;

pub struct OpenLoop {
    // The open-loop control is running or not.
    pub is_running: bool,
    // Actuators
    pub actuators: Vec<Actuator>,
    // Selected 0-based actuator IDs to do the movement.
    _selected_actuators: Vec<usize>,
    // Displacement of steps to do the movement.
    _displacement_steps: Vec<i32>,
    // Is the simulation mode enabled or not.
    _is_simulation_mode: bool,
}

impl OpenLoop {
    /// Open-loop control.
    ///
    /// # Arguments
    /// * `is_simulation_mode` - Enable the simulation mode or not.
    ///
    /// # Returns
    /// A new OpenLoop object.
    pub fn new(is_simulation_mode: bool) -> Self {
        Self {
            is_running: false,
            actuators: Actuator::from_cell_mapping_file(Path::new(
                "config/cell/cell_actuator_mapping.yaml",
            )),
            _selected_actuators: Vec::new(),
            _displacement_steps: Vec::new(),
            _is_simulation_mode: is_simulation_mode,
        }
    }

    /// Start the movement.
    ///
    /// # Arguments
    /// * `actuators` - 0-based actuator IDs to move.
    /// * `displacement` - Displacement in millimeter or step to move.
    /// * `unit` - Unit of the displacement.
    ///
    /// # Returns
    /// Result of the movement.
    ///
    /// # Errors
    /// * The actuators are moving now.
    /// * No actuators are selected to move.
    /// * The unit of the displacement is not set.
    pub fn start(
        &mut self,
        actuators: &[usize],
        displacement: f64,
        unit: ActuatorDisplacementUnit,
    ) -> Result<(), &'static str> {
        if self.is_running {
            return Err("The actuators are moving now.");
        }

        if actuators.is_empty() {
            return Err("No actuators are selected to move.");
        }

        if unit == ActuatorDisplacementUnit::None {
            return Err("The unit of the displacement is not set.");
        }

        self.is_running = true;
        self._selected_actuators = actuators.to_vec();
        self._displacement_steps = self.calculate_steps(actuators, displacement, unit);

        Ok(())
    }

    /// Calculate the steps of displacement.
    ///
    /// # Arguments
    /// * `actuators` - 0-based actuator IDs to move.
    /// * `displacement` - Displacement in millimeter or step to move.
    /// * `unit` - Unit of the displacement.
    ///
    /// # Returns
    /// Steps of displacement.
    ///
    /// # Panics
    /// If the simulation mode is disabled.
    fn calculate_steps(
        &self,
        actuators: &[usize],
        displacement: f64,
        unit: ActuatorDisplacementUnit,
    ) -> Vec<i32> {
        if !self._is_simulation_mode {
            panic!("Not implemented yet.");
        }

        if unit == ActuatorDisplacementUnit::Millimeter {
            actuators
                .iter()
                .map(|actuator| self.actuators[*actuator].displacement_to_step(displacement))
                .collect()
        } else {
            vec![displacement as i32; actuators.len()]
        }
    }

    /// Stop the movement. The internal data will be reset.
    pub fn stop(&mut self) {
        self.is_running = false;

        self._selected_actuators.clear();
        self._displacement_steps.clear();
    }

    /// Pause the movement.
    pub fn pause(&mut self) {
        self.is_running = false;
    }

    /// Resume the movement.
    ///
    /// # Returns
    /// Result of the movement.
    ///
    /// # Errors
    /// * The movement is done.
    pub fn resume(&mut self) -> Result<(), &'static str> {
        let is_able_to_resume = (!self._displacement_steps.is_empty())
            && self._displacement_steps.iter().any(|step| *step != 0);
        if is_able_to_resume {
            self.is_running = true;
            return Ok(());
        } else {
            return Err("The movement is done.");
        }
    }

    /// Get the steps to move.
    ///
    /// # Notes
    /// If the requested displacement is done or the actuator force is out of
    /// limit, the value of self.is_running will change to False.
    ///
    /// # Arguments
    /// * `steps_axial` - Absolute axial steps (>=0) to move. The internal
    /// calculation will consider the direction of target displacement by
    /// itself.
    /// * `steps_tangent` - Absolute tangent steps (>=0) to move. The internal
    /// calculation will consider the direction of target displacement by
    /// itself.
    ///
    /// # Returns
    /// Steps to move for each actuator.
    ///
    /// # Errors
    /// * The actuators are not running now.
    /// * The axial steps should be >= 0.
    /// * The tangent steps should be >= 0.
    pub fn get_steps_to_move(
        &mut self,
        steps_axial: i32,
        steps_tangent: i32,
    ) -> Result<Vec<i32>, &'static str> {
        if !self.is_running {
            return Err("The actuators are not running now.");
        }

        if steps_axial < 0 {
            return Err("The axial steps should be >= 0.");
        }

        if steps_tangent < 0 {
            return Err("The tangent steps should be >= 0.");
        }

        // Deside the step to move
        let steps_to_move: Vec<i32> = self
            ._selected_actuators
            .iter()
            .zip(self._displacement_steps.iter())
            .map(|(idx, step)| {
                if *idx < NUM_AXIAL_ACTUATOR {
                    clip(*step, -steps_axial, steps_axial)
                } else {
                    clip(*step, -steps_tangent, steps_tangent)
                }
            })
            .collect();

        self._displacement_steps
            .iter_mut()
            .zip(steps_to_move.iter())
            .for_each(|(displacement_step, step)| {
                *displacement_step -= *step;
            });

        // Finish the running after the final movement
        if self._displacement_steps.iter().all(|step| *step == 0) {
            self.is_running = false;
        }

        // // Return the actuator steps to move
        let mut actuator_steps = vec![0; NUM_ACTUATOR];
        self._selected_actuators
            .iter()
            .enumerate()
            .for_each(|(idx, actuator)| {
                actuator_steps[*actuator] = steps_to_move[idx];
            });

        Ok(actuator_steps)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_open_loop(is_simulation_mode: bool) -> OpenLoop {
        OpenLoop::new(is_simulation_mode)
    }

    #[test]
    fn test_start_fail() {
        let mut open_loop = create_open_loop(true);

        assert_eq!(
            open_loop.start(&Vec::new(), 10.0, ActuatorDisplacementUnit::Millimeter),
            Err("No actuators are selected to move.")
        );

        assert_eq!(
            open_loop.start(&vec![0], 10.0, ActuatorDisplacementUnit::None),
            Err("The unit of the displacement is not set.")
        );

        open_loop.is_running = true;
        assert_eq!(
            open_loop.start(&vec![0], 10.0, ActuatorDisplacementUnit::Millimeter),
            Err("The actuators are moving now.")
        );
    }

    #[test]
    fn test_start_success() {
        let mut open_loop = create_open_loop(true);

        assert_eq!(
            open_loop.start(&vec![1], 2.0, ActuatorDisplacementUnit::Step),
            Ok(())
        );

        assert_eq!(open_loop.is_running, true);
        assert_eq!(open_loop._displacement_steps, vec![2]);
    }

    #[test]
    fn test_calculate_steps() {
        let open_loop = create_open_loop(true);

        let idx = 1;
        assert_eq!(
            open_loop.calculate_steps(
                &[idx],
                open_loop.actuators[idx].gain_step_to_mm,
                ActuatorDisplacementUnit::Millimeter
            ),
            vec![1]
        );

        assert_eq!(
            open_loop.calculate_steps(&[idx], 10.0, ActuatorDisplacementUnit::Step),
            vec![10]
        );
    }

    #[test]
    fn test_stop() {
        let mut open_loop = create_open_loop(true);
        open_loop.is_running = true;
        open_loop._selected_actuators = vec![10];
        open_loop._displacement_steps = vec![10];

        open_loop.stop();

        assert_eq!(open_loop.is_running, false);
        assert!(open_loop._selected_actuators.is_empty());
        assert!(open_loop._displacement_steps.is_empty());
    }

    #[test]
    fn test_pause() {
        let mut open_loop = create_open_loop(true);
        open_loop.is_running = true;

        open_loop.pause();

        assert_eq!(open_loop.is_running, false);
    }

    #[test]
    fn test_resume_fail() {
        let mut open_loop = create_open_loop(true);

        assert_eq!(open_loop.resume(), Err("The movement is done."));
    }

    #[test]
    fn test_resume_success() {
        let mut open_loop = create_open_loop(true);
        open_loop._displacement_steps = vec![10];

        assert_eq!(open_loop.resume(), Ok(()));
        assert_eq!(open_loop.is_running, true);
    }

    #[test]
    fn test_get_steps_to_move_fail() {
        let mut open_loop = create_open_loop(true);

        assert_eq!(
            open_loop.get_steps_to_move(1, 2),
            Err("The actuators are not running now.")
        );

        open_loop.is_running = true;
        assert_eq!(
            open_loop.get_steps_to_move(-1, 2),
            Err("The axial steps should be >= 0.")
        );
        assert_eq!(
            open_loop.get_steps_to_move(1, -2),
            Err("The tangent steps should be >= 0.")
        );
    }

    #[test]
    fn test_get_steps_to_move_done() {
        let mut open_loop = create_open_loop(true);
        let _ = open_loop.start(&vec![1, 74], -2.0, ActuatorDisplacementUnit::Step);

        let actuator_steps = open_loop.get_steps_to_move(3, 3).unwrap();

        assert_eq!(actuator_steps[1], -2);
        assert_eq!(actuator_steps[74], -2);
        assert_eq!(open_loop.is_running, false);
    }

    #[test]
    fn test_run_steps_not_done() {
        let mut open_loop = create_open_loop(true);
        let _ = open_loop.start(&vec![1, 73], 4.0, ActuatorDisplacementUnit::Step);

        let actuator_steps = open_loop.get_steps_to_move(2, 3).unwrap();

        assert_eq!(actuator_steps[1], 2);
        assert_eq!(actuator_steps[73], 3);

        assert_eq!(open_loop.is_running, true);
        assert_eq!(open_loop._displacement_steps[0], 2);
        assert_eq!(open_loop._displacement_steps[1], 1);
    }
}
