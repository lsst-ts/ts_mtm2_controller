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

pub struct EventQueue {
    // Events to publish
    _events: Vec<Value>,
}

impl EventQueue {
    /// Create a new instance of the event queue.
    ///
    /// # Returns
    /// New instance of the event queue.
    pub fn new() -> Self {
        Self {
            _events: Vec::new(),
        }
    }

    /// Check if there are any events to publish.
    ///
    /// # Returns
    /// True if there are events to publish, false otherwise.
    pub fn has_event(&self) -> bool {
        !self._events.is_empty()
    }

    /// Add an event.
    pub fn add_event(&mut self, event: Value) {
        self._events.push(event);
    }

    /// Get the events to publish and clear the internal events.
    ///
    /// # Returns
    /// Events.
    pub fn get_events_and_clear(&mut self) -> Vec<Value> {
        let events = self._events.clone();
        self._events.clear();

        events
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use serde_json::json;

    #[test]
    fn test_has_event() {
        let mut event_queue = EventQueue::new();

        assert!(!event_queue.has_event());

        let event = json!({
            "id": "test",
        });
        event_queue.add_event(event);

        assert!(event_queue.has_event());
    }

    #[test]
    fn test_add_event() {
        let mut event_queue = EventQueue::new();

        let event = json!({
            "id": "test",
        });
        event_queue.add_event(event);

        assert_eq!(event_queue._events.len(), 1);
    }

    #[test]
    fn test_get_events_and_clear() {
        let mut event_queue = EventQueue::new();

        let event = json!({
            "id": "test",
        });
        event_queue.add_event(event);

        let events = event_queue.get_events_and_clear();

        assert_eq!(events.len(), 1);
        assert_eq!(event_queue._events.len(), 0);
    }
}
