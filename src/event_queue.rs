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
