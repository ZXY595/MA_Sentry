use ros2_client::builtin_interfaces::Time;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

impl Default for Header {
    fn default() -> Self {
        Self {
            stamp: Time::ZERO,
            frame_id: Default::default(),
        }
    }
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Empty {}
