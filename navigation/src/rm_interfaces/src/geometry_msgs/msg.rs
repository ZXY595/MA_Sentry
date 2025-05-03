use serde::{Deserialize, Serialize};

use crate::std_msgs::msg::Header;

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
