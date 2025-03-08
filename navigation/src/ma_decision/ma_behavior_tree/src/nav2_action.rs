use ros2_client::{ActionTypes, Message};
use ros2_interfaces_humble::{builtin_interfaces::msg::Duration, geometry_msgs::msg::PoseStamped};
use serde::{Deserialize, Serialize};

pub struct NavigateToPose;

#[derive(Clone, Serialize, Deserialize)]
pub struct GoalType {
    pub pose: PoseStamped,
    pub behavior_tree: String,
}

impl Message for GoalType {}


#[derive(Clone, Serialize, Deserialize)]
pub struct ResultType {
// uint16 NONE=0
// uint16 UNKNOWN=9000
// uint16 FAILED_TO_LOAD_BEHAVIOR_TREE=9001
// uint16 TF_ERROR=9002

// uint16 error_code
// string error_msg
    // NONE: u16,
    // UNKNOWN: u16,
    // FAILED_TO_LOAD_BEHAVIOR_TREE: u16,
    // TF_ERROR: u16,
    pub error_code: u16,
    pub error_msg: String,
}

impl Message for ResultType {}

impl ResultType {
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 9000;
    pub const FAILED_TO_LOAD_BEHAVIOR_TREE: u16 = 9001;
    pub const TF_ERROR: u16 = 9002;
}

#[derive(Serialize, Deserialize)]
pub struct FeedbackType {
    pub current_pose: PoseStamped,
    pub navigation_time: Duration,
    pub estimated_time_remaining: Duration,
    pub number_of_recoveries: i16,
    pub distance_remaining: f32
}

impl Message for FeedbackType {}


impl ActionTypes for NavigateToPose {
    type GoalType = GoalType;

    type ResultType = ResultType;

    type FeedbackType = FeedbackType;

    fn goal_type_name(&self) -> &str {
        todo!()
    }

    fn result_type_name(&self) -> &str {
        todo!()
    }

    fn feedback_type_name(&self) -> &str {
        todo!()
    }
}
