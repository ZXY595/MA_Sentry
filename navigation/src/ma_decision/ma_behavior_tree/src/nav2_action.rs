use ros2_client::{
    ActionTypes, Message,
    action::{ActionClient, GoalStatusEnum},
};
use ros2_interfaces_humble::{builtin_interfaces::msg::Duration, geometry_msgs::msg::PoseStamped};
use serde::{Deserialize, Serialize};

pub struct NavigateToPose;

#[derive(Clone, Serialize, Deserialize)]
pub struct NavigateToPoseGoal {
    pub pose: PoseStamped,
    pub behavior_tree: String,
}

impl Message for NavigateToPoseGoal {}

#[derive(Clone, Serialize, Deserialize)]
pub struct NavigateToPoseResult {
    // uint16 NONE=0
    // uint16 UNKNOWN=9000
    // uint16 FAILED_TO_LOAD_BEHAVIOR_TREE=9001
    // uint16 TF_ERROR=9002

    // uint16 error_code
    // string error_msg
    pub error_code: u16,
    pub error_msg: String,
}

impl Message for NavigateToPoseResult {}

#[allow(unused)]
impl NavigateToPoseResult {
    pub const NONE: u16 = 0;
    pub const UNKNOWN: u16 = 9000;
    pub const FAILED_TO_LOAD_BEHAVIOR_TREE: u16 = 9001;
    pub const TF_ERROR: u16 = 9002;
}

#[derive(Serialize, Deserialize)]
pub struct NavigateToPoseFeedback {
    pub current_pose: PoseStamped,
    pub navigation_time: Duration,
    pub estimated_time_remaining: Duration,
    pub number_of_recoveries: i16,
    pub distance_remaining: f32,
}

impl Message for NavigateToPoseFeedback {}

impl ActionTypes for NavigateToPose {
    type GoalType = NavigateToPoseGoal;

    type ResultType = NavigateToPoseResult;

    type FeedbackType = NavigateToPoseFeedback;

    fn goal_type_name(&self) -> &str {
        "/navigate_to_pose/goal"
    }

    fn result_type_name(&self) -> &str {
        "/navigate_to_pose/result"
    }

    fn feedback_type_name(&self) -> &str {
        "/navigate_to_pose/feedback"
    }
}

/// ### process of action NavigateToPose
///
/// # Error
/// If the goal fails to be sent, rejected, or not successful, an error is returned.
pub async fn process_navigate_to_pose(
    client: &ActionClient<NavigateToPose>,
    pose: PoseStamped,
) -> Result<(), anyhow::Error> {
    let uuid = client
        .async_send_goal(NavigateToPoseGoal {
            pose,
            behavior_tree: String::new(),
        })
        .await
        .map_err(|e| anyhow::anyhow!("{e:?}"))
        .and_then(|(id, response)| {
            if response.accepted {
                Ok(id)
            } else {
                Err(anyhow::anyhow!("Goal was rejected by server"))
            }
        })?;
    let (status, _) = client
        .async_request_result(uuid)
        .await
        .map_err(|e| anyhow::anyhow!("{e:?}"))?;
    matches!(status, GoalStatusEnum::Succeeded)
        .then_some(())
        .ok_or(anyhow::anyhow!("Goal failed"))
}
