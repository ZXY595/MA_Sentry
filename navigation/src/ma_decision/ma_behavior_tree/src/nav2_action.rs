pub use navigate_to_pose::NavigateToPose;
use ros2_client::{
    Message,
    action::{self, ActionClient, GoalStatusEnum},
};
use ros2_interfaces_humble::{
    builtin_interfaces::msg::Duration, geometry_msgs::msg::PoseStamped, std_msgs,
};
use serde::{Deserialize, Serialize};

pub mod navigate_to_pose {
    use super::*;

    pub type NavigateToPose = action::Action<Goal, Result, FeedBack>;
    #[derive(Clone, Serialize, Deserialize)]
    pub struct Goal {
        pub pose: PoseStamped,
        pub behavior_tree: String,
    }

    impl Message for Goal {}

    #[derive(Clone, Serialize, Deserialize)]
    pub struct Result {
        pub result: std_msgs::msg::Empty,
    }

    impl Message for Result {}

    #[derive(Serialize, Deserialize)]
    pub struct FeedBack {
        pub current_pose: PoseStamped,
        pub navigation_time: Duration,
        pub estimated_time_remaining: Duration,
        pub number_of_recoveries: i16,
        pub distance_remaining: f32,
    }

    impl Message for FeedBack {}
}

/// ### process of action NavigateToPose
///
/// # Error
/// If the goal fails to be sent, rejected, or not successful, an error is returned.
pub async fn send_navigate_to_pose(
    client: &ActionClient<NavigateToPose>,
    pose: PoseStamped,
) -> Result<(), anyhow::Error> {
    let uuid = client
        .async_send_goal(navigate_to_pose::Goal {
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
