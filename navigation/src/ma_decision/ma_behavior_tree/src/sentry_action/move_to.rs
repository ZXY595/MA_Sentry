use std::{ops::Deref, sync::Arc, time::Duration};

use r2r::{
    GoalStatus, builtin_interfaces::msg::Time, geometry_msgs::msg::PoseStamped,
    std_msgs::msg::Header,
};

use super::*;

pub async fn move_to_pose(
    client: Arc<ActionClient<NavigateToPose::Action>>,
    pose: Pose,
    goal_handle_state: Arc<Mutex<Option<ActionClientGoal<NavigateToPose::Action>>>>,
) -> Result<(), anyhow::Error> {
    log::info!("move to {:?}", &pose);
    let goal = PoseStamped {
        header: Header {
            stamp: Time::default(),
            frame_id: "map".to_string(),
        },
        pose,
    };
    while let Err(e) = send_navigate_to_pose(client.deref(), goal, goal_handle_state).await {
        log::error!("Failed to send goal: {:?}, retrying...");
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
    Ok(())
}

/// ### process of action NavigateToPose
///
/// # Error
/// If the goal fails to be sent, rejected, or not successful, an error is returned.
pub async fn send_navigate_to_pose(
    client: &ActionClient<NavigateToPose::Action>,
    pose: PoseStamped,
    goal_handle_state: Arc<Mutex<Option<ActionClientGoal<NavigateToPose::Action>>>>,
) -> Result<(), anyhow::Error> {
    let send_goal_future = client.send_goal_request(NavigateToPose::Goal {
        pose,
        behavior_tree: String::new(),
    })?;
    let send_goal_with_time_out = tokio::time::timeout(Duration::from_secs(3), send_goal_future);

    let (goal_handle, result, _feedback) = send_goal_with_time_out.await??;
    log::info!(
        "Goal was accepted by the server, uuid: {}",
        goal_handle.uuid
    );

    {
        *goal_handle_state.lock().await = Some(goal_handle);
    }

    let (status, _result) = result.await?;

    {
        *goal_handle_state.lock().await = None;
    }

    log::info!("Goal status: {}", status);

    matches!(status, GoalStatus::Succeeded)
        .then_some(())
        .ok_or(anyhow::anyhow!("Goal failed"))
}

pub async fn cancel_moving(
    goal_handle_state: Arc<Mutex<Option<ActionClientGoal<NavigateToPose::Action>>>>,
) -> Result<(), anyhow::Error> {
    {
        goal_handle_state
            .lock()
            .await
            .as_ref()
            .ok_or(anyhow::anyhow!(
                "Attemping to cancel the goal but found no goal_handle."
            ))?
            .cancel()?
            .await?
    }
    Ok(())
}
