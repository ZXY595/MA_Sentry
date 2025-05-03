use anyhow::anyhow;
use futures_util::TryFutureExt;
use ma_interfaces::geometry_msgs::msg::Point;
use smol::future::FutureExt;

use crate::sentry_action::nav_to_pose as nav_action;

use super::{GameStatus, StatusReceivers, wait_util};

pub async fn nav_to_point(
    clinet: &nav_action::Client,
    status_rx: &StatusReceivers,
    point: Point,
) -> Result<(), anyhow::Error> {
    let (goal_id, stamp) = match nav_action::start_nav(clinet, point.clone()).await {
        Err(e @ nav_action::SendGoalError::Timeout(_)) => {
            log::error!("Goal response timeout: {e}");
            return Ok(());
        }
        other => other?,
    };
    nav_action::request_result(clinet, goal_id)
        .map_err(|e| anyhow!(e))
        .or(async {
            wait_util(status_rx.game_status.new_receiver(), |status| {
                status != GameStatus::InGame as u8
            })
            .await?;
            let _ = nav_action::cancel_goal(clinet, goal_id, stamp).await?;
            Err(anyhow!("Game is end, stop the task."))
        })
        .or(async {
            wait_util(status_rx.hp.new_receiver(), |hp| hp <= 0).await?;
            let _ = nav_action::cancel_goal(clinet, goal_id, stamp).await?;
            Ok(())
        })
        .await
}
