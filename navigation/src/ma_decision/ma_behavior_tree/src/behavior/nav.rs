use std::time::Duration;

use anyhow::anyhow;
use futures_util::TryFutureExt;
use ma_interfaces::geometry_msgs::msg::Point;
use smol::{Timer, future::FutureExt as SmolFutureExt};
use thiserror::Error;

use crate::sentry_action::nav_to_pose::{
    self as nav_action, CancelGoalError, RequestResultError, SendGoalError,
};

use super::{SentryCtx, Shutdown, ShutdownReason};

pub async fn patrol(ctx: &SentryCtx, shutdown: Shutdown) -> Result<(), anyhow::Error> {
    loop {
        nav_to_point(
            ctx,
            Point {
                x: 1.6,
                y: 0.0,
                z: 0.0,
            },
            shutdown.clone(),
        )
        .await?;

        Timer::after(Duration::from_secs(1)).await;

        nav_to_point(
            ctx,
            Point {
                x: 1.6,
                y: -2.0,
                z: 0.0,
            },
            shutdown.clone(),
        )
        .await?;

        Timer::after(Duration::from_secs(1)).await;
    }
}

/// when shutdown signal is triggered, cancel the goal and return an error
pub async fn nav_to_point(
    ctx: &SentryCtx,
    point: Point,
    shutdown: Shutdown,
) -> Result<(), anyhow::Error> {
    log::info!("Start nav to point: {point:?}");
    let client = &ctx.nav_clinet;
    let (goal_id, stamp) = loop {
        match nav_action::start_nav(client, point.clone()).await {
            Ok(result) => break result,
            Err(e) => log::warn!("Fail to send goal: {e}"),
        }
    };
    async {
        if let Err(e) = nav_action::request_result(client, goal_id).await {
            log::error!("Fail to request result: {e}")
        }
        Ok(())
    }
    .or(async {
        let reason = shutdown.wait_shutdown_triggered().await;
        nav_action::cancel_goal(client, goal_id, stamp).await?;
        Err(anyhow!("Nav canceled with reason: {reason}"))
    })
    .await
}


