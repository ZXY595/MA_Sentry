use ma_interfaces::geometry_msgs::msg::Point;

use super::{
    nav, sentry_state::{HpState, State}, SentryCtx, Shutdown
};

pub async fn retreat(
    ctx: &SentryCtx,
    shutdown: Shutdown,
) -> Result<(), anyhow::Error> {
    nav::nav_to_point(
        ctx,
        Point {
            x: 0.0,
            y: -2.0,
            z: 0.0,
        },
        shutdown,
    )
    .await?;
    ctx.states.hp.wait_until(|hp| hp >= HpState::FULL).await;
    Ok(())
}
