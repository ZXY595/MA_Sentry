use ma_interfaces::geometry_msgs::msg::Point;

use super::{nav, SentryCtx, Shutdown};

#[expect(unused)]
pub async fn attack_outpost(
    ctx: &SentryCtx,
    shutdown: Shutdown,
) -> Result<(), anyhow::Error> {
    nav::nav_to_point(
        ctx,
        Point {
            x: 3.0,
            y: 2.0,
            z: 0.0,
        },
        shutdown,
    )
    .await
}
