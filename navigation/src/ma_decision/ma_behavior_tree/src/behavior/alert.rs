use std::time::Duration;

use futures_util::FutureExt;
use ma_interfaces::msg::BTState;
use ros2_client::Publisher;
use smol::{Timer, future::FutureExt as SmolFutureExt};

use super::{
    Shutdown,
    sentry_state::{SentryStates, State},
};

pub async fn alert(
    status: &SentryStates,
    state_publisher: &Publisher<BTState>,
    shutdown: Shutdown,
) -> Result<(), anyhow::Error> {
    status.spin.store(true);
    state_publisher
        .async_publish(BTState { spin: true })
        .await?;
    log::info!("start spin");

    Timer::after(Duration::from_secs(3))
        .map(|_| ())
        .or(async {
            shutdown.wait_shutdown_triggered().await;
        })
        .await;

    status.spin.store(false);
    state_publisher
        .async_publish(BTState { spin: false })
        .await?;
    log::info!("cancel spin");

    Ok(())
    // if let AlertState::Cancel = result {
    //     break;
    // }
}
