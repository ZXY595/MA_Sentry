mod nav;
use anyhow::anyhow;
use async_broadcast::Receiver;
use futures_util::{Stream, StreamExt};
use ma_interfaces::{geometry_msgs::msg::Point, nav2_msgs::action::NavigateToPose};
use ros2_client::action::ActionClient;

pub struct SentryCtx {
    nav_clinet: ActionClient<NavigateToPose::Action>,
    status_rx: StatusReceivers,
}
struct StatusReceivers {
    game_status: Receiver<u8>,
    hp: Receiver<i16>,
}

#[repr(u8)]
enum GameStatus {
    Idle = 0,
    Prepare = 3,
    InGame = 4,
}

impl SentryCtx {
    pub fn new(
        nav_clinet: ActionClient<NavigateToPose::Action>,
        game_status_rx: Receiver<u8>,
        hp_rx: Receiver<i16>,
    ) -> Self {
        Self {
            nav_clinet,
            status_rx: StatusReceivers {
                game_status: game_status_rx,
                hp: hp_rx,
            },
        }
    }
}

pub async fn sentry_task(ctx: SentryCtx) -> Result<(), anyhow::Error> {
    wait_util(ctx.status_rx.game_status.new_receiver(), |game_status| {
        game_status == GameStatus::InGame as u8
    })
    .await?;
    log::info!("Game is started");
    loop {
        nav::nav_to_point(
            &ctx.nav_clinet,
            &ctx.status_rx,
            Point {
                x: 1.6,
                y: 0.0,
                z: 0.0,
            },
        )
        .await?;
        nav::nav_to_point(
            &ctx.nav_clinet,
            &ctx.status_rx,
            Point {
                x: 1.6,
                y: -2.0,
                z: 0.0,
            },
        )
        .await?;
    }
}

pub(crate) async fn wait_util<T>(
    stream: impl Stream<Item = T>,
    mut f: impl FnMut(T) -> bool,
) -> Result<(), anyhow::Error>
where
    T: Clone,
{
    stream
        .any(|game_status| std::future::ready(f(game_status)))
        .await
        .then_some(())
        .ok_or(anyhow!("Fail to receive game status"))
}
