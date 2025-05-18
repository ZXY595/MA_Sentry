mod alert;
mod attack;
mod nav;
mod retreat;

pub(crate) mod sentry_state;
use std::{sync::Arc, time::Duration};

use async_shutdown::ShutdownManager;
use futures_util::FutureExt;
use ma_interfaces::{
    msg::{BTState, SerialReceiveData},
    nav2_msgs::action::NavigateToPose,
};
use ros2_client::{Publisher, Subscription, action::ActionClient};
use sentry_state::{GameStatus, SentryStates, State};
use smol::{Timer, future::FutureExt as SmolFutureExt};

use crate::dedup::Dedup;

type Shutdown = ShutdownManager<isize>;

pub struct SentryCtx {
    pub(crate) nav_clinet: ActionClient<NavigateToPose::Action>,
    pub(crate) states: Arc<SentryStates>,
    pub(crate) state_publisher: Publisher<BTState>,
}

impl SentryCtx {
    pub fn new(
        nav_clinet: ActionClient<NavigateToPose::Action>,
        serial_subscription: Subscription<SerialReceiveData>,
        state_publisher: Publisher<BTState>,
    ) -> Result<Self, anyhow::Error> {
        let states = SentryStates::default();
        let states = Arc::new(states);

        let sender_states = Arc::clone(&states);

        smol::spawn(async move {
            use futures_util::{StreamExt, TryStreamExt};
            use ma_interfaces::msg::{JudgeSystemData, SerialReceiveData};
            let serial_stream = serial_subscription
                .async_stream()
                .map_ok(|(msg, _)| {
                    let SerialReceiveData {
                        judge_system_data:
                            JudgeSystemData {
                                game_status,
                                hp,
                                outpost_hp,
                                ..
                            },
                        ..
                    } = msg;
                    (game_status, hp, outpost_hp)
                })
                .inspect_err(|e| log::error!("Error: {e}"))
                .filter_map(|x| async { x.ok() });

            Dedup::new_default(serial_stream)
                .for_each(async |status| {
                    sender_states.store(status);
                    // log::trace!("broadcast: game_status: {game_status:?}, hp: {hp:?}, outpost_hp: {outpost_hp:?}")
                })
                .await;
        })
        .detach();

        Ok(Self {
            nav_clinet,
            states,
            state_publisher,
        })
    }
    pub async fn sentry_task(&mut self) -> Result<(), anyhow::Error> {
        let status = &self.states;

        status
            .game_status
            .wait_until(|game_status| game_status == GameStatus::InGame)
            .inspect(|_| log::info!("Game is started"))
            .await;

        let shutdown = async_shutdown::ShutdownManager::new();

        // smol::spawn(alert::alert(Arc::clone(status), terminate_signal.clone())).detach();

        nav::patrol(self, shutdown.clone())
            .or(async {
                status
                    .game_status
                    .wait_until(|game_status| game_status != GameStatus::InGame)
                    .inspect(|_| log::info!("Game is ended"))
                    .or(status
                        .hp
                        .wait_until(|hp| hp == 0)
                        .inspect(|_| log::info!("HP is 0")))
                    .await;
                let _ = shutdown.trigger_shutdown(0);
                Ok(())
            })
            .or(async {
                Timer::after(Duration::from_secs(30)).await;
                status.hp.wait_until(|hp| hp < 50).await;
                let _ = shutdown.trigger_shutdown(1);
                retreat::retreat(self, shutdown.clone()).await?;
                Ok(())
            })
            .or(async {
                let mut last_hp = status.hp.load();
                status
                    .hp
                    .wait_until(|hp| {
                        let should_alert = hp < last_hp;
                        last_hp = hp;
                        should_alert
                    })
                    .await;
                let _ = shutdown.trigger_shutdown(1);
                alert::alert(status, &self.state_publisher, shutdown.clone()).await
            })
            // .or(async {
            //     status.outpost_hp.wait_until(|outpost_hp| outpost_hp > 0).await;
            //     attack::attack_outpost(&self, terminate_signal.clone()).await
            // })
            .await
    }
}
