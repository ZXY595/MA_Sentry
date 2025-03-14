mod behavior;
mod sentry_action;

use std::{pin::pin, time::Duration};

use bonsai_bt::{BT, Event, Timer, UpdateArgs};
use r2r::{QosProfile, qos, rm_interfaces};
use sentry_action::SentryState;
use tokio::{task, time};
use futures_util::StreamExt;

fn main() -> Result<(), anyhow::Error> {
    env_logger::init();
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "behavior_tree", "/ma")?;

    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_time()
        .build()?;

    let behavior = behavior::get_behavior();

    let serial_stream = node.subscribe::<rm_interfaces::msg::SerialReceiveData>(
        "/serial/receive",
        QosProfile::default()
            .reliability(qos::ReliabilityPolicy::SystemDefault)
            .history(qos::HistoryPolicy::KeepLast)
            .durability(qos::DurabilityPolicy::SystemDefault),
    )?;

    let client = node.create_action_client("/navigate_to_pose")?;
    let (mut sentry_state, sync_state) = SentryState::new(client);

    let mut behavior_tree = BT::new(behavior, ());
    let graph_viz = behavior_tree.get_graphviz();
    log::info!("{graph_viz}");

    runtime.spawn(async move {
        let mut pin_serial = pin!(serial_stream);
        let mut cached_hp = UpdateDetecter(0);
        let mut cached_ammo = UpdateDetecter(0);
        let mut cached_game_started = UpdateDetecter(false);
        while let Some(data) = pin_serial.next().await {
            if cached_hp.update(data.judge_system_data.hp as i32) {
                *sync_state.hp.write().await = cached_hp.0;
            }
            if cached_ammo.update(data.judge_system_data.ammo as u32) {
                *sync_state.ammo.write().await = cached_ammo.0;
            }
            if cached_game_started.update(data.judge_system_data.game_status == 1) {
                *sync_state.is_game_started.write().await = cached_game_started.0;
            }
        }
    });

    runtime.block_on(async move {
        let mut timer = Timer::init_time();
        loop {
            let event = Event::from(UpdateArgs { dt: timer.get_dt() });
            behavior_tree.tick(&event, &mut |event, _| {
                let dt = event.dt;
                event.action.tick(&mut sentry_state, dt)
            });
            node.spin_once(Duration::from_millis(100));
            // time::sleep(Duration::from_millis(50)).await;
            task::yield_now().await;
        }
    });
    Ok(())
}

struct UpdateDetecter<T>(pub T);

impl<T> UpdateDetecter<T>
where
    T: PartialEq,
{
    pub fn update(&mut self, data: T) -> bool {
        if self.0 != data {
            self.0 = data;
            true
        } else {
            false
        }
    }
}
