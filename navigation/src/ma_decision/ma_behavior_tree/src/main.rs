mod sentry_action;

use std::time::Duration;

use bonsai_bt::{BT, Behavior, Event, Timer, UpdateArgs};
use r2r::geometry_msgs::msg::{Point, Pose};
use sentry_action::{SentryAction, SentryState};
use tokio::task;

fn main() -> Result<(), anyhow::Error> {
    // env_logger::init();
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "ma_behavior_tree", "/")?;

    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_time()
        .build()?;

    let sequence = Behavior::Sequence(vec![Behavior::Action(SentryAction::MoveTo(Pose {
        position: Point {
            x: 4.0,
            y: 0.0,
            z: 0.0,
        },
        ..Default::default()
    }))]);

    let client = node.create_action_client("/navigate_to_pose")?;
    let mut sentry_state = SentryState::new(client);

    let mut behavior_tree = BT::new(sequence, ());

    runtime.block_on(async move {
        let mut timer = Timer::init_time();
        loop {
            let event = Event::from(UpdateArgs { dt: timer.get_dt() });
            behavior_tree.tick(&event, &mut |event, _| {
                let dt = event.dt;
                (event.action.tick(&mut sentry_state, dt), dt)
            });
            node.spin_once(Duration::from_millis(50));
            task::yield_now().await;
        }
    });
    Ok(())
}
