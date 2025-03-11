mod nav2_action;
mod sentry_action;

use bonsai_bt::{BT, Behavior, Event, Timer, UpdateArgs};
use ros2_client::NodeName;
use ros2_interfaces_humble::geometry_msgs::msg::Pose;
use sentry_action::{SentryAction, SentryState};
use tokio::task;

fn main() -> Result<(), anyhow::Error> {
    let mut node = ros2_client::Context::new()?.new_node(
        NodeName::new("/ma", "ma_behavior_tree")?,
        Default::default(),
    )?;

    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_time()
        .build()?;

    let sequence = Behavior::Sequence(vec![Behavior::Action(
        SentryAction::MoveTo(Pose::default()),
    )]);

    let mut behavior_tree = BT::new(sequence, ());

    runtime.spawn(node.spinner()?.spin());

    runtime.block_on(async move {
        let mut sentry_state = SentryState::new(&mut node);
        let mut timer = Timer::init_time();
        loop {
            let event = Event::from(UpdateArgs { dt: timer.get_dt() });
            behavior_tree.tick(&event, &mut |event, _| {
                let dt = event.dt;
                (event.action.tick(&mut sentry_state, dt), dt)
            });
            task::yield_now().await;
        }
    });
    Ok(())
}
