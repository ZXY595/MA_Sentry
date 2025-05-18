mod behavior;
mod dedup;
mod sentry_action;
#[cfg(test)]
mod tests;

use ma_dds::prelude::Ros2NodeExt;
use ma_interfaces::nav2_msgs::action::NavigateToPose;
use ros2_client::action::ActionClientQosPolicies;

fn main() -> Result<(), anyhow::Error> {
    env_logger::init();
    let mut node = ma_dds::Ros2NodeBuilder::new()
        .name("/ma", "behavior_tree")
        .build()?;

    let nav_client = node
        .action_client_builder::<NavigateToPose::Action>()
        .action_base_name("navigate_to_pose")
        .type_name("nav2", "NavigateToPose")
        .qos(ActionClientQosPolicies {
            goal_service: Default::default(),
            result_service: Default::default(),
            cancel_service: Default::default(),
            feedback_subscription: Default::default(),
            status_subscription: Default::default(),
        })
        .build()?;

    let serial_subscription = {
        use ma_dds::prelude::Ros2NodeExt;
        node.pub_sub_builder::<ma_interfaces::msg::SerialReceiveData>()
            .topic_name("/serial", "receive")
            .build_subscription()?
    };

    let bt_state_publisher = {
        use ma_dds::prelude::Ros2NodeExt;
        node.pub_sub_builder::<ma_interfaces::msg::BTState>()
            .topic_base_name("cmd_bt_state")
            .build_publisher()?
    };

    smol::spawn(node.spinner()?.spin()).detach();

    smol::block_on(async {
        let mut ctx =
            behavior::SentryCtx::new(nav_client,  serial_subscription, bt_state_publisher)?;
        while let Err(e) = ctx.sentry_task().await {
            log::warn!("sentry task failed: {e}, restart the bt");
        }
        Ok(())
    })
}
