mod nav2_action;

use bonsai_bt::{
    BT,
    Behavior::{Action, Sequence},
};
use nav2_action::NavigateToPose;
use ros2_client::action::ActionClientQosPolicies;
use ros2_client::ros2::QosPolicies;
use ros2_client::{ActionTypeName, Name, NodeName};
use ros2_interfaces_humble::geometry_msgs::msg::PoseStamped;

fn main() -> Result<(), anyhow::Error> {
    let mut node = ros2_client::Context::new()?.new_node(
        NodeName::new("/ma", "ma_behavior_tree")?,
        Default::default(),
    )?;
    // let topic = node.create_topic(&Name::new("/", "t"), type_name, qos)?;
    let client = node.create_action_client::<NavigateToPose>(
        ros2_client::ServiceMapping::Basic,
        &Name::new("/", "navigate_to_pose")?,
        &ActionTypeName::new("nav2_msgs", "NavigateToPose"),
        ActionClientQosPolicies {
            goal_service: QosPolicies::default(),
            result_service: QosPolicies::default(),
            cancel_service: QosPolicies::default(),
            feedback_subscription: QosPolicies::default(),
            status_subscription: QosPolicies::default(),
        },
    )?;

    let seq = Sequence(vec![Action(())]);

    let bt = BT::new(seq, ());

    let runtime = tokio::runtime::Builder::new_current_thread().build()?;

    runtime.spawn(node.spinner()?.spin());

    runtime.block_on(async {
        loop {
            let (_, response) = match client
                .async_send_goal(nav2_action::GoalType {
                    pose: PoseStamped::default(),
                    behavior_tree: String::new(),
                })
                .await
            {
                Ok(response) => response,
                Err(e) => {
                    println!("Failed to send goal: {e:?}");
                    continue;
                }
            };
            if !response.accepted {
                continue;
            }
        }
    })
}
