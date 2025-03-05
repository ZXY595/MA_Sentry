use ros2_client::{
    MessageTypeName, Name, NodeName,
    ros2::{self, QosPolicies, policy},
};
use ros2_interfaces_humble::geometry_msgs::msg::{Twist, Vector3};

fn main() -> Result<(), anyhow::Error> {
    let ctx = ros2_client::Context::new()?;
    let mut node = ctx.new_node(NodeName::new("/ma", "bringup")?, Default::default())?;
    let chassis_topic = node.create_topic(
        &Name::new("/", "cmd_vel_chassis")?,
        MessageTypeName::new("geometry_msgs", "Twist"),
        &QosPolicies::builder()
            .history(policy::History::KeepLast { depth: 10 })
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: ros2::Duration::from_millis(100),
            })
            .durability(policy::Durability::TransientLocal)
            .build(),
    )?;
    let chassis_publisher = node.create_publisher::<Twist>(&chassis_topic, None)?;

    smol::spawn(node.spinner()?.spin()).detach();

    smol::block_on::<Result<(), anyhow::Error>>(async {
        chassis_publisher.wait_for_subscription(&node).await;
        chassis_publisher
            .async_publish(Twist {
                linear: Vector3::default(),
                angular: Vector3::default(),
            })
            .await?;
        Ok(())
    })
}
