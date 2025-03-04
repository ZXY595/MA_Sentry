use ros2_client::{MessageTypeName, NodeName, ros2::QosPolicies};
use ros2_interfaces_humble::geometry_msgs::msg::{Twist, Vector3};

fn main() -> Result<(), anyhow::Error> {
    let ctx = ros2_client::Context::new()?;
    let mut node = ctx.new_node(NodeName::new("/ma", "bringup")?, Default::default())?;
    let chassis_topic = ctx.create_topic(
        "cmd_vel_chassis".into(),
        MessageTypeName::new("geometry_msgs", "Twist"),
        &QosPolicies::default(),
    )?;
    let chassis_publisher = node.create_publisher::<Twist>(&chassis_topic, None)?;
    chassis_publisher.publish(Twist {
        linear: Vector3::default(),
        angular: Vector3::default(),
    })?;
    Ok(())
}
