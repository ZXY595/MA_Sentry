use std::{ops::Deref, sync::Arc};

use super::*;
use ros2_interfaces_humble::{builtin_interfaces::msg::Time, std_msgs::msg::Header};

pub async fn tick(
    client: Arc<ActionClient<NavigateToPose>>,
    pose: Pose,
) -> Result<(), anyhow::Error> {
    println!("move to {:?}", &pose);
    let goal = PoseStamped {
        header: Header {
            stamp: Time::default(),
            frame_id: "map".to_string(),
        },
        pose,
    };
    nav2_action::send_navigate_to_pose(client.deref(), goal).await?;
    Ok(())
}
