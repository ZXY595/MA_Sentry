use super::*;
use ros2_interfaces_humble::std_msgs::msg::Header;

pub fn tick(
    goal_reached: bool,
    goal_sender: &mpsc::Sender<PoseStamped>,
    pose: Pose,
) -> Status {
    if goal_reached {
        return Status::Success;
    }

    let goal = PoseStamped {
        header: Header::default(),
        pose,
    };
    match goal_sender.blocking_send(goal) {
        Ok(_) => Status::Running,
        Err(e) => {
            println!("Error sending goal: {e}");
            Status::Failure
        }
    }
}
