#[expect(non_snake_case)]
pub mod NavigateToPose {
    pub type Action = ros2_client::Action<Goal, Result, FeedBack>;
    pub use super::navigate_to_pose_impl::{FeedBack, Goal, Result};
}

mod navigate_to_pose_impl {
    use ros2_client::{Message, builtin_interfaces::Duration};
    use serde::{Deserialize, Serialize};

    use crate::{geometry_msgs::msg::PoseStamped, std_msgs};

    #[derive(Clone, Serialize, Deserialize)]
    pub struct Goal {
        pub pose: PoseStamped,
        pub behavior_tree: String,
    }

    impl Message for Goal {}

    #[derive(Clone, Serialize, Deserialize)]
    pub struct Result {
        pub result: std_msgs::msg::Empty,
    }

    impl Message for Result {}

    #[derive(Serialize, Deserialize)]
    pub struct FeedBack {
        pub current_pose: PoseStamped,
        pub navigation_time: Duration,
        pub estimated_time_remaining: Duration,
        pub number_of_recoveries: i16,
        pub distance_remaining: f32,
    }

    impl Message for FeedBack {}
}
