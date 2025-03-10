use anyhow::Context;
use bonsai_bt::Status;
use ros2_client::{
    ActionTypeName, Name,
    action::{ActionClient, ActionClientQosPolicies},
    ros2::QosPolicies,
};
use ros2_interfaces_humble::geometry_msgs::msg::{Pose, PoseStamped};
use tokio::{runtime::Runtime, sync::mpsc};

use crate::nav2_action::{self, NavigateToPose};

mod move_to;

#[derive(Debug, Clone)]
pub enum SentryAction {
    MoveTo(Pose),
    Idle,
}

impl SentryAction {
    pub fn tick(&self, state: &mut SentryState, _dt: f64) -> Status {
        match self {
            SentryAction::MoveTo(pose) => {
                move_to::tick(state.goal_reached, &state.goal_sender, pose.clone())
            }
            SentryAction::Idle => Status::Success,
        }
    }
}

#[derive(Debug)]
pub struct SentryState {
    hp: usize,
    ammo: usize,
    goal_reached: bool,
    goal_sender: mpsc::Sender<PoseStamped>,
}

impl SentryState {
    pub fn new(node: &mut ros2_client::Node, runtime: &Runtime) -> Self {
        let (goal_sender, goal_receiver) = mpsc::channel::<PoseStamped>(3);
        runtime.spawn(navigate_to_pose_server_task(
            create_navigate_to_pose_client(node)
                .context("Failed to create NavigateToPose client")
                .unwrap(),
            goal_receiver,
        ));

        Self {
            hp: 0,
            ammo: 0,
            goal_reached: false,
            goal_sender,
        }
    }
}

fn create_navigate_to_pose_client(
    node: &mut ros2_client::Node,
) -> Result<ActionClient<NavigateToPose>, anyhow::Error> {
    Ok(node.create_action_client::<NavigateToPose>(
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
    )?)
}

async fn navigate_to_pose_server_task(
    client: ActionClient<NavigateToPose>,
    mut goal_receiver: mpsc::Receiver<PoseStamped>,
) {
    loop {
        if let Err(err) = (async || {
            let goal = goal_receiver
                .recv()
                .await
                .ok_or(anyhow::anyhow!("goal_receiver.recv() failed"))?;
            nav2_action::process_navigate_to_pose(&client, goal).await?;
            Result::<(), anyhow::Error>::Ok(())
        })()
        .await
        {
            println!("navigate_to_pose_server_task failed: {err:?}")
        }
    }
}
