use std::sync::Arc;

use bonsai_bt::Status;
use ros2_client::{
    ActionTypeName, Name,
    action::{ActionClient, ActionClientQosPolicies},
    ros2::QosPolicies,
};
use ros2_interfaces_humble::geometry_msgs::msg::{Pose, PoseStamped};
use tokio::{sync::oneshot::{self, error::TryRecvError}, task};

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
                let receiver = state.move_status.get_or_insert_with(|| {
                    let (task_status_sender, task_status_receiver) = oneshot::channel();
                    wrap_async_task(
                        task_status_sender,
                        move_to::tick(Arc::clone(&state.nav_client), pose.clone()),
                    );
                    task_status_receiver
                });
                match receiver.try_recv() {
                    Err(TryRecvError::Empty) => Status::Running,
                    Ok(TaskStatus::Finished) => {
                        state.move_status = None;
                        Status::Success
                    }
                    Err(TryRecvError::Closed) | Ok(TaskStatus::Error) => {
                        state.move_status = None;
                        Status::Failure
                    }
                }
            }
            SentryAction::Idle => Status::Success,
        }
    }
}

pub struct SentryState {
    hp: usize,
    ammo: usize,
    nav_client: Arc<ActionClient<NavigateToPose>>,
    move_status: Option<oneshot::Receiver<TaskStatus>>,
}

impl SentryState {
    pub fn new(node: &mut ros2_client::Node) -> Self {
        let client = create_navigate_to_pose_client(node).unwrap();

        Self {
            hp: 0,
            ammo: 0,
            nav_client: Arc::new(client),
            move_status: None,
        }
    }
}

fn wrap_async_task(
    task_status_sender: oneshot::Sender<TaskStatus>,
    task: impl Future<Output = Result<(), anyhow::Error>> + Send + 'static,
) {
    task::spawn(async move {
        if let Err(err) = task.await {
            println!("task failed: {err:?}");
            task_status_sender.send(TaskStatus::Error).unwrap() // receiver should not be dropped now;
        } else {
            task_status_sender.send(TaskStatus::Finished).unwrap() // same as above
        }
    });
}

#[derive(Debug)]
enum TaskStatus {
    Finished,
    Error,
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
