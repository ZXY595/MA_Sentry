use std::{sync::Arc, task::Poll};

use bonsai_bt::Status;
use r2r::{
    ActionClient, ActionClientGoal,
    geometry_msgs::msg::{Point, Pose},
    nav2_msgs::action::NavigateToPose,
};
use tokio::{
    sync::{
        Mutex, RwLock,
        oneshot::{self, error::TryRecvError},
    },
    task,
};

mod move_to;

const MOVING_SPREAD: f64 = 0.01;

#[derive(Debug, Clone)]
pub enum SentryAction {
    MoveTo(Pose),
    RandomMoveTo(Pose),
    CancelMoving,
    // SpinArmor,
    UntilGameStarted,
    UntilGameOver,
    UntilIsAlive,
    UntilIsHpLessThan(i32),
    UntilIsHpGreaterThan(i32),
    IsHpGreaterThan(i32),
    // IsAmmoGreaterThan(u32),
}

impl SentryAction {
    pub fn tick(&self, state: &mut SentryState, dt: f64) -> (Status, f64) {
        let status = match self {
            SentryAction::MoveTo(pose) => wrap_async_task(
                &mut state.moving_status,
                move_to::move_to_pose(
                    Arc::clone(&state.nav_client),
                    Arc::clone(&state.moving_goal_handle),
                    pose.clone(),
                ),
            ),
            SentryAction::CancelMoving => wrap_async_task(
                &mut state.cancelling_status,
                move_to::cancel_moving(Arc::clone(&state.moving_goal_handle)),
            ),
            SentryAction::UntilGameStarted => {
                if let Poll::Ready(true) = state.is_game_started() {
                    log::info!("Game is started");
                    Status::Success
                } else {
                    Status::Running
                }
            }
            SentryAction::UntilGameOver => {
                if let Poll::Ready(false) = state.is_game_started() {
                    log::info!("Game is over");
                    Status::Success
                } else {
                    Status::Running
                }
            }
            SentryAction::UntilIsAlive => {
                if let Poll::Ready(true) = state.get_hp().map(|hp| hp > 0) {
                    log::info!("Sentry is alive");
                    Status::Success
                } else {
                    Status::Running
                }
            }
            SentryAction::UntilIsHpLessThan(max) => {
                if let Poll::Ready(true) = state.get_hp().map(|hp| hp < *max) {
                    log::info!("Sentry is low hp");
                    Status::Success
                } else {
                    Status::Running
                }
            }
            SentryAction::UntilIsHpGreaterThan(min) => {
                if let Poll::Ready(true) = state.get_hp().map(|hp| hp >= *min) {
                    log::info!("Sentry is high hp");
                    Status::Success
                } else {
                    Status::Running
                }
            }
            SentryAction::IsHpGreaterThan(min) => state
                .get_hp()
                .map(|hp| hp >= *min)
                .into_status(|| log::info!("Sentry hp is greater than {}", min)),
            SentryAction::RandomMoveTo(pose) => wrap_async_task(
                &mut state.moving_status,
                move_to::move_to_pose(
                    Arc::clone(&state.nav_client),
                    Arc::clone(&state.moving_goal_handle),
                    get_rand_point(pose),
                ),
            ),
        };
        (status, dt)
    }
}

fn get_rand_point(pose: &Pose) -> Pose {
    Pose {
        position: Point {
            x: pose.position.x + rand::random_range(-MOVING_SPREAD..MOVING_SPREAD),
            y: pose.position.y + rand::random_range(-MOVING_SPREAD..MOVING_SPREAD),
            z: pose.position.z,
        },
        orientation: pose.orientation.clone(),
    }
}

pub struct SentryState {
    sync_state: SyncState,
    nav_client: Arc<ActionClient<NavigateToPose::Action>>,
    moving_status: Option<oneshot::Receiver<TaskStatus>>,
    // for cancelling the goal
    moving_goal_handle: Arc<Mutex<Option<ActionClientGoal<NavigateToPose::Action>>>>,
    cancelling_status: Option<oneshot::Receiver<TaskStatus>>,
}

/// A struct that is shared across non behavior tree tasks
#[derive(Clone)]
pub struct SyncState {
    pub is_game_started: Arc<RwLock<bool>>,
    pub hp: Arc<RwLock<i32>>,
    pub ammo: Arc<RwLock<u32>>,
}

impl Default for SyncState {
    fn default() -> Self {
        Self {
            is_game_started: Arc::new(RwLock::new(false)),
            hp: Arc::new(RwLock::new(0)),
            ammo: Arc::new(RwLock::new(0)),
        }
    }
}

impl SentryState {
    pub fn new(client: ActionClient<NavigateToPose::Action>) -> (Self, SyncState) {
        let sync_state = SyncState::default();
        (
            Self {
                sync_state: sync_state.clone(),
                nav_client: Arc::new(client),
                moving_status: None,
                moving_goal_handle: Arc::new(Mutex::new(None)),
                cancelling_status: None,
            },
            sync_state,
        )
    }
    /// Pending if the lock is currently held by an exclusive writer.
    #[inline]
    pub fn is_game_started(&self) -> Poll<bool> {
        self.sync_state
            .is_game_started
            .try_read()
            .ok()
            .map(|x| *x)
            .into_poll()
    }

    /// Pending if the lock is currently held by an exclusive writer.
    #[inline]
    pub fn get_hp(&self) -> Poll<i32> {
        self.sync_state.hp.try_read().ok().map(|x| *x).into_poll()
    }

    // Pending if the lock is currently held by an exclusive writer.
    // #[inline]
    // pub fn get_ammo(&self) -> Poll<u32> {
    //     self.sync_state.ammo.try_read().ok().map(|x| *x).into_poll()
    // }
}

trait IntoPoll<T> {
    fn into_poll(self) -> Poll<T>;
}

impl<T> IntoPoll<T> for Option<T> {
    fn into_poll(self) -> Poll<T> {
        match self {
            Some(x) => Poll::Ready(x),
            None => Poll::Pending,
        }
    }
}

trait IntoStatus {
    fn into_status(self, when_success: impl FnOnce()) -> Status;
}

impl IntoStatus for Poll<bool> {
    fn into_status(self, when_success: impl FnOnce()) -> Status {
        match self {
            Poll::Ready(true) => {
                when_success();
                Status::Success
            }
            Poll::Ready(false) => Status::Failure,
            Poll::Pending => Status::Running,
        }
    }
}
impl IntoStatus for Poll<()> {
    fn into_status(self, when_success: impl FnOnce()) -> Status {
        match self {
            Poll::Ready(_) => {
                when_success();
                Status::Success
            }
            Poll::Pending => Status::Running,
        }
    }
}

fn wrap_async_task(
    task_status: &mut Option<oneshot::Receiver<TaskStatus>>,
    task: impl Future<Output = Result<(), anyhow::Error>> + Send + 'static,
) -> Status {
    let receiver = task_status.get_or_insert_with(|| {
        let (task_status_sender, task_status_receiver) = oneshot::channel();
        spawn_async_task(task_status_sender, task);
        task_status_receiver
    });
    match receiver.try_recv() {
        Err(TryRecvError::Empty) => Status::Running,
        Ok(TaskStatus::Finished) => {
            *task_status = None;
            log::trace!("Some async task successed");
            Status::Success
        }
        Err(TryRecvError::Closed) | Ok(TaskStatus::Error) => {
            *task_status = None;
            Status::Failure
        }
    }
}

fn spawn_async_task(
    task_status_sender: oneshot::Sender<TaskStatus>,
    task: impl Future<Output = Result<(), anyhow::Error>> + Send + 'static,
) {
    task::spawn(async move {
        if let Err(err) = task.await {
            log::error!("task failed: {err:?}");
            task_status_sender.send(TaskStatus::Error).unwrap() // receiver would not have dropped now;
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
