use std::{ops::Not, sync::Arc, task::Poll};

use bonsai_bt::Status;
use r2r::{
    ActionClient, ActionClientGoal, geometry_msgs::msg::Pose, nav2_msgs::action::NavigateToPose,
};
use tokio::{
    sync::{
        Mutex, RwLock,
        oneshot::{self, error::TryRecvError},
    },
    task,
};

mod move_to;

#[derive(Debug, Clone)]
pub enum SentryAction {
    MoveTo(Pose),
    // CancelMoving,
    SpinArmor,
    UntilGameStarted,
    UntilGameOver,
    UntilIsAlive,
    IsGameStarted,
    IsGameOver,
    IsDead,
    IsAlive,
    IsHpLessThan(i32),
    IsAmmoLessThan(u32),

    IsHpGreaterThan(i32),
    IsAmmoGreaterThan(u32),
    Idle,
}

impl SentryAction {
    pub fn tick(&self, state: &mut SentryState, dt: f64) -> (Status, f64) {
        let status = match self {
            SentryAction::MoveTo(pose) => wrap_async_task(
                &mut state.moving_status,
                move_to::move_to_pose(
                    Arc::clone(&state.nav_client),
                    pose.clone(),
                ),
            ),
            // SentryAction::CancelMoving => wrap_async_task(
            //     &mut state.cancelling_status,
            //     move_to::cancel_moving(Arc::clone(&state.moving_goal_handle)),
            // ),
            SentryAction::SpinArmor => todo!(),
            SentryAction::IsGameStarted => state
                .is_game_started()
                .into_status(|| log::info!("Game is started")),
            SentryAction::Idle => Status::Success,

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
            SentryAction::IsGameOver => state
                .is_game_started()
                .map(Not::not)
                .into_status(|| log::info!("Game is over")),
            SentryAction::IsDead => state
                .get_hp()
                .map(|hp| hp <= 0)
                .into_status(|| log::info!("Sentry is dead")),
            SentryAction::UntilIsAlive => {
                if let Poll::Ready(true) = state.get_hp().map(|hp|hp > 0) {
                    log::info!("Sentry is alive");
                    Status::Success
                } else {
                    Status::Running
                }
            }
            SentryAction::IsAlive => state
                .get_hp()
                .map(|hp| hp > 0)
                .into_status(|| log::info!("Sentry is dead")),
            SentryAction::IsHpLessThan(max) => state
                .get_hp()
                .map(|hp| hp <= *max)
                .into_status(|| log::info!("Sentry hp is less than {}", max)),
            SentryAction::IsAmmoLessThan(max) => state
                .get_ammo()
                .map(|ammo| ammo <= *max)
                .into_status(|| log::info!("Sentry ammo is less than {}", max)),
            SentryAction::IsHpGreaterThan(min) => state
                .get_hp()
                .map(|hp| hp > *min)
                .into_status(|| log::info!("Sentry hp is greater than {}", min)),
            SentryAction::IsAmmoGreaterThan(min) => state
                .get_ammo()
                .map(|ammo| ammo > *min)
                .into_status(|| log::info!("Sentry ammo is greater than {}", min)),
        };
        (status, dt)
    }
}

pub struct SentryState {
    sync_state: SyncState,
    nav_client: Arc<ActionClient<NavigateToPose::Action>>,
    moving_status: Option<oneshot::Receiver<TaskStatus>>,
    // for cancelling the goal
    // moving_goal_handle: Arc<Mutex<Option<ActionClientGoal<NavigateToPose::Action>>>>,
    // cancelling_status: Option<oneshot::Receiver<TaskStatus>>,
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

    /// Pending if the lock is currently held by an exclusive writer.
    #[inline]
    pub fn get_ammo(&self) -> Poll<u32> {
        self.sync_state.ammo.try_read().ok().map(|x| *x).into_poll()
    }
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

trait IntoStatusThen<T> {
    fn into_status_then(self, when_success: impl FnOnce(T)) -> Status;
}

impl<T> IntoStatusThen<T> for Poll<Option<T>> {
    fn into_status_then(self, when_success: impl FnOnce(T)) -> Status {
        match self {
            Poll::Ready(Some(x)) => {
                when_success(x);
                Status::Success
            }
            Poll::Ready(None) => Status::Failure,
            Poll::Pending => Status::Running,
        }
    }
}

impl<T> IntoStatusThen<T> for Poll<T> {
    fn into_status_then(self, when_success: impl FnOnce(T)) -> Status {
        match self {
            Poll::Ready(x) => {
                when_success(x);
                Status::Success
            }
            Poll::Pending => Status::Running,
        }
    }
}

impl<T> IntoStatusThen<T> for Option<T> {
    fn into_status_then(self, when_success: impl FnOnce(T)) -> Status {
        match self {
            Some(x) => {
                when_success(x);
                Status::Success
            }
            None => Status::Failure,
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
