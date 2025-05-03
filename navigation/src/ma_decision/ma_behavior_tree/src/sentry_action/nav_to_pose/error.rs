use std::time::Instant;

use ros2_client::service::CallServiceError;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum SendGoalError {
    #[error("Timeout at {0:?} while waiting for action server to accept goal")]
    Timeout(Instant),
    #[error("Goal was Rejected")]
    Rejected,
    #[error("Call service with error: {0:?}")]
    CallServiceError(CallServiceError<()>),
}

impl From<CallServiceError<()>> for SendGoalError {
    fn from(value: CallServiceError<()>) -> Self {
        Self::CallServiceError(value)
    }
}

#[derive(Debug, Error)]
pub enum RequestResultError {
    #[error("Call service with error: {0:?}")]
    CallServiceError(CallServiceError<()>),
    #[error("Goal was not successful")]
    GoalNotSuccessful,
}

impl From<CallServiceError<()>> for RequestResultError {
    fn from(value: CallServiceError<()>) -> Self {
        Self::CallServiceError(value)
    }
}

#[derive(Debug, Error)]
pub enum CancelGoalError {
    #[error("Call service with error: {0:?}")]
    CallServiceError(CallServiceError<()>),

    #[error("Request was rejected")]
    Rejected,

    #[error("Requested goal ID does not exist")]
    UnknownGoal,

    #[error("Goal is already in terminal state")]
    GoalTerminated,
}

impl From<CallServiceError<()>> for CancelGoalError {
    fn from(value: CallServiceError<()>) -> Self {
        Self::CallServiceError(value)
    }
}
