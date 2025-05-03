use ros2_client::{NodeCreateError, names::NameError, ros2::CreateError};
use thiserror::Error;
#[derive(Debug, Error)]
#[error("Missing argument: {0}")]
pub struct MissingArgumentError(pub &'static str);

#[derive(Debug, Error)]
pub enum CreateNodeError {
    #[error(transparent)]
    FailToCreateContext(#[from] CreateError),
    #[error(transparent)]
    FailToCreateNode(#[from] NodeCreateError),
    #[error(transparent)]
    FailToParseName(#[from] NameError),
    #[error(transparent)]
    MissingArgument(#[from] MissingArgumentError),
}

#[derive(Debug, Error)]
pub enum CreatePubSubError {
    #[error(transparent)]
    FailToCreate(#[from] CreateError),
    #[error(transparent)]
    FailToParseName(#[from] NameError),
    #[error(transparent)]
    MissingArgument(#[from] MissingArgumentError),
}
