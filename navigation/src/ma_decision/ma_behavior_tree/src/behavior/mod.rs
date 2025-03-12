use bonsai_bt::Behavior::{self, Action, Sequence};
use r2r::geometry_msgs::msg::{Point, Pose};

use crate::sentry_action::SentryAction;
pub fn get_behavior() -> Behavior<SentryAction> {
    Sequence(vec![
        Action(SentryAction::IsGameStarted),
        Action(SentryAction::MoveTo(Pose {
            position: Point {
                x: 4.0,
                y: 0.0,
                z: 0.0,
            },
            ..Default::default()
        })),
        Action(SentryAction::Idle),
        Action(SentryAction::IsGameOver),
    ])
}
