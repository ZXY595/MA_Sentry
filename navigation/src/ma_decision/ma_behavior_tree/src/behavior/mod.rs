use bonsai_bt::Behavior::{self, *};
use r2r::geometry_msgs::msg::{Point, Pose};

use crate::sentry_action::SentryAction;
pub fn get_behavior() -> Behavior<SentryAction> {
    WhileAll(
        Box::new(Action(SentryAction::UntilGameOver)),
        vec![AlwaysSucceed(Box::new(alive_task()))],
    )
}

fn alive_task() -> Behavior<SentryAction> {
    Sequence(vec![
        Action(SentryAction::UntilIsAlive),
        attack_move_to_buff_area_and_retreat(),
    ])
}

fn attack_move_to_buff_area_and_retreat() -> Behavior<SentryAction> {
    If(
        Box::new(when_healthy()),
        Box::new(move_to_buff_area()),
        Box::new(move_to_supply()),
    )
}

fn move_to_buff_area() -> Behavior<SentryAction> {
    Box::new(Action(SentryAction::MoveTo(Pose {
        position: Point {
            x: 4.0,
            y: 0.0,
            z: 0.0,
        },
        ..Default::default()
    })))
}
fn when_healthy() -> Behavior<SentryAction> {
    WhenAll(vec![
        Action(SentryAction::IsHpGreaterThan(50)),
        Action(SentryAction::IsAmmoGreaterThan(10)),
    ])
}

fn move_to_supply() -> Behavior<SentryAction> {
    Box::new(Action(SentryAction::MoveTo(Pose {
        position: Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        ..Default::default()
    })))
}
