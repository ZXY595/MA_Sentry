use bonsai_bt::Behavior::{self, *};
use r2r::geometry_msgs::msg::{Point, Pose};

use crate::sentry_action::SentryAction;
pub const LOW_HP_THRESHOLD: i32 = 110;
pub const HIGH_HP_THRESHOLD: i32 = 399;

const BUFF_POINT: Point = Point {
    x: 3.5,
    y: 5.3,
    // x: 1.0,
    // y: 0.0,
    z: 0.0,
};

const BASE_POINT: Point = Point {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};
pub fn get_behavior() -> Behavior<SentryAction> {
    Sequence(vec![
        Action(SentryAction::UntilGameStarted),
        WhileAll(
            Box::new(Action(SentryAction::UntilGameOver)),
            vec![AlwaysSucceed(Box::new(alive_task()))],
        ),
    ])
}

fn alive_task() -> Behavior<SentryAction> {
    Sequence(vec![
        Action(SentryAction::UntilIsAlive),
        attack_move_to_buff_area_and_retreat(),
    ])
}

fn attack_move_to_buff_area_and_retreat() -> Behavior<SentryAction> {
    If(
        Box::new(Action(SentryAction::IsHpGreaterThan(LOW_HP_THRESHOLD))),
        Box::new(move_to_buff_area()),
        Box::new(move_to_supply()),
    )
}

fn move_to_buff_area() -> Behavior<SentryAction> {
    WhenAny(vec![
        Sequence(vec![
            Action(SentryAction::UntilIsHpLessThan(LOW_HP_THRESHOLD)),
            Action(SentryAction::CancelMoving),
            move_to_supply(),
        ]),
        move_to_random_point(BUFF_POINT),
    ])
}

fn move_to_supply() -> Behavior<SentryAction> {
    Sequence(vec![
        move_to_point(BASE_POINT),
        Action(SentryAction::UntilIsHpGreaterThan(HIGH_HP_THRESHOLD)),
    ])
}

fn move_to_point(point: Point) -> Behavior<SentryAction> {
    Action(SentryAction::MoveTo(Pose {
        position: point,
        ..Default::default()
    }))
}

fn move_to_random_point(point: Point) -> Behavior<SentryAction> {
    Action(SentryAction::RandomMoveTo(Pose {
        position: point,
        ..Default::default()
    }))
}
