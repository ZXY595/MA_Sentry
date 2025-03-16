use bonsai_bt::Behavior::{self, *};
use r2r::geometry_msgs::msg::{Point, Pose};

use crate::sentry_action::SentryAction;
pub const LOWHP_THRESHOLD: i32 = 110;
const MOVING_SPREAD: f64 = 0.45;
const BUFF_POINT: Point = Point {
    x: 4.5,
    y: -4.0,
    z: 0.0,
};

const BASE_POINT: Point = Point {
    x: -0.1,
    y: -0.9,
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
        Box::new(Action(SentryAction::IsHpGreaterThan(LOWHP_THRESHOLD))),
        Box::new(move_to_buff_area()),
        Box::new(move_to_supply()),
    )
}

fn move_to_buff_area() -> Behavior<SentryAction> {
    WhenAny(vec![
        Sequence(vec![Action(SentryAction::UntilIsLowHp), move_to_supply()]),
        move_to_point(get_rand_point(&BUFF_POINT)),
    ])
}

fn move_to_supply() -> Behavior<SentryAction> {
    move_to_point(BASE_POINT)
}

fn move_to_point(point: Point) -> Behavior<SentryAction> {
    Action(SentryAction::MoveTo(Pose {
        position: point,
        ..Default::default()
    }))
}

fn get_rand_point(point: &Point) -> Point {
    Point {
        x: point.x + rand::random_range(-MOVING_SPREAD..MOVING_SPREAD),
        y: point.y + rand::random_range(-MOVING_SPREAD..MOVING_SPREAD),
        z: point.z,
    }
}
