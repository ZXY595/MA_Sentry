use bonsai_bt::Behavior::{self, *};
use r2r::geometry_msgs::msg::{Point, Pose};

use crate::sentry_action::SentryAction;
pub fn get_behavior() -> Behavior<SentryAction> {
    Select(vec![alive_task(), dead_task()])
}

fn alive_task() -> Behavior<SentryAction> {
    WhenAll(vec![
        Action(SentryAction::IsAlive),
        attack_move_to_buff_area_and_retreat(),
    ])
}

fn dead_task() -> Behavior<SentryAction> {
    WhenAll(vec![Action(SentryAction::IsDead)])
}

fn when_reborn() -> Behavior<SentryAction> {
    todo!()
}

fn healthy_task() -> Behavior<SentryAction> {
    WhenAll(vec![
        when_healthy(),
        Select(vec![
            engaging_enemy_task(),
            move_to_buff_area(),
        ]),
    ])
}

fn engaging_enemy_task() -> Behavior<SentryAction> {
    WhenAll(vec![
        Action(SentryAction::IsEngagingEmemy),
        stop_and_guard(),
    ])
}

fn attack_move_to_buff_area_and_retreat() -> Behavior<SentryAction> {
    Select(vec![healthy_task(), move_to_supply()])
}

fn stop_and_guard() -> Behavior<SentryAction> {
    WhenAll(vec![
        Action(SentryAction::CancelMoving),
        Action(SentryAction::SpinArmor),
        Wait(5.0), // at least guard for 5s
    ])
}

fn when_healthy() -> Behavior<SentryAction> {
    WhenAll(vec![
        Action(SentryAction::IsHpGreaterThan(50)),
        Action(SentryAction::IsAmmoGreaterThan(10)),
    ])
}

fn move_to_buff_area() -> Behavior<SentryAction> {
    WhenAll(vec![
        Invert(Box::new(Action(SentryAction::IsEngagingEmemy))),
        Action(SentryAction::MoveTo(Pose {
            position: Point {
                x: 4.0,
                y: 0.0,
                z: 0.0,
            },
            ..Default::default()
        })),
    ])
}

fn move_to_supply() -> Behavior<SentryAction> {
    WhenAll(vec![
        Select(vec![
            Action(SentryAction::IsHpLessThan(50)),
            Action(SentryAction::IsAmmoLessThan(10)),
        ]),
        Action(SentryAction::MoveTo(Pose {
            position: Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            ..Default::default()
        })),
    ])
}

// fn guard_area() -> Behavior<SentryAction> {
//     While(Action(SentryAction::IsAlive), Action())
// }
