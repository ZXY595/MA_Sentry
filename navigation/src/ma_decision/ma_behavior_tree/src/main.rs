mod behavior;
#[cfg(feature = "fake_serial")]
mod fake_serial;
mod sentry_action;

use std::pin::pin;

use behavior::SentryCtx;
use futures_util::{Stream, StreamExt};
use ma_dds::prelude::Ros2NodeExt;
use ma_interfaces::nav2_msgs::action::NavigateToPose;
use pin_project::pin_project;
use ros2_client::action::ActionClientQosPolicies;
use std::task::ready;

fn main() -> Result<(), anyhow::Error> {
    env_logger::init();
    let mut node = ma_dds::Ros2NodeBuilder::new()
        .name("/ma", "behavior_tree")
        .build()?;

    #[cfg(not(feature = "fake_serial"))]
    let serial_subscription = node
        .pub_sub_builder::<ma_interfaces::msg::SerialReceiveData>()
        .topic_name("/serial", "receive")
        .build_subscription()?;

    //     "/serial/receive",
    //     QosProfile::default()
    //         .reliability(qos::ReliabilityPolicy::SystemDefault)
    //         .history(qos::HistoryPolicy::KeepLast)
    //         .durability(qos::DurabilityPolicy::SystemDefault),
    // )?;

    let nav_client = node
        .action_client_builder::<NavigateToPose::Action>()
        .action_base_name("navigate_to_pose")
        .type_name("nav2", "NavigateToPose")
        .qos(ActionClientQosPolicies {
            goal_service: Default::default(),
            result_service: Default::default(),
            cancel_service: Default::default(),
            feedback_subscription: Default::default(),
            status_subscription: Default::default(),
        })
        .build()?;

    smol::spawn(node.spinner()?.spin()).detach();

    let (hp_tx, hp_rx) = async_broadcast::broadcast(2);
    let (game_status_tx, game_status_rx) = async_broadcast::broadcast(1);

    smol::spawn(async move {
        #[cfg(not(feature = "fake_serial"))]
        let serial_stream = serial_subscription
            .async_stream()
            .map_ok(|(msg, _)| {
                let SerialReceiveData {
                    judge_system_data:
                        JudgeSystemData {
                            game_status, hp, ..
                        },
                    ..
                } = msg;
                (hp, game_status)
            })
            .inspect_err(|e| log::error!("Error: {e}"))
            .filter_map(|x| async { x.ok() });
        #[cfg(feature = "fake_serial")]
        let serial_stream = fake_serial::fake_serial();

        Dedup::new(serial_stream, (0, 0))
            .for_each(async |(hp, game_status)| {
                hp_tx.broadcast_direct(hp).await.unwrap();
                game_status_tx.broadcast_direct(game_status).await.unwrap();
            })
            .await;
    })
    .detach();

    smol::block_on(behavior::sentry_task(SentryCtx::new(
        nav_client,
        game_status_rx,
        hp_rx,
    )))
}

#[pin_project]
struct Dedup<S: Stream> {
    #[pin]
    inner: S,
    last: S::Item,
}

impl<S: Stream> Dedup<S> {
    fn new(inner: S, last_default: S::Item) -> Self {
        Self {
            inner,
            last: last_default,
        }
    }
}

impl<S> Stream for Dedup<S>
where
    S: Stream<Item: PartialEq + Clone>,
{
    type Item = S::Item;

    fn poll_next(
        self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        use std::task::Poll;
        let mut this = self.project();
        let last = this.last;
        let next = ready!(this.inner.as_mut().poll_next(cx));
        match next {
            Some(next) => {
                if next.ne(last) {
                    *last = next.clone();
                    Poll::Ready(Some(next))
                } else {
                    Poll::Pending
                }
            }
            None => Poll::Ready(None),
        }
    }
}
