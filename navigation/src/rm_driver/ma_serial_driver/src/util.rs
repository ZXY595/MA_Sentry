use std::{
    fmt::Display,
    task::{Poll, ready},
};

use futures_util::Stream;
use pin_project::pin_project;

#[pin_project]
/// A stream that caches the last yielded item from either of two streams.
///
/// the cached item is updated and yielded when either stream yields the item.
pub struct CachedOr<T, S1, S2> {
    cache: T,
    #[pin]
    stream1: S1,
    #[pin]
    stream2: S2,
}

impl<T, S1, S2> CachedOr<T, S1, S2>
where
    S1: Stream,
    S2: Stream,
    T: UpdateFrom<S1::Item> + UpdateFrom<S2::Item> + Default + Clone,
{
    pub fn new(stream1: S1, stream2: S2) -> Self {
        Self {
            stream1,
            stream2,
            cache: Default::default(),
        }
    }
}

impl<T, S1, S2> Stream for CachedOr<T, S1, S2>
where
    S1: Stream,
    S2: Stream,
    T: UpdateFrom<S1::Item> + UpdateFrom<S2::Item> + Default + Clone,
{
    type Item = T;

    fn poll_next(
        self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> Poll<Option<Self::Item>> {
        let mut project = self.project();
        let cache = project.cache;

        let stream1_result = ready!(project.stream1.as_mut().poll_next(cx));
        let stream2_result = ready!(project.stream2.as_mut().poll_next(cx));

        match (stream1_result, stream2_result) {
            (Some(item), None) => {
                cache.update_from(item);
                Poll::Ready(Some(cache.clone()))
            }
            (None, Some(item)) => {
                cache.update_from(item);
                Poll::Ready(Some(cache.clone()))
            }
            (Some(item1), Some(item2)) => {
                cache.update_from(item1);
                cache.update_from(item2);
                Poll::Ready(Some(cache.clone()))
            }
            (None, None) => Poll::Ready(None),
        }
    }
}

pub trait UpdateFrom<T> {
    fn update_from(&mut self, other: T);
}

impl<F, T, E> UpdateFrom<Result<F, E>> for T
where
    T: UpdateFrom<F>,
    E: Display,
{
    fn update_from(&mut self, other: Result<F, E>) {
        match other {
            Ok(other) => self.update_from(other),
            Err(e) => {
                log::error!("Error value updating from other: {e}")
            }
        }
    }
}
