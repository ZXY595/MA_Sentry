use std::{
    fmt::Display,
    task::{Poll, ready},
};

use futures_util::Stream;
use pin_project_lite::pin_project;

pin_project! {
    /// A stream that caches the last yielded item from either of two streams.
    ///
    /// the cached item is updated and yielded when either stream yields the item.
    pub struct CachedOr<T, S1, S2, S3> {
        cache: T,
        #[pin]
        stream1: S1,
        #[pin]
        stream2: S2,
        #[pin]
        stream3: S3,
    }
}

impl<T, S1, S2, S3> CachedOr<T, S1, S2, S3>
where
    S1: Stream,
    S2: Stream,
    S3: Stream,
    T: UpdateFrom<S1::Item> + UpdateFrom<S2::Item> + UpdateFrom<S3::Item> + Clone,
{
    pub fn new(default: T, stream1: S1, stream2: S2, stream3: S3) -> Self {
        Self {
            stream1,
            stream2,
            stream3,
            cache: default,
        }
    }
}

impl<T, S1, S2, S3> Stream for CachedOr<T, S1, S2, S3>
where
    S1: Stream,
    S2: Stream,
    S3: Stream,
    T: UpdateFrom<S1::Item> + UpdateFrom<S2::Item> + UpdateFrom<S3::Item> + Clone,
{
    type Item = T;

    fn poll_next(
        self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> Poll<Option<Self::Item>> {
        let mut this = self.project();
        let cache = this.cache;

        fn update_cache<T, I>(cache: &mut T, stream_result: Option<I>) -> bool
        where
            T: UpdateFrom<I>,
        {
            if let Some(item) = stream_result {
                cache.update_from(item);
                true
            } else {
                false
            }
        }
        let has_updated = update_cache(cache, ready!(this.stream1.as_mut().poll_next(cx)))
            | update_cache(cache, ready!(this.stream2.as_mut().poll_next(cx)))
            | update_cache(cache, ready!(this.stream3.as_mut().poll_next(cx)));

        Poll::Ready(has_updated.then_some(cache.clone()))
    }
}

pub trait UpdateFrom<T> {
    fn update_from(&mut self, value: T);
}

impl<F, T, E> UpdateFrom<Result<F, E>> for T
where
    T: UpdateFrom<F>,
    E: Display,
{
    fn update_from(&mut self, value: Result<F, E>) {
        match value {
            Ok(other) => self.update_from(other),
            Err(e) => {
                log::error!("Error value updating from other: {e}")
            }
        }
    }
}

pub fn cached_or<T, S1, S2, S3>(
    default: T,
    stream1: S1,
    stream2: S2,
    stream3: S3,
) -> CachedOr<T, S1, S2, S3>
where
    S1: Stream,
    S2: Stream,
    S3: Stream,
    T: UpdateFrom<S1::Item> + UpdateFrom<S2::Item> + UpdateFrom<S3::Item> + Default + Clone,
{
    CachedOr::new(default, stream1, stream2, stream3)
}
