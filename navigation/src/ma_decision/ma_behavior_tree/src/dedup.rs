use std::task::ready;

use futures_util::Stream;
use pin_project_lite::pin_project;

pin_project! {
    pub struct Dedup<S: Stream> {
        #[pin]
        inner: S,
        last: S::Item,
    }
}

impl<S: Stream> Dedup<S> {
    #[expect(unused)]
    pub fn new(inner: S, last_init: S::Item) -> Self {
        Self {
            inner,
            last: last_init,
        }
    }
    pub fn new_default(inner: S) -> Self
    where
        S::Item: Default,
    {
        Self {
            inner,
            last: S::Item::default(),
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
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
            }
            None => Poll::Ready(None),
        }
    }
}

#[test]
fn test_simple_numeric_stream() {
    use futures_util::{StreamExt, stream};
    let input = stream::repeat(1).take(3).chain(stream::repeat(2).take(2));
    smol::block_on(async {
        let result = Dedup::new_default(input).collect::<Vec<_>>().await;
        assert_eq!(result, vec![1, 2]);
    })
}

#[test]
fn test_tuple_stream() {
    use futures_util::{StreamExt, stream};
    let input = stream::repeat((1, 2))
        .take(3)
        .chain(stream::repeat((2, 3)).take(2));
    smol::block_on(async {
        let result = Dedup::new_default(input).collect::<Vec<_>>().await;
        assert_eq!(result, vec![(1, 2), (2, 3)]);
    })
}
