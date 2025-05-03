use futures_util::{Stream, StreamExt};

pub fn fake_serial() -> impl Stream<Item = (i16, u8)> {
    futures_util::stream::repeat((500, 4)).take(100)
}
