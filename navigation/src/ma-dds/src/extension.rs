use crate::builder::{Ros2ActionClientBuilder, Ros2PubSubBuilder};
pub trait Ros2NodeExt {
    fn pub_sub_builder<M>(&mut self) -> Ros2PubSubBuilder<M>;
    fn action_client_builder<A>(&mut self) -> Ros2ActionClientBuilder<A>;
}
impl Ros2NodeExt for ros2_client::Node {
    fn pub_sub_builder<M>(&mut self) -> Ros2PubSubBuilder<M> {
        Ros2PubSubBuilder::new().node(self)
    }

    fn action_client_builder<A>(&mut self) -> Ros2ActionClientBuilder<A> {
        Ros2ActionClientBuilder::new().node(self)
    }
}
