use std::marker::PhantomData;

use crate::error::{CreateNodeError, CreatePubSubError, MissingArgumentError};
use ros2_client::{
    ActionTypeName, ActionTypes, MessageTypeName, Name, Node, NodeName, NodeOptions, Publisher,
    Subscription,
    action::{ActionClient, ActionClientQosPolicies},
    ros2::QosPolicies,
};
use serde::Serialize;

pub struct Ros2NodeBuilder<'a> {
    name_space: Result<&'a str, MissingArgumentError>,
    base_name: Result<&'a str, MissingArgumentError>,
}

impl<'a> Ros2NodeBuilder<'a> {
    pub const fn new() -> Self {
        Self {
            name_space: Ok("/"),
            base_name: Err(MissingArgumentError("base_name")),
        }
    }
    pub const fn name_space(mut self, name_space: &'a str) -> Self {
        self.name_space = Ok(name_space);
        self
    }
    pub const fn base_name(mut self, base_name: &'a str) -> Self {
        self.base_name = Ok(base_name);
        self
    }
    pub const fn name(mut self, name_space: &'a str, base_name: &'a str) -> Self {
        self.name_space = Ok(name_space);
        self.base_name = Ok(base_name);
        self
    }
    pub fn build(self) -> Result<Node, CreateNodeError> {
        ros2_client::Context::new()?
            .new_node(
                NodeName::new(self.name_space?, self.base_name?)?,
                NodeOptions::default(),
            )
            .map_err(From::from)
    }
}

pub struct Ros2PubSubBuilder<'a, M> {
    node: Result<&'a mut Node, MissingArgumentError>,
    topic_name_space: Result<&'a str, MissingArgumentError>,
    topic_base_name: Result<&'a str, MissingArgumentError>,
    message_package_name: Result<&'a str, MissingArgumentError>,
    message_type_name: Result<&'a str, MissingArgumentError>,
    qos: Result<&'a QosPolicies, MissingArgumentError>,
    message_type: PhantomData<M>,
}

impl<'a, M> Ros2PubSubBuilder<'a, M> {
    pub const fn new() -> Self {
        Self {
            node: Err(MissingArgumentError("node")),
            topic_name_space: Ok("/"),
            topic_base_name: Err(MissingArgumentError("topic_base_name")),
            message_package_name: Err(MissingArgumentError("message_package_name")),
            message_type_name: Err(MissingArgumentError("message_type_name")),
            qos: Err(MissingArgumentError("qos")),
            message_type: PhantomData,
        }
    }
    pub const fn node(mut self, node: &'a mut Node) -> Self {
        self.node = Ok(node);
        self
    }
    pub const fn topic_name_space(mut self, topic_name_space: &'a str) -> Self {
        self.topic_name_space = Ok(topic_name_space);
        self
    }
    pub const fn topic_base_name(mut self, topic_base_name: &'a str) -> Self {
        self.topic_base_name = Ok(topic_base_name);
        self
    }
    pub const fn topic_name(mut self, name_space: &'a str, base_name: &'a str) -> Self {
        self.topic_name_space = Ok(name_space);
        self.topic_base_name = Ok(base_name);
        self
    }
    pub const fn message_package_name(mut self, message_package_name: &'a str) -> Self {
        self.message_package_name = Ok(message_package_name);
        self
    }
    pub const fn message_type_name(mut self, message_type_name: &'a str) -> Self {
        self.message_type_name = Ok(message_type_name);
        self
    }
    pub const fn type_name(mut self, package_name: &'a str, type_name: &'a str) -> Self {
        self.message_package_name = Ok(package_name);
        self.message_type_name = Ok(type_name);
        self
    }
    pub const fn qos(mut self, qos: &'a QosPolicies) -> Self {
        self.qos = Ok(qos);
        self
    }
    pub fn build_publisher(self) -> Result<Publisher<M>, CreatePubSubError>
    where
        M: Serialize,
    {
        let node = self.node?;
        let topic = node.create_topic(
            &Name::new(self.topic_name_space?, self.topic_base_name?)?,
            MessageTypeName::new(self.message_package_name?, self.message_type_name?),
            self.qos.unwrap_or(&QosPolicies::default()),
        )?;
        node.create_publisher(&topic, None).map_err(From::from)
    }
    pub fn build_subscription(self) -> Result<Subscription<M>, CreatePubSubError>
    where
        M: 'static,
    {
        let node = self.node?;
        let topic = node.create_topic(
            &Name::new(self.topic_name_space?, self.topic_base_name?)?,
            MessageTypeName::new(self.message_package_name?, self.message_type_name?),
            self.qos.unwrap_or(&QosPolicies::default()),
        )?;
        node.create_subscription(&topic, None).map_err(From::from)
    }
}

pub struct Ros2ActionClientBuilder<'a, A> {
    node: Result<&'a mut Node, MissingArgumentError>,
    action_name_space: Result<&'a str, MissingArgumentError>,
    action_base_name: Result<&'a str, MissingArgumentError>,
    type_package_name: Result<&'a str, MissingArgumentError>,
    type_base_name: Result<&'a str, MissingArgumentError>,
    qos: Result<ActionClientQosPolicies, MissingArgumentError>,
    action_type: PhantomData<A>,
}

impl<'a, A> Ros2ActionClientBuilder<'a, A> {
    pub const fn new() -> Self {
        Self {
            node: Err(MissingArgumentError("node")),
            action_name_space: Ok("/"),
            action_base_name: Err(MissingArgumentError("action_base_name")),
            type_package_name: Err(MissingArgumentError("type_package_name")),
            type_base_name: Err(MissingArgumentError("type_base_name")),
            qos: Err(MissingArgumentError("qos")),
            action_type: PhantomData,
        }
    }
    pub const fn node(mut self, node: &'a mut Node) -> Self {
        self.node = Ok(node);
        self
    }
    pub const fn action_name_space(mut self, action_name_space: &'a str) -> Self {
        self.action_name_space = Ok(action_name_space);
        self
    }
    pub const fn action_base_name(mut self, action_base_name: &'a str) -> Self {
        self.action_base_name = Ok(action_base_name);
        self
    }
    pub const fn action_name(mut self, name_space: &'a str, base_name: &'a str) -> Self {
        self.action_name_space = Ok(name_space);
        self.action_base_name = Ok(base_name);
        self
    }
    pub const fn type_package_name(mut self, action_package_name: &'a str) -> Self {
        self.type_package_name = Ok(action_package_name);
        self
    }
    pub const fn type_base_name(mut self, action_type_name: &'a str) -> Self {
        self.type_base_name = Ok(action_type_name);
        self
    }
    pub const fn type_name(mut self, package_name: &'a str, type_name: &'a str) -> Self {
        self.type_package_name = Ok(package_name);
        self.type_base_name = Ok(type_name);
        self
    }
    pub const fn qos(mut self, qos: ActionClientQosPolicies) -> Self {
        self.qos = Ok(qos);
        self
    }
    pub fn build(self) -> Result<ActionClient<A>, CreatePubSubError>
    where
        A: ActionTypes + 'static,
    {
        let node = self.node?;
        node.create_action_client(
            ros2_client::ServiceMapping::Enhanced,
            &Name::new(self.action_name_space?, self.action_base_name?)?,
            &ActionTypeName::new(self.type_package_name?, self.type_base_name?),
            self.qos?,
        )
        .map_err(From::from)
    }
}
