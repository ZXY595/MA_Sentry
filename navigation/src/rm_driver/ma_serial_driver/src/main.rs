mod decoder;
mod encoder;

use std::pin::pin;

use anyhow::Context;
use futures_util::{SinkExt, StreamExt, TryStreamExt};
use ros2_client::{MessageTypeName, Name, NodeName, ros2::QosPolicies};
use ros2_interfaces_humble::geometry_msgs::msg::Twist;
use tokio::runtime;
use tokio_serial::SerialPortBuilderExt;
use tokio_util::codec::FramedWrite;

const SERIAL_PATH: &str = "/dev/ttyACM0";
const SERIAL_BAUD: u32 = 9600;

fn main() -> Result<(), anyhow::Error> {
    let mut node = ros2_client::Context::new()?
        .new_node(NodeName::new("/ma", "serial_driver")?, Default::default())?;

    let qos = QosPolicies::default();

    let topic = node.create_topic(
        &Name::new("/", "cmd_vel_chassis")?,
        MessageTypeName::new("geometry_msgs", "Twist"),
        &qos,
    )?;

    let chassis_subscription = node.create_subscription::<Twist>(&topic, None)?;

    let runtime = runtime::Builder::new_current_thread().enable_io().build()?;

    runtime.spawn(node.spinner()?.spin());

    runtime.block_on(async {
        let mut serial_port = tokio_serial::new(SERIAL_PATH, SERIAL_BAUD)
            .open_native_async()
            .context("Unable to open serial port")?;

        serial_port
            .set_exclusive(false)
            .context("Unable to set serial port exclusive to false")?;

        let serial_sink = FramedWrite::new(serial_port, encoder::TwistEncoder::default());

        chassis_subscription.wait_for_publisher(&node).await;

        let chassis_stream = chassis_subscription.async_stream();

        chassis_stream
            .map_ok(|(twist, _)| twist)
            .map_err(anyhow::Error::from)
            .forward(pin!(serial_sink.sink_map_err(anyhow::Error::from)))
            .await?;

        Ok(())
    })
}
