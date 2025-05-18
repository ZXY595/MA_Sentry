#[deny(clippy::undocumented_unsafe_blocks)]
mod frames;

mod serial_io;
mod util;

use anyhow::{Context, anyhow};
use futures_util::{AsyncReadExt, AsyncWriteExt, SinkExt, StreamExt, TryStreamExt};
use ma_dds::prelude::Ros2NodeExt;
use ma_interfaces::{
    geometry_msgs::msg::Twist,
    msg::{BTState, GimbalCmd, SerialReceiveData},
};
use ros2_client::{Publisher, Subscription};
use zerocopy::FromBytes;

const SERIAL_PATH: &str = "/dev/ttyACM0";
const SERIAL_BAUD: u32 = 115200;

fn main() -> Result<(), anyhow::Error> {
    let mut node = ma_dds::Ros2NodeBuilder::new()
        .name("/ma", "serial_driver")
        .build()?;

    let chassis_subscription: Subscription<Twist> = node
        .pub_sub_builder()
        .topic_base_name("cmd_vel_chassis")
        .type_name("geometry_msgs", "Twist")
        .build_subscription()?;

    let bt_state_subscription: Subscription<BTState> = node
        .pub_sub_builder()
        .topic_base_name("cmd_bt_state")
        .type_name("rm_interfaces", "BTState")
        .build_subscription()?;

    let gimbal_subscription: Subscription<GimbalCmd> = node
        .pub_sub_builder()
        .topic_base_name("cmd_gimbal")
        .type_name("rm_interfaces", "GimbalCmd")
        .build_subscription()?;

    let serial_receive_publisher: Publisher<SerialReceiveData> = node
        .pub_sub_builder()
        .topic_base_name("serial_receive")
        .type_name("rm_interfaces", "SerialReceiveData")
        .build_publisher()?;

    let serial_port = serialport::new(SERIAL_PATH, SERIAL_BAUD)
        .open_native()
        .context("Unable to open serial port")?;

    let (mut serial_rx, serial_tx) = serial_io::SerialPort::new_async(serial_port)?.split();

    smol::spawn(async move { node.spinner()?.spin().await }).detach();

    smol::spawn::<anyhow::Result<()>>(async move {
        let mut serial_buf = [0u8; size_of::<frames::SerialReceiveFrame>()];

        loop {
            serial_rx.read_exact(&mut serial_buf).await?;
            let frame = frames::SerialReceiveFrame::ref_from_bytes(&serial_buf)
                .map_err(|received| anyhow!("Received invalid frame: {received:?}"))?;
            serial_receive_publisher.async_publish(frame.into()).await?;
        }
    })
    .detach();

    smol::block_on(async {
        let gimbal_stream = gimbal_subscription
            .async_stream()
            .map_err(anyhow::Error::from);
        let chassis_stream = chassis_subscription
            .async_stream()
            .map_err(anyhow::Error::from);
        let bt_state_stream = bt_state_subscription
            .async_stream()
            .map_err(anyhow::Error::from);

        util::cached_or(frames::SerialSendFrame::default(), gimbal_stream, chassis_stream, bt_state_stream)
            .map(frames::SerialFrame::from)
            .map(Result::Ok)
            .forward(serial_tx.into_sink().sink_map_err(anyhow::Error::from))
            .await
    })
}
