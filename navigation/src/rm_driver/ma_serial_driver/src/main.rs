mod decoder;
mod encoder;

use std::{pin::pin, time::Duration};

use anyhow::Context;
use futures_util::{SinkExt, StreamExt};
use r2r::geometry_msgs::msg::Twist;
use tokio::runtime;
use tokio_serial::SerialPortBuilderExt;
use tokio_util::codec::FramedWrite;

const SERIAL_PATH: &str = "/dev/ttyACM0";
const SERIAL_BAUD: u32 = 9600;

fn main() -> Result<(), anyhow::Error> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "serial_driver", "/ma")?;

    let chassis_subscription =
        node.subscribe::<Twist>("/cmd_vel_chassis", r2r::QosProfile::default())?;

    let runtime = runtime::Builder::new_current_thread().enable_io().build()?;

    runtime.spawn(async move {
        node.spin_once(Duration::from_millis(50));
        tokio::task::yield_now().await;
    });

    runtime.block_on(async {
        let mut serial_port = tokio_serial::new(SERIAL_PATH, SERIAL_BAUD)
            .open_native_async()
            .context("Unable to open serial port")?;

        serial_port
            .set_exclusive(false)
            .context("Unable to set serial port exclusive to false")?;

        let serial_sink = FramedWrite::new(serial_port, encoder::TwistEncoder::default());

        chassis_subscription
            .map(|twist| Ok(twist))
            .forward(pin!(serial_sink.sink_map_err(anyhow::Error::from)))
            .await?;

        Ok(())
    })
}
