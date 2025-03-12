use r2r::geometry_msgs::msg::{Twist, Vector3};
use tokio_util::{bytes::BufMut, codec::Encoder};

#[derive(Default)]
pub struct TwistEncoder(Vector3Encoder);

impl Encoder<Twist> for TwistEncoder {
    type Error = std::io::Error;

    fn encode(
        &mut self,
        item: Twist,
        dst: &mut tokio_util::bytes::BytesMut,
    ) -> Result<(), Self::Error> {
        let Twist { linear, angular } = item;
        self.0.encode(linear, dst)?;
        self.0.encode(angular, dst)?;
        Ok(())
    }
}

#[derive(Default)]
pub struct Vector3Encoder;

impl Encoder<Vector3> for Vector3Encoder {
    type Error = std::io::Error;

    fn encode(
        &mut self,
        item: Vector3,
        dst: &mut tokio_util::bytes::BytesMut,
    ) -> Result<(), Self::Error> {
        let Vector3 { x, y, z } = item;
        dst.put_f64(x);
        dst.put_f64(y);
        dst.put_f64(z);
        Ok(())
    }
}
