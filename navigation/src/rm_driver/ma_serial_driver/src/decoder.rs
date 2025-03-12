use r2r::geometry_msgs::msg::{Twist, Vector3};
use tokio_util::{bytes::Buf, codec::Decoder};

pub struct TwistDecoder(Vector3Decoder);

impl Decoder for TwistDecoder {
    type Item = Twist;

    type Error = std::io::Error;

    fn decode(
        &mut self,
        src: &mut tokio_util::bytes::BytesMut,
    ) -> Result<Option<Self::Item>, Self::Error> {
        let linear = self.0.decode(src)?;
        let angular = self.0.decode(src)?;
        Ok(linear.and_then(|linear| {
            let angular = angular?;
            Some(Twist { linear, angular })
        }))
    }
}

#[derive(Default)]
pub struct Vector3Decoder;

impl Decoder for Vector3Decoder {
    type Item = Vector3;

    type Error = std::io::Error;

    fn decode(
        &mut self,
        src: &mut tokio_util::bytes::BytesMut,
    ) -> Result<Option<Self::Item>, Self::Error> {
        let x = src.get_f64();
        let y = src.get_f64();
        let z = src.get_f64();
        Ok(Some(Vector3 { x, y, z }))
    }
}
