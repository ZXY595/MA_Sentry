use ma_interfaces::{
    geometry_msgs::msg::Twist,
    msg::{GimbalCmd, JudgeSystemData, SerialReceiveData},
};
use ros2_client::MessageInfo;
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

use crate::util::UpdateFrom;

#[derive(IntoBytes, Immutable)]
#[repr(packed)]
pub struct SerialFrame<T> {
    #[expect(unused)]
    head: u8,
    #[expect(unused)]
    inner: T,
}

#[derive(IntoBytes, Immutable, Default, Clone)]
pub struct GimbalTwistFrame {
    pub gimbal: GimbalFrame,
    pub twist: TwistFrame,
}

#[derive(IntoBytes, Immutable, Default, Clone)]
#[repr(packed)]
pub struct GimbalFrame {
    #[expect(unused)]
    fire_advice: bool,
    #[expect(unused)]
    pitch: f64,
    #[expect(unused)]
    yaw: f64,
    #[expect(unused)]
    distance: f64,
}

#[derive(IntoBytes, Immutable, Default, Clone)]
#[repr(packed)]
pub struct TwistFrame {
    #[expect(unused)]
    linear: [f64; 3],
    #[expect(unused)]
    angular: [f64; 3],
    #[expect(unused)]
    spin_advice: bool,
}

#[derive(FromBytes, Immutable, KnownLayout)]
#[repr(packed)]
pub struct SerialReceiveFrame {
    pub mode: u8,
    pub bullet_speed: f32,
    pub roll: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub game_status: u8,
    pub hp: i16,
    pub ammo: u16,
    pub outpost_hp: u16,
}

impl<T> From<T> for SerialFrame<T> {
    fn from(value: T) -> Self {
        Self {
            head: 0xFF,
            inner: value,
        }
    }
}

impl<T> AsRef<[u8]> for SerialFrame<T>
where
    T: IntoBytes + Immutable,
{
    fn as_ref(&self) -> &[u8] {
        self.as_bytes()
    }
}

impl From<Twist> for TwistFrame {
    fn from(value: Twist) -> Self {
        Self {
            linear: [value.linear.x, value.linear.y, value.linear.z],
            angular: [value.angular.x, value.angular.y, value.angular.z],
            spin_advice: false,
        }
    }
}

impl From<(Twist, MessageInfo)> for TwistFrame {
    fn from(value: (Twist, MessageInfo)) -> Self {
        Self::from(value.0)
    }
}

impl AsRef<[u8]> for TwistFrame {
    fn as_ref(&self) -> &[u8] {
        self.as_bytes()
    }
}

impl From<GimbalCmd> for GimbalFrame {
    fn from(value: GimbalCmd) -> Self {
        Self {
            fire_advice: value.fire_advice,
            pitch: value.pitch,
            yaw: value.yaw,
            distance: value.distance,
        }
    }
}

impl From<(GimbalCmd, MessageInfo)> for GimbalFrame {
    fn from(value: (GimbalCmd, MessageInfo)) -> Self {
        Self::from(value.0)
    }
}

impl AsRef<[u8]> for GimbalFrame {
    fn as_ref(&self) -> &[u8] {
        self.as_bytes()
    }
}

impl From<&SerialReceiveFrame> for SerialReceiveData {
    fn from(value: &SerialReceiveFrame) -> Self {
        Self {
            header: Default::default(),
            mode: value.mode,
            bullet_speed: value.bullet_speed,
            roll: value.roll,
            yaw: value.yaw,
            pitch: value.pitch,
            judge_system_data: JudgeSystemData {
                game_status: value.game_status,
                hp: value.hp,
                ammo: value.ammo,
                ..Default::default()
            },
        }
    }
}

impl UpdateFrom<GimbalFrame> for GimbalTwistFrame {
    fn update_from(&mut self, other: GimbalFrame) {
        self.gimbal = other;
    }
}

impl UpdateFrom<TwistFrame> for GimbalTwistFrame {
    fn update_from(&mut self, other: TwistFrame) {
        self.twist = other;
    }
}
