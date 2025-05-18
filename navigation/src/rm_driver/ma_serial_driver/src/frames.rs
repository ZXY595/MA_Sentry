use ma_interfaces::{
    geometry_msgs::msg::Twist,
    msg::{BTState, GimbalCmd, JudgeSystemData, SerialReceiveData},
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
pub struct SerialSendFrame {
    pub gimbal: GimbalFrame,
    pub twist: TwistFrame,
    pub flags: FlagsFrame,
}

#[derive(IntoBytes, Immutable, Default, Clone)]
#[repr(packed)]
pub struct GimbalFrame {
    pitch: f32,
    yaw: f32,
    distance: f32,
}

#[derive(IntoBytes, Immutable, Default, Clone)]
#[repr(packed)]
pub struct TwistFrame {
    linear: [f32; 2],
    #[expect(unused)]
    angular_z: f32,
}

#[derive(IntoBytes, Immutable, Clone)]
#[repr(packed)]
pub struct FlagsFrame {
    fire_advice: bool,
    spin_advice: bool,
    #[expect(unused)]
    follow_gimbal_advice: bool,
}

#[derive(FromBytes, Immutable, KnownLayout)]
#[repr(packed)]
pub struct SerialReceiveFrame {
    pub mode: u8,
    pub roll: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub game_status: u8,
    pub bullet_speed: f32,
    pub hp: u16,
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
impl Default for FlagsFrame {
    fn default() -> Self {
        Self {
            fire_advice: Default::default(),
            spin_advice: Default::default(),
            follow_gimbal_advice: Default::default(),
        }
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

impl UpdateFrom<(GimbalCmd, MessageInfo)> for SerialSendFrame {
    fn update_from(&mut self, (value, _): (GimbalCmd, MessageInfo)) {
        let gimbal = &mut self.gimbal;
        gimbal.pitch = value.pitch as f32;
        gimbal.yaw = value.yaw as f32;
        gimbal.distance = value.distance as f32;
        self.flags.fire_advice = value.fire_advice;
        // self.flags.spin_advice = value.distance >= 0.0;
    }
}

impl UpdateFrom<(Twist, MessageInfo)> for SerialSendFrame {
    fn update_from(&mut self, (value, _): (Twist, MessageInfo)) {
        let twist = &mut self.twist;
        twist.linear = [value.linear.x, value.linear.y].map(|x| x as f32);
    }
}

impl UpdateFrom<(BTState, MessageInfo)> for SerialSendFrame {
    fn update_from(&mut self, (value, _): (BTState, MessageInfo)) {
        self.flags.spin_advice = value.spin;
    }
}
