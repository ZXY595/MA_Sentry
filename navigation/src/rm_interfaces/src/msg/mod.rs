use serde::{Deserialize, Serialize};

use crate::std_msgs;

#[derive(Debug, Serialize, Deserialize)]
pub struct SerialReceiveData {
    pub header: std_msgs::msg::Header,
    pub mode: u8,
    pub bullet_speed: f32,
    pub roll: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub judge_system_data: JudgeSystemData,
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct JudgeSystemData {
    pub game_status: u8,
    pub remaining_time: i16,
    pub hp: u16,
    pub outpost_hp: u16,
    pub ammo: u16,
    pub operator_command: OperatorCommand,
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct OperatorCommand {
    pub is_retreating: u8,
    pub is_drone_avoiding: u8,
    pub is_outpost_attacking: u8,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct GimbalCmd {
    pub header: std_msgs::msg::Header,
    pub pitch: f64,
    pub yaw: f64,
    pub yaw_diff: f64,
    pub pitch_diff: f64,
    pub distance: f64,
    pub fire_advice: bool,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct BTState {
    pub spin: bool,
}
