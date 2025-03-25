// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_serial_driver/protocol/sentry_protocol.hpp"
// ros2
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>

namespace fyt::serial_driver::protocol {
ProtocolSentry::ProtocolSentry(std::string_view port_name, bool enable_data_print) {
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name));
  packet_tool_ = std::make_shared<FixedPacketTool<32>>(uart_transporter);
  packet_tool_->enbaleDataPrint(enable_data_print);
}

void ProtocolSentry::send(const rm_interfaces::msg::GimbalCmd &data) {
  packet_.loadData<unsigned char>(data.fire_advice ? FireState::Fire : FireState::NotFire, 1);
  // packet_.loadData<unsigned char>(FireState::NotFire, 1);
  // is_spin
  // packet_.loadData<unsigned char>(0x00, 2);
  // gimbal control
  packet_.loadData<float>(static_cast<float>(data.pitch), 2);
  // FYT_DEBUG("serail send","pitch {}",data.pitch);
  packet_.loadData<float>(static_cast<float>(data.yaw), 6);
  packet_.loadData<float>(static_cast<float>(data.distance), 10);
  // // chassis control
  // // linear x
  // packet_.loadData<float>(0, 16);
  // // linear y
  // packet_.loadData<float>(0, 20);
  // // angular z
  // packet_.loadData<float>(0, 24);
  // // useless data
  // packet_.loadData<float>(0, 28);
  // 视觉
  packet_.loadData<unsigned char>(0x02, 14);
  packet_tool_->sendPacket(packet_);
}

void ProtocolSentry::send(const geometry_msgs::msg::Twist &msg) {

  float x = msg.linear.x;
  float y = msg.linear.y;
  float z = msg.angular.z;
  uint8_t spin = 1;
  //uint16_t check_sum = (uint32_t)head + *(uint32_t*)&x + *(uint32_t*)&y + *(uint32_t*)&z + (uint32_t)is_navigating;
  //uint16_t check_sum = 0;

  // packet_.loadData<unsigned char>(0x00, 1);
  // is_spin
  //packet_.loadData<unsigned char>(data.is_spining ? 0x01 : 0x00, 2);
  //packet_.loadData<unsigned char>(data.is_navigating ? 0x01 : 0x00, 3);
 
  packet_.loadData<unsigned char>(spin, 1);
  // gimbal control
  // packet_.loadData<float>(0, 4);
  // packet_.loadData<float>(0, 8);
  // packet_.loadData<float>(0, 12);
  // chassis control
  // linear x
  packet_.loadData<float>(x, 2);
  // linear y
  packet_.loadData<float>(y, 6);
  // if (x == 0.0 && y == 0.0) {
  //   FYT_WARN("serial_driver","Sending a stop speed!");
  // }
  // angular z
  packet_.loadData<float>(z, 10);
  // useless data
  // packet_.loadData<float>(0, 28);
  // 导航
  packet_.loadData<unsigned char>(0x01, 14);

  packet_tool_->sendPacket(packet_);
}

void ProtocolSentry::send() {
  const auto& vision_data = this->vision_data_;
  const auto& nav_data = this->nav_data_;

  packet_.loadData<unsigned char>(vision_data.fire_advice ? FireState::Fire : FireState::NotFire, 1);
  packet_.loadData<float>(static_cast<float>(vision_data.pitch), 2);
  packet_.loadData<float>(static_cast<float>(vision_data.yaw), 6);
  packet_.loadData<float>(static_cast<float>(vision_data.distance), 10);
 
  float x = nav_data.linear.x;
  float y = nav_data.linear.y;
  float z = nav_data.angular.z;
  uint8_t spin = 1;
 
  packet_.loadData<unsigned char>(spin, 14);
  // linear x
  packet_.loadData<float>(x, 15);
  // linear y
  packet_.loadData<float>(y, 19);
  // angular z
  packet_.loadData<float>(z, 23);

  packet_tool_->sendPacket(packet_);
}

bool ProtocolSentry::receive(rm_interfaces::msg::SerialReceiveData &data) {
  FixedPacket<32> packet;
  if (packet_tool_->recvPacket(packet)) {
     // game status
    packet.unloadData(data.mode, 1);
    
    // FYT_INFO("serial_node","enemy_color = {}",enemy_color);
    packet.unloadData(data.roll, 2);
    packet.unloadData(data.pitch, 6);
    packet.unloadData(data.yaw, 10);

    packet.unloadData(data.judge_system_data.game_status, 14);
    packet.unloadData(data.judge_system_data.ammo, 15);
    packet.unloadData(data.judge_system_data.hp, 17);
    // packet.unloadData(data.pitch, 2);
    // packet.unloadData(data.yaw, 6);
    // // 实际上是底盘角度
    // // packet.unloadData(data.chassis_yaw, 10);
    // // blood
    //packet.unloadData(data.judge_system_data.blood, 16);
    // // remaining time
    // packet.unloadData(data.judge_system_data.remaining_time, 16);
    // // outpost hp
    // packet.unloadData(data.judge_system_data.outpost_hp, 20);
    // // operator control message
    // packet.unloadData(data.judge_system_data.operator_command.is_outpost_attacking, 22);
    // packet.unloadData(data.judge_system_data.operator_command.is_retreating, 23);
    // packet.unloadData(data.judge_system_data.operator_command.is_drone_avoiding, 24);

    // packet.unloadData(data.judge_system_data.game_status, 25);

    // data.bullet_speed = 24;

    //auto ammunition_msg = std_msgs::msg::Int32();
    //ammunition_msg.data = 


    return true;
  } else {
    return false;
  }
}

std::vector<rclcpp::SubscriptionBase::SharedPtr> ProtocolSentry::getSubscriptions(
  rclcpp::Node::SharedPtr node) {
  auto sub1 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "armor_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { 
      this->vision_data_ = *msg;
      this->send(); 
    });
  auto sub2 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "rune_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { 
      this->vision_data_ = *msg;
      this->send(); 
    });
  //auto sub3 = node->create_subscription<rm_interfaces::msg::ChassisCmd>(
  //  "cmd_vel_chassis",
  //  rclcpp::SensorDataQoS(),
  //  [this](const rm_interfaces::msg::ChassisCmd::SharedPtr msg) { this->send(*msg); });
  auto sub3= node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_chassis",
     rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { 
      this->nav_data_ = *msg;
      this->send(); 
    });

  //rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ammunition_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/my_ammunition",10);

  return {sub1, sub2, sub3};
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> ProtocolSentry::getClients(
  rclcpp::Node::SharedPtr node) const {
  auto client1 = node->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode",
                                                                  rmw_qos_profile_services_default);
  auto client2 = node->create_client<rm_interfaces::srv::SetMode>("armor_solver/set_mode",
                                                                  rmw_qos_profile_services_default);
  return {client1, client2};
}
}  // namespace fyt::serial_driver::protocol
