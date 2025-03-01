// Copyright 2021 RoboMaster-OSS
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

#include "rmoss_base/simple_robot_base_node.hpp"

#include <thread>
#include <memory>

#include "rmoss_base/uart_transporter.hpp"
#include "rmoss_base/udp_transporter.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace rmoss_base
{

namespace protocol_example
{
typedef enum : unsigned char
{
  GimbalAngleControl = 0x01,
  ChangeMode = 0xa1
} ProtocolExample;
}

SimpleRobotBaseNode::SimpleRobotBaseNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("simple_robot_base", options);
  // init
  auto transporter = std::make_shared<rmoss_base::UartTransporter>("/dev/ttyACM0");
  packet_tool_ = std::make_shared<FixedPacketTool<16>>(transporter);
  //// sub
  // cmd_gimbal_sub_ = node_->create_subscription<rmoss_interfaces::msg::GimbalCmd>(
  //   "cmd_gimbal", 10,
  //   std::bind(&SimpleRobotBaseNode::gimbal_cmd_cb, this, std::placeholders::_1));
  cmd_vel_sub_= node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_chassis",10,
     std::bind(&SimpleRobotBaseNode::cmd_vel_cb, this, std::placeholders::_1) );//chassis
  
 
  // task thread
  //listen_thread_ = std::make_unique<std::thread>(&SimpleRobotBaseNode::listen_loop, this);
}


void SimpleRobotBaseNode::cmd_vel_cb(const geometry_msgs::msg::Twist msg)
{
  FixedPacket<16> packet;
  uint8_t head = 0x5A;
  float x = msg.linear.x;
  float y = msg.linear.y;
  float z = msg.angular.z;
  uint8_t is_navigating = true;
  //uint16_t check_sum = (uint32_t)head + *(uint32_t*)&x + *(uint32_t*)&y + *(uint32_t*)&z + (uint32_t)is_navigating;
  uint16_t check_sum = 0;

  packet.load_data(head, 1); // head
  packet.load_data(x,2); // x
  packet.load_data(y,6); // y
  packet.load_data(z,10); // angular
  packet.load_data(is_navigating,14); // navigating
  packet.load_data(check_sum,15); // checksum
  if (!packet_tool_->send_packet(packet)) {
    RCLCPP_ERROR(node_->get_logger(),"Fail to send nav packet!");
  }
  
   //RCLCPP_INFO(node_->get_logger(),"serial send data  : x == %f , y== %f",x,y);

  // delay for data send.
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

// void SimpleRobotBaseNode::gimbal_cmd_cb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
// {
//   FixedPacket<16> packet;
//   packet.load_data<unsigned char>(protocol_example::GimbalAngleControl, 1);
//   packet.load_data<unsigned char>(0x00, 2);
//   packet.load_data<float>(msg->linear.x,3);
//   packet.load_data<float>(msg->linear.y,7);
//   packet.load_data<float>(msg->angular.z,11);
//   packet_tool_->send_packet(packet);
//   // delay for data send.
//   std::this_thread::sleep_for(std::chrono::milliseconds(5));
// }

/*void SimpleRobotBaseNode::listen_loop()
{
  FixedPacket<16> packet;
  while (rclcpp::ok()) {
    if (packet_tool_->recv_packet(packet)) {
      uint8_t head;
      uint16_t sentry_hp;
      uint16_t bullet;
      uint16_t checksum;
      packet.unload_data(head, 1);
      packet.unload_data(sentry_hp, 2);
      packet.unload_data(bullet, 4);
      packet.unload_data(checksum, 6);
      RCLCPP_INFO(node_->get_logger(),"Get packet from microcontroler: head: %d, hp: %d, bullet: %d, checksum: %d", head, sentry_hp, bullet, checksum);
    }
  }
}
*/
}  // namespace rmoss_base

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmoss_base::SimpleRobotBaseNode)
