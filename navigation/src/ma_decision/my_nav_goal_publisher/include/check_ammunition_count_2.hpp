#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"

class CheckAmmunitionCount_2 : public BT::ConditionNode
{
public:
  CheckAmmunitionCount_2(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config)
  {
    // 创建一个ROS节点并设置参数
    node_1 = rclcpp::Node::make_shared("check_ammunition_count_2");
    node_1->declare_parameter<int>("ammunition_count", 30);  // 初始化弹丸数量

    t_s = node_1->now().seconds();

    // 创建订阅器来接收弹丸数量
    ammunition_subscriber_ = node_1->create_subscription<rm_interfaces::msg::SerialReceiveData>(
        "/my_ammunition", rclcpp::SensorDataQoS(),
        std::bind(&CheckAmmunitionCount_2::onAmmunitionReceived, this, std::placeholders::_1));

        // 创建一个线程来处理ROS事件循环
        spin_thread_ = std::thread([this]() {rclcpp::spin(node_1);});

    RCLCPP_INFO(node_1->get_logger(), "CheckAmmunitionCount_2 node created and subscribed to my_ammunition topic.");
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<int>("threshold", 30, "Minimum ammunition threshold")};
  }

  BT::NodeStatus tick() override
  {
    // 获取阈值
    int threshold;
    if (!getInput("threshold", threshold))
    {
      RCLCPP_ERROR(node_1->get_logger(), "Threshold not provided");
      return BT::NodeStatus::FAILURE;
    }

    // 如果弹丸数量尚未接收到，返回RUNNING
    if (!has_received_ammo_)
    {
      RCLCPP_INFO(node_1->get_logger(), "Waiting for ammunition data...");
      return BT::NodeStatus::RUNNING;  // 等待数据
      //return BT::NodeStatus::SUCCESS;  // 等待数据
    }

    RCLCPP_INFO(node_1->get_logger(), "Current ammunition count: %d", ammunition_count_);

    // 判断弹丸数量是否高于阈值
    if (ammunition_count_ >= threshold && blood_count_  >= 400)
    {
      RCLCPP_WARN(node_1->get_logger(), "Ammunition below threshold!");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

  ~CheckAmmunitionCount_2()
  {
    rclcpp::shutdown();
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
  }


private:
  rclcpp::Node::SharedPtr node_1;
  rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr ammunition_subscriber_;
  int ammunition_count_ = 0;  // 使用类成员变量来存储弹丸数量
  int blood_count_ = 500;
  double t=0;
  double t_s=0;
  bool has_received_ammo_ = false;  // 标记是否接收到弹丸数据
  std::thread spin_thread_; // 声明 spin_thread_ 变量

  // 订阅到弹丸数量后回调
  void onAmmunitionReceived(const rm_interfaces::msg::SerialReceiveData::SharedPtr msg)
  {
    //ammunition_count_ = msg->judge_system_data.ammo;
    //blood_count_ = msg->judge_system_data.hp;
    t = node_1->now().seconds();
    if(t-t_s >= 30)
    {
      ammunition_count_ = 500;
    }
    else 
    {
      ammunition_count_ = 0;
    }
    
    has_received_ammo_ = true;
    RCLCPP_INFO(node_1->get_logger(), "Received ammunition_2222 count: %d", ammunition_count_);
  }
};