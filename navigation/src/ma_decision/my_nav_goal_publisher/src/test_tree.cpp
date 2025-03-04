#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "my_nav_goal_publisher.hpp"
#include "check_ammunition_count.hpp"
#include "check_ammunition_count_2.hpp"
#include "repeat_until_fail.hpp"
#include <unistd.h>  // 用于检查文件是否存在

int main(int argc, char ** argv)
{
  // 初始化 ROS 2
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("behavior_tree"), "ROS 2 Node initialized.");

  // 创建行为树工厂
  bt::BehaviorTreeFactory factory;

  // 注册 NavigateToGoal 节点，确保自定义节点已被注册
  RCLCPP_INFO(rclcpp::get_logger("behavior_tree"), "Registering NavigateToGoalNode.");
  factory.registerNodeType<NavigateToGoalNode>("NavigateToGoal");
  factory.registerNodeType<CheckAmmunitionCount>("CheckAmmunitionCount");
  factory.registerNodeType<CheckAmmunitionCount_2>("CheckAmmunitionCount_2");
  factory.registerNodeType<RepeatUntilFail>("RepeatUntilFail");

  // 加载行为树 XML 配置文件，指定行为树结构
  std::string tree_file = "/home/ma/MA_Sentry/navigation/src/ma_decision/my_nav_goal_publisher/config/test_tree.xml";
  
  // 检查 XML 文件是否存在
  if (access(tree_file.c_str(), F_OK) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("behavior_tree"), "Behavior tree file not found: %s", tree_file.c_str());
    return 1;
  }

  // 加载行为树
  RCLCPP_INFO(rclcpp::get_logger("behavior_tree"), "Loading behavior tree from file: %s", tree_file.c_str());
  bt::Tree tree = factory.createTreeFromFile(tree_file);
  
  // 检查是否成功加载行为树
  if (tree.rootNode() == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("behavior_tree"), "The behavior tree does not have a root node.");
    return 1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("behavior_tree"), "Behavior tree successfully loaded.");
  }

  // 输出当前树的根节点名称
  if (tree.rootNode() != nullptr) {
    RCLCPP_INFO(rclcpp::get_logger("behavior_tree"), "Root Node: %s", tree.rootNode()->name().c_str());
  }

  // 创建一个 ROS 2 节点来运行行为树
  auto node = rclcpp::Node::make_shared("behavior_tree_node");

  // 定期执行行为树
  rclcpp::Rate rate(0.05);  // 设置每秒执行0.05次
  while (rclcpp::ok()) {
    RCLCPP_INFO(rclcpp::get_logger("behavior_tree"), "Starting to tick behavior tree.");
    BT::NodeStatus status = tree.tickRoot();  // 执行行为树根节点
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("behavior_tree"), "Behavior tree execution succeeded.");
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(rclcpp::get_logger("behavior_tree"), "Behavior tree execution failed.");
      break;  // 如果执行失败，则退出循环
    } else if (status == BT::NodeStatus::RUNNING) {
      RCLCPP_INFO(rclcpp::get_logger("behavior_tree"), "Behavior tree is running.");
    }
    rate.sleep();  // 等待下一次 tick
  }

  rclcpp::shutdown();
  return 0;
}
