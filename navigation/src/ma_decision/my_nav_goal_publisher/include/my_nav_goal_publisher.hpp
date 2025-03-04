#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace std::chrono_literals;
namespace bt = BT; // Behavior Tree namespace

class NavigateToGoalNode : public bt::AsyncActionNode
{
public:
  NavigateToGoalNode(const std::string& name, const BT::NodeConfiguration& config)
    : AsyncActionNode(name, config), client_ptr_(nullptr), goal_reached_(false)
  {
    node_ = rclcpp::Node::make_shared("navigate_to_goal_node");
    client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      node_, "navigate_to_pose");

    // 从行为树节点的配置中获取目标位置
    getInput("goal_x", goal_x_);
    getInput("goal_y", goal_y_);
    getInput("goal_orientation", goal_orientation_);
  }

  virtual BT::NodeStatus tick() override
  {
    if (!client_ptr_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
      return BT::NodeStatus::FAILURE;
    }

    // 如果目标还未到达，发送新的目标
    if (!goal_reached_) {
      // 设置目标位置
      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = node_->now();
      goal_msg.pose.pose.position.x = goal_x_;  // 使用从 XML 获取的目标位置
      goal_msg.pose.pose.position.y = goal_y_;
      goal_msg.pose.pose.orientation.w = goal_orientation_; // 使用从 XML 获取的朝向

      RCLCPP_INFO(node_->get_logger(), "Sending goal to navigate");

      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.goal_response_callback =
        std::bind(&NavigateToGoalNode::goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback =
        std::bind(&NavigateToGoalNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback =
        std::bind(&NavigateToGoalNode::result_callback, this, std::placeholders::_1);

      client_ptr_->async_send_goal(goal_msg, send_goal_options);

      // 目标尚未到达，返回运行状态
      //return BT::NodeStatus::RUNNING;
      return BT::NodeStatus::SUCCESS;
    }

    // 如果目标已经到达，返回成功
    return BT::NodeStatus::SUCCESS;
  }

  // 目标响应的回调函数
  void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      this->setStatus(BT::NodeStatus::FAILURE); // 目标被拒绝，失败
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  // 反馈回调函数
  void feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>,
                         const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(node_->get_logger(), "Current progress: %.2f", feedback->distance_remaining);
  }

  // 结果回调函数
  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "Goal reached successfully!");
        goal_reached_ = true; // 设置目标已到达标志
        this->setStatus(BT::NodeStatus::SUCCESS);  // 成功
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
        this->setStatus(BT::NodeStatus::FAILURE); // 被中止，失败
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(node_->get_logger(), "Goal was canceled");
        this->setStatus(BT::NodeStatus::FAILURE); // 被取消，失败
        break;
      default:
        RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
        this->setStatus(BT::NodeStatus::FAILURE); // 未知错误，失败
        break;
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("goal_x"),
      BT::InputPort<double>("goal_y"),
      BT::InputPort<double>("goal_orientation")
    };
  }

private:
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::Node::SharedPtr node_;

  double goal_x_, goal_y_, goal_orientation_;
  bool goal_reached_;  // 目标到达标志
};

// 注册 NavigateToGoalNode
void registerNavigateToGoalNode(bt::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<NavigateToGoalNode>("NavigateToGoal");
}
