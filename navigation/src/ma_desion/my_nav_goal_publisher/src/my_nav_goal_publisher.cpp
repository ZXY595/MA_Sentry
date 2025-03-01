// #include <chrono>
// #include <memory>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"

// using namespace std::chrono_literals;

// class NavGoalPublisher : public rclcpp::Node
// {
// public:
//   using NavigateToPose = nav2_msgs::action::NavigateToPose;
//   using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

//   NavGoalPublisher()
//   : Node("nav_goal_publisher")
//   {
//     // 创建导航到目标点的 action 客户端
//     this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

//     // 等待服务器启动
//     if (!this->client_ptr_->wait_for_action_server(10s)) {
//       RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
//       rclcpp::shutdown();
//     }

//     // 发送目标点
//     this->send_goal();
//   }

// private:
//   rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

//   void send_goal()
//   {
//     auto goal_msg = NavigateToPose::Goal();
//     goal_msg.pose.header.frame_id = "map";
//     goal_msg.pose.header.stamp = this->now();
    
//     // 设置目标位置和朝向
//     goal_msg.pose.pose.position.x = 4.0;
//     goal_msg.pose.pose.position.y = 2.0;
//     goal_msg.pose.pose.orientation.w = 1.0;

//     RCLCPP_INFO(this->get_logger(), "Sending goal");

//     auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//     send_goal_options.goal_response_callback = std::bind(&NavGoalPublisher::goal_response_callback, this, std::placeholders::_1);
//     send_goal_options.feedback_callback = std::bind(&NavGoalPublisher::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
//     send_goal_options.result_callback = std::bind(&NavGoalPublisher::result_callback, this, std::placeholders::_1);

//     // 发送目标
//     this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
//   }

//   // 修改后的回调函数
//   void goal_response_callback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
//   {
//     if (!goal_handle) {
//       RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//     } else {
//       RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
//     }
//   }

//   void feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
//   {
//     RCLCPP_INFO(this->get_logger(), "Current progress: %.2f", feedback->distance_remaining);
//   }

//   void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
//   {
//     switch (result.code) {
//       case rclcpp_action::ResultCode::SUCCEEDED:
//         RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
//         break;
//       case rclcpp_action::ResultCode::ABORTED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//         break;
//       case rclcpp_action::ResultCode::CANCELED:
//         RCLCPP_WARN(this->get_logger(), "Goal was canceled");
//         break;
//       default:
//         RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//         break;
//     }
//     rclcpp::shutdown();
//   }
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<NavGoalPublisher>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


