#include "behaviortree_cpp_v3/behavior_tree.h"

class RepeatUntilFail : public BT::ControlNode
{
public:
    RepeatUntilFail(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ControlNode(name, config)
         {

         }

    static BT::PortsList providedPorts()
    {
        return {};  // 不需要任何输入或输出端口
    }

    BT::NodeStatus tick() override
    {
        // 确保至少有一个子节点
        if (childrenCount() == 0)
        {
            return BT::NodeStatus::FAILURE;  // 没有子节点，返回失败
        }

        // 获取所有子节点，children() 返回的是 std::vector<BT::TreeNode*>
        const std::vector<BT::TreeNode*>& children = this->children();  // 保持类型一致

        // 获取第一个子节点
        auto* child_node = children[0];  // 使用指针类型

        // 执行子节点，直到它返回 FAILURE
        rclcpp::Rate rate(0.5);  // 设置每秒执行0.5次
        while (rclcpp::ok())
        {
            BT::NodeStatus status = child_node->executeTick();

            // 如果子节点返回 FAILURE，表示完成
            if (status == BT::NodeStatus::FAILURE)
            {
                return BT::NodeStatus::SUCCESS;
            }

            // 如果子节点返回 RUNNING，继续执行
            if (status == BT::NodeStatus::RUNNING)
            {
                return BT::NodeStatus::RUNNING;
            }
            rate.sleep();  // 等待下一次 tick
            

        }
        // 如果 ROS 被关闭，则返回 FAILURE,有这个才能cral+c
        return BT::NodeStatus::FAILURE;
    }

};

