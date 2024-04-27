#pragma once

// ROS2 headers
#include <std_msgs/msg/int32.hpp>

// Project headers
#include "bt_sample/topic_pub_condition_node.hpp"

class PositiveIntPubCondition : public TopicPubConditionNode<std_msgs::msg::Int32>
{
public:
    PositiveIntPubCondition(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node,
                            const std::string &topic_name, std::chrono::milliseconds timeout = 1000ms)
        : TopicPubConditionNode<std_msgs::msg::Int32>(name, config, node, topic_name, timeout)
        , node_(node)
    {
    }

protected:
    bool OnTopicReceived(void) override
    {
        const std_msgs::msg::Int32 msg = this->GetMessage();

        if (msg.data > 0)
        {
            RCLCPP_INFO(node_->get_logger(), "Received message: %d", msg.data);
            return true;
        }

        return false;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
};