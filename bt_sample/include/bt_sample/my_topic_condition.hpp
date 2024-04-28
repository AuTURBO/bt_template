#pragma once

// ROS2 headers
#include <std_msgs/msg/int32.hpp>

// Project headers
#include "topic_node/general_topic_condition_node.hpp"

/**
 * @brief A condition node that checks if the topic value is positive.
 */
class IsTopicPositive : public GeneralTopicCondition
{
public:
    /**
     * @brief Constructor for the IsTopicPositive class.
     * @param name The name of the topic.
     * @param config The configuration for the BT node.
     * @param node A shared pointer to the ROS 2 node.
     */
    IsTopicPositive(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
        : GeneralTopicCondition(name, config, node)
    {
        this->AddTopicCondition<std_msgs::msg::Int32>(
            "/topic", [](std::shared_ptr<std_msgs::msg::Int32> msg) { return msg->data > 0; });
    }
};

/**
 * @brief A condition node that checks if the topic value is bigger than 10.
 */
class IsTopicBiggerThan10 : public GeneralTopicCondition
{
public:
    /**
     * @brief Constructor for IsTopicBiggerThan10.
     * @param name The name of the topic.
     * @param config The configuration for the BT node.
     * @param node A shared pointer to the ROS 2 node.
     */
    IsTopicBiggerThan10(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
        : GeneralTopicCondition(name, config, node)
    {
        this->AddTopicCondition<std_msgs::msg::Int32>(
            "/topic", [](std::shared_ptr<std_msgs::msg::Int32> msg) { return msg->data > 10; });
    }
};