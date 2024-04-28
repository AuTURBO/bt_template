#pragma once

// standard headers
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

// BehaviorTree.CPP headers
#include <behaviortree_cpp/condition_node.h>

using namespace std::chrono_literals;

class GeneralTopicCondition : public BT::ConditionNode
{
public:
    GeneralTopicCondition(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
        : BT::ConditionNode(name, config)
        , node_(node)
    {
    }

    // Behavior Tree 노드의 실행 함수
    BT::NodeStatus tick() override
    {
        if (!node_)
        {
            RCLCPP_ERROR(node_->get_logger(), "ROS2 node not registered via init() method");
            return BT::NodeStatus::FAILURE;
        }

        for (auto &topic_condition : topic_condition_map_)
        {
            if (!topic_condition.second())
            {
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::SUCCESS;
    }

    // Behavior Tree 노드에서 사용할 수 있는 포트 목록을 제공
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("message") };
    }

protected:
    /**
     * Adds a topic condition to the GeneralTopicCondition node.
     *
     * @tparam TopicMsgType The ROS2 message type of the topic.
     * @param topic_name The name of the topic to subscribe to.
     * @param condition A function that takes a shared pointer to the topic message type and returns a boolean value
     * indicating whether the condition is met.
     * @param timeout The timeout for subscribing to the topic. Default is 1000ms.
     * @return A reference to the GeneralTopicCondition node.
     */
    template <typename TopicMsgType>
    GeneralTopicCondition &AddTopicCondition(const std::string &topic_name,
                                             std::function<bool(std::shared_ptr<TopicMsgType>)> condition,
                                             std::chrono::milliseconds timeout = 1000ms)
    {
        topic_condition_map_[topic_name] = []() { return false; };

        sub_ = node_->create_subscription<TopicMsgType>(
            topic_name, 10, [this, topic_name, condition, timeout](std::shared_ptr<TopicMsgType> msg) {
                const auto sub_time = std::chrono::system_clock::now();

                topic_condition_map_[topic_name] = [this, topic_name, condition, sub_time, timeout, msg]() {
                    return (condition(msg) && (std::chrono::system_clock::now() - sub_time) < timeout);
                };
            });

        return *this;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::SubscriptionBase::SharedPtr sub_;
    std::unordered_map<std::string, std::function<bool()>> topic_condition_map_;
};