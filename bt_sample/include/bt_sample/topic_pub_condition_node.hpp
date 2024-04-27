// Standard headers
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <memory>
#include <string>

// ROS2 headers
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

// BehaviorTree.CPP headers
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

// BehaviorTree.CPP에서 사용되는 네임스페이스와 타입 정의

// Condition node 이기에 tick의 반환값은 SUCCESS or FAILURE
// ConditionNode는 논리 연산을 수행하는 노드이다.
// 로봇의 상태를 확인하거나, 센서의 값을 확인하는 노드를 구현할 때 사용된다.
// 현재 로봇의 상태를 확인하고, 이에 따라 다른 노드를 실행하거나 중지하는 노드를 구현할 때 사용된다.
// ROS2의 토픽 메시지를 구독하는 노드를 구현할 때 사용된다.
// 즉, 특정 토픽 메시지를 구독하고, 이에 따라 다른 노드를 실행하거나 중지하는 노드를 구현할 때 사용된다.

// 비동기적 구현이 가능한 stateful action node와 달리, condition node는 동기적으로 구현된다.
// statful action에서 실행이 되는 동안 로봇의 상태를 확인하고, 이에 따라 다른 노드를 실행하거나 중지하는 노드를 구현하면
// 된다.

class TopicPubConditionNode : public BT::ConditionNode
{
public:
    // 생성자: Behavior Tree 노드와 ROS 2 노드를 초기화
    TopicPubConditionNode(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node,
                          std::chrono::milliseconds timeout = 1000ms)
        : BT::ConditionNode(name, config)
        , node_(node)
        , timeout_(timeout)
    {
        this->setPreTickFunction(std::bind(&TopicPubConditionNode::CreateSubscription, this));
        this->setPostTickFunction(std::bind(&TopicPubConditionNode::DestroySubscription, this));
    }

    // Behavior Tree 노드의 실행 함수
    BT::NodeStatus tick() override
    {
        if (!node_)
        {
            RCLCPP_ERROR(node_->get_logger(), "ROS2 node not registered via init() method");
            return BT::NodeStatus::FAILURE;
        }

        if (future_.wait_for(timeout_) == std::future_status::timeout)
        {
            RCLCPP_ERROR(node_->get_logger(), "Timeout while waiting for message");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS;
    }

    // Behavior Tree 노드에서 사용할 수 있는 포트 목록을 제공
    static BT::PortsList providedPorts()
    {
        const char *description = "Simply print the target on console...";
        return { BT::InputPort<std::string>("message") };
    }

private:
    BT::NodeStatus CreateSubscription(void)
    {
        promise_ = std::promise<void>();
        future_ = promise_.get_future();
        sub_ = node_->create_subscription<std_msgs::msg::Int32>("/topic", 10, [this](std_msgs::msg::Int32::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "Received: %d", msg->data);
            promise_.set_value();
        });
    }

    BT::NodeStatus DestroySubscription(void)
    {
        sub_.reset();
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    std::chrono::milliseconds timeout_;
    std::promise<void> promise_;
    std::future<void> future_;
};