#pragma once

// standard headers
#include <chrono>
#include <future>
#include <memory>
#include <string>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>

// BehaviorTree.CPP headers
#include <behaviortree_cpp/condition_node.h>

using namespace std::chrono_literals;

template <typename TopicMsgType>
class SubTopicConditionNode : public BT::ConditionNode
{
public:
    /**
     *
     * 특정 ROS2 topic 메시지에 대한 subscriber를 생성하고 해당 토픽이 발행되면 SUCCESS를 반환하는 Condition node를
     * 생성합니다. 지정된 시간(timeout) 내에 메시지가 수신되지 않으면 FAILURE를 반환합니다.
     *
     * @tparam TopicMsgType ROS2 message type of the topic
     *
     * @param name Behavior Tree 노드의 이름
     * @param config Behavior Tree 노드의 구성 설정
     * @param node ROS 2 노드의 shared pointer
     * @param topic_name 발행을 확인할 ROS2 topic 이름
     * @param timeout 타임아웃 값 (기본값: 1000ms)
     */
    SubTopicConditionNode(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node,
                          const std::string &topic_name, std::chrono::milliseconds timeout = 1000ms)
        : BT::ConditionNode(name, config)
        , node_(node)
        , timeout_(timeout)
    {
        this->setPreTickFunction(std::bind(&SubTopicConditionNode::CreateSubscription, this, topic_name));
        this->setPostTickFunction(std::bind(&SubTopicConditionNode::DestroySubscription, this));
    }

    // Behavior Tree 노드의 실행 함수
    BT::NodeStatus tick() override
    {
        if (!node_)
        {
            RCLCPP_ERROR(node_->get_logger(), "ROS2 node not registered via init() method");
            return BT::NodeStatus::FAILURE;
        }

        // rclcpp::spin_some(node_);

        if (future_.wait_for(timeout_) == std::future_status::timeout)
        {
            RCLCPP_ERROR(node_->get_logger(), "Timeout while waiting for message");
            return BT::NodeStatus::FAILURE;
        }

        if (!OnTopicReceived())
        {
            RCLCPP_ERROR(node_->get_logger(), "OnTopicReceived() returned false");
            return BT::NodeStatus::FAILURE;
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
     * 본 함수는 topic 수신시 호출되어 bool 값을 반환합니다.
     * 본 함수를 override하여 원하는 조건을 구현할 수 있습니다.
     *
     * @return bool 기본값은 true입니다.
     */
    virtual bool OnTopicReceived(void)
    {
        return true;
    }
    /**
     * 수신한 topic 메시지를 반환합니다.
     *
     * @return TopicMsgType 수신한 topic 메시지
     */
    const TopicMsgType &GetMessage(void) const
    {
        return *msg_;
    }

private:
    [[maybe_unused]] BT::NodeStatus CreateSubscription(const std::string &topic_name)
    {
        promise_ = std::promise<void>();
        future_ = promise_.get_future();
        sub_ =
            node_->create_subscription<TopicMsgType>(topic_name, 10, [this, &topic_name](std::shared_ptr<TopicMsgType> msg) {
                msg_ = msg;
                promise_.set_value();
            });
    }

    [[maybe_unused]] BT::NodeStatus DestroySubscription(void)
    {
        sub_.reset();
        msg_.reset();
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::Subscription<TopicMsgType>> sub_;
    std::shared_ptr<TopicMsgType> msg_;
    std::chrono::milliseconds timeout_;
    std::promise<void> promise_;
    std::future<void> future_;
};

template <typename T>
using SimpleSubTopicCondition = SubTopicConditionNode<T>;