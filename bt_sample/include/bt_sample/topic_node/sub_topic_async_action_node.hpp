#pragma once

// Standard headers
#include <chrono>
#include <cmath>
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

// StatefulAction node 이기에 tick의 반환값은 RUNNING, SUCCESS, FAILURE
// StatefulActionNode는 상태를 가지고 있기 때문에, onHalted, onRunning, onStart 함수를 구현해야 한다.
// onHalted: 노드가 다른 노드에 의해 중지되었을 때 호출되는 함수
// onRunning: 노드가 실행 중일 때 호출되는 함수
// onStart: 노드가 처음 실행될 때 호출되는 함수
// 이 함수들은 Behavior Tree의 노드 상태를 변경하기 위해 사용된다.
// ROS2의 액션 클라이언트로 동작하는 노드를 구현할 때 사용된다.
// 즉, 행위가 긴 액션을 수행하는 노드를 구현할 때 사용된다.

// 비동기적 구현이 가능하여, 로봇이 동작하는 동안 로봇의 상태를 체크하고, 이에 따라 다른 노드를 실행하거나 중지하는
// 노드를 구현할 때 사용될 수 있다.

class SubTopicAsyncActionNode : public BT::StatefulActionNode
{
public:
    // 생성자: Behavior Tree 노드와 ROS 2 노드를 초기화
    SubTopicAsyncActionNode(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
        : BT::StatefulActionNode(name, config)
        , node_ptr_{ node_ptr }
    {
        subscription_ = node_ptr_->create_subscription<std_msgs::msg::Int32>(
            "/topic", 10, std::bind(&SubTopicAsyncActionNode::topic_callback, this, _1));
        publisher_ = node_ptr_->create_publisher<std_msgs::msg::Int32>("/camera/detected", 10);
    }

    // 해당 노드의 초기화 함수
    BT::NodeStatus onStart() override
    {

        if (!node_ptr_)
        {
            std::cout << "ROS2 node not registered via init() method" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // 메시지가 감지되었는지 여부에 따라 Behavior Tree의 실행 결과를 반환
        flag = true;
        detected = false;
        std::string msg;
        getInput<std::string>("message", msg);
        //       RCLCPP_INFO(node_ptr_->get_logger(), "message: ", msg);
        return BT::NodeStatus::RUNNING;
    }

    // 해당 노드의 실행 함수
    BT::NodeStatus onRunning() override
    {
        if (detected == true)
        {
            detected = false;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    // 해당 노드가 다른 노드에 의해 중지되었을 때 호출되는 함수
    void onHalted() override
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Halted");
    }

    // Behavior Tree 노드에서 사용할 수 있는 포트 목록을 제공
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("message") }; // 이 예제에서는 string 타입의 입력 포트를 제공
    }

private:
    // ROS 2 토픽 메시지 콜백 함수
    void topic_callback(const std_msgs::msg::Int32::SharedPtr _msg)
    {
        // 받은 메시지를 바탕으로 로직 처리
        std_msgs::msg::Int32 tmp;
        tmp.data = _msg->data;    // 받은 데이터를 임시 메시지에 저장
        publisher_->publish(tmp); // 임시 메시지를 발행
        // detected = true;          // 메시지가 감지되었음을 표시
        if (flag == true)
        {
            detected = true;
            flag = false;
        }
    }

    // ROS 2 노드 객체
    rclcpp::Node::SharedPtr node_ptr_;
    // ROS 2 구독자 및 발행자 객체
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

    // 메시지가 감지되었는지 여부를 나타내는 플래그
    bool detected = false;
    bool flag = false;
};