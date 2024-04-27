#include "std_msgs/msg/int32.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::chrono::milliseconds;
using std::placeholders::_1;

using namespace BT;
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

class TopicDetectedCondition : public BT::ConditionNode
{
  private:
    // ROS 2 노드 객체
    rclcpp::Node::SharedPtr node_ptr_;
    // ROS 2 구독자 및 발행자 객체
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

    // 메시지가 감지되었는지 여부를 나타내는 플래그
    bool detected = false;

    // ROS 2 토픽 메시지 콜백 함수
    void topic_callback(const std_msgs::msg::Int32::SharedPtr _msg)
    {
        // 받은 메시지를 바탕으로 로직 처리

        std_msgs::msg::Int32 tmp;
        tmp.data = _msg->data;    // 받은 데이터를 임시 메시지에 저장
        publisher_->publish(tmp); // 임시 메시지를 발행
        detected = true;          // 메시지가 감지되었음을 표시
    }

  public:
    // 생성자: Behavior Tree 노드와 ROS 2 노드를 초기화
    TopicDetectedCondition(const std::string &name, const NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
        : BT::ConditionNode(name, config), node_ptr_{node_ptr}
    {
        // ROS 2 구독자 및 발행자 초기화
        subscription_ = node_ptr_->create_subscription<std_msgs::msg::Int32>(
            "/topic", 10, std::bind(&TopicDetectedCondition::topic_callback, this, _1));
        publisher_ = node_ptr_->create_publisher<std_msgs::msg::Int32>("/topic/detected", 10);
    }

    // Behavior Tree 노드의 실행 함수
    NodeStatus tick() override
    {
        if (!node_ptr_)
        {
            // std::cout << "ROS2 node not registered via init() method" << std::endl;
            RCLCPP_ERROR(node_ptr_->get_logger(), "ROS2 node not registered via init() method");
            return BT::NodeStatus::FAILURE;
        }

        if (detected == true)
        {
            detected = false;
            return NodeStatus::SUCCESS;
        }
        else
        {
            return NodeStatus::FAILURE;
        }
    }

    // Behavior Tree 노드에서 사용할 수 있는 포트 목록을 제공
    static PortsList providedPorts()
    {
        const char *description = "Simply print the target on console...";
        return {InputPort<std::string>("message")}; // 이 예제에서는 string 타입의 입력 포트를 제공
    }
};