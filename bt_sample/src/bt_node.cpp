// Standard headers
#include <chrono>
#include <filesystem>
#include <random>
#include <string>

// ROS2 headers
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

// BehaviorTree.CPP headers
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>

// Project headers
#include "bt_sample/my_topic_condition.hpp"
#include "bt_sample/topic_node/general_topic_condition_node.hpp"
#include "bt_sample/topic_node/sub_topic_async_action_node.hpp"
#include "bt_sample/topic_node/sub_topic_condition_node.hpp"
#include "bt_sample/topic_node/sub_topic_sync_action_node.hpp"

using namespace std::chrono_literals;

class ROS2Node : public rclcpp::Node
{
public:
    ROS2Node(std::string node_name)
        : rclcpp::Node(node_name)
    {
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto ros2_node = std::make_shared<ROS2Node>("bt_node");

    auto factory = BT::BehaviorTreeFactory();
    factory.registerNodeType<IsTopicPositive>("IsTopicPositive", ros2_node);
    factory.registerNodeType<IsTopicBiggerThan10>("IsTopicBiggerThan10", ros2_node);
    // factory.registerNodeType<SimpleSubTopicCondition<std_msgs::msg::Int32>>("SimpleSubTopic", ros2_node, "/topic", 2s);
    // factory.registerNodeType<SubTopicAsyncActionNode>("SubTopicAsyncActionNode", ros2_node);
    // factory.registerNodeType<SubTopicSyncActionNode>("SubTopicSyncActionNode", ros2_node);

    const auto default_bt_xml_file = ament_index_cpp::get_package_share_directory("bt_sample") + "/config/main_bt.xml";
    const auto default_bt_log_file = ament_index_cpp::get_package_share_directory("bt_sample") + "/log/bt_trace.btlog";

    auto blackboard = BT::Blackboard::create();
    auto tree = factory.createTreeFromFile(default_bt_xml_file, blackboard);

    auto groot2_publisher = std::make_unique<BT::Groot2Publisher>(tree, 5555);
    auto bt_file_logger = std::make_unique<BT::FileLogger2>(tree, default_bt_log_file);
    auto bt_cout_logger = std::make_unique<BT::StdCoutLogger>(tree);

    while (rclcpp::ok())
    {
        const auto &now = std::chrono::steady_clock::now();

        rclcpp::spin_some(ros2_node);
        tree.tickOnce();

        std::this_thread::sleep_until(now + 100ms);
    }

    rclcpp::shutdown();
    RCLCPP_INFO(ros2_node->get_logger(), "Shutting down.");
    return 0;
}