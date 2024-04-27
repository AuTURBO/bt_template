// Standard headers
#include <chrono>
#include <filesystem>
#include <random>
#include <string>

// ROS2 headers
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// BehaviorTree.CPP headers
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>

// Project headers
#include "bt_sample/topic_detect_stateful_action.hpp"
#include "bt_sample/topic_detect_sync_action.hpp"
#include "bt_sample/topic_pub_condition_node.hpp"

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
    factory.registerNodeType<TopicDetected>("TopicDetected", ros2_node);
    factory.registerNodeType<TopicPubConditionNode>("TopicPubConditionNode", ros2_node, 2s);
    factory.registerNodeType<TopicDetectedSyncAction>("TopicDetectedSyncAction", ros2_node);

    const auto default_bt_xml_file = ament_index_cpp::get_package_share_directory("bt_sample") + "/config/main_bt.xml";
    const auto default_bt_log_file = ament_index_cpp::get_package_share_directory("bt_sample") + "/log/bt_trace.btlog";

    auto blackboard = BT::Blackboard::create();
    auto tree = factory.createTreeFromFile(default_bt_xml_file, blackboard);

    auto groot2_publisher = std::make_unique<BT::Groot2Publisher>(tree, 5555);
    auto bt_file_logger = std::make_unique<BT::FileLogger2>(tree, default_bt_log_file);
    auto bt_cout_logger = std::make_unique<BT::StdCoutLogger>(tree);

    auto bt_executer = std::thread([&]() {
        tree.tickWhileRunning();
        rclcpp::shutdown();
    });

    rclcpp::spin(ros2_node);
    bt_executer.join();

    RCLCPP_INFO(ros2_node->get_logger(), "Shutting down.");
    return 0;
}