#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <random>
#include <string>

#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "bt_nav2/bt_tree.h"
#include "yaml-cpp/yaml.h"

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
    auto ros2_node = std::make_shared<ROS2Node>("bt_docking_node");
    auto factory = BT::BehaviorTreeFactory();
    factory.registerNodeType<SetLocations>("SetLocations");
    factory.registerNodeType<GetLocationFromQueue>("GetLocationFromQueue");
    factory.registerNodeType<GoToPose>("GoToPose", ros2_node);

    // tree xml 파일 경로 설정
    const auto default_bt_xml_file = 
        ament_index_cpp::get_package_share_directory("bt_nav2") + "/config/bt_nav2.xml";

    // logger 파일 경로 설정
    const auto default_bt_log_file =
        ament_index_cpp::get_package_share_directory("bt_nav2") + "/log/bt_nav2_trace.btlog";

    // waypoint 파일 경로 설정
    const auto location_file =
        ament_index_cpp::get_package_share_directory("bt_nav2") + "/maps/nav2_waypoints.yaml";
    RCLCPP_INFO(ros2_node->get_logger(), "Loading locations file: %s", location_file.c_str());

 
    auto blackboard = BT::Blackboard::create();
    blackboard->set<std::string>("location_file", location_file);
    auto tree = factory.createTreeFromFile(default_bt_xml_file, blackboard);
    auto groot2_publisher = std::make_unique<BT::Groot2Publisher>(tree, 5555);
    auto bt_file_logger = std::make_unique<BT::FileLogger2>(tree,default_bt_log_file);
    auto bt_cout_logger = std::make_unique<BT::StdCoutLogger>(tree);

    while(rclcpp::ok())
    {
        const auto &now = std::chrono::steady_clock::now();

        rclcpp::spin_some(ros2_node);

        // Tick the behavior tree.
        BT::NodeStatus tree_status = tree.tickOnce();
        // if (tree_status == BT::NodeStatus::RUNNING) {
        //     return;
        // }
        // Cancel the timer if we hit a terminal state.
        if (tree_status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(ros2_node->get_logger(), "Finished with status SUCCESS");
           break;
        } else if (tree_status == BT::NodeStatus::FAILURE) {
            RCLCPP_INFO(ros2_node->get_logger(), "Finished with status FAILURE");
            break;
        }
        std::this_thread::sleep_until(now + 100ms);
    }

    rclcpp::shutdown();
    RCLCPP_INFO(ros2_node->get_logger(),"Shutting down");

    return 0;
}
