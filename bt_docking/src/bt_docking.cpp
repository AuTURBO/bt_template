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

#include "bt_sample/laser_detection.h"
#include "yaml-cpp/yaml.h"

#include "laser_line_extraction/msg/line_segment.hpp" // 사용자 정의 메시지 타입은 그대로 유지
#include "laser_line_extraction/msg/line_segment_list.hpp" //

// tree xml 파일 경로 설정
const std::string default_bt_xml_file = 
    ament_index_cpp::get_package_share_directory("bt_sample") + "/config/docking_bt.xml";

// logger 파일 경로 설정
const std::string default_bt_log_file =
    ament_index_cpp::get_package_share_directory("bt_sample") + "/log/bt_sample_trace.btlog";

// Nav2 waypoint 파일 경로 설정 (대차의 예상 위치와, 이송할 목적지 위치)
const std::string default_location_file =
     "/root/laser_line_extract/src/bt_template/bt_sample/maps/docking_waypoints.yaml";



class AutonomyNode : public rclcpp::Node
{
private:
    // Configuration parameters.
    std::string tree_xml_file_;
    std::string tree_log_file_;   
    std::string location_file_;
    // ROS and BehaviorTree.CPP variables.
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
    std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
    std::unique_ptr<BT::FileLogger2> logger_file_;
    std::unique_ptr<BT::StdCoutLogger> logger_cout_;

public:
    AutonomyNode():Node("bt_docking_node")
    {
        // xml 파일 경로 설정
        this->declare_parameter<std::string>("tree_xml_file", default_bt_xml_file);
        tree_xml_file_ = this->get_parameter("tree_xml_file").as_string();

        // log 파일 경로 설정
        this->declare_parameter<std::string>("tree_log_file", default_bt_log_file);
        tree_log_file_ = this->get_parameter("tree_log_file").as_string();        
        RCLCPP_INFO(this->get_logger(),
                "Using tree_log_file %s", tree_log_file_.c_str());


        this->declare_parameter<std::string>("location_file", default_location_file);
        location_file_ = this->get_parameter("location_file").as_string();
        RCLCPP_INFO(this->get_logger(),
                "Using location file %s", location_file_.c_str());



    }

    void execute()
    {
        // Behavior Tree 생성
        create_behavior_tree();

        // Behavior Tree 실행 Tick 주기 설정
        const auto timer_period = 500ms;
        timer_ = this->create_wall_timer(timer_period, std::bind(&AutonomyNode::update_behavior_tree, this));

        // rclcpp::spin(shared_from_this());
        // rclcpp::shutdown();
    }

    void create_behavior_tree()
    {
        BT::BehaviorTreeFactory factory;
        factory.registerNodeType<SetLocations>("SetLocations");
        factory.registerNodeType<GetLocationFromQueue>("GetLocationFromQueue");
        factory.registerNodeType<GoToPose>("GoToPose", shared_from_this());
        factory.registerNodeType<DockingPointDetected>("DockingPointDetected",shared_from_this());

        // factory.registerNodeType<DockingController>("DockingController");
        auto blackboard = BT::Blackboard::create();
        blackboard->set<std::string>("location_file", location_file_);
        tree_ = factory.createTreeFromFile(tree_xml_file_, blackboard);

        publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 5555);
        
        // bt file logger
        BT::FileLogger2 logger_file(tree_, tree_log_file_);
        // bt cout logger
        // BT::StdCoutLogger logger_cout(tree_);
        logger_file_ = std::make_unique<BT::FileLogger2>(tree_, tree_log_file_);
        logger_cout_ = std::make_unique<BT::StdCoutLogger>(tree_);

    }

    void update_behavior_tree()
    {
            // Tick the behavior tree.
            BT::NodeStatus tree_status = tree_.tickOnce();
            if (tree_status == BT::NodeStatus::RUNNING) {
                return;
            }
            // Cancel the timer if we hit a terminal state.
            if (tree_status == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Finished with status SUCCESS");
                timer_->cancel();
            } else if (tree_status == BT::NodeStatus::FAILURE) {
                RCLCPP_INFO(this->get_logger(), "Finished with status FAILURE");
                timer_->cancel();
            }


    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AutonomyNode>();
    node->execute();
    rclcpp::spin(node);
    rclcpp::shutdown();
    std::cout <<" bt docking " << std::endl;
    return 0;
}
