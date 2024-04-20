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
// #include "bt_sample/topic_detect_sync_action.hpp"
#include "bt_sample/topic_detect_stateful_action.hpp"

// 시간 측정을 위한 헤더 파일
using namespace std::chrono_literals;
using std::chrono::milliseconds;
using std::placeholders::_1;

// Behavior Tree 실행을 제어하기 위한 전역 atomic 변수 정의
std::atomic_bool switchActive{true};

// tree xml 파일 경로 설정
const std::string default_bt_xml_file =
    ament_index_cpp::get_package_share_directory("bt_sample") + "/config/main_bt.xml";

// logger 파일 경로 설정
const std::string default_bt_log_file =
    ament_index_cpp::get_package_share_directory("bt_sample") + "/log/bt_trace.btlog";

class BTNode : public rclcpp::Node
{
  public:
    BTNode() : Node("bt_node")
    {
        this->declare_parameter<std::string>("tree_xml_file", default_bt_xml_file);
        tree_xml_file_ = this->get_parameter("tree_xml_file").as_string();
        this->declare_parameter<std::string>("tree_log_file", default_bt_log_file);
        tree_log_file_ = this->get_parameter("tree_log_file").as_string();
    }

    void execute()
    {
        create_behavior_tree();

        const auto timer_period = 500ms;
        timer_ = this->create_wall_timer(timer_period, std::bind(&BTNode::update_behavior_tree, this));
        RCLCPP_INFO(this->get_logger(), "Behavior Tree is running...");
    }

    void create_behavior_tree()
    {
        BT::BehaviorTreeFactory factory;
        factory.registerNodeType<TopicDetected>("TopicDetected", shared_from_this());

        auto blackboard = BT::Blackboard::create();
        tree_ = factory.createTreeFromFile(tree_xml_file_, blackboard);

        // Set up tree logging to monitor the tree in Groot2.
        publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 5555);
        // Logger의 생명주기를 관리하기 위해 클래스 멤버로 선언
        logger_file_ = std::make_unique<BT::FileLogger2>(tree_, tree_log_file_);
        logger_cout_ = std::make_unique<BT::StdCoutLogger>(tree_);
    }

    void update_behavior_tree()
    {
        BT::NodeStatus tree_status = tree_.tickOnce();
        if (tree_status != BT::NodeStatus::RUNNING)
        {
            RCLCPP_INFO(this->get_logger(), "Behavior Tree finished with status: %s",
                        tree_status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
            timer_->cancel();
        }
    }

    // Configuration parameters.
    std::string tree_xml_file_;
    std::string tree_log_file_;

    // ROS and BehaviorTree.CPP variables.
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
    std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
    std::unique_ptr<BT::FileLogger2> logger_file_;
    std::unique_ptr<BT::StdCoutLogger> logger_cout_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BTNode>();
    node->execute();
    // ros ok 이거나 behavior tree의 root node가 running 상태일 때까지 노드를 spin
    while (rclcpp::ok() && node->tree_.tickOnce() == BT::NodeStatus::RUNNING)
    {
        rclcpp::spin_some(node);
    }
    RCLCPP_INFO(node->get_logger(), "Shutting down.");
    rclcpp::shutdown();
    return 0;
}