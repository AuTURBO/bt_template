#include "bt_sample/laser_detection.h"
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

SetLocations::SetLocations(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

BT::NodeStatus SetLocations::tick()
{
    std::string location_file;
    const auto result = config().blackboard->get("location_file", location_file);
    if (!result) {
        std::cerr << "[" << this->name() << "] Could not read locations file from blackboard." << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        YAML::Node locations = YAML::LoadFile(location_file);
        int num_locs = locations.size();
        std::cout <<"num_locs !! " <<num_locs<< std::endl;
        if (num_locs == 0) {
            std::cerr << "[" << this->name() << "] No locations found." << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        setOutput("num_locs", num_locs);
        std::cout << "[" << this->name() << "] Found " << num_locs << " locations." << std::endl;

        std::deque<std::string> location_names{};
        std::map<std::string, Pose> location_poses{};
        for (YAML::const_iterator it=locations.begin(); it!=locations.end(); ++it) {
            const auto name = it->first.as<std::string>();
            location_names.push_back(name);
            const Pose pose = it->second.as<Pose>();
            location_poses.emplace(name, pose);
        }
        // Shuffle location names to get random order in each run
        // std::random_device rd;
        // std::mt19937 rng(rd());
        // std::shuffle(location_names.begin(), location_names.end(), rng);
        setOutput("loc_names", location_names);
        setOutput("loc_poses", location_poses);
        
    } catch (YAML::Exception const& e) {
        std::cerr << "Couldn't load locations file: " << location_file << ". Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetLocations::providedPorts()
{
    return { BT::OutputPort<int>("num_locs"),
             BT::OutputPort<std::deque<std::string>>("loc_names"),
             BT::OutputPort<std::map<std::string, Pose>>("loc_poses")
         };
}

////////////////////////////////

GetLocationFromQueue::GetLocationFromQueue(const std::string& name,
                                           const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}


BT::NodeStatus GetLocationFromQueue::tick()
{   
    // Get the locations from the port and select first one as the next target
    auto location_queue_ = getInput<std::deque<std::string>>("loc_names");
    if (!location_queue_) {
        std::cerr << "Couldn't get loc_names!" << std::endl;
    }
    if (location_queue_.value().empty()) {
        std::cout << "[" << this->name() << "] No more locations!" << std::endl;
        return BT::NodeStatus::FAILURE;
    } else {
        std::string tgt_loc = location_queue_.value().front();
        setOutput("target_location", tgt_loc);
        location_queue_.value().pop_front();
        std::cout << "[" << this->name() << "] Targeting location: " << tgt_loc << std::endl;
        setOutput("loc_names", location_queue_.value());
        return BT::NodeStatus::SUCCESS;
    }
}



BT::PortsList GetLocationFromQueue::providedPorts()
{
    return { BT::OutputPort<std::string>("target_location"),
             BT::BidirectionalPort<std::deque<std::string>>("loc_names") };
}



////////////////////////////////

// GOTOPOSE
// Wrapper behavior around the `navigate_to_pose` action client,
// whose status reflects the status of the ROS action.
GoToPose::GoToPose(const std::string& name, const BT::NodeConfig& config,
                   rclcpp::Node::SharedPtr node_ptr) :
    BT::StatefulActionNode(name, config), node_ptr_{node_ptr} {}

BT::NodeStatus GoToPose::onStart() {
    // Validate that a node exists
    if (!node_ptr_) {
        std::cout << "ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Read the YAML file
    BT::Expected<std::string> loc = getInput<std::string>("loc");

    auto location_poses = getInput<std::map<std::string,Pose>>("loc_poses");
    if (!location_poses){
        std::cerr << "Couldn't get loc_poses!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    auto target_loc = getInput<std::string>("loc");
    std::cout <<"target_loc : " << target_loc.value() << std::endl;
    if (!target_loc) {
        std::cerr << "Couldn't get target loc!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    auto target_pose = location_poses.value().at(target_loc.value());
    
    std::cout <<target_loc.value()<<" target_pose_x : " << target_pose.x << std::endl;
    std::cout <<target_loc.value()<<" target_pose_y : " << target_pose.y << std::endl;
    std::cout <<target_loc.value()<<" target_pose_theta : " << target_pose.theta << std::endl;

    // Set up the action client
    using namespace std::placeholders;
    auto send_goal_options = 
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&GoToPose::result_callback, this, _1);
    client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      node_ptr_, "/navigate_to_pose");

    // Package up the the goal
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = target_pose.x;
    goal_msg.pose.pose.position.y = target_pose.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, target_pose.theta);
    q.normalize();
    goal_msg.pose.pose.orientation = tf2::toMsg(q);


    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) 
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Action server not available after waiting");
        return BT::NodeStatus::FAILURE;
    }

    // Send the navigation action goal.
    done_flag_ = false;
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    std::cout << "[" << this->name() << "] Sent goal message to Nav2 Server" << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning() {
    // If there is a result, we can check the status of the action directly.
    // Otherwise, the action is still running.
    std::cout << "done_flag_ " << done_flag_ << std::endl; 
    if (done_flag_) {
        if (nav_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
            std::cout << "[" << this->name() << "] Goal reached" << std::endl;
            return BT::NodeStatus::SUCCESS;   
        } else {
            std::cout << "[" << this->name() << "] Failed to reach goal" << std::endl;
            return BT::NodeStatus::FAILURE;   
        }
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

BT::PortsList GoToPose::providedPorts() {
    return { BT::InputPort<std::string>("loc"),
             BT::InputPort<std::map<std::string, Pose>>("loc_poses") };
}

void GoToPose::result_callback(const GoalHandleNav::WrappedResult& result) {
    // If there is a result, we consider navigation completed and save the
    // result code to be checked in the `onRunning()` method.
    if (result.result) {
        done_flag_ = true;
        nav_result_ = result.code;
    }
}



/////////////////////////////////
//DockingPointDetected 생성자 definition
DockingPointDetected::DockingPointDetected(const std::string& name, const BT::NodeConfig& config,rclcpp::Node::SharedPtr node_ptr)
:BT::SyncActionNode(name, config),node_ptr_{node_ptr}
{
        subscription_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/docking_point", 10, std::bind(&DockingPointDetected::docking_point_callback, this, _1));
        publisher_ = node_ptr_->create_publisher<std_msgs::msg::Int32>("/docking_point_detected", 10);
}
 
// DockingPointDetected tick함수
BT::NodeStatus DockingPointDetected::tick()
{
    std::cout <<"DockingPointDetected tick함수 tick" << std::endl;
    std::this_thread::sleep_for(3s);
    std::vector<int> ballLocation{1,2,3}; 
    BT::TreeNode::setOutput("ball_location", ballLocation); // output port : ball_location , key : location , value : ballLocation{1,2,3}  // 출력 포트(ball_location)에 위치 정보(ballLocation)를 설정합니다.

    return  BT::NodeStatus::SUCCESS;
}


BT::PortsList DockingPointDetected::providedPorts()
{
    return 
    { 
        // BT::OutputPort<int>("num_locs"),
        // BT::OutputPort<std::deque<std::string>>("loc_names"),
        // BT::OutputPort<std::map<std::string, Pose>>("loc_poses")
        BT::OutputPort<std::vector<int>>("ball_location")
    };
}


// // Behavior Tree 실행을 제어하기 위한 전역 atomic 변수 정의
// std::atomic_bool switchActive{true};

// // tree xml 파일 경로 설정
// const std::string default_bt_xml_file =
//     ament_index_cpp::get_package_share_directory("bt_sample") + "/config/main_bt.xml";

// // logger 파일 경로 설정
// const std::string default_bt_log_file =
//     ament_index_cpp::get_package_share_directory("bt_sample") + "/log/bt_sample_trace.btlog";

// class BTNode : public rclcpp::Node
// {
// private:


// public:
//     BTNode() : Node("bt_node")
//     {
//         // xml 파일 경로 설정
//         this->declare_parameter<std::string>("tree_xml_file", default_bt_xml_file);
//         tree_xml_file_ = this->get_parameter("tree_xml_file").as_string();
//         this->declare_parameter<std::string>("tree_log_file", default_bt_log_file);
//         tree_log_file_ = this->get_parameter("tree_log_file").as_string();
//     }
//     // void init()
//     // {
//     //     topic_detected = std::make_unique<RosHandler>(shared_from_this());
//     // }

//     void execute()
//     {
//         // Behavior Tree 생성
//         create_behavior_tree();

//         // Behavior Tree 실행 주기 설정
//         const auto timer_period = 500ms;
//         timer_ = this->create_wall_timer(timer_period, std::bind(&BTNode::update_behavior_tree, this));

//         // rclcpp::spin(shared_from_this());
//         RCLCPP_INFO(this->get_logger(), "Before spinning node");
//         // rclcpp::spin(shared_from_this());
//         // RCLCPP_INFO(this->get_logger(), "After spinning node");
//         // rclcpp::shutdown();
//     }
//     void create_behavior_tree()
//     {
//         // xml 파일에서 Behavior Tree 로드
//         BT::BehaviorTreeFactory factory;
//         factory.registerNodeType<TopicDetected>("TopicDetected",shared_from_this());

//         // callback 함수 활성화를 위해 노드 설정
//         BT::NodeConfiguration con = {};
//         //auto lc_topic_detected_state = std::make_shared<TopicDetected>("lc_topic_detected_state", con,shared_from_this());

//         auto blackboard = BT::Blackboard::create();
//         // blackboard->set<std::string>("location_file", location_file_);
//         tree_ = factory.createTreeFromFile(tree_xml_file_, blackboard);

//         // Set up tree logging to monitor the tree in Groot2.
//         // Default ports (1666/1667) are used by the Nav2 behavior tree, so we use another port.
//         // NOTE: You must have the PRO version of Groot2 to view live tree updates.
//         publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 5555);
//         // bt file logger
//         BT::FileLogger2 logger_file(tree_, tree_log_file_);
//         // bt cout logger
//         BT::StdCoutLogger logger_cout(tree_);
//     }
//     void update_behavior_tree()
//     {
//         // Tick the behavior tree.
//         BT::NodeStatus tree_status = tree_.tickOnce();

//         // Check the status of the tree.
//         if (tree_status == BT::NodeStatus::RUNNING)
//         {
//             return;
//         }
//         // Cancel the timer if we hit a terminal state.
//         if (tree_status == BT::NodeStatus::SUCCESS)
//         {
//             RCLCPP_INFO(this->get_logger(), "Finished with status SUCCESS");
//             timer_->cancel();
//         }
//         else if (tree_status == BT::NodeStatus::FAILURE)
//         {
//             RCLCPP_INFO(this->get_logger(), "Finished with status FAILURE");
//             timer_->cancel();
//         }
// }
// // Configuration parameters.
// std::string tree_xml_file_;
// std::string tree_log_file_;

// // ROS and BehaviorTree.CPP variables.
// rclcpp::TimerBase::SharedPtr timer_;
// BT::Tree tree_;
// std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<BTNode>();
//     // node->init();
//     node->execute();

// //  여기 바꾸기
//     rclcpp::spin(node);
//     //RCLCPP_INFO(this->get_logger(), "After spinning node");
//     rclcpp::shutdown();

//     // rclcpp::shutdown();
//     return 0;
// }