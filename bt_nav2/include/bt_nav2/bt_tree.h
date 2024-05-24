#pragma once
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/bool.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "yaml-cpp/yaml.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::chrono::milliseconds;
using std::placeholders::_1;


// Struct to keep location pose data
struct Pose
{
    double x;
    double y;
    double theta;
};

// YAML parsing template specialization for the Pose type, which is used to parse locations directly
namespace YAML {
    template<>
    struct convert<Pose> {
        static bool decode(const Node& node, Pose& pose) {
            if (!node.IsSequence() || node.size() != 3) {
                return false;
            }
            pose.x = node[0].as<double>();
            pose.y = node[1].as<double>();
            pose.theta = node[2].as<double>();
            return true;
        }
    };
}

// Sets number of locations from list.
class SetLocations : public BT::SyncActionNode
{
  public:
    SetLocations(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

// Gets location from a queue of locations read from a list.
class GetLocationFromQueue : public BT::SyncActionNode
{
  public:
    GetLocationFromQueue(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
  
  private:
    std::deque<std::string> location_queue_;
};

// Go to a target location (wraps around `navigate_to_pose` action).
class GoToPose : public BT::StatefulActionNode
{
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    bool done_flag_;
    rclcpp_action::ResultCode nav_result_;
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    std_msgs::msg::Bool message;    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_publisher;

    // Method overrides
    GoToPose(const std::string& name, const BT::NodeConfig& config,
             rclcpp::Node::SharedPtr node_ptr);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {};
    static BT::PortsList providedPorts();

    // Action client callbacks
    void result_callback(const GoalHandleNav::WrappedResult& result);
    void feedbackCallback(GoalHandleNav::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    int m_count;
};



