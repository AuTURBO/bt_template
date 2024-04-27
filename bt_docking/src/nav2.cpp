#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class Nav2Client : public rclcpp::Node
{
public:
  Nav2Client() : Node("nav2_client_node")
  {
    this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");

    this->goal_msg_.pose.pose.position.x = 0.0;  // 예시 좌표
    this->goal_msg_.pose.pose.position.y = 1.5;
    this->goal_msg_.pose.pose.orientation.w = 1.0;
    this->goal_msg_.pose.header.frame_id = "map";
    this->goal_msg_.pose.header.stamp = this->get_clock()->now();
  }

  void send_goal()
  {
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_handle_future = this->client_ptr_->async_send_goal(this->goal_msg_);

    if (rclcpp::spin_until_future_complete(this->shared_from_this(), goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "send goal call failed :(");
      return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return;
    }

    auto result_future = client_ptr_->async_get_result(goal_handle);

    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
    } else {
        RCLCPP_INFO(this->get_logger(), "Failed to reach goal with status: %d", static_cast<int>(result.code));

    }

    } else {
      RCLCPP_ERROR(this->get_logger(), "get result call failed :(");
    }
  }

private:
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
  nav2_msgs::action::NavigateToPose::Goal goal_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Client>();
  node->send_goal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
