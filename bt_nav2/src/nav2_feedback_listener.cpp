#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Nav2Client : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

  explicit Nav2Client(): Node("nav2_send_goal")
  {
    // 액션 클라이언트 생성
    this->client_ptr_  = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  void sendGoal(void) {
    // 액션 서버가 제공될 때까지 대기
    while (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }

    // 액션 목표(Goal) 생성
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = this->now(); // 현재 시간으로 스탬프 설정
    goal_msg.pose.header.frame_id = "map"; // 좌표계 설정

    // 목표 위치와 자세 설정
    goal_msg.pose.pose.position.x = 1;
    goal_msg.pose.pose.position.y = 0;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;
    goal_msg.pose.pose.orientation.z = 0.0;

    // 진행 상황을 표시하는 피드백 콜백 설정
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&Nav2Client::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&Nav2Client::resultCallback, this, _1);

    // Goal을 서버로 비동기 전송
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  // 피드백 콜백 함수: 목표까지 남은 거리를 로그에 출력
  void feedbackCallback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance remaining = %f", feedback->distance_remaining);
  }

  // 결과 콜백 함수: 목표 도달 여부를 로그에 출력
  void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Success!!!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); // ROS 2 초기화
  auto node = std::make_shared<Nav2Client>(); // 노드 생성
  node->sendGoal(); // 목표 전송 함수 호출
  rclcpp::spin(node); // 노드가 종료될 때까지 실행

  rclcpp::shutdown(); // ROS 2 종료
  return 0;
}
