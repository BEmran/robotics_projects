#include <coding_test_msgs/action/move_to_point.hpp>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp {
class MyClient : public rclcpp::Node {
 public:
  using MoveToPoint = coding_test_msgs::action::MoveToPoint;
  using GoalHandleMoveToPoint = rclcpp_action::ClientGoalHandle<MoveToPoint>;

  explicit MyClient(const rclcpp::NodeOptions& options)
      : Node("MoveToPoint_action_client", options) {
    this->client_ptr_ =
        rclcpp_action::create_client<MoveToPoint>(this, "move_to_point");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&MyClient::send_goal, this));
  }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = MoveToPoint::Goal();
    goal_msg.target_point.x = -1;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<MoveToPoint>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MyClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&MyClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&MyClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

 private:
  rclcpp_action::Client<MoveToPoint>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(
      std::shared_future<GoalHandleMoveToPoint::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleMoveToPoint::SharedPtr,
      const std::shared_ptr<const MoveToPoint::Feedback> feedback) {
    std::stringstream ss;
    ss << feedback->current_point.x << " " << feedback->current_point.y << " "
       << feedback->current_point.z << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleMoveToPoint::WrappedResult& result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "result %s",
                result.result->success ? "Success" : "failed");
  }
};  // class MyClient

}  // namespace action_tutorials_cpp
RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::MyClient)
