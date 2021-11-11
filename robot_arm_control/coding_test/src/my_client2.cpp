#include <coding_test_msgs/action/move_to_point.hpp>
#include <functional>
#include <future>
#include <memory>
#include <random>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp {

/// Utility to convert a `rclcpp_action::ResultCode` to a string
constexpr const char *action_code_to_string(rclcpp_action::ResultCode code) {
  switch (code) {
    default:
    case rclcpp_action::ResultCode::UNKNOWN:
      return "UNKNOWN";
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    case rclcpp_action::ResultCode::ABORTED:
      return "ABORTED";
  }
}

constexpr const char *action_code_to_string(
    std::underlying_type_t<rclcpp_action::ResultCode> code) {
  return action_code_to_string(static_cast<rclcpp_action::ResultCode>(code));
}

class MyClient2 : public rclcpp::Node {
 public:
  using MoveToPoint = coding_test_msgs::action::MoveToPoint;
  using GoalHandleMoveToPoint = rclcpp_action::ClientGoalHandle<MoveToPoint>;

  constexpr static char move_to_point_client_action_name[] = "move_to_point";

  /// How often to (attempt to) send a `coding_test_msgs::action::MoveToPoint`
  constexpr static auto send_goal_period = std::chrono::seconds{5};

  /// Send goals with variables between negative and positive this value
  constexpr static double point_range = 2.0;

  MyClient2(const std::string &node_name, const std::string &namespace_,
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("MoveToPoint_action_client", options),
        rng_((std::random_device())()),
        move_to_point_client_(rclcpp_action::create_client<MoveToPoint>(
            this, move_to_point_client_action_name)),
        timer_(this->create_wall_timer(
            send_goal_period, std::bind(&MyClient2::send_goal, this))) {}

  explicit MyClient2(const std::string &node_name,
                     const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : MyClient2(node_name, "", options) {}

  explicit MyClient2(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : MyClient2("high_level_controller", "", options) {}

  ~MyClient2() override = default;

  void send_goal() {
    using namespace std::placeholders;
    // If the action server is not found, fail
    if (!move_to_point_client_->action_server_is_ready()) {
      RCLCPP_ERROR(
          get_logger(),
          "coding_test_msgs::action::MoveToPoint server %s is not ready",
          move_to_point_client_action_name);
      return;
    }

    constexpr auto apoint_range = std::abs(point_range);
    std::uniform_real_distribution<double> point_distribution(-apoint_range,
                                                              apoint_range);

    // Generate a goal, to move to a random point.
    MoveToPoint::Goal goal{};
    goal.header.stamp = now();
    goal.target_point.x = point_distribution(rng_);
    goal.target_point.y = 0.0;  // This robot rotates around y... we assume it
                                // is 2D in the plane y = 0
    goal.target_point.z = point_distribution(rng_);

    RCLCPP_INFO(get_logger(), "Sending Goal (%f, %f, %f)", goal.target_point.x,
                goal.target_point.y, goal.target_point.z);

    auto send_goal_options =
        rclcpp_action::Client<MoveToPoint>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MyClient2::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&MyClient2::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&MyClient2::result_callback, this, _1);
    move_to_point_client_->async_send_goal(goal, send_goal_options);
  }

 protected:
  void goal_response_callback(
      std::shared_future<GoalHandleMoveToPoint::SharedPtr> future) {
    const auto &goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Goal accepted by server, response callback (%s) [%s]",
                  rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
                  action_code_to_string(goal_handle->get_status()));
    }
  }

  void feedback_callback(
      GoalHandleMoveToPoint::SharedPtr,
      const std::shared_ptr<const MoveToPoint::Feedback> feedback) {
    RCLCPP_INFO(get_logger(), "Received goal feedback: (%f, %f, %f)",
                feedback->current_point.x, feedback->current_point.y,
                feedback->current_point.z);

    // std::stringstream ss;
    // ss << feedback->current_point.x << " " << feedback->current_point.y << "
    // "
    //    << feedback->current_point.z << " ";
    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleMoveToPoint::WrappedResult &result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal Result (%s) [%s]: %s (%f, %f, %f)",
                    rclcpp_action::to_string(result.goal_id).c_str(),
                    action_code_to_string(result.code),
                    result.result->success ? "SUCCESS" : "FAILURE",
                    result.result->error.x, result.result->error.y,
                    result.result->error.z);
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
  }

  rclcpp_action::Client<MoveToPoint>::SharedPtr move_to_point_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 rng_;
};  // class MyClient2

}  // namespace action_tutorials_cpp
RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::MyClient2)
