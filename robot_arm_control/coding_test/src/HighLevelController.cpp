#include <coding_test/HighLevelController.hpp>
#include <rclcpp_action/create_client.hpp>

namespace high_level_controller {

/// Utility to convert a `rclcpp_action::ResultCode` to a string
constexpr const char* action_code_to_string(rclcpp_action::ResultCode code) {
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

constexpr const char* action_code_to_string(
    std::underlying_type_t<rclcpp_action::ResultCode> code) {
  return action_code_to_string(static_cast<rclcpp_action::ResultCode>(code));
}

HighLevelController::HighLevelController(const std::string& node_name,
                                         const std::string& namespace_,
                                         const rclcpp::NodeOptions& options)
    : Node(node_name, namespace_, options),
      rng_((std::random_device())()),
      // create ROS constructs
      move_to_point_client_(rclcpp_action::create_client<MoveToPoint>(
          this, move_to_point_client_action_name)),
      send_goal_timer_(
          create_wall_timer(send_goal_period, [this] { send_goal(); })) {}

void HighLevelController::send_goal() {
  // If the action server is not found, fail
  if (!move_to_point_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_logger(),
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
  goal.target_point.y = 0.0;  // This robot rotates around y... we assume it is
                              // 2D in the plane y = 0
  goal.target_point.z = point_distribution(rng_);

  RCLCPP_INFO(get_logger(), "Sending Goal (%f, %f, %f)", goal.target_point.x,
              goal.target_point.y, goal.target_point.z);

  // Setup the goal to use our callbacks

  using namespace std::placeholders;
  auto send_goal_options =
      rclcpp_action::Client<MoveToPoint>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&HighLevelController::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&HighLevelController::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&HighLevelController::result_callback, this, _1);
  move_to_point_client_->async_send_goal(goal, send_goal_options);
}

void HighLevelController::goal_response_callback(
    std::shared_future<Client::GoalHandle::SharedPtr> future) {
  const auto& goal_handle = future.get();
  RCLCPP_INFO(get_logger(), "Goal Response Callback (%s) [%s]",
              rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
              action_code_to_string(goal_handle->get_status()));
}

void HighLevelController::result_callback(
    const Client::GoalHandle::WrappedResult& result) {
  RCLCPP_INFO(get_logger(), "Goal Result (%s) [%s]: %s (%f, %f, %f)",
              rclcpp_action::to_string(result.goal_id).c_str(),
              action_code_to_string(result.code),
              result.result->success ? "SUCCESS" : "FAILURE",
              result.result->error.x, result.result->error.y,
              result.result->error.z);
}

void HighLevelController::feedback_callback(
    Client::GoalHandle::SharedPtr handle,
    const std::shared_ptr<const MoveToPoint::Feedback> feedback) {
  (void)handle;
  // RCLCPP_INFO(get_logger(), "Goal Feedback (%s) [%s]: (%f, %f, %f)",
  //             rclcpp_action::to_string(handle->get_goal_id()).c_str(),
  //             action_code_to_string(handle->get_status()),
  //             feedback->current_point.x, feedback->current_point.y,
  //             feedback->current_point.z);
  RCLCPP_INFO(get_logger(), "Received goal feedback: (%f, %f, %f)",
              feedback->current_point.x, feedback->current_point.y,
              feedback->current_point.z);
}

}  // namespace high_level_controller
