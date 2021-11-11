#include <algorithm>  // for std::clamp
#include <coding_test/PathPlanner.hpp>
#include <functional>  // for placeholder _1, _2
#include <thread>

namespace path_planner {

PathPlanner::PathPlanner(const std::string& node_name,
                         const std::string& namespace_,
                         const rclcpp::NodeOptions& options)
    : Node(node_name, namespace_, options) {
  using namespace std::placeholders;
  // {  // initialize our cached JointState
  //   std::transform(joint_names.begin(), joint_names.end(),
  //                  std::back_inserter(joint_state_.name),
  //                  [](const auto& sv) { return std::string(sv); });

  //   joint_state_.position.assign(num_joints, 0.0);
  //   joint_state_.velocity.assign(num_joints, 0.0);
  //   for (size_t i = 0; i < num_joints; i++) {
  //     current_joint_states_[i] = 0.0;
  //     current_point_state_[i] = 0.0;
  //   }
  // }
  move_to_point_action_server_ = rclcpp_action::create_server<MoveToPoint>(
      this, move_to_point_client_action_name,
      std::bind(&PathPlanner::handle_goal, this, _1, _2),
      std::bind(&PathPlanner::handle_cancel, this, _1),
      std::bind(&PathPlanner::handle_accepted, this, _1));

  RCLCPP_INFO(get_logger(), "Start PathPlanner (%s)", get_name());
  // RCLCPP_INFO(get_logger(), "PathPlaning for Joints:");
  // for (size_t i = 0; i < num_joints; i++) {
  //   RCLCPP_INFO(get_logger(), "  - %s", joint_names[i].data());
  // }
}

rclcpp_action::GoalResponse PathPlanner::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const MoveToPoint::Goal> goal) {
  RCLCPP_INFO(
      this->get_logger(),
      "Received goal request at [%d:%d] for the following point %f %f %f",
      goal->header.stamp.sec, goal->header.stamp.nanosec, goal->target_point.x,
      goal->target_point.y, goal->target_point.z);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PathPlanner::handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPoint> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PathPlanner::handle_accepted(
    const std::shared_ptr<GoalHandleMoveToPoint> goal_handle) {
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&PathPlanner::execute, this, std::placeholders::_1),
              goal_handle}
      .detach();
}

void PathPlanner::execute(
    const std::shared_ptr<GoalHandleMoveToPoint> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  current_point_target_[0] = goal->target_point.x;
  current_point_target_[1] = goal->target_point.y;
  current_point_target_[2] = goal->target_point.z;

  auto result = std::make_shared<MoveToPoint::Result>();
  auto feedback = std::make_shared<MoveToPoint::Feedback>();
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok() && goal_handle->is_active()) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->error.x = current_point_target_[0] - current_point_state_[0];
      result->error.y = current_point_target_[1] - current_point_state_[1];
      result->error.z = current_point_target_[2] - current_point_state_[2];
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    if (check()) {
      break;
    }
    // publish
    feedback->header.stamp = now();
    feedback->error.x = current_point_target_[0] - current_point_state_[0];
    feedback->error.y = current_point_target_[1] - current_point_state_[1];
    feedback->error.z = current_point_target_[2] - current_point_state_[2];
    feedback->current_point.x = current_point_state_[0];
    feedback->current_point.y = current_point_state_[1];
    feedback->current_point.z = current_point_state_[2];
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish feedback");
    tick();
    loop_rate.sleep();
  }
  // send result
  result->success = true;
  result->error.x = current_point_target_[0] - current_point_state_[0];
  result->error.y = current_point_target_[1] - current_point_state_[1];
  result->error.z = current_point_target_[2] - current_point_state_[2];
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Publish Result as SUCCEED");
}

void PathPlanner::tick() {
  constexpr static auto period = std::chrono::milliseconds{100};
  // Convert our tick period into seconds. We do this via nanoseconds for the
  // highest resolution.
  constexpr auto tick_period_seconds =
      1e-3 *
      std::chrono::duration_cast<std::chrono::milliseconds>(period).count();
  // Alternatively, we could measure the delta_t between ticks.
  // That would be more accurate, since there might be some jitter in the tick
  // execution, and this is not a realtime context.

  // Since velocity_limit is per seconds, we need it to be per one tick period.
  constexpr auto velocity_limit_in_tick =
      std::abs(velocity_limit) * tick_period_seconds;

  for (size_t i = 0; i < num_joints; i++) {
    // This is how much the joint needs to move
    const auto error = current_point_target_[i] - current_point_state_[i];

    // but we can't move faster than the velocity limit
    const auto delta =
        std::clamp(error, -velocity_limit_in_tick, velocity_limit_in_tick);

    // Compute new point_state
    current_point_state_[i] += delta;
  }
}

bool PathPlanner::check() {
  double error = 0;
  for (size_t i = 0; i < num_joints; i++) {
    error += current_point_target_[i] - current_point_state_[i];
  }
  if (-0.01 <= error && error <= 0.01) {
    return true;
  }
  return false;
}
}  // namespace path_planner