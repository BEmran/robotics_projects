#include <algorithm>  // for std::clamp
#include <coding_test/PathPlanner.hpp>
#include <coding_test/utils.hpp>
#include <functional>  // for placeholder _1, _2
#include <thread>

namespace path_planner {

using namespace std::placeholders;
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

PathPlanner::PathPlanner(const std::string& node_name,
                         const std::string& namespace_,
                         const rclcpp::NodeOptions& options)
    : Node(node_name, namespace_, options),

      move_to_point_action_server_(rclcpp_action::create_server<MoveToPoint>(
          this, move_to_point_action_name,
          std::bind(&PathPlanner::handle_goal, this, _1, _2),
          std::bind(&PathPlanner::handle_cancel, this, _1),
          std::bind(&PathPlanner::handle_accepted, this, _1))),

      move_joints_action_client_(rclcpp_action::create_client<MoveJoints>(
          this, move_joints_action_name)),

      js_subscriber_(create_subscription<JointState>(
          joint_state_topic, 10,
          std::bind(&PathPlanner::js_callback, this, std::placeholders::_1))) {
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
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&PathPlanner::move_point_send_goal, this, _1),
              goal_handle}
      .detach();
}

void PathPlanner::move_point_send_goal(
    const std::shared_ptr<GoalHandleMoveToPoint> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  current_point_target_[0] = goal->target_point.x;
  current_point_target_[1] = goal->target_point.y;
  current_point_target_[2] = goal->target_point.z;
  // utils::Point point_target;
  // point_target.x = goal->target_point.x;
  // point_target.y = goal->target_point.y;
  // point_target.z = goal->target_point.z;
  // const auto joints_target =
  //     utils::CartesianToJoint({0.9, 0.9, 0.975}, point_target);
  // move_joints_send_goal(joints_target);
  rclcpp::Rate loop_rate(100);
  while (rclcpp::ok() && goal_handle->is_active()) {
    if (move_point_check_reached()) {
      {
        auto result = std::make_shared<MoveToPoint::Result>();
        result->success = true;
        result->error.x = current_point_target_[0] - current_point_state_[0];
        result->error.y = current_point_target_[1] - current_point_state_[1];
        result->error.z = current_point_target_[2] - current_point_state_[2];
        RCLCPP_INFO(this->get_logger(), "Publish Result as SUCCEED");
        goal_handle->succeed(result);
      }
      // move_point_send_results();
      break;
    }
    // move_point_send_feedback();
    {
      const auto goal_point = goal->target_point;
      const auto current_point =
          utils::JointToCartesian({0.9, 0.9, 0.975}, current_joint_);
      // publish
      auto feedback = std::make_shared<MoveToPoint::Feedback>();
      feedback->header.stamp = now();
      feedback->error.x = goal_point.x - current_point.x;
      feedback->error.y = goal_point.y - current_point.y;
      feedback->error.z = goal_point.z - current_point.z;
      feedback->current_point.x = current_point.x;
      feedback->current_point.y = current_point.y;
      feedback->current_point.z = current_point.z;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
    }
    tick();
    loop_rate.sleep();
  }
}

void PathPlanner::move_point_send_results() {
  auto result = std::make_shared<MoveToPoint::Result>();
  result->success = true;
  result->error.x = current_point_target_[0] - current_point_state_[0];
  result->error.y = current_point_target_[1] - current_point_state_[1];
  result->error.z = current_point_target_[2] - current_point_state_[2];
  RCLCPP_INFO(this->get_logger(), "Publish Result as SUCCEED");
  move_point_current_goal_handle_->succeed(result);
}

void PathPlanner::move_point_send_feedback() {
  const auto goal_point =
      move_point_current_goal_handle_->get_goal()->target_point;
  const auto current_point =
      utils::JointToCartesian({0.9, 0.9, 0.975}, current_joint_);
  // publish
  auto feedback = std::make_shared<MoveToPoint::Feedback>();
  feedback->header.stamp = now();
  feedback->error.x = goal_point.x - current_point.x;
  feedback->error.y = goal_point.y - current_point.y;
  feedback->error.z = goal_point.z - current_point.z;
  feedback->current_point.x = current_point.x;
  feedback->current_point.y = current_point.y;
  feedback->current_point.z = current_point.z;
  move_point_current_goal_handle_->publish_feedback(feedback);
  RCLCPP_INFO(this->get_logger(), "Publish feedback");
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
    // std::cout << "[" << error << ", " << current_point_state_[i] << "]";
  }
  std::cout << std::endl;
}

bool PathPlanner::move_point_check_reached() {
  // const auto goal_point =
  //     move_point_current_goal_handle_->get_goal()->target_point;
  // const auto current_point =
  //     utils::JointToCartesian({0.9, 0.9, 0.975}, current_joint_);
  double error = 0;
  // error += goal_point.x - current_point.x;
  // error += goal_point.y - current_point.y;
  // error += goal_point.z - current_point.z;
  error += current_point_target_[0] - current_point_state_[0];
  error += current_point_target_[1] - current_point_state_[1];
  error += current_point_target_[2] - current_point_state_[2];
  if (-0.01 <= error && error <= 0.01) {
    return true;
  }
  return false;
}

void PathPlanner::move_joints_send_goal(const std::array<double, 3> joints) {
  // using namespace std::placeholders;
  // If the action server is not found, fail
  if (!move_joints_action_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_logger(),
                 "coding_test_msgs::action::MoveJoints server %s is not ready",
                 move_joints_action_name);
    return;
  }

  // Generate a goal, to move to a random point.
  MoveJoints::Goal goal{};
  goal.header.stamp = now();
  goal.target_state.position.push_back(joints[0]);
  goal.target_state.position.push_back(joints[1]);
  goal.target_state.position.push_back(joints[2]);

  RCLCPP_INFO(get_logger(), "Sending Goal MoveJoints (%f, %f, %f)",
              goal.target_state.position[0], goal.target_state.position[1],
              goal.target_state.position[2]);

  auto options = ClientMoveJoints::SendGoalOptions();
  options.goal_response_callback =
      std::bind(&PathPlanner::move_joints_goal_response_callback, this, _1);
  options.result_callback =
      std::bind(&PathPlanner::move_joints_result_callback, this, _1);
  options.feedback_callback =
      std::bind(&PathPlanner::move_joints_feedback_callback, this, _1, _2);
  // Send the Goal
  move_joints_action_client_->async_send_goal(goal);
}

void PathPlanner::move_joints_goal_response_callback(
    std::shared_future<ClientMoveJoints::GoalHandle::SharedPtr> future) {
  const auto& goal_handle = future.get();
  RCLCPP_INFO(get_logger(), "Move Joints Goal Response Callback (%s) [%s]",
              rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
              action_code_to_string(goal_handle->get_status()));
}

void PathPlanner::move_joints_result_callback(
    const ClientMoveJoints::GoalHandle::WrappedResult& result) {
  RCLCPP_INFO(get_logger(),
              "Move Joints Goal Result (%s) [%s]: %s (%f, %f, %f)",
              rclcpp_action::to_string(result.goal_id).c_str(),
              action_code_to_string(result.code),
              result.result->success ? "SUCCESS" : "FAILURE");
  move_point_send_results();
}

void PathPlanner::move_joints_feedback_callback(
    ClientMoveJoints::GoalHandle::SharedPtr handle,
    std::shared_ptr<const MoveJoints::Feedback> feedback) {
  (void)handle;
  RCLCPP_INFO(get_logger(), "Move Joints Goal Feedback: (%f, %f, %f)",
              // rclcpp_action::to_string(handle->get_goal_id()).c_str(),
              // action_code_to_string(handle->get_status()),
              feedback->current_state.position[0],
              feedback->current_state.position[1],
              feedback->current_state.position[2]);
  move_point_send_feedback();
}

void PathPlanner::js_callback(const JointState::SharedPtr msg) {
  joint_state_ = *msg.get();
  for (size_t i = 0; i < current_joint_.size(); ++i) {
    current_joint_[i] = joint_state_.position[i];
  }
}
}  // namespace path_planner