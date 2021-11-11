#include <algorithm>  // for std::clamp
#include <coding_test/MyDummyJointController.hpp>
#include <coding_test/utils.hpp>
#include <functional>  // for placeholder _1, _2
#include <thread>

namespace my_dummy_joint_controller {

MyDummyJointController::MyDummyJointController(
    const std::string& node_name, const std::string& namespace_,
    const rclcpp::NodeOptions& options)
    : Node(node_name, namespace_, options),
      feedback_joint_state_(std::make_shared<MoveJoints::Feedback>()),
      // create ROS constructs
      js_publisher_(create_publisher<JointState>(joint_state_topic, 10)),
      js_publish_timer_(create_wall_timer(joint_publishing_period,
                                          [this] { publish_js(); })) {
  using namespace std::placeholders;

  {  // initialize our cached JointState
    std::transform(joint_names.begin(), joint_names.end(),
                   std::back_inserter(joint_state_.name),
                   [](const auto& sv) { return std::string(sv); });
    joint_state_.position.assign(num_joints, 0.0);
    joint_state_.velocity.assign(num_joints, 0.0);

    std::transform(
        joint_names.begin(), joint_names.end(),
        std::back_inserter(feedback_joint_state_->current_state.name),
        [](const auto& sv) { return std::string(sv); });

    feedback_joint_state_->current_state.position.assign(num_joints, 0.0);
    feedback_joint_state_->current_state.velocity.assign(num_joints, 0.0);
  }

  this->move_joints_action_server_ = rclcpp_action::create_server<MoveJoints>(
      this, move_joints_topic,
      std::bind(&MyDummyJointController::handle_goal, this, _1, _2),
      std::bind(&MyDummyJointController::handle_cancel, this, _1),
      std::bind(&MyDummyJointController::handle_accepted, this, _1));

  RCLCPP_INFO(get_logger(), "Initialized DummyJointController (%s)",
              get_name());
  RCLCPP_INFO(get_logger(), "Controlling Joints:");
  for (size_t i = 0; i < num_joints; i++) {
    RCLCPP_INFO(get_logger(), "  - %s", joint_names[i].data());
  }
}

rclcpp_action::GoalResponse MyDummyJointController::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const MoveJoints::Goal> goal) {
  RCLCPP_INFO(
      this->get_logger(),
      "Received goal request at [%d:%d] for the following point %f %f %f",
      goal->header.stamp.sec, goal->header.stamp.nanosec,
      goal->target_state.position[0], goal->target_state.position[1],
      goal->target_state.position[2]);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MyDummyJointController::handle_cancel(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MyDummyJointController::handle_accepted(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle) {
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&MyDummyJointController::execute, this, _1),
              goal_handle}
      .detach();
}

void MyDummyJointController::publish_js() {
  joint_state_.header.stamp = now();

  // joint_names is already set in the constructor.
  // likewise, position and velocity were allocated in the constructor, and
  // updated by tick

  js_publisher_->publish(joint_state_);
}

void MyDummyJointController::execute(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  for (size_t i = 0; i < num_joints; i++) {
    const auto joint_goal =
        utils::constrain_angle(goal->target_state.position[i]);

    // joint_state_.position[i] =
    // utils::constrain_angle(joint_state_.position[i]);

    const auto diff = joint_goal - joint_state_.position[i];
    if (diff > M_PI) {
      current_joint_target_[i] = joint_state_.position[i] - 2 * M_PI + diff;
    } else if (diff < -M_PI) {
      current_joint_target_[i] = joint_state_.position[i] + 2 * M_PI + diff;
    } else {
      current_joint_target_[i] = joint_goal;
    }
  }

  auto result = std::make_shared<MoveJoints::Result>();
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok()) {
    if (check_reached()) {
      send_results(goal_handle);
      break;
    }
    send_feedback(goal_handle);
    tick();
    loop_rate.sleep();
  }
  // stop movement
  for (size_t i = 0; i < num_joints; i++) {
    joint_state_.velocity[i] = 0;
  }
}

void MyDummyJointController::tick() {
  // Convert our tick period into seconds. We do this via nanoseconds for the
  // highest resolution.
  constexpr auto tick_period_seconds =
      1e-3 * std::chrono::duration_cast<std::chrono::milliseconds>(tick_period)
                 .count();
  // Alternatively, we could measure the delta_t between ticks.
  // That would be more accurate, since there might be some jitter in the tick
  // execution, and this is not a realtime context.

  // Since velocity_limit is per seconds, we need it to be per one tick period.
  constexpr auto velocity_limit_in_tick =
      std::abs(velocity_limit) * tick_period_seconds;

  for (size_t i = 0; i < num_joints; i++) {
    // This is how much the joint needs to move
    const auto error = current_joint_target_[i] - joint_state_.position[i];
    // but we can't move faster than the velocity limit
    const auto delta =
        std::clamp(error, -velocity_limit_in_tick, velocity_limit_in_tick);

    // Compute new JointState
    // joint_state_.position[i] =
    //     utils::constrain_angle(joint_state_.position[i] + delta);
    joint_state_.position[i] += delta;
    joint_state_.velocity[i] = delta / tick_period_seconds;
  }
}

void MyDummyJointController::send_results(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle) {
  auto result = std::make_shared<MoveJoints::Result>();
  result->success = true;
  RCLCPP_INFO(this->get_logger(), "Publish Result as %s",
              result->success ? "SUCCESS" : "FAILURE");
  goal_handle->succeed(result);
}

void MyDummyJointController::send_feedback(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle) {
  // publish
  feedback_joint_state_->header.stamp = now();
  for (size_t i = 0; i < num_joints; i++) {
    feedback_joint_state_->current_state.position[i] = joint_state_.position[i];
    feedback_joint_state_->current_state.velocity[i] = joint_state_.velocity[i];
  }
  goal_handle->publish_feedback(feedback_joint_state_);
  RCLCPP_INFO(this->get_logger(), "Publish feedback!");
}

bool MyDummyJointController::check_reached() {
  double error = 0;
  for (size_t i = 0; i < num_joints; i++) {
    error += current_joint_target_[i] - joint_state_.position[i];
  }
  if (-0.01 <= error && error <= 0.01) {
    return true;
  }
  return false;
}

}  // namespace my_dummy_joint_controller
