#include <coding_test/DummyJointController.hpp>

namespace dummy_joint_controller {

DummyJointController::DummyJointController(const std::string& node_name,
                                           const std::string& namespace_,
                                           const rclcpp::NodeOptions& options)
    : Node(node_name, namespace_, options),
      // Seed our RNG
      rng_((std::random_device())()),
      // create ROS constructs
      js_publisher_(create_publisher<JointState>(joint_state_topic, 10)),
      js_publish_timer_(create_wall_timer(tick_period, [this] { tick(); })),
      randomize_timer_(
          create_wall_timer(randomize_period, [this] { randomize_target(); })) {
  {  // initialize our cached JointState
    std::transform(joint_names.begin(), joint_names.end(),
                   std::back_inserter(joint_state_.name),
                   [](const auto& sv) { return std::string(sv); });

    joint_state_.position.assign(num_joints, 0.0);
    joint_state_.velocity.assign(num_joints, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Initialized DummyJointController (%s)",
              get_name());
  RCLCPP_INFO(get_logger(), "Controlling Joints:");
  for (size_t i = 0; i < num_joints; i++) {
    RCLCPP_INFO(get_logger(), "  - %s", joint_names[i].data());
  }

  // choose an initial random position
  randomize_target();
}

void DummyJointController::tick() {
  // Convert our tick period into seconds. We do this via nanoseconds for the
  // highest resolution.
  constexpr auto tick_period_seconds =
      1e-9 *
      std::chrono::duration_cast<std::chrono::nanoseconds>(tick_period).count();
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
    joint_state_.position[i] += delta;
    joint_state_.velocity[i] = delta / tick_period_seconds;
  }

  publish_js();
}

void DummyJointController::publish_js() {
  joint_state_.header.stamp = now();

  // joint_names is already set in the constructor.
  // likewise, position and velocity were allocated in the constructor, and
  // updated by tick

  js_publisher_->publish(joint_state_);
}

void DummyJointController::randomize_target() {
  // Simply choose a new angle target for each joint

  std::uniform_real_distribution<double> revolute_distribution(-M_PI, M_PI);

  for (size_t i = 0; i < num_joints; i++) {
    current_joint_target_[i] = revolute_distribution(rng_);
  }

  {
    RCLCPP_INFO(get_logger(), "Generated new random target:");
    for (size_t i = 0; i < num_joints; i++) {
      RCLCPP_INFO(get_logger(), "  - %s : %f", joint_names[i].data(),
                  current_joint_target_[i]);
    }
  }
}

}  // namespace dummy_joint_controller
