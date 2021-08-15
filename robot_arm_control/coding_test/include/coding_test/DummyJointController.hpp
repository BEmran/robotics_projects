#ifndef CODING_TEST_DUMMYJOINTCONTROLLER_HPP
#define CODING_TEST_DUMMYJOINTCONTROLLER_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <random>

namespace dummy_joint_controller
{

/**
 * A simple robot controller&driver emulator.
 *
 * It publishes JointStates at the period specified by `tick_period`, for the joints listed in `joint_names`.
 *
 * It will also compute a random joint state target every `randomize_period`, that the joints will move towards
 */
class DummyJointController : public rclcpp::Node
{
public:
  constexpr static char joint_state_topic[] = "joint_states";

  using JointState = sensor_msgs::msg::JointState;

  /// The Joints that this controller is concerned with
  constexpr static std::array<std::string_view, 3> joint_names = {"joint1", "joint2", "joint3"};

  /// Helper for the # of joints in joint_names
  constexpr static auto num_joints = joint_names.size();

  /// Limit joint movement to this value (rad/s).
  /// eg. To emulate servo motors
  constexpr static auto velocity_limit = M_PI;

  /// How often to calculate position and velocity, and publish JointState
  constexpr static auto tick_period = std::chrono::milliseconds {10};

  /// How often to randomize the joint target
  constexpr static auto randomize_period = std::chrono::seconds {2};

  DummyJointController(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit DummyJointController(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : DummyJointController(node_name, "", options)
  {}

  explicit DummyJointController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : DummyJointController("dummy_joint_controller", "", options)
  {}


  ~DummyJointController() override = default;

protected:
  /// Each tick of the controller.
  /// 1. Compute `joint_state_.position` and `joint_state_.velocity`, moving towards `current_joint_target_`.
  ///    Movement speed is limited by `velocity_limit`
  /// 2. Call `publish_js`
  void tick();

  /// Publishes JointState from joint_state_; is responsible for updating JointState::header
  void publish_js();

  /// Generate a new target, randomly, for `current_joint_target_`
  void randomize_target();

  /// Current JointState Goal
  std::array<double, num_joints> current_joint_target_ {};

  /// Cache a constructed JointState (this will reduce allocations later)
  JointState joint_state_ {};

  /// Random number generator
  /// (by storing this, we don't have to leverage std::random_device, thereby making syscall, each time we need rng)
  std::mt19937 rng_;

  rclcpp::Publisher<JointState>::SharedPtr js_publisher_;
  rclcpp::TimerBase::SharedPtr js_publish_timer_, randomize_timer_;
};

}

#endif //CODING_TEST_DUMMYJOINTCONTROLLER_HPP
