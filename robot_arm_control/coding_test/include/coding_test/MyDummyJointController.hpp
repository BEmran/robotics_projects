#ifndef CODING_TEST_MYDUMMYJOINTCONTROLLER_HPP
#define CODING_TEST_MYDUMMYJOINTCONTROLLER_HPP

#include <coding_test_msgs/action/move_joints.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string_view>

namespace my_dummy_joint_controller {

/**
 * A simple robot controller&driver emulator.
 *
 * It publishes JointStates at the period specified by `tick_period`, for the
 * joints listed in `joint_names`.
 *
 * It will also compute a random joint state target every `randomize_period`,
 * that the joints will move towards
 */
class MyDummyJointController : public rclcpp::Node {
 public:
  constexpr static char joint_state_topic[] = "joint_states";
  constexpr static char move_joints_topic[] = "move_joints";

  using JointState = sensor_msgs::msg::JointState;
  using MoveJoints = coding_test_msgs::action::MoveJoints;
  using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<MoveJoints>;

  /// The Joints that this controller is concerned with
  constexpr static std::array<std::string_view, 3> joint_names = {
      "joint1", "joint2", "joint3"};

  /// How often to calculate position and velocity, and publish JointState
  constexpr static auto tick_period = std::chrono::milliseconds{100};

  /// How often to calculate position and velocity, and publish JointState
  constexpr static auto joint_publishing_period = std::chrono::milliseconds{10};

  /// Helper for the # of joints in joint_names
  constexpr static auto num_joints = joint_names.size();

  /// Limit joint movement to this value (rad/s).
  /// eg. To emulate servo motors
  constexpr static auto velocity_limit = M_PI;

  MyDummyJointController(
      const std::string &node_name, const std::string &namespace_,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  explicit MyDummyJointController(
      const std::string &node_name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : MyDummyJointController(node_name, "", options) {}

  explicit MyDummyJointController(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : MyDummyJointController("my_dummy_joint_controller", "", options) {}

  ~MyDummyJointController() override = default;

 protected:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveJoints::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

  void execute(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

  bool check();

  /// Each tick of the controller.
  /// 1. Compute `joint_state_.position` and `joint_state_.velocity`, moving
  /// towards `current_joint_target_`.
  ///    Movement speed is limited by `velocity_limit`
  /// 2. Call `publish_js`
  void tick();

  /// Publishes JointState from joint_state_; is responsible for updating
  /// JointState::header
  void publish_js();

  rclcpp_action::Server<MoveJoints>::SharedPtr move_joints_action_server_;

  /// Current JointState Goal
  std::array<double, num_joints> current_joint_target_{};

  /// Cache a constructed JointState (this will reduce allocations later)
  JointState joint_state_{};
  MoveJoints::Feedback::SharedPtr feedback_joint_state_{};

  rclcpp::Publisher<JointState>::SharedPtr js_publisher_;
  rclcpp::TimerBase::SharedPtr js_publish_timer_;
};

}  // namespace my_dummy_joint_controller

#endif  // CODING_TEST_MYDUMMYJOINTCONTROLLER_HPP
