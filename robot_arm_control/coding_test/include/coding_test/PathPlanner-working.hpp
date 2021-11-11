#ifndef CODING_TEST_PATHPLANNER_HPP
#define CODING_TEST_PATHPLANNER_HPP

#include <coding_test_msgs/action/move_joints.hpp>
#include <coding_test_msgs/action/move_to_point.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string_view>

namespace path_planner {

/**
 * A simple planner robot path planner.
 *
 * It recives a MoveToPoint goal for the end effector form HighLevelController
 * and create a suitable path for the robot joints to follow
 *
 * It sends JointTarget goal at the period specified by `publish_period`, for
 * the joints listed in `joint_names` to follow.
 *
 */
class PathPlanner : public rclcpp::Node {
 public:
  //   constexpr static char joint_state_topic[] = "joint_states";

  using JointState = sensor_msgs::msg::JointState;
  using MoveJoints = coding_test_msgs::action::MoveJoints;
  using MoveToPoint = coding_test_msgs::action::MoveToPoint;
  using GoalHandleMoveToPoint = rclcpp_action::ServerGoalHandle<MoveToPoint>;

  constexpr static char move_to_point_client_action_name[] = "move_to_point";

  /// The Joints that this controller is concerned with
  constexpr static std::array<std::string_view, 3> joint_names = {
      "joint1", "joint2", "joint3"};

  /// Helper for the # of joints in joint_names
  constexpr static auto num_joints = joint_names.size();

  //   /// Limit joint movement to this value (m/s).
  //   /// eg. To emulate servo motors
  constexpr static auto velocity_limit = 1.0;

  //   /// How often to calculate position and velocity, and publish JointState
  //   constexpr static auto tick_period = std::chrono::milliseconds{10};

  //   /// How often to randomize the joint target
  //   constexpr static auto randomize_period = std::chrono::seconds{2};

  PathPlanner(const std::string &node_name, const std::string &namespace_,
              const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  explicit PathPlanner(
      const std::string &node_name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : PathPlanner(node_name, "", options) {}

  explicit PathPlanner(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : PathPlanner("dummy_joint_controller", "", options) {}

  ~PathPlanner() override = default;

 protected:
  //   /// Each tick of the controller.
  //   /// 1. Compute `joint_state_.position` and `joint_state_.velocity`,
  //   moving
  //   /// towards `current_joint_target_`.
  //   ///    Movement speed is limited by `velocity_limit`
  //   /// 2. Call `publish_js`
  //   void tick();

  //   /// Publishes JointState from joint_state_; is responsible for updating
  //   /// JointState::header
  //   void publish_js();

  //   /// Generate a new target, randomly, for `current_joint_target_`
  //   void randomize_target();

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveToPoint::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMoveToPoint> goal_handle);

  void handle_accepted(
      const std::shared_ptr<GoalHandleMoveToPoint> goal_handle);

  void execute(const std::shared_ptr<GoalHandleMoveToPoint> goal_handle);

  void tick();
  bool check();

  rclcpp_action::Server<MoveToPoint>::SharedPtr move_to_point_action_server_;

  //   /// Current JointState Goal
  //   std::array<double, num_joints> current_joint_target_{};

  /// Current JointState Goal
  std::array<double, num_joints> current_point_state_{};

  //   /// Current JointState Goal
  //   std::array<double, num_joints> current_joint_states_{};

  /// Current JointState Goal
  std::array<double, num_joints> current_point_target_{};

  //   /// Cache a constructed JointState (this will reduce allocations later)
  //   JointState joint_state_{};
};

}  // namespace path_planner

#endif  // CODING_TEST_PATHPLANNER_HPP
