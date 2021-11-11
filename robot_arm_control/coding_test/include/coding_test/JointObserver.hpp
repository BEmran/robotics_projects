#ifndef CODING_TEST_DUMMYJOINTCONTROLLER_HPP
#define CODING_TEST_DUMMYJOINTCONTROLLER_HPP

#include <coding_test/utils.hpp>
#include <coding_test_msgs/srv/joints_to_point.hpp>
#include <coding_test_msgs/srv/point_to_joints.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <random>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace joint_observer {
using Point = geometry_msgs::msg::Point;
Point JointToCartesian(std::vector<double> joints);
std::vector<double> CartesianToJoint(Point point);

/**
 * A simple robot observer.
 *
 * It publishes robot's end effector point in Cartesian space
 * *
 * It will also compute a conversion from joint space to Cartesian space and
 * vice versa
 *
 */
class JointObserver : public rclcpp::Node {
 public:
  constexpr static char joint_state_topic[] = "joint_states";
  constexpr static char end_effector_topic[] = "end_effector";
  constexpr static char point_to_joints_topic[] = "point_to_joints";
  constexpr static char joints_to_point_topic[] = "joints_to_point";

  using JointState = sensor_msgs::msg::JointState;
  using PointStamped = geometry_msgs::msg::PointStamped;
  using JointsToPoint = coding_test_msgs::srv::JointsToPoint;
  using PointToJoints = coding_test_msgs::srv::PointToJoints;

  /// The length of the links that this observer is concerned with
  constexpr static std::array<double, 3> link_lengths = {0.9, 0.9, 0.975};

  /// Helper for the # of joints in joint_names
  constexpr static auto num_joints = link_lengths.size();

  JointObserver(const std::string& node_name, const std::string& namespace_,
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  explicit JointObserver(
      const std::string& node_name,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : JointObserver(node_name, "", options) {}

  explicit JointObserver(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : JointObserver("joint_observer", "", options) {}

  ~JointObserver() override = default;

 protected:
  void publish_ef();

  void js_callback(const JointState::SharedPtr msg);

  void joints_to_point_callback(
      const std::shared_ptr<JointsToPoint::Request> request,
      std::shared_ptr<JointsToPoint::Response> response);

  void point_to_joints_callback(
      const std::shared_ptr<PointToJoints::Request> request,
      std::shared_ptr<PointToJoints::Response> response);

  /// Cache a constructed JointState (this will reduce allocations later)
  JointState joint_state_{};

  /// Cache a constructed JointState (this will reduce allocations later)
  PointStamped ef_point_{};

  rclcpp::Publisher<PointStamped>::SharedPtr ef_publisher_;
  rclcpp::Subscription<JointState>::SharedPtr js_subscriber_;

  rclcpp::Service<JointsToPoint>::SharedPtr joints_to_points_service;
  rclcpp::Service<PointToJoints>::SharedPtr points_to_joints_service;
};

}  // namespace joint_observer

#endif  // CODING_TEST_DUMMYJOINTCONTROLLER_HPP
