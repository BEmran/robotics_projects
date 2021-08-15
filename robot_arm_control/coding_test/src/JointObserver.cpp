#include <math.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <coding_test/JointObserver.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace joint_observer {

JointObserver::JointObserver(const std::string& node_name,
                             const std::string& namespace_,
                             const rclcpp::NodeOptions& options)
    : Node(node_name, namespace_, options),
      ef_publisher_(create_publisher<PointStamped>(end_effector_topic, 10)),
      // create ROS constructs
      js_subscriber_(create_subscription<JointState>(
          joint_state_topic, 10,
          std::bind(&JointObserver::js_callback, this, std::placeholders::_1))),
      joints_to_points_service(create_service<JointsToPoint>(
          joints_to_point_topic,
          std::bind(&JointObserver::joints_to_point_callback, this,
                    std::placeholders::_1, std::placeholders::_2))),
      points_to_joints_service(create_service<PointToJoints>(
          point_to_joints_topic,
          std::bind(&JointObserver::point_to_joints_callback, this,
                    std::placeholders::_1, std::placeholders::_2))) {
  RCLCPP_INFO(get_logger(), "Start Joint Observer");
}

void JointObserver::js_callback(const JointState::SharedPtr msg) {
  joint_state_ = *msg.get();
  std::array<double, 3> joints = {joint_state_.position[0],
                                  joint_state_.position[1],
                                  joint_state_.position[2]};

  ef_point_.point = utils::JointToCartesian(link_lengths, joints);
  publish_ef();
}

void JointObserver::publish_ef() {
  ef_point_.header.stamp = now();
  ef_publisher_->publish(ef_point_);
}

double constrain_angle(double j) {
  double res = std::fmod(j + M_PI, 2 * M_PI);
  if (res < 0) {
    res += 2 * M_PI;
  }
  return res - M_PI;
}

void JointObserver::joints_to_point_callback(
    const std::shared_ptr<JointsToPoint::Request> request,
    std::shared_ptr<JointsToPoint::Response> response) {
  if (request->joints.size() != 3) {
    return;
  }
  RCLCPP_INFO(get_logger(),
              "Incoming request to transfer point [%+.2f, %+.2f, %+.2f] to "
              "joints",
              request->joints[0], request->joints[1], request->joints[2]);

  const std::array<double, 3> joints = {request->joints[0], request->joints[1],
                                        request->joints[2]};

  response->point = utils::JointToCartesian(link_lengths, joints);

  RCLCPP_INFO(get_logger(),
              "Sending back response point: [%+.2f, %+.2f, %+.2f]",
              response->point.x, response->point.y, response->point.z);
}

void JointObserver::point_to_joints_callback(
    const std::shared_ptr<PointToJoints::Request> request,
    std::shared_ptr<PointToJoints::Response> response) {
  RCLCPP_INFO(get_logger(),
              "Incoming request to transfer point [%+.2f, %+.2f, %+.2f] to "
              "joints",
              request->point.x, request->point.y, request->point.z);

  const auto joints = utils::CartesianToJoint(link_lengths, request->point);

  for (const auto& j : joints) {
    response->joints.push_back(j);
  }

  RCLCPP_INFO(get_logger(),
              "Sending back response joints: [%+.2f, %+.2f, %+.2f]",
              response->joints[0], response->joints[1], response->joints[2]);
}
}  // namespace joint_observer
