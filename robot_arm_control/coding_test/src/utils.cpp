
#include <math.h>  // sin, cos, asin, acos

#include <algorithm>  // clamp
#include <coding_test/utils.hpp>
#include <iostream>  // cout
#include <numeric>   // accumulate

namespace utils {
Point JointToCartesian(std::array<double, 3> link_lengths,
                       std::array<double, 3> joints) {
  Point p;
  p.x = link_lengths[0] * std::sin(joints[0]) +
        link_lengths[1] * std::sin(joints[0] + joints[1]) +
        link_lengths[2] * std::sin(joints[0] + joints[1] + joints[2]);

  p.z = link_lengths[0] * std::cos(joints[0]) +
        link_lengths[1] * std::cos(joints[0] + joints[1]) +
        link_lengths[2] * std::cos(joints[0] + joints[1] + joints[2]);
  return p;
}

Point constrain_point_reach(std::array<double, 3> link_lengths, Point point) {
  // last joint angle with respect to x-axis
  const auto theta = std::atan2(point.x, point.z);
  std::cout << " point: " << point.x << " " << point.z << std::endl;
  std::cout << " theta: " << theta << std::endl;
  // find the distance between the goal point and origin
  const auto distance_to_goal =
      std::sqrt(std::pow(point.x, 2) + std::pow(point.z, 2));
  // find maximum distance robot can go to
  const auto max_distance_to_goal =
      std::accumulate(link_lengths.begin(), link_lengths.end(), 0.0);
  std::cout << " max_distance_to_goal: " << max_distance_to_goal << std::endl;
  if (distance_to_goal <= max_distance_to_goal) {
    return point;
  }
  // clamp the actual distance and return the new distance
  const auto restricted_distance_to_goal =
      std::clamp(distance_to_goal, -max_distance_to_goal, max_distance_to_goal);
  std::cout << "goal: " << distance_to_goal
            << " new goal: " << restricted_distance_to_goal << std::endl;
  // calculate the new goal point
  Point restricted_point;
  restricted_point.x = restricted_distance_to_goal * std::sin(theta);
  restricted_point.z = restricted_distance_to_goal * std::cos(theta);

  std::cout << " new point: " << restricted_point.x << " " << restricted_point.z
            << std::endl;

  return restricted_point;
}

std::array<double, 3> CartesianToJoint(std::array<double, 3> link_lengths,
                                       Point point) {
  const auto goal_point = constrain_point_reach(link_lengths, point);
  // last joint angle with respect to x-axis
  const auto theta = std::atan2(goal_point.x, goal_point.z);

  // length from origin to the joint of the last (third) link
  auto r = std::sqrt(std::pow(goal_point.x, 2) + std::pow(goal_point.z, 2)) -
           link_lengths[2];

  // angle from x-axis to end of second link
  auto alpha = theta;
  // correct r and inverse angle theta
  if (r <= 0) {
    r *= -1;
    alpha = constrain_angle(alpha + M_PI);
  }

  // r^2 = l0 + l1 - l0*l1*cos(beta)
  const auto num1 = std::pow(link_lengths[0], 2) +
                    std::pow(link_lengths[1], 2) - std::pow(r, 2);
  const auto den1 = 2 * link_lengths[0] * link_lengths[1];
  const auto beta = clamp_acos(num1, den1);

  // l1^2 = r^2 + l0^2 - 2*r*l0 cos(gama)
  const auto num2 = std::pow(r, 2) + std::pow(link_lengths[0], 2) -
                    std::pow(link_lengths[1], 2);
  const auto den2 = 2 * r * link_lengths[0];
  const auto gama = clamp_acos(num2, den2);

  std::cout << " alpha: " << alpha << std::endl;
  std::cout << " theta: " << theta << std::endl;
  std::cout << " beta: " << beta << std::endl;
  std::cout << " gama: " << gama << std::endl;

  std::array<double, 3> joints;
  joints[0] = constrain_angle(alpha - gama);
  joints[1] = constrain_angle(M_PI - beta);
  joints[2] = constrain_angle(theta - joints[0] - joints[1]);
  std::cout << " joints: " << joints[0] << " " << joints[1] << " " << joints[2]
            << std::endl;

  return joints;
}

double constrain_angle(double j) {
  double res = std::fmod(j + M_PI, 2 * M_PI);
  if (res < 0) {
    res += 2 * M_PI;
  }
  return res - M_PI;
}

double clamp_acos(const double num, const double den) {
  const auto ratio = num / den;
  const auto clamped = std::clamp(ratio, -1.0, 1.0);
  return std::acos(clamped);
}

}  // namespace utils