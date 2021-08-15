#ifndef CODING_TEST_UTILS_HPP
#define CODING_TEST_UTILS_HPP

#include <array>
#include <geometry_msgs/msg/point.hpp>

namespace utils {

using Point = geometry_msgs::msg::Point;

Point JointToCartesian(std::array<double, 3> link_lengths,
                       std::array<double, 3> joints);

std::array<double, 3> CartesianToJoint(std::array<double, 3> link_lengths,
                                       Point point);

Point constrain_point_reach(std::array<double, 3> link_lengths, Point point);

double constrain_angle(double j);

double clamp_acos(const double num, const double den);
}  // namespace utils

#endif  // CODING_TEST_UTILS_HPP