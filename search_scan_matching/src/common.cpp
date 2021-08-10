#include "search_scan_matching/common.h"

#include <math.h>

#include <ctime>
#include <random>

namespace utils {

double Norm(const Pose2D& p1, const Pose2D& p2) {
  // find difference between tow poses
  Pose2D diff = p1 - p2;
  // find norm as sqrt(dx^2 + dy^2 + dtheta^2)
  return std::sqrt(diff.point.x * diff.point.x +  // dx^2
                   diff.point.y * diff.point.y +  // dy^2
                   diff.theta * diff.theta);      // dtheta^2
}

void Display(const std::vector<std::vector<uint8_t>>& occupancy) {
  // loop through all rows
  for (const auto& row : occupancy) {
    // loop through all columns in a single row
    for (const auto& cell : row) {
      if (cell == FREE)
        std::cout << "|-";
      else if (cell == OCCUPIED)
        std::cout << "|x";
      else if (cell == SENSOR)
        std::cout << "|o";
      else
        std::cerr << "undefined cell value of: " << cell << std::endl;
    }
    std::cout << "|" << std::endl;
  }
}

std::vector<std::vector<uint8_t>> Creat2DArray(int h, int w,
                                               const uint8_t val) {
  // create a single row of size width and initialize it by val
  std::vector<uint8_t> row(w, val);
  // create a a 2nd dimintion of size height and initialize it be vector row
  std::vector<std::vector<uint8_t>> arr(h, row);
  return arr;
}

Pose2D GenerateRandPose(const double linear_stddev,
                        const double angular_stddev) {
  // Generate a normal distribution around that mean
  std::mt19937 mt(time(nullptr));
  std::normal_distribution<> linear_normal_dist(0, linear_stddev);
  std::normal_distribution<> angular_normal_dist(0, angular_stddev);
  // generate a random position
  double x = linear_normal_dist(mt);
  double y = linear_normal_dist(mt);
  double t = angular_normal_dist(mt);
  return utils::Pose2D(x, y, t);
}
}  // namespace utils