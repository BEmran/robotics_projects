/**
 * @file utils.cpp
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief implementation of utility functions
 * @version 0.1
 * @date 2021-08-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "search_scan_matching/utils.h"

#include <ctime>   // time
#include <random>  // mt19937, normal_distribution

namespace utils {

double L2Norm(const comm::Pose2D& p1, const comm::Pose2D& p2) {
  // find difference between tow poses
  comm::Pose2D diff = p1 - p2;
  // find L2Norm as sqrt(dx^2 + dy^2 + dtheta^2)
  return std::sqrt(diff.point.x * diff.point.x +  // dx^2
                   diff.point.y * diff.point.y +  // dy^2
                   diff.theta * diff.theta);      // dtheta^2
}

void Display(const std::vector<std::vector<uint8_t>>& occupancy) {
  // loop through all rows
  for (const auto& row : occupancy) {
    // loop through all columns in a single row
    for (const auto& cell : row) {
      if (cell == comm::FREE)
        std::cout << "|-";
      else if (cell == comm::OCCUPIED)
        std::cout << "|x";
      else if (cell == comm::SENSOR)
        std::cout << "|o";
      else
        std::cerr << "undefined cell value of: " << cell << std::endl;
    }
    std::cout << "|" << std::endl;
  }
}

std::vector<std::vector<uint8_t>> Creat2DArray(const comm::Size& size,
                                               const uint8_t val) {
  // create a single row of size width and initialize it by val
  std::vector<uint8_t> row(size.width, val);
  // create a a 2nd dimintion of size height and initialize it be vector row
  std::vector<std::vector<uint8_t>> arr(size.height, row);
  return arr;
}

comm::Pose2D GenerateRandPose(const double linear_stddev,
                              const double angular_stddev) {
  // Generate a normal distribution around that mean
  std::mt19937 mt(time(nullptr));
  std::normal_distribution<> linear_normal_dist(0, linear_stddev);
  std::normal_distribution<> angular_normal_dist(0, angular_stddev);
  // generate a random position
  double x = linear_normal_dist(mt);
  double y = linear_normal_dist(mt);
  double t = angular_normal_dist(mt);
  return comm::Pose2D(x, y, t);
}

void PrintVec(std::vector<double> vec) {
  std::cout << "Data: [";
  // loop through data
  for (size_t i = 0; i < vec.size(); ++i) {
    std::cout << vec[i];
    if (i < vec.size() - 1) {
      std::cout << ", ";
    }
  }
  std::cout << std::endl;
}

comm::Point2D PolarToCaretssian(const double angle, const double radius) {
  const double x = radius * std::cos(angle);
  const double y = radius * std::sin(angle);
  return comm::Point2D(x, y);
}

comm::CellInfo PointToCell(const comm::Point2D& point, const comm::Size& size,
                           const double resolution) {
  // calculate approxmate row and column indicies of the point in the grid
  const int approx_row = std::round(point.x / resolution);
  const int approx_col = std::round(point.y / resolution);
  const comm::Cell cell(approx_row, approx_col);
  // assume cell is not valid
  bool valid = false;
  // check of the approximated indicies are within the grid boundaries
  if (0 <= cell.row && cell.row < size.height &&  // [0, max row)
      0 <= cell.col && cell.col < size.width) {   // [0, max col)
    valid = true;
  }
  // return result
  return comm::CellInfo(valid, cell);
}
}  // namespace utils
