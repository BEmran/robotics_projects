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

std::vector<std::vector<uint8_t>> RangeDataToOccupancyGrid(
    const comm::Pose2D& range_finder_pose,
    const comm::RangeData& range_finder_data, const double range_max_range,
    const comm::Size& size, const double resolution) {
  // create sensor frame to map all points from sensor frame to grid frame
  comm::Frame2D sensor_frame(range_finder_pose, "sensor");
  // create empty grid to fill it with lasser finding
  auto sensor_occupancy = utils::Creat2DArray(size, comm::FREE);
  // place sensor pose on grid
  const auto sensor_pose_cell =
      PointToCell(range_finder_pose.point, size, resolution);
  // mark sensor pose, make sure the cell index are within the grid
  if (sensor_pose_cell.valid) {
    sensor_occupancy[sensor_pose_cell.cell.row][sensor_pose_cell.cell.col] =
        comm::SENSOR;
  }
  // loop through all sensor rays
  for (const auto& d : range_finder_data) {
    // skip ray if range is bigger than sensor mas range
    if (d.range > range_max_range) continue;
    // transform the measured data into a cartesian point
    const auto range_point = utils::PolarToCaretssian(d.angle, d.range);
    // transform calculated point from sensor frame to grid frame
    const auto transfeared_point = sensor_frame.TransformBack(range_point);
    // transfer calculated point to grid cell coordination
    const auto transfeared_cell_info =
        PointToCell(transfeared_point, size, resolution);
    // make sure the cell index are within the grid
    if (transfeared_cell_info.valid) {
      // register the cell as occupied
      sensor_occupancy[transfeared_cell_info.cell.row]
                      [transfeared_cell_info.cell.col] = comm::OCCUPIED;
    }
  }
  // return sensor occupancy
  return sensor_occupancy;
}

}  // namespace utils