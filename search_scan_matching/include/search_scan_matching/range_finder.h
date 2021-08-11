/**
 * @file range_finder.h
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief defines RangeFinder class
 * @version 0.1
 * @date 2021-08-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <search_scan_matching/common.h>  // Pose2D
#include <search_scan_matching/grid.h>    // Grid2D

#include <memory>  // shared_ptr

/**
 * @brief Simulate Range Finder sensor by generating its
 * measured data with respect to a grid and its pose wrt to the grid
 *
 */
class RangeFinder {
 public:
  /**
   * @brief Construct a new Range Finder object
   *
   * @param max_range maximum ray range
   * @param fov field of view
   * @param res ray resolution
   */
  RangeFinder(const double max_range, const double fov, const double res);

  /**
   * @brief sweep through all range finder rays and record distance to obstacles
   * @details This is used to simulate real sensor and generate its measured
   * data  with respect to a grid obstacles
   *
   * @param grid grid with actual obstacles are placed
   * @param pose pose of the range finder in the grid
   * @return std::vector<double> range finder distance
   */
  std::vector<double> Execute(const std::shared_ptr<Grid2D> grid,
                              const comm::Pose2D pose);

  /**
   * @brief convert range finder data to occupancy grid
   *
   * @param grid a 2D Grid
   * @param pose pose of the range finder in the grid
   * @return std::vector<std::vector<uint8_t>> occupancy grid
   */

  std::vector<std::vector<uint8_t>> ToGrid(const std::shared_ptr<Grid2D> grid,
                                           const comm::Pose2D pose);
  /**
   * @brief Get RangeFinder measured Data
   *
   * @return std::vector<double> last  measured data
   */
  std::vector<double> GetData();

 private:
  double max_range_;  // range finder maximum range
  double fov_;  // range finder field of view where rays are in the range of
                // [-fov/2, fov/2)
  double res_;  // range finder angle resolution
  std::vector<double> data_;  // distance to obstacle at each ray. values are
                              // between 0 and max_range_ + 1
  std::vector<double> rays_;  // rays angle to scan
};