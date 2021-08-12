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

#include <vector>  // vector

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
   * @return comm::RangeData
   */
  comm::RangeData Execute(const Grid2D& grid, const comm::Pose2D pose);

  /**
   * @brief Get RangeFinder measured Data
   *
   * @return RangeData last measured data
   */
  comm::RangeData GetData();

 private:
  double max_range_;  // range finder maximum range
  double fov_;  // range finder field of view where rays are in the range of
                // [-fov/2, fov/2)
  double res_;  // range finder angle resolution
  comm::RangeData data_;  // a vector contains range finder data
};

/**
 * @brief convert range finder data to occupancy grid
 *
 * @param grid grid with actual obstacles are placed
 * @param range_finder_pose range finder pose
 * @param range_finder_data range finder data
 * @param range_finder_max_range range finder maximum range
 * @return std::vector<std::vector<uint8_t>> occupancy grid
 */
std::vector<std::vector<uint8_t>> RangeDataToOccupancyGrid(
    const Grid2D& grid, const comm::Pose2D& range_finder_pose,
    const comm::RangeData& range_finder_data,
    const double range_finder_max_range);
