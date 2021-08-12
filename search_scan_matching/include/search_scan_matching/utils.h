/**
 * @file utils.h
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief defines all utility functions used all over the code
 * @version 0.1
 * @date 2021-08-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <vector>  // vector

#include "search_scan_matching/common.h"  // Pose2D

namespace utils {
/**
 * @brief Calculated L2 Norm of two points
 *
 * @param p1 first pose
 * @param p2 second pose
 * @return double Euclidean distance
 */
double L2Norm(const comm::Pose2D& p1, const comm::Pose2D& p2);

/**
 * @brief Display occupancy of a 2d Array for debugging purposes
 *
 * @param occupancy a 2d container defines the states of its cell
 */
void Display(const std::vector<std::vector<uint8_t>>& occupancy);

/**
 * @brief creates a 2d Array of hegitXwidth size with an initial value
 *
 * @param size grid size height x width
 * @param val initialized value value
 * @return std::vector<std::vector<uint8_t>> an initialized 2d array
 */
std::vector<std::vector<uint8_t>> Creat2DArray(const comm::Size& size,
                                               const uint8_t val);

/**
 * @brief generate a random pose with mean 0 and passed variance used to
 * randomize the starting searching point
 *
 * @param linear_stddev linear standard deviation, default value 0.01
 * @param angular_stddev angular standard deviation, default value 0.01
 * @return Pose2D random pose
 */
comm::Pose2D GenerateRandPose(const double linear_stddev = 0.01,
                              const double angular_stddev = 0.01);

/**
 * @brief printout vector data for debugging purposes
 *
 */
void PrintVec(std::vector<double> vec);

/**
 * @brief convert point information from Polar system to cartesian
 *
 * @param angle angle from x-axis counterclocwise
 * @param radius distance from origin to the point
 * @return comm::Point2D cartesian point in x-axis and y-axs
 */
comm::Point2D PolarToCaretssian(const double angle, const double radius);

/**
 * @brief Get cell indices defined by projection of the passed point on a
 * 2d array size
 *
 * @param point a grid size height x width
 * @param size point to the desired cell
 * @param resolution grid resolution
 * @return comm::CellInfo contines the cell indices and if the projected cell
 * is within the grid boundaries
 */
comm::CellInfo PointToCell(const comm::Point2D& point, const comm::Size& size,
                           const double resolution);

/**
 * @brief convert range finder data to occupancy grid
 *
 * @param range_finder range finder pose
 * @param range_finder_data range finder data
 * @param range_max_range range finder maximum range
 * @param size a 2D Grid size
 * @param resolution a 2D Grid resolution
 * @return std::vector<std::vector<uint8_t>> occupancy grid
 */
std::vector<std::vector<uint8_t>> RangeDataToOccupancyGrid(
    const comm::Pose2D& range_finder_pose,
    const comm::RangeData& range_finder_data, const double range_max_range,
    const comm::Size& size, const double resolution);

}  // namespace utils