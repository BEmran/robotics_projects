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
 * @param h array height, number of rows
 * @param w array width, number of columns
 * @param val initialized value value
 * @return std::vector<std::vector<uint8_t>> an initialized 2d array
 */
std::vector<std::vector<uint8_t>> Creat2DArray(int h, int w, const uint8_t val);

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
void Print(std::vector<double> vec);
}  // namespace utils