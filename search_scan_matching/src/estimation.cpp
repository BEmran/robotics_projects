/**
 * @file estimation.cpp
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief Implementation of estimation algorithm class
 * @version 0.1
 * @date 2021-08-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "search_scan_matching/estimation.h"

#include <algorithm>  // max, min
#include <iomanip>    // std::setprecision

#include "search_scan_matching/utils.h"

int MatchingScore(std::vector<std::vector<uint8_t>> grid,
                  std::vector<std::vector<uint8_t>> sensor) {
  int score = 0;
  for (int i = 0; i < grid.size(); ++i) {
    for (int j = 0; j < grid.size(); ++j) {
      if (grid[i][j] == sensor[i][j]) score++;
    }
  }
  return score;
}

Estimation2D::Estimation2D() {}

est::EstimationInfo Estimation2D::BruteSearch(const Grid2D& grid,
                                              const comm::Pose2D& init_pose,
                                              const comm::RangeData& range_data,
                                              const double range_max_range,
                                              const est::SearchConfig& config) {
  // store initial pose
  init_pose_ = init_pose;
  // clear data
  max_ = -1;
  est_info_vec_.clear();
  max_est_info_vec_.clear();
  // calculate number of steps
  int linear_steps = config.linear_tolerance / config.linear_resolution + 1;
  int angular_steps = config.angular_tolerance / config.angular_resolution + 1;
  // calculate starting pose
  comm::Pose2D start(init_pose);
  start.point.x -= config.linear_tolerance / 2;
  start.point.y -= config.linear_tolerance / 2;
  start.theta -= config.angular_tolerance / 2;
  comm::Pose2D end(init_pose);
  end.point.x += config.linear_tolerance / 2;
  end.point.y += config.linear_tolerance / 2;
  end.theta += config.angular_tolerance / 2;
  // printout information
  std::cout << std::fixed << std::setprecision(3)
            << "Number of approximate positions to be check = "
            << linear_steps * linear_steps * angular_steps << ".\nsearch range:"
            << "\n\t- x\tin range of: " << start.point.x << " - " << end.point.x
            << "\t\twith increment of: " << config.linear_resolution
            << "\n\t- y\tin range of: " << start.point.y << " - " << end.point.y
            << "\t\twith increment of: " << config.linear_resolution
            << "\n\t- theta in range of: " << start.theta << " - " << end.theta
            << "\t\twith increment of: " << config.angular_resolution
            << std::endl;
  // loop through all approximate poses in grid frame
  for (int i = 0; i < angular_steps; ++i) {
    // update theta
    double t = start.theta + i * config.angular_resolution;
    for (int j = 0; j < linear_steps; ++j) {
      // update x position
      double x = start.point.x + j * config.linear_resolution;
      for (int k = 0; k < linear_steps; ++k) {
        // update y position
        double y = start.point.y + k * config.linear_resolution;
        comm::Pose2D estimate(x, y, t);
        // calculate range_finder occupancy
        auto rf_occupancy = RangeDataToOccupancyGrid(grid, estimate, range_data,
                                                     range_max_range);
        // calculate score
        auto score = MatchingScore(grid.GetOccupancy(), rf_occupancy);
        // record estimation score
        est_info_vec_.push_back(est::EstimationInfo(score, estimate));
      }
    }
  }
  return ClosestEstimate();
}

int Estimation2D::MaximumScore() {
  // find maximum score
  auto max_ptr = std::max_element(
      est_info_vec_.begin(), est_info_vec_.end(),
      [](auto& p1, auto& p2) { return p1.matching_score < p2.matching_score; });
  // store result
  max_ = max_ptr->matching_score;
  // printout result
  std::cout << "maximum score: " << max_ << std::endl;
  return max_;
}

std::vector<est::EstimationInfo> Estimation2D::MaximumEstimates() {
  // calculate maximum score if not calculated
  if (max_ == -1) {
    MaximumScore();
  }
  // loop through all estimates and find the one with same maximum score
  for (const auto& est : est_info_vec_) {
    if (est.matching_score == max_) {
      max_est_info_vec_.push_back(est);
    }
  }
  // printout result
  std::cout << "Found [" << max_est_info_vec_.size()
            << "] estimation points with the same maximum score of [" << max_
            << "]" << std::endl;
  return max_est_info_vec_;
}

est::EstimationInfo Estimation2D::ClosestEstimate() {
  // extract maximum estimation vector if not calculated
  if (max_est_info_vec_.empty()) {
    MaximumEstimates();
  }

  // find norm between initial pose and estimation poses of maximum score
  for (auto& est : max_est_info_vec_) {
    double norm = utils::L2Norm(init_pose_, est.pose);
    est.closeness = norm;
  }

  // find minimum closeness
  auto min_ptr = std::min_element(
      max_est_info_vec_.begin(), max_est_info_vec_.end(),
      [](auto& p1, auto& p2) { return p1.closeness < p2.closeness; });
  return *min_ptr;
}
