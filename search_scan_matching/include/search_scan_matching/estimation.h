/**
 * @file estimation.h
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief defines estimation algorithm class
 * @version 0.1
 * @date 2021-08-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <memory>
#include <vector>

#include "search_scan_matching/common.h"
#include "search_scan_matching/grid.h"
#include "search_scan_matching/range_finder.h"

namespace est {
/**
 * @brief estimation information of a single estimation pose
 *
 */
struct EstimationInfo {
  int matching_score;    // matching score at this pose
  comm::Pose2D pose;     // estimation pose
  double closeness = 0;  // how close to inital pose

  /**
   * @brief Construct a new Estimation Info object
   *
   * @param s matching score
   * @param p estimation pose
   */
  EstimationInfo(const int ms, const comm::Pose2D& p)
      : matching_score(ms), pose(p) {}
};

/**
 * @brief defines the search cateria used by the Estimation2D class
 *
 */
struct SearchConfig {
  double linear_tolerance = 0;    // linear tolerance in x and y
  double angular_tolerance = 0;   // angular tolerance in theta
  double linear_resolution = 1;   // linear tolerance in x and y
  double angular_resolution = 1;  // angular resolution

  /**
   * @brief Construct a default Search Config object
   *
   */
  SearchConfig();

  /**
   * @brief Construct a new Search Config object
   *
   * @param lin_tol linear search tolerance
   * @param ang_tol angular search tolerance
   * @param lin_res linear search resolution
   * @param ang_res angular search resolution
   */
  SearchConfig(const double lin_tol, const double ang_tol, const double lin_res,
               const double ang_res)
      : linear_tolerance(lin_tol),
        angular_tolerance(ang_tol),
        linear_resolution(lin_res),
        angular_resolution(ang_res) {}
};
}  // namespace est

//
/**
 * @brief calculate a matching score between to two 2d Arrays
 *
 * @param grid occupancy grid defined by the grid
 * @param laser occupancy grid seen by the sensor
 * @return int matching score
 */
int MatchingScore(std::vector<std::vector<uint8_t>> grid,
                  std::vector<std::vector<uint8_t>> laser);

/**
 * @brief runs an score matching estimation algorithm to estimate the best pose
 * of the sensor with respect to occupancy grid
 *
 */
class Estimation2D {
 public:
  /**
   * @brief Construct a new Estimation 2 D object
   *
   */
  Estimation2D();

  /**
   * @brief Check all possible poses defined by the search caretria around the
   * initial pose and calculates their estimation matching score
   *
   * @param grid 2D grid
   * @param init_pose initali estimation pose
   * @param range_data range finder data
   * @param range_max_range range finder maximum range
   * @param config searching configuration
   * @return est::EstimationInfo closes estimated pose with maximum matching
   * score
   */
  est::EstimationInfo BruteSearch(const Grid2D& grid,
                                  const comm::Pose2D& init_pose,
                                  const comm::RangeData& range_data,
                                  const double range_max_range,
                                  const est::SearchConfig& config);

 private:
  /**
   * @brief returns maximum matching score found
   *
   * @return int maximum score
   */
  int MaximumScore();

  /**
   * @brief returns estimation ionformation of all poses with maximum score
   *
   * @return std::vector<est::EstimationInfo> vector of EstimationInfo
   */
  std::vector<est::EstimationInfo> MaximumEstimates();

  /**
   * @brief returns the estimation infomation of the closes pose to inital pose
   * among all maximum estimation poses
   *
   * @return est::EstimationInfo
   */
  est::EstimationInfo ClosestEstimate();

  comm::Pose2D init_pose_;  // initial pose estimation
  int max_;                 // maximum score
  std::vector<est::EstimationInfo>
      est_info_vec_;  // holds estimation information of all checked poses
  std::vector<est::EstimationInfo>
      max_est_info_vec_;  // holds estimation information of poses with he
                          // maximum score
};
