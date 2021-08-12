#include <algorithm>
#include <chrono>
#include <iostream>

#include "search_scan_matching/common.h"
#include "search_scan_matching/estimation.h"
#include "search_scan_matching/grid.h"
#include "search_scan_matching/range_finder.h"
#include "search_scan_matching/utils.h"

int main(int argc, char const* argv[]) {
  std::cout << "start ..." << std::endl;

  // Create grid and fill with obstacles ------------------------------------//
  // create a grid with specific dimintion in meter
  comm::Pose2D grid_pose(0, -0.5, 0);
  Grid2D grid(grid_pose, 0.05, 1, 1.25);

  // create obstacles
  grid.CreateObstacle(comm::Pose2D(0.2, 1.0, 0.2), 0.50, 0.20);
  grid.CreateObstacle(comm::Pose2D(0.1, 0.3, 0), 0.45, 0.45);
  grid.CreateObstacle(comm::Pose2D(0.9, 0.0, 0), 0.10, 1.20);
  grid.CreateObstacle(comm::Pose2D(0.0, 0.0, 0), 0.70, 0.05);

  // display actual grid
  std::cout << "Actual grid:" << std::endl;
  utils::Display(grid.GetOccupancy());

  // Create Range finder sensor and simulate its actual data ----------------//
  // create range finder
  const double sensor_max_range = 1.2;
  const double sensor_fov = 3.14 * 2;
  const double sensor_res = 0.05;
  RangeFinder rf(sensor_max_range, sensor_fov, sensor_res);
  // define exact sensor pose
  comm::Pose2D exact_sensor_pose(0.8, 0.7, 3.14);
  // generate range finder data
  auto range_data = rf.Execute(grid, exact_sensor_pose);

  // Printout ground truth information --------------------------------------//
  // display exact occupancy grid seen by laser
  std::cout << "\n\nExact occupancy seen by laser" << std::endl;
  auto sensor_occupancy = RangeDataToOccupancyGrid(
      grid, exact_sensor_pose, range_data, sensor_max_range);
  utils::Display(sensor_occupancy);

  // calculate matching score of the exact pose
  auto exact_sensor_cell = utils::PointToCell(
      exact_sensor_pose.point, grid.GetSize(), grid.GetResolution());
  std::cout << "\n\nPerfect Score: "
            << MatchingScore(grid.GetOccupancy(), sensor_occupancy) << " at "
            << exact_sensor_pose << " at index of " << exact_sensor_cell.cell
            << "\n\n"
            << std::endl;

  // Configure searching criteria -------------------------------------------//
  auto begin_timer = std::chrono::steady_clock::now();

  // define search configuration
  est::SearchConfig search_config(0.4, 0.6, 0.05, 0.05);

  // create initial estimation pose
  auto initial_estimate = exact_sensor_pose + utils::GenerateRandPose(0.1, 0.2);
  auto initial_estimate_cell = utils::PointToCell(
      initial_estimate.point, grid.GetSize(), grid.GetResolution());
  std::cout << "Initial pose estimate: " << initial_estimate << " at index of "
            << initial_estimate_cell.cell << "\n\n"
            << std::endl;

  // Run Estimation algorithm -----------------------------------------------//
  Estimation2D est;
  auto closest_estimate = est.BruteSearch(grid, initial_estimate, range_data,
                                          sensor_max_range, search_config);
  auto closest_estimate_cell = utils::PointToCell(
      closest_estimate.pose.point, grid.GetSize(), grid.GetResolution());

  // printout result
  std::cout << "Closest estimate has norm of [" << closest_estimate.closeness
            << "] found at: " << closest_estimate.pose << " and index of "
            << closest_estimate_cell.cell << "\n\n"
            << std::endl;

  // calculate elapsed time
  auto end_timer = std::chrono::steady_clock::now();
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end_timer -
                                                                    begin_timer)
                  .count();

  std::cout << "Time elapsed = " << diff << " [ms]" << std::endl;
  return 0;
}
