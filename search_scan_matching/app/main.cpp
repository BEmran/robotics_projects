/**
 * @file main.cpp
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief an executable to demonstrate the project functionality by an example
 * @version 0.1
 * @date 2021-08-14
 *
 * @copyright Copyright (c) 2021
 * @details The implementation code goes through the following steps:
 * 1) Creates a Grid2D object at a certain pose, size and resolution
 * 2) Create four rectangle obstacles in order to simulate real life scenario.
 * 3) The grid is then displayed in the terminal where ‘-’ represent a FREE
 *    cell while ‘x’ represent an OCCUPIED cell.
 * 4) Create a RangeFinder object with certain FOV and resolution.
 * 5) The RangeFinder sensor s placed at pa ose in the world which defined as
 *    the ground truth pose.
 * 6) The grid object is passed to the RangeFinder in order to generate the
 *    sensor data.
 * 7) The range sensor occupancy grid is then displayed on the terminal. The
 *    ‘o’ cell represent the sensor truth pose.
 * 8) A random pose is generated around the exact pose.
 * 9) The Estimation2d object is then created along side the searching criteria.
 * 10) The Estimation2D object is run using the random pose to  calculate the
 *     best estimate according the suggested algorithm
 * 11) The result are then displayed on the the terminal.
 *
 */
/*****************************************************************************/

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

  // Step 1: Create grid ----------------------------------------------------//
  // create a grid with specific dimintion in meter
  comm::Pose2D grid_pose(0, 0.0, 0);
  Grid2D grid(grid_pose, 0.05, 1, 1.25);

  // Step 2: create obstacles -----------------------------------------------//
  grid.CreateObstacle(comm::Pose2D(0.2, 1.0, 0.2), 0.50, 0.20);
  grid.CreateObstacle(comm::Pose2D(0.1, 0.3, 0), 0.45, 0.45);
  grid.CreateObstacle(comm::Pose2D(0.9, 0.0, 0), 0.10, 1.20);
  grid.CreateObstacle(comm::Pose2D(0.0, 0.0, 0), 0.70, 0.05);

  // Step 3: display actual grid --------------------------------------------//
  std::cout << "Actual grid:" << std::endl;
  utils::Display(grid.GetOccupancy());

  // Step 4: create range finder --------------------------------------------//
  const double sensor_max_range = 1.2;
  const double sensor_fov = 3.14 * 2;
  const double sensor_res = 0.05;
  RangeFinder rf(sensor_max_range, sensor_fov, sensor_res);

  // Step 5: define exact sensor pose ---------------------------------------//
  comm::Pose2D exact_sensor_pose(0.8, 0.7, 3.14);

  // Step 6: simulate its actual data ---------------------------------------//
  auto range_data = rf.Execute(grid, exact_sensor_pose);

  // Step 7: Printout ground truth information ------------------------------//
  // display exact occupancy grid seen by laser
  std::cout << "\n\nExact occupancy seen by laser" << std::endl;
  const auto sensor_occupancy = RangeDataToOccupancyGrid(
      grid, exact_sensor_pose, range_data, sensor_max_range);
  utils::Display(sensor_occupancy);

  // calculate matching score of the exact pose
  const auto sensor_grid_cells = RangeDataToGridCells(
      grid, exact_sensor_pose, range_data, sensor_max_range);
  const auto exact_sensor_cell = utils::PointToCell(
      exact_sensor_pose.point, grid.GetSize(), grid.GetResolution());
  std::cout << "\n\nPerfect Score: "
            << MatchingScore(grid.GetOccupancy(), sensor_grid_cells) << " at "
            << exact_sensor_pose << " at index of " << exact_sensor_cell.cell
            << "\n\n"
            << std::endl;

  // Configure searching criteria -------------------------------------------//
  auto begin_timer = std::chrono::steady_clock::now();

  // Step 8: create initial estimation pose ---------------------------------//
  auto initial_estimate = exact_sensor_pose + utils::GenerateRandPose(0.1, 0.2);
  auto initial_estimate_cell = utils::PointToCell(
      initial_estimate.point, grid.GetSize(), grid.GetResolution());
  std::cout << "Initial pose estimate: " << initial_estimate << " at index of "
            << initial_estimate_cell.cell << "\n\n"
            << std::endl;

  // Step 9: run Estimation algorithm ---------------------------------------//
  Estimation2D est;

  // define search configuration
  est::SearchConfig search_config(0.4, 1.0, 0.02, 0.02);

  // Step 10: run estimation algorithm --------------------------------------//
  auto closest_estimate = est.BruteSearch(grid, initial_estimate, range_data,
                                          sensor_max_range, search_config);
  auto closest_estimate_cell = utils::PointToCell(
      closest_estimate.pose.point, grid.GetSize(), grid.GetResolution());

  // Step 11: printout result -----------------------------------------------//
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
