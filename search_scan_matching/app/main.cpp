#include <algorithm>
#include <chrono>
#include <iostream>

#include "search_scan_matching/common.h"
#include "search_scan_matching/estimation.h"
#include "search_scan_matching/grid.h"
#include "search_scan_matching/range_finder.h"
;

int main(int argc, char const* argv[]) {
  std::cout << "start ..." << std::endl;

  // create a grid with specific hight and width
  auto grid = std::make_shared<Grid2D>(20, 25);

  // define exact sensor pose
  utils::Pose2D exact_sensor_pose(0.6, 0.7, 3.14);

  // create range finder
  auto rf = std::make_shared<RangeFinder>(1.2, 3.14 * 2, 0.05);

  // create obstacles
  grid->CreatBox(utils::Pose2D(0.2, 1.0, 0), 0.70, 0.20);
  grid->CreatBox(utils::Pose2D(0.1, 0.3, 0), 0.46, 0.46);
  grid->CreatBox(utils::Pose2D(0.9, 0.0, 0), 0.10, 1.20);
  grid->CreatBox(utils::Pose2D(0.0, 0.0, 0), 0.70, 0.05);

  // display actual grid
  std::cout << "Actual grid:" << std::endl;
  utils::Display(grid->GetOccupancy());

  // generate range finder data
  rf->Execute(grid, exact_sensor_pose);

  // display exact occupancy grid seen by laser
  std::cout << "\n\nExact occupancy seen by laser" << std::endl;
  auto sensor_occupancy = rf->ToGrid(grid, exact_sensor_pose);
  utils::Display(sensor_occupancy);

  // calculate matching score of the exact pose
  std::cout << "\n\nPerfect Score: "
            << MatchingScore(grid->GetOccupancy(), sensor_occupancy) << " at "
            << exact_sensor_pose << " at index of "
            << grid->GetCell(exact_sensor_pose.point).cell << "\n\n"
            << std::endl;

  auto begin = std::chrono::steady_clock::now();

  // define search configuration
  SearchConfig search_config(0.4, 0.6, 0.05, 0.05);

  // create initial estimation pose
  auto initial_estimate = exact_sensor_pose + utils::GenerateRandPose(0.1, 0.2);
  std::cout << "Initial pose estimate: " << initial_estimate << " at index of "
            << grid->GetCell(initial_estimate.point).cell << "\n\n"
            << std::endl;

  // run estimation algorithm
  Estimation2D est(grid);
  est.BruteSearch(initial_estimate, rf, search_config);

  // calculate elapsed time
  auto end = std::chrono::steady_clock::now();
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin)
                  .count();

  std::cout << "Time elapsed = " << diff << " [ms]" << std::endl;
  return 0;
}
