/**
 * @file range_finder.cpp
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief Implementation of RangeFinder class
 * @version 0.1
 * @date 2021-08-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "search_scan_matching/range_finder.h"

#include "search_scan_matching/utils.h"

RangeFinder::RangeFinder(const double max_range, const double fov,
                         const double res)
    : max_range_(max_range), fov_(fov), res_(res) {
  // initialize data vector
  const size_t num_rays = fov_ / res_;
  data_ = comm::RangeData(num_rays);
}

comm::RangeData RangeFinder::Execute(const Grid2D& grid,
                                     const comm::Pose2D pose) {
  // create sensor frame to map all points from sensor frame to grid frame
  comm::Frame2D sensor_frame(pose, "sensor");
  // define sensor resolution as twice as the grid resolution
  const double distance_res = grid.GetResolution() / 2;
  // calculate range steps
  const size_t num_steps = max_range_ / distance_res + 1;
  // calculate starting angle
  const double ray0 = -fov_ / 2;
  // loop through all sensor rays
  for (double r = 0; r < data_.size(); ++r) {
    // define ray angle
    const double ray = ray0 + r * res_;
    double measurement = max_range_;  // initialize by maximum range
    // loop through sensor range [0 , max_range]
    for (int d = 0; d < num_steps; d++) {
      const double range = d * distance_res;
      // transform the measured data into a cartesian point
      const auto range_point = utils::PolarToCaretssian(ray, range);
      // transform calculated point from sensor frame to world frame
      auto point_w = sensor_frame.TransformBack(range_point);
      // transform calculated point from world frame to grid frame
      auto point_g = grid.GetFrame().TransformForward(point_w);
      // check if point is occupied
      const bool occupied = grid.IsOccupied(point_g);
      // if occupied stop searching
      if (occupied) {
        measurement = range;
        break;
      }
    }
    // record range and stop searching on this ray
    data_[r] = comm::RangeFinderData(ray, measurement);
  }
  // return sensor data
  return data_;
}

comm::RangeData RangeFinder::GetData() { return data_; }

std::vector<std::vector<uint8_t>> RangeDataToOccupancyGrid(
    const Grid2D& grid, const comm::Pose2D& range_finder_pose,
    const comm::RangeData& range_finder_data,
    const double range_finder_max_range) {
  // create sensor frame to map all points from sensor frame to grid frame
  comm::Frame2D sensor_frame(range_finder_pose, "sensor");
  // create empty grid to fill it with lasser finding
  auto sensor_occupancy = utils::Creat2DArray(grid.GetSize(), comm::FREE);
  // transform calculated point from world frame to grid frame
  auto laser_point_g =
      grid.GetFrame().TransformForward(range_finder_pose.point);
  // place sensor pose on grid
  const auto sensor_pose_cell =
      utils::PointToCell(laser_point_g, grid.GetSize(), grid.GetResolution());
  // mark sensor pose, make sure the cell index are within the grid
  if (sensor_pose_cell.valid) {
    sensor_occupancy[sensor_pose_cell.cell.row][sensor_pose_cell.cell.col] =
        comm::SENSOR;
  }
  // loop through all sensor rays
  for (const auto& d : range_finder_data) {
    // skip ray if range is bigger than sensor mas range
    if (d.range > range_finder_max_range) continue;
    // transform the measured data into a cartesian point
    const auto range_point = utils::PolarToCaretssian(d.angle, d.range);
    // transform calculated point from sensor frame to world frame
    auto point_w = sensor_frame.TransformBack(range_point);
    // transform calculated point from world frame to grid frame
    auto point_g = grid.GetFrame().TransformForward(point_w);
    // transfer calculated point to grid cell coordination
    const auto transfeared_cell_info =
        utils::PointToCell(point_g, grid.GetSize(), grid.GetResolution());
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
