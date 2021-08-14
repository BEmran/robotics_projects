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
  // assert max_range value
  if (max_range <= 0) {
    std::string e_msg =
        "Error occurred when defning RangeFinder object where Max Range is "
        "set [" +
        std::to_string(max_range_) + "] however a positive value is expected.";
    throw std::runtime_error(e_msg.c_str());
  }

  // assert res value
  if (res < 0) {
    std::string e_msg =
        "Error occurred when defning RangeFinder object where resolution is "
        "set [" +
        std::to_string(res_) + "] however a non negative value is expected.";
    throw std::runtime_error(e_msg.c_str());
  }

  // initialize data vector:
  // define a single data if fov or res is set to zero or res > fov
  if (res_ == 0 || fov_ == 0 || res_ >= fov_) {
    data_ = comm::RangeData(1);
  } else {
    const size_t num_rays = fov_ / res_;
    data_ = comm::RangeData(num_rays + 1);
  }
}

comm::RangeData RangeFinder::Execute(const Grid2D& grid,
                                     const comm::Pose2D pose) {
  // create sensor frame to map all points from sensor frame to grid frame
  comm::Frame2D sensor_frame(pose, "sensor");
  // define sensor resolution as twice as the grid resolution
  const double distance_res = grid.GetResolution() / 1;
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
      const comm::RangeFinderData data(ray, range);
      // transform the measured data into to grid frame
      auto cell_info = SingleRangeDataToGridCell(grid, sensor_frame, data);
      // check if point is occupied
      const bool occupied = grid.IsOccupied(cell_info.cell);
      // std::cout << ray << " " << range << " " << cell_info << " " << occupied
      //           << std::endl;
      // if occupied stop searching
      if (occupied) {
        measurement = range;
        break;
      }
    }
    // record range and stop searching on this ray
    data_[r] = comm::RangeFinderData(ray, measurement);
    // std::cout << "is set to: " << comm::RangeFinderData(ray, measurement)
    //           << std::endl;
  }
  // return sensor data
  return data_;
}

comm::RangeData RangeFinder::GetData() const { return data_; }

comm::CellInfo SingleRangeDataToGridCell(
    const Grid2D& grid, const comm::Frame2D& sensor_frame,
    const comm::RangeFinderData& range_finder_data) {
  // transform the measured data into a cartesian point
  const auto range_point = utils::PolarToCaretssian(range_finder_data.angle,
                                                    range_finder_data.range);
  // transform calculated point from sensor frame to world frame
  auto point_w = sensor_frame.TransformBack(range_point);
  // transform calculated point from world frame to grid frame
  auto point_g = grid.GetFrame().TransformForward(point_w);
  // transfer calculated point to grid cell coordination
  const auto transfeared_cell_info =
      utils::PointToCell(point_g, grid.GetSize(), grid.GetResolution());
  return transfeared_cell_info;
}

std::vector<comm::Cell> RangeDataToGridCells(
    const Grid2D& grid, const comm::Pose2D& range_finder_pose,
    const comm::RangeData& range_data, const double range_finder_max_range) {
  std::vector<comm::Cell> range_to_grid_indices;
  // create sensor frame to map all points from sensor frame to grid frame
  comm::Frame2D sensor_frame(range_finder_pose, "sensor");
  // loop through all sensor rays
  for (const auto& d : range_data) {
    // skip ray if range is bigger than sensor mas range
    if (d.range > range_finder_max_range) continue;
    const auto cell_info = SingleRangeDataToGridCell(grid, sensor_frame, d);
    // make sure the cell index are within the grid
    if (cell_info.valid) {
      // register the cell
      range_to_grid_indices.push_back(cell_info.cell);
    }
  }
  // return sensor occupancy
  return range_to_grid_indices;
}

std::vector<std::vector<uint8_t>> RangeDataToOccupancyGrid(
    const Grid2D& grid, const comm::Pose2D& range_finder_pose,
    const comm::RangeData& range_data, const double range_finder_max_range) {
  // convert range data to grid cells
  const auto rf_grid_cells = RangeDataToGridCells(
      grid, range_finder_pose, range_data, range_finder_max_range);
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
  // loop through all sensor cells
  for (const auto& cell : rf_grid_cells) {
    sensor_occupancy[cell.row][cell.col] = comm::OCCUPIED;
  }
  // return sensor occupancy
  return sensor_occupancy;
}
