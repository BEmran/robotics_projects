/**
 * @file grid.h
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief Implementation of Grid class
 * @version 0.1
 * @date 2021-08-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "search_scan_matching/grid.h"

#include "search_scan_matching/utils.h"

#define DEFAULT_RESOLUTION 0.05

Grid2D::Grid2D(const double length, const double width)
    : frame_(comm::Frame2D(comm::Pose2D(), "grid")) {
  Initialize(DEFAULT_RESOLUTION, length, width);
}

Grid2D::Grid2D(const comm::Pose2D pose, const double resolution,
               const double length, const double width)
    : frame_(comm::Frame2D(pose, "grid")) {
  Initialize(resolution, length, width);
}

void Grid2D::Initialize(const double res, const double length,
                        const double width) {
  // assert res value
  if (res <= 0) {
    std::string e_msg =
        "Error occurred when defning Grid2D object where resolution is "
        "set [" +
        std::to_string(res) + "] however  a positive  value is expected.";
    throw std::runtime_error(e_msg.c_str());
  }

  // assert length value
  if (length <= 0) {
    std::string e_msg =
        "Error occurred when defning Grid2D object where length is set to [" +
        std::to_string(length) + "] however a positive value is expected.";
    throw std::runtime_error(e_msg.c_str());
  }

  // assert width value
  if (width <= 0) {
    std::string e_msg =
        "Error occurred when defning Grid2D object where width is set to [" +
        std::to_string(width) + "] however a positive value is expected.";
    throw std::runtime_error(e_msg.c_str());
  }

  // initialize parameters
  res_ = res;
  size_ = comm::Size(length / res_, width / res_);
  occupancy_ = utils::Creat2DArray(size_, comm::FREE);
}

void Grid2D::CreateObstacle(const comm::Pose2D origin, const double length,
                            const double width) {
  // create box frame using box origin pose
  comm::Frame2D obs_frame(origin, "box");
  // convert rectangle's actual dimintion to cell resolution
  const double tmp_res = res_ / 2;  // increase resolution
  const size_t rectangle_height = length / tmp_res + 1;
  const size_t rectangle_width = width / tmp_res + 1;
  // loop through all rectangle's points
  for (size_t row = 0; row < rectangle_height; ++row) {
    for (size_t col = 0; col < rectangle_width; ++col) {
      // transfer point from box frame to grid frame
      const comm::Point2D box_point(row * tmp_res, col * tmp_res);
      const comm::Point2D box_point_w = obs_frame.TransformBack(box_point);
      SetCellOccupancy(box_point_w, comm::OCCUPIED);
    }
  }
}

bool Grid2D::SetCellOccupancy(const comm::Cell& cell, const uint8_t val) {
  bool result = false;
  // make sure the cell index are within the grid
  if (0 <= cell.row && cell.row < size_.height &&  // [0, max row)
      0 <= cell.col && cell.col < size_.width) {   // [0, max col)
    occupancy_[cell.row][cell.col] = val;
    result = true;
  }
  return result;
}

bool Grid2D::SetCellOccupancy(const comm::Point2D& point_world,
                              const uint8_t val) {
  // transform point from world frame to grid frame
  auto point_g = frame_.TransformForward(point_world);
  // transfer calculated point to grid cell coordination
  const auto cellinfo = utils::PointToCell(point_g, size_, res_);
  return SetCellOccupancy(cellinfo.cell, val);
}

bool Grid2D::IsOccupied(comm::Point2D point) const {
  // calculate approxmate row and column indicies of the point in the grid
  auto cell_info = utils::PointToCell(point, size_, res_);
  // initialize cell value as free
  uint8_t cell_value = comm::FREE;
  // get occupancy if cell is valid
  if (cell_info.valid) {
    cell_value = occupancy_[cell_info.cell.row][cell_info.cell.col];
  }
  // return occupied if cell has value of 255
  if (cell_value == comm::OCCUPIED) {
    return true;
  } else if (cell_value != comm::FREE) {  // if value is not defined
    std::cerr << "undefined cell value of: " << cell_value << std::endl;
  }
  return false;
}

bool Grid2D::IsOccupied(comm::Cell cell) const {
  // initialize cell value as free
  uint8_t cell_value = comm::FREE;
  // check of indicies are within the grid boundaries
  bool valid = false;
  if (0 <= cell.row && cell.row < size_.height &&  // [0, max row)
      0 <= cell.col && cell.col < size_.width) {   // [0, max col)
    valid = true;
  }
  // get occupancy if cell is valid
  if (valid) {
    cell_value = occupancy_[cell.row][cell.col];
  }
  // return occupied if cell has value of 255
  if (cell_value == comm::OCCUPIED) {
    return true;
  } else if (cell_value != comm::FREE) {  // if value is not defined
    std::cerr << "undefined cell value of: " << cell_value << std::endl;
  }
  return false;
}
comm::Size Grid2D::GetSize() const { return size_; }

comm::Frame2D Grid2D::GetFrame() const { return frame_; }

double Grid2D::GetResolution() const { return res_; }

std::vector<std::vector<uint8_t>> Grid2D::GetOccupancy() const {
  return occupancy_;
}
