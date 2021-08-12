#include "search_scan_matching/grid.h"

#include "search_scan_matching/utils.h"

#define DEFAULT_RESOLUTION 0.05

Grid2D::Grid2D(const double length, const double width)
    : frame_(comm::Frame2D(comm::Pose2D(0, 0, 0), "grid")),
      res_(DEFAULT_RESOLUTION),
      size_(
          comm::Size(length / DEFAULT_RESOLUTION, width / DEFAULT_RESOLUTION)) {
  // initialize FREE occupancy
  occupancy_ = utils::Creat2DArray(size_, comm::FREE);
}

Grid2D::Grid2D(const comm::Frame2D frame, const double resolution,
               const double length, const double width)
    : frame_(frame),
      res_(resolution),
      size_(comm::Size(length / resolution, width / resolution)) {
  // initialize FREE occupancy
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
      const comm::Point2D point = obs_frame.TransformBack(box_point);
      // calculate approxmate row and column indicies of the point in the
      // grid
      const comm::CellInfo cell_info = utils::PointToCell(point, size_, res_);
      // if cell is valid place as occupied
      if (cell_info.valid) {
        occupancy_[cell_info.cell.row][cell_info.cell.col] = comm::OCCUPIED;
      }
    }
  }
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
  }  // if value is not defined
  else if (cell_value != comm::FREE) {
    std::cerr << "undefined cell value of: " << cell_value << std::endl;
  }
  return false;
}

comm::Size Grid2D::GetSize() const { return size_; }

double Grid2D::GetResolution() const { return res_; }

std::vector<std::vector<uint8_t>> Grid2D::GetOccupancy() const {
  return occupancy_;
}

void Grid2D::SetOccupancy(const std::vector<std::vector<uint8_t>>& occ) {
  // update occupancy grid information
  occupancy_ = occ;
  size_ = comm::Size(occ.size(), occ[0].size());
}