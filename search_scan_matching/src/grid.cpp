#include "search_scan_matching/grid.h"

#include "search_scan_matching/utils.h"

#define DEFAULT_RESOLUTION 0.05

comm::Cell Point2DToCell(const comm::Point2D& point, const double res) {
  const int approx_row = std::round(point.x / res);
  const int approx_col = std::round(point.y / res);
  return comm::Cell(approx_row, approx_col);
}

Grid2D::Grid2D(const double length, const double width)
    : frame_(comm::Frame2D(comm::Pose2D(0, 0, 0), "grid")),
      res_(DEFAULT_RESOLUTION),
      height_(length / DEFAULT_RESOLUTION),
      width_(width / DEFAULT_RESOLUTION) {
  // initialize FREE occupancy
  occupancy_ = utils::Creat2DArray(height_, width_, comm::FREE);
}

Grid2D::Grid2D(const comm::Frame2D frame, const double resolution,
               const double length, const double width)
    : frame_(frame),
      res_(resolution),
      height_(length / resolution),
      width_(width / resolution) {
  // initialize FREE occupancy
  occupancy_ = utils::Creat2DArray(height_, width_, comm::FREE);
}

void Grid2D::CreateObstacle(const comm::Pose2D origin, const double length,
                            const double width) {
  // create box frame using box origin pose
  comm::Frame2D obs_frame(origin, "box");
  // convert rectangle's actual dimintion to cell resolution
  const double tmp_res = res_ / 2;  // increase resolution
  const size_t rectangle_height = length / tmp_res + 1;
  const size_t rectangle_width = width / tmp_res + 1;
  // loop through all box's points
  for (size_t row = 0; row < rectangle_height; ++row) {
    for (size_t col = 0; col < rectangle_width; ++col) {
      // transfer point from box frame to grid frame
      const comm::Point2D box_point(row * tmp_res, col * tmp_res);
      const comm::Point2D point = obs_frame.TransformBack(box_point);
      // calculate approxmate row and column indicies of the point in the
      // grid
      const comm::CellInfo cell_info = GetCell(point);
      // if cell is valid place as occupied
      if (cell_info.valid) {
        occupancy_[cell_info.cell.row][cell_info.cell.col] = comm::OCCUPIED;
      }
    }
  }
}

bool Grid2D::IsOccupied(comm::Point2D point) {
  // initialize cell value as free
  uint8_t cell_value = comm::FREE;
  // calculate approxmate row and column indicies of the point in the grid
  auto cell_info = GetCell(point);
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

comm::CellInfo Grid2D::GetCell(comm::Point2D point) const {
  // calculate approxmate row and column indicies of the point in the grid
  comm::Cell cell = Point2DToCell(point, res_);
  // assume cell is not valid
  bool valid = false;
  // check of the approximated indicies are within the grid boundaries
  if (0 <= cell.row && cell.row < occupancy_.size() &&     // [0, max row)
      0 <= cell.col && cell.col < occupancy_[0].size()) {  // [0, max col)
    valid = true;
  }
  // return result
  return comm::CellInfo(valid, cell);
}

std::pair<int, int> Grid2D::GetSize() const {
  return std::pair<int, int>{height_, width_};
}

double Grid2D::GetResolution() const { return res_; }

std::vector<std::vector<uint8_t>> Grid2D::GetOccupancy() const {
  return occupancy_;
}

void Grid2D::SetOccupancy(const std::vector<std::vector<uint8_t>>& occ) {
  // update occupancy grid information
  occupancy_ = occ;
  height_ = occ.size();
  width_ = occ[0].size();
}