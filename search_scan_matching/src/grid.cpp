#include "search_scan_matching/grid.h"

#define DEFAULT_RESOLUTION 0.05

utils::Cell Point2DToCell(const utils::Point2D& point, const double res) {
  int approx_row = std::round(point.x / res);
  int approx_col = std::round(point.y / res);
  return utils::Cell(approx_row, approx_col);
}

Grid2D::Grid2D(const int h, const int w)
    : height_(h),
      width_(w),
      res_(DEFAULT_RESOLUTION),
      frame_(utils::Frame2D(utils::Pose2D(0, 0, 0), "grid")) {
  // initialize free occupancy
  occupancy_ = utils::Creat2DArray(height_, width_, utils::FREE);
}

Grid2D::Grid2D(const utils::Frame2D f, const int h, const int w,
               const double resolution)
    : height_(h), width_(w), res_(resolution), frame_(f) {
  // initialize free occupancy
  occupancy_ = utils::Creat2DArray(height_, width_, utils::FREE);
}

Grid2D::Grid2D(const utils::Frame2D f, const int h, const int w,
               const double resolution,
               const std::vector<std::vector<uint8_t>>& occ)
    : height_(h), width_(w), res_(resolution), frame_(f), occupancy_(occ){};

void Grid2D::CreatBox(const utils::Pose2D origin, const double h,
                      const double w) {
  // create box frame using box origin pose
  utils::Frame2D obs_frame(origin, "box");
  // convert box's actual dimintion to cell resolution
  int box_height = h / res_;
  int box_width = w / res_;
  // loop through all box's points
  for (size_t row = 0; row < box_height; ++row) {
    for (size_t col = 0; col < box_width; ++col) {
      // transfer point from box frame to grid frame
      utils::Point2D box_point(row * res_, col * res_);
      auto point = obs_frame.TransformBack(box_point);
      // calculate approxmate row and column indicies of the point in the grid
      auto cell_info = GetCell(point);
      // if cell is valid place as occupied
      if (cell_info.valid) {
        occupancy_[cell_info.cell.row][cell_info.cell.col] = utils::OCCUPIED;
      }
    }
  }
}

bool Grid2D::IsOccupied(utils::Point2D point) {
  // initialize cell value as free
  uint8_t cell_value = utils::FREE;
  // calculate approxmate row and column indicies of the point in the grid
  auto cell_info = GetCell(point);
  // get occupancy if cell is valid
  if (cell_info.valid) {
    cell_value = occupancy_[cell_info.cell.row][cell_info.cell.col];
  }

  // return occupied if cell has value of 255
  if (cell_value == utils::OCCUPIED) {
    return true;
  }  // if value is not defined
  else if (cell_value != utils::FREE) {
    std::cerr << "undefined cell value of: " << cell_value << std::endl;
  }
  return false;
}

utils::CellInfo Grid2D::GetCell(utils::Point2D point) const {
  // calculate approxmate row and column indicies of the point in the grid
  utils::Cell cell = Point2DToCell(point, res_);
  // assume cell is not valid
  bool valid = false;
  // check of the approximated indicies are within the grid boundaries
  if (0 <= cell.row && cell.row < occupancy_.size() &&     // [0, max row)
      0 <= cell.col && cell.col < occupancy_[0].size()) {  // [0, max col)
    valid = true;
  }
  // return result
  return utils::CellInfo(valid, cell);
}

std::pair<int, int> Grid2D::GetSize() const {
  return std::pair<int, int>{height_, width_};
}

void AddObstacle(const std::vector<std::vector<uint8_t>>& object,
                 const utils::Pose2D origin) {}

double Grid2D::GetResolution() const { return res_; }

std::vector<std::vector<uint8_t>> Grid2D::GetOccupancy() const {
  return occupancy_;
}