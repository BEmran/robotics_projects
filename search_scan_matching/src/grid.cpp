#include "search_scan_matching/grid.h"

Grid2D::Grid2D(const int h, const int w)
    : height_(h),
      width_(w),
      res_(0.05),
      frame_(utils::Frame2D(utils::Pose2D(0, 0, 0), "grid")) {
  occupancy_ = Creat2DArray(height_, width_, 0);
}

Grid2D::Grid2D(const utils::Frame2D f, const int h, const int w,
               const double resolution)
    : height_(h), width_(w), res_(resolution), frame_(f) {
  occupancy_ = Creat2DArray(height_, width_, 0);
}

Grid2D::Grid2D(const utils::Frame2D f, const int h, const int w,
               const double resolution,
               const std::vector<std::vector<uint8_t>>& occ)
    : height_(h), width_(w), res_(resolution), frame_(f), occupancy_(occ){};

std::vector<std::vector<uint8_t>> Creat2DArray(int h, int w,
                                               const uint8_t val) {
  std::vector<uint8_t> row(w, 0);
  std::vector<std::vector<uint8_t>> arr(h, row);
  return arr;
}

void Grid2D::Display() {
  for (const auto& row : occupancy_) {
    std::cout << "|";
    for (const auto& c : row) {
      if (c == 0)
        std::cout << '-' << "|";
      else if (c == 255)
        std::cout << 'x' << "|";
      else
        std::cerr << "undefined" << std::endl;
    }
    std::cout << std::endl;
  }
}

void Grid2D::CreatBox(const double h, const double w,
                      const utils::Pose2D origin) {
  int box_height = h / res_;
  int box_width = w / res_;
  utils::Frame2D obs_frame(origin, "box");
  for (size_t i = 0; i < box_height; ++i) {
    for (size_t j = 0; j < box_width; ++j) {
      uint8_t ele = 255;
      auto point = obs_frame.TransformBack(utils::Point2D(i * res_, j * res_));
      int approx_row = std::round(point.x / res_);
      int approx_col = std::round(point.y / res_);
      if (0 <= approx_row && approx_row < occupancy_.size() &&
          0 <= approx_col && approx_col < occupancy_[0].size()) {
        occupancy_[approx_row][approx_col] = ele;
      }
    }
  }
}

bool Grid2D::IsOccupied(utils::Point2D point) {
  uint8_t cell = 0;
  int approx_row = std::round(point.x / res_);
  int approx_col = std::round(point.y / res_);
  // std::cout << approx_row << " " << approx_col << std::endl;
  if (0 <= approx_row && approx_row < occupancy_.size() && 0 <= approx_col &&
      approx_col < occupancy_[0].size()) {
    cell = occupancy_[approx_row][approx_col];
  }
  if (cell == 0) {
    return false;
  } else if (cell == 255) {
    return true;
  } else {
    std::cerr << "undefined cell value" << std::endl;
  }
}
std::pair<int, int> Grid2D::GetCell(utils::Point2D point) const {
  std::pair<int, int> cell{-1, -1};
  int approx_row = std::round(point.x / res_);
  int approx_col = std::round(point.y / res_);
  if (0 <= approx_row && approx_row < occupancy_.size() && 0 <= approx_col &&
      approx_col < occupancy_[0].size()) {
    cell.first = approx_row;
    cell.second = approx_col;
  } else {
    // std::cerr << "the point: " << point << " is out of range" << std::endl;
  }
  return cell;
}

std::pair<int, int> Grid2D::GetSize() const {
  return std::pair<int, int>{height_, width_};
}

void AddObstacle(const std::vector<std::vector<uint8_t>>& object,
                 const utils::Pose2D origin) {}

double Grid2D::Resolution() const { return res_; }

std::vector<std::vector<uint8_t>> Grid2D::GetOccupancy() const {
  return occupancy_;
}