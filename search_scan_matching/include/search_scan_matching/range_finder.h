#pragma once

#include <search_scan_matching/common.h>
#include <search_scan_matching/grid.h>

#include <memory>

class RangeFinder {
 public:
  RangeFinder(const double max_range, const double fov, const double res);
  std::vector<double> Execute(const std::shared_ptr<Grid2D> grid,
                              const utils::Pose2D pose);
  std::vector<std::vector<uint8_t>> ToGrid(const std::shared_ptr<Grid2D> grid,
                                           const utils::Pose2D pose);
  std::vector<double> GetData();
  void Print();

 private:
  double max_range_;
  double fov_;
  double res_;
  int num_rays_;
  std::vector<double> data_;
  std::vector<double> rays_;
};