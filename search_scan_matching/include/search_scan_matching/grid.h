#pragma once

#include <iostream>
#include <vector>

#include "search_scan_matching/common.h"

class Grid2D {
 public:
  Grid2D(const int h, const int w);
  Grid2D(const utils::Frame2D f, const int h, const int w,
         const double resolution);
  Grid2D(const utils::Frame2D f, const int h, const int w,
         const double resolution, const std::vector<std::vector<uint8_t>>& occ);
  void AddObstacle(const std::vector<std::vector<uint8_t>>& object,
                   const utils::Pose2D origin);

  void CreatBox(const utils::Pose2D origin, const double h, const double w);
  bool IsOccupied(utils::Point2D point);
  utils::CellInfo GetCell(utils::Point2D point) const;
  std::pair<int, int> GetSize() const;
  double GetResolution() const;
  std::vector<std::vector<uint8_t>> GetOccupancy() const;

 private:
  int height_;
  int width_;
  double res_;
  utils::Frame2D frame_;
  std::vector<std::vector<uint8_t>> occupancy_;
};