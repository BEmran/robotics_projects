#pragma once

#include <iostream>
#include <vector>

#include "search_scan_matching/common.h"

std::vector<std::vector<uint8_t>> Creat2DArray(int h, int w, const uint8_t val);

class Grid2D {
 public:
  Grid2D(const int h, const int w);
  Grid2D(const utils::Frame2D f, const int h, const int w,
         const double resolution);
  Grid2D(const utils::Frame2D f, const int h, const int w,
         const double resolution, const std::vector<std::vector<uint8_t>>& occ);
  void Display();
  void AddObstacle(const std::vector<std::vector<uint8_t>>& object,
                   const utils::Pose2D origin);

  void CreatBox(const double h, const double w, const utils::Pose2D origin);
  bool IsOccupied(utils::Point2D point);
  double Resolution() const;
  std::pair<int, int> GetCell(utils::Point2D point) const;
  std::pair<int, int> GetSize() const;
  std::vector<std::vector<uint8_t>> GetOccupancy() const;

 private:
  int height_;
  int width_;
  double res_;
  utils::Frame2D frame_;
  std::vector<std::vector<uint8_t>> occupancy_;
};

void dis();