/**
 * @file helper.cpp
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief Implemntation of helping functions
 * @version 0.1
 * @date 2021-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "search_scan_matching_test/helper.h"

#include <math.h>

#include "gtest/gtest.h"

std::array<double, 3> MapAToB(double x_g2w, double y_g2w, double t_g2w,
                              double x_g, double y_g, double t_g) {
  double x_w = std::cos(t_g2w) * x_g - std::sin(t_g2w) * y_g + x_g2w;
  double y_w = std::sin(t_g2w) * x_g + std::cos(t_g2w) * y_g + y_g2w;
  double t_w = t_g + t_g2w;
  std::array<double, 3> result({x_w, y_w, t_w});
  return result;
}

std::array<double, 3> Inverse(double x, double y, double t) {
  // inverse translation: R^-1 * (-t)
  double xi = +std::cos(t) * x + std::sin(t) * y;
  double yi = -std::sin(t) * x + std::cos(t) * y;
  // flip angle:
  // double ti = -t;
  std::array<double, 3> result({-xi, -yi, -t});
  return result;
}

void EXPECT_SIZE(const comm::Size& s1, const comm::Size& s2) {
  EXPECT_EQ(s1.height, s2.height);
  EXPECT_EQ(s1.width, s2.width);
}

void EXPECT_POINT2D(const comm::Point2D& p1, const comm::Point2D& p2) {
  EXPECT_NEAR(p1.x, p2.x, EPS);
  EXPECT_NEAR(p1.y, p2.y, EPS);
}

void EXPECT_POSE2D(const comm::Pose2D& p1, const comm::Pose2D& p2) {
  EXPECT_POINT2D(p1.point, p2.point);
  EXPECT_NEAR(p2.theta, p2.theta, EPS);
}

void EXPECT_FRAME2D(const comm::Frame2D& f1, const comm::Frame2D& f2) {
  EXPECT_POSE2D(f1.origin, f2.origin);
  EXPECT_EQ(f1.name, f2.name);
}

void EXPECT_CELL(const comm::Cell& c1, const comm::Cell& c2) {
  EXPECT_EQ(c1.row, c2.row);
  EXPECT_EQ(c1.col, c2.col);
}

void EXPECT_CELLINFO(const comm::CellInfo& c1, const comm::CellInfo& c2) {
  EXPECT_CELL(c1.cell, c2.cell);
  EXPECT_EQ(c1.valid, c2.valid);
}

void EXPECT_RANGE_FINDER(const comm::RangeFinderData& rfd1,
                         const comm::RangeFinderData& rfd2) {
  EXPECT_NEAR(rfd1.range, rfd2.range, EPS);
  EXPECT_NEAR(rfd1.angle, rfd2.angle, EPS);
}