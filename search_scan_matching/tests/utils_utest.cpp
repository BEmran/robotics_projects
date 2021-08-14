/**
 * @file utils_utest.cpp
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief unit tests for utils fintions
 * @version 0.1
 * @date 2021-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <math.h>

#include "gtest/gtest.h"
#include "search_scan_matching/common.h"
#include "search_scan_matching/utils.h"
#include "search_scan_matching_test/helper.h"

using namespace utils;
using namespace comm;

/**
 * @brief tests results of L2Norm function
 *
 */
TEST(L2Norm, Construction) {
  // arrange
  const double x1 = 1.6, y1 = 2.0, t1 = 0.5;
  const double x2 = 2.7, y2 = -2.0, t2 = 1.1;
  const Pose2D p1(x1, y1, t1);
  const Pose2D p2(x2, y2, t2);
  // act
  // zero poses
  const double l0 = utils::L2Norm(Pose2D(), Pose2D());
  // same pose
  const double l1 = utils::L2Norm(p1, p1);
  const double l2 = utils::L2Norm(p2, p2);
  // check for similarity
  const double l3 = utils::L2Norm(p1, p2);
  const double l4 = utils::L2Norm(p2, p1);
  // assert
  EXPECT_NEAR(0, l0, EPS);
  EXPECT_NEAR(0, l1, EPS);
  EXPECT_NEAR(0, l2, EPS);
  const double ans = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2) +
                               std::pow(t1 - t2, 2));
  EXPECT_NEAR(ans, l3, EPS);
  EXPECT_NEAR(l3, l4, EPS);
}

/**
 * @brief tests creation of Creat2DArray function
 *
 */
TEST(Creat2DArray, Construction) {
  // arrange
  const size_t height = 5, width = 7;
  const uint8_t val = 1;
  const Size size(height, width);
  // act
  const std::vector<std::vector<uint8_t>> zero_arr =
      Creat2DArray(Size(0, 0), 0);
  const std::vector<std::vector<uint8_t>> col_arr =
      Creat2DArray(Size(height, 1), 0);
  const std::vector<std::vector<uint8_t>> row_arr =
      Creat2DArray(Size(1, width), 0);
  const std::vector<std::vector<uint8_t>> arr =
      Creat2DArray(Size(height, width), val);
  int sum = 0;
  for (const auto& vec : arr) {
    for (const auto& ele : vec) {
      sum += ele;
    }
  }

  // assert
  EXPECT_EQ(0, zero_arr.size());
  EXPECT_EQ(height, col_arr.size());
  EXPECT_EQ(1, col_arr[0].size());
  EXPECT_EQ(1, row_arr.size());
  EXPECT_EQ(width, row_arr[0].size());
  EXPECT_EQ(height, arr.size());
  EXPECT_EQ(width, arr[0].size());
  EXPECT_EQ(sum, height * width * val);
}

/**
 * @brief tests result of GenerateRandPose function
 *
 */
TEST(GenerateRandPose, Construction) {
  // arrange
  const double linear_stddev = 0.01;
  const double angular_stddev = 0.02;
  // act
  const Pose2D zero_pose = GenerateRandPose(0, 0);
  const Pose2D zero_ang = GenerateRandPose(linear_stddev, 0);
  const Pose2D zero_lin = GenerateRandPose(0, linear_stddev);
  const Pose2D pose = GenerateRandPose(linear_stddev, linear_stddev);
  // assert
  const double lin_max = 4 * linear_stddev;
  const double lin_min = -4 * linear_stddev;
  const double ang_max = 4 * angular_stddev;
  const double ang_min = -4 * angular_stddev;

  EXPECT_POSE2D(comm::Pose2D(0, 0, 0), zero_pose);
  EXPECT_TRUE(lin_min < zero_ang.point.x && zero_ang.point.x < lin_max);
  EXPECT_TRUE(lin_min < zero_ang.point.y && zero_ang.point.y < lin_max);
  EXPECT_EQ(0, zero_ang.theta);
  EXPECT_EQ(0, zero_lin.point.x);
  EXPECT_EQ(0, zero_lin.point.y);
  EXPECT_TRUE(ang_min < zero_lin.theta && zero_lin.theta < ang_max);
  EXPECT_TRUE(lin_min < pose.point.x && pose.point.x < lin_max);
  EXPECT_TRUE(lin_min < pose.point.y && pose.point.y < lin_max);
  EXPECT_TRUE(ang_min < pose.theta && pose.theta < ang_max);
}

/**
 * @brief tests transformaition result of PolarToCaretssian function
 *
 */
TEST(PolarToCaretssian, Construction) {
  // arrange
  const double angle = 1.2, radius = 7.1;
  const Point2D ans(radius * std::cos(angle), radius * std::sin(angle));
  // act
  const Point2D zero_ang = PolarToCaretssian(0, radius);
  const Point2D zero_rad = PolarToCaretssian(angle, 0);
  const Point2D point = PolarToCaretssian(angle, radius);
  const Point2D neg_ang = PolarToCaretssian(-angle, radius);
  const Point2D neg_rad = PolarToCaretssian(angle, -radius);
  const Point2D double_neg = PolarToCaretssian(-angle, -radius);

  // assert
  EXPECT_POINT2D(comm::Point2D(radius, 0), zero_ang);
  EXPECT_POINT2D(comm::Point2D(0, 0), zero_rad);
  EXPECT_POINT2D(ans, point);
  EXPECT_POINT2D(comm::Point2D(ans.x, -ans.y), neg_ang);
  EXPECT_POINT2D(comm::Point2D(-ans.x, -ans.y), neg_rad);
  EXPECT_POINT2D(comm::Point2D(-ans.x, ans.y), double_neg);
}

/**
 * @brief tests transformaition result of PointToCell function
 *
 */
TEST(PointToCell, Construction) {
  // arrange
  const double x = 1, y = 2;
  const size_t h = 4, w = 8;
  const double res = 0.5;
  const Point2D point(x, y);
  const Size size(h, w);
  const Cell cell(x / res, y / res);
  // act
  const CellInfo zero = PointToCell(Point2D(0, 0), size, res);
  const CellInfo not_valid1 = PointToCell(Point2D(res, -res), size, res);
  const CellInfo not_valid2 = PointToCell(Point2D(-res, res), size, res);
  const CellInfo not_valid3 = PointToCell(Point2D(h / res, 0), size, res);
  const CellInfo not_valid4 = PointToCell(Point2D(0, w / res), size, res);
  const CellInfo not_valid5 = PointToCell(Point2D(0, 0), Size(0, 0), res);
  const CellInfo cell_ans = PointToCell(point, size, res);
  // assert
  EXPECT_CELLINFO(CellInfo(true, Cell(0, 0)), zero);
  EXPECT_FALSE(not_valid1.valid);
  EXPECT_FALSE(not_valid2.valid);
  EXPECT_FALSE(not_valid3.valid);
  EXPECT_FALSE(not_valid4.valid);
  EXPECT_FALSE(not_valid5.valid);
  EXPECT_CELLINFO(CellInfo(true, cell), cell_ans);
}