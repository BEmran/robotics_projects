/**
 * @file comm_utest.cpp
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief unit tests for common objects
 * @version 0.1
 * @date 2021-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "gtest/gtest.h"
#include "search_scan_matching/common.h"
#include "search_scan_matching_test/helper.h"

using namespace comm;

/**
 * @brief tests contstraction of Point2D class
 *
 */
TEST(Point2D, Construction) {
  // arrange
  const double x = 1;
  const double y = 2;
  // act
  const Point2D p00;
  const Point2D p12(x, y);
  const Point2D pn1n2(-x, -y);
  const Point2D tmp1(p12);
  const Point2D tmp2 = pn1n2;
  // assert
  EXPECT_POINT2D(Point2D(0, 0), p00);
  EXPECT_POINT2D(Point2D(x, y), p12);
  EXPECT_POINT2D(Point2D(-x, -y), pn1n2);
  EXPECT_POINT2D(Point2D(x, y), tmp1);
  EXPECT_POINT2D(Point2D(-x, -y), tmp2);
}

/**
 * @brief tests Point2D class arithmetic operations
 *
 */
TEST(Point2D, operations) {
  // arrange
  const double x1 = 1, y1 = 2;
  const double x2 = 3, y2 = 4;
  const double mul = 2;
  const double div = 3;
  // act
  const Point2D p1(x1, y1);
  const Point2D p2(x2, y2);
  // assert
  EXPECT_POINT2D(Point2D(x1 + x2, y1 + y2), p1 + p2);
  EXPECT_POINT2D(Point2D(x1 - x2, y1 - y2), p1 - p2);
}

/**
 * @brief tests operation of Pose2D class
 *
 */
TEST(Pose2D, Construction) {
  // arrange
  const double x = 3;
  const double y = 4;
  const double t = 1.5;
  // act
  const Pose2D p00;
  const Pose2D p12(x, y, t);
  const Pose2D pn1n2(Point2D(-x, -y), -t);
  const Pose2D tmp1(p12);
  const Pose2D tmp2 = pn1n2;
  // assert
  EXPECT_POSE2D(Pose2D(0, 0, 0), p00);
  EXPECT_POSE2D(Pose2D(x, y, t), p12);
  EXPECT_POSE2D(Pose2D(-x, -y, -t), pn1n2);
  EXPECT_POSE2D(Pose2D(x, y, t), tmp1);
  EXPECT_POSE2D(Pose2D(-x, -y, -t), tmp2);
}

/**
 * @brief tests Pose2D class arithmetic operation
 *
 */
TEST(Pose2D, operations) {
  // arrange
  const double x1 = 1, y1 = 2, t1 = 1.3;
  const double x2 = 3, y2 = 4, t2 = -0.3;
  // act
  const Pose2D p1(x1, y1, t1);
  const Pose2D p2(x2, y2, t2);
  // assert
  EXPECT_POSE2D(Pose2D(x1 + x2, y1 + y2, t1 + t2), p1 + p2);
  EXPECT_POSE2D(Pose2D(x1 - x2, y1 - y2, t1 - t2), p1 - p2);
}

/**
 * @brief tests contstraction of Frame2D class
 *
 */
TEST(Frame2D, Construction) {
  // arrange
  const double x = 3;
  const double y = 4;
  const double t = 1.5;
  const std::string id = "frame1";
  // act
  const Frame2D f0;
  const Pose2D origin(x, y, t);
  const Frame2D f1(origin, id);
  // assert
  EXPECT_POSE2D(Pose2D(0, 0, 0), f0.origin);
  EXPECT_POSE2D(origin, f1.origin);
}

/**
 * @brief tests Frame2D class inverse
 *
 */
TEST(Frame2D, inverse) {
  // point in frame g
  const double x = 1, y = 2;
  // coordination of frame g wrt to frame w
  const double x_g = 2, y_g = 1, t_g = 0.785398;
  Frame2D grid(Pose2D(x_g, y_g, t_g), "");
  // coordination of frame w seen by frame g
  const auto ans = Inverse(x_g, y_g, t_g);
  const auto world = grid.Inverse();
  EXPECT_POSE2D(Pose2D(ans[0], ans[1], ans[2]), world.origin);

  const auto grid_tmp = world.Inverse();
  EXPECT_POSE2D(grid.origin, grid_tmp.origin);
}

/**
 * @brief tests Frame2D class forward and and backward transformation functions
 *
 */
TEST(Frame2D, transform) {
  // point in frame g
  const double x = 1, y = 2;
  // coordination of frame g seen by frame w
  const double x_g = 2, y_g = 1, t_g = 0.785398;
  // act
  const Point2D grid_point(x_g, y_g);
  const Frame2D grid(Pose2D(x_g, y_g, t_g), "grid wrt world");
  const Frame2D world = grid.Inverse();

  // Transform point from grid to world
  const Point2D world_point = grid.TransformBack(grid_point);
  const auto ans1 = MapAToB(grid.origin.point.x, grid.origin.point.y,
                            grid.origin.theta, grid_point.x, grid_point.y, 0);
  EXPECT_POINT2D(Point2D(ans1[0], ans1[1]), world_point);

  // two way grid->world->grid
  const Point2D back_to_grid = world.TransformBack(world_point);
  EXPECT_POINT2D(grid_point, back_to_grid);

  // Transform point from world to grid using grid frame
  const Point2D back_point_forward = grid.TransformForward(world_point);
  EXPECT_POINT2D(grid_point, back_point_forward);
}

/**
 * @brief tests contstraction of RangeFinderData class
 *
 */
TEST(RangeFinderData, Construction) {
  // arrange
  const double angle = 1.5;
  const double range = 4.3;
  // act
  const RangeFinderData rfd;
  const RangeFinderData rfd1(angle, range);
  // assert
  EXPECT_RANGE_FINDER(RangeFinderData(0, 0), rfd);
  EXPECT_RANGE_FINDER(RangeFinderData(angle, range), rfd1);
}

/**
 * @brief tests contstraction of Size class
 *
 */
TEST(Size, Construction) {
  // arrange
  const size_t height = 1.5;
  const size_t width = 4.3;
  // act
  const Size s;
  const Size s1(height, width);
  // assert
  EXPECT_SIZE(Size(0, 0), s);
  EXPECT_SIZE(Size(height, width), s1);
}

/**
 * @brief tests contstraction of Size class
 *
 */
TEST(Construction, Occupancy) {
  // assert
  EXPECT_EQ(0, FREE);
  EXPECT_EQ(1, SENSOR);
  EXPECT_EQ(255, OCCUPIED);
}

/**
 * @brief tests contstraction of Size class
 *
 */
TEST(Cell, Construction) {
  // arrange
  const int row = 1;
  const int col = 4;
  // act
  const Cell c;
  const Cell c1(row, col);
  // assert
  EXPECT_CELL(Cell(0, 0), c);
  EXPECT_CELL(Cell(row, col), c1);
}

/**
 * @brief tests contstraction of Size class
 *
 */
TEST(CellInfo, Construction) {
  // arrange
  const int row = 1;
  const int col = 4;
  const bool valid = true;
  // act
  CellInfo ci;
  CellInfo ci1(valid, Cell(row, col));
  // assert
  EXPECT_CELLINFO(CellInfo(false, Cell(0, 0)), ci);
  EXPECT_CELLINFO(CellInfo(true, Cell(row, col)), ci1);
}