#include <math.h>
#include <search_scan_matching/common.h>

#include "gtest/gtest.h"

using namespace comm;

const double EPS = 0.001;

/*****************************************************************************
 * HELPING FUNCTIONS
 *****************************************************************************/

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

// std::array<double, 3> FromWorldToGrid(double x_w2g, double y_w2g, double
// t_w2g,
//                                       double x_w, double y_w, double t_w) {
//   double x_g =
//       std::cos(t_w2g) * (x_w - x_w2g) + std::sin(t_w2g) * (y_w - y_w2g);
//   double y_g =
//       -std::sin(t_w2g) * (x_w - x_w2g) + std::cos(t_w2g) * (y_w - y_w2g);
//   double t_g = t_w - t_w2g;
//   std::array<double, 3> result({x_g, y_g, t_g});
//   return result;
// }

/*****************************************************************************
 * Extra ASSERTIONS
 *****************************************************************************/
void EXPECT_POINT2D(const double x, const double y, const Point2D& p) {
  EXPECT_NEAR(x, p.x, EPS);
  EXPECT_NEAR(y, p.y, EPS);
}

void EXPECT_POINT2D(const Point2D& p1, const Point2D& p2) {
  EXPECT_NEAR(p1.x, p2.x, EPS);
  EXPECT_NEAR(p1.y, p2.y, EPS);
}

void EXPECT_POSE2D(const double x, const double y, const double t,
                   const Pose2D& p) {
  EXPECT_NEAR(x, p.point.x, EPS);
  EXPECT_NEAR(y, p.point.y, EPS);
  EXPECT_NEAR(t, p.theta, EPS);
}

void EXPECT_POSE2D(const Pose2D& p1, const Pose2D& p2) {
  EXPECT_NEAR(p2.point.x, p2.point.x, EPS);
  EXPECT_NEAR(p2.point.y, p2.point.y, EPS);
  EXPECT_NEAR(p2.theta, p2.theta, EPS);
}

void EXPECT_FRAME2D(const Frame2D& f1, const Frame2D& f2) {
  EXPECT_POSE2D(f1.origin, f2.origin);
  EXPECT_EQ(f1.name, f2.name);
}

/*****************************************************************************
 * TESTING CASES
 *****************************************************************************/

/**
 * @brief tests contstraction of Point2D class
 *
 */
TEST(Point2D, Definition) {
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
  EXPECT_POINT2D(0, 0, p00);
  EXPECT_POINT2D(x, y, p12);
  EXPECT_POINT2D(-x, -y, pn1n2);
  EXPECT_POINT2D(x, y, tmp1);
  EXPECT_POINT2D(-x, -y, tmp2);
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
  EXPECT_POINT2D(x1 + x2, y1 + y2, p1 + p2);
  EXPECT_POINT2D(x1 - x2, y1 - y2, p1 - p2);
}

/**
 * @brief tests operation of Pose2D class
 *
 */
TEST(Pose2D, Definition) {
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
  EXPECT_POSE2D(0, 0, 0, p00);
  EXPECT_POSE2D(x, y, t, p12);
  EXPECT_POSE2D(-x, -y, -t, pn1n2);
  EXPECT_POSE2D(x, y, t, tmp1);
  EXPECT_POSE2D(-x, -y, -t, tmp2);
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
  EXPECT_POSE2D(x1 + x2, y1 + y2, t1 + t2, p1 + p2);
  EXPECT_POSE2D(x1 - x2, y1 - y2, t1 - t2, p1 - p2);
}

/**
 * @brief tests contstraction of Frame2D class
 *
 */
TEST(Frame2D, Definition) {
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
  EXPECT_POSE2D(0, 0, 0, f0.origin);
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
  EXPECT_POSE2D(ans[0], ans[1], ans[2], world.origin);

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
  EXPECT_POINT2D(ans1[0], ans1[1], world_point);

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
TEST(RangeFinderData, Definition) {
  // arrange
  const double angle = 1.5;
  const double range = 4.3;
  // act
  const RangeFinderData rfd;
  const RangeFinderData rfd1(angle, range);
  // assert
  EXPECT_EQ(0, rfd.angle);
  EXPECT_EQ(0, rfd.range);
  EXPECT_EQ(angle, rfd1.angle);
  EXPECT_EQ(range, rfd1.range);
}

/**
 * @brief tests contstraction of Size class
 *
 */
TEST(Size, Definition) {
  // arrange
  const size_t height = 1.5;
  const size_t width = 4.3;
  // act
  const Size s;
  const Size s1(height, width);
  // assert
  EXPECT_EQ(0, s.height);
  EXPECT_EQ(0, s.width);
  EXPECT_EQ(height, s1.height);
  EXPECT_EQ(width, s1.width);
}

/**
 * @brief tests contstraction of Size class
 *
 */
TEST(Definition, Occupancy) {
  // assert
  EXPECT_EQ(0, FREE);
  EXPECT_EQ(1, SENSOR);
  EXPECT_EQ(255, OCCUPIED);
}

/**
 * @brief tests contstraction of Size class
 *
 */
TEST(Cell, Definition) {
  // arrange
  const int row = 1;
  const int col = 4;
  // act
  const Cell c;
  const Cell c1(row, col);
  // assert
  EXPECT_EQ(0, c.row);
  EXPECT_EQ(0, c.col);
  EXPECT_EQ(row, c1.row);
  EXPECT_EQ(col, c1.col);
}

/**
 * @brief tests contstraction of Size class
 *
 */
TEST(CellInfo, Definition) {
  // arrange
  const int row = 1;
  const int col = 4;
  const bool valid = true;
  // act
  CellInfo ci;
  CellInfo ci1(valid, Cell(row, col));
  // assert
  EXPECT_EQ(0, ci.cell.row);
  EXPECT_EQ(0, ci.cell.col);
  EXPECT_EQ(false, ci.valid);
  EXPECT_EQ(row, ci1.cell.row);
  EXPECT_EQ(col, ci1.cell.col);
  EXPECT_EQ(valid, ci1.valid);
}