#include <math.h>
#include <search_scan_matching/common.h>

#include <array>

#include "gtest/gtest.h"

using namespace utils;

const double EPS = 0.000001;

// helping_function
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

/****************************************************************************/
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
  EXPECT_EQ(f1.id, f2.id);
}
/****************************************************************************/
TEST(Point2D, Definition) {
  // arrange
  double x = 1;
  double y = 2;
  // act
  Point2D p00;
  Point2D p12(x, y);
  Point2D pn1n2(-x, -y);
  Point2D tmp1(p12);
  Point2D tmp2 = pn1n2;
  // assert
  EXPECT_POINT2D(0, 0, p00);
  EXPECT_POINT2D(x, y, p12);
  EXPECT_POINT2D(-x, -y, pn1n2);
  EXPECT_POINT2D(x, y, tmp1);
  EXPECT_POINT2D(-x, -y, tmp2);
}

TEST(Point2D, operations) {
  // arrange
  double x1 = 1, y1 = 2;
  double x2 = 3, y2 = 4;
  double mul = 2;
  double div = 3;
  // act
  Point2D p1(x1, y1);
  Point2D p2(x2, y2);
  // assert
  // add:
  EXPECT_POINT2D(x1 + x2, y1 + y2, p1 + p2);
  Point2D tmp_add(p1);
  tmp_add += p2;
  EXPECT_POINT2D(x1 + x2, y1 + y2, tmp_add);
  // sub:
  EXPECT_POINT2D(x1 - x2, y1 - y2, p1 - p2);
  Point2D tmp_sub(p1);
  tmp_sub -= p2;
  EXPECT_POINT2D(x1 - x2, y1 - y2, tmp_sub);
  // mul
  EXPECT_POINT2D(x1 * mul, y1 * mul, p1 * mul);
  Point2D tmp_mul(p1);
  tmp_mul *= mul;
  EXPECT_POINT2D(x1 * mul, y1 * mul, tmp_mul);
  // div
  EXPECT_POINT2D(x1 / div, y1 / div, p1 / div);
  Point2D tmp_div(p1);
  tmp_div /= -div;
  EXPECT_POINT2D(x1 / -div, y1 / -div, tmp_div);
}

TEST(Pose2D, Definition) {
  // arrange
  double x = 3;
  double y = 4;
  double t = 1.5;
  // act
  Pose2D p00;
  Pose2D p12(x, y, t);
  Pose2D pn1n2(Point2D(-x, -y), -t);
  Pose2D tmp1(p12);
  Pose2D tmp2 = pn1n2;
  // assert
  EXPECT_POSE2D(0, 0, 0, p00);
  EXPECT_POSE2D(x, y, t, p12);
  EXPECT_POSE2D(-x, -y, -t, pn1n2);
  EXPECT_POSE2D(x, y, t, tmp1);
  EXPECT_POSE2D(-x, -y, -t, tmp2);
}

TEST(Pose2D, operations) {
  // arrange
  double x1 = 1, y1 = 2, t1 = 1.3;
  double x2 = 3, y2 = 4, t2 = -0.3;
  // act
  Pose2D p1(x1, y1, t1);
  Pose2D p2(x2, y2, t2);
  // assert
  // add:
  EXPECT_POSE2D(x1 + x2, y1 + y2, t1 + t2, p1 + p2);
  Pose2D tmp_add(p1);
  tmp_add += p2;
  EXPECT_POSE2D(x1 + x2, y1 + y2, t1 + t2, tmp_add);
  // sub:
  EXPECT_POSE2D(x1 - x2, y1 - y2, t1 - t2, p1 - p2);
  Pose2D tmp_sub(p1);
  tmp_sub -= p2;
  EXPECT_POSE2D(x1 - x2, y1 - y2, t1 - t2, tmp_sub);
}

TEST(Frame2D, Definition) {
  // arrange
  double x = 3;
  double y = 4;
  double t = 1.5;
  std::string id = "frame1";
  // act
  Frame2D f0;
  Pose2D origin(x, y, t);
  Frame2D f1(origin, id);
  // assert
  EXPECT_POSE2D(0, 0, 0, f0.origin);
  EXPECT_POSE2D(origin, f1.origin);
}

TEST(Frame2D, inverse) {
  // point in frame g
  double x = 1, y = 2;
  // coordination of frame g wrt to frame w
  double x_g = 2, y_g = 1, t_g = 0.785398;
  Frame2D grid(Pose2D(x_g, y_g, t_g), "");
  // coordination of frame w seen by frame g
  auto ans = Inverse(x_g, y_g, t_g);
  auto world = grid.Inverse();
  EXPECT_POSE2D(ans[0], ans[1], ans[2], world.origin);

  auto grid_tmp = world.Inverse();
  EXPECT_POSE2D(grid.origin, grid_tmp.origin);
}

TEST(Frame2D, transform) {
  // point in frame g
  double x = 1, y = 2;
  // coordination of frame g seen by frame w
  double x_g = 2, y_g = 1, t_g = 0.785398;
  // act
  Point2D grid_point(x_g, y_g);
  Frame2D grid(Pose2D(x_g, y_g, t_g), "grid wrt world");
  Frame2D world = grid.Inverse();

  // Transform point from grid to world
  Point2D world_point = grid.TransformBack(grid_point);
  auto ans1 = MapAToB(grid.origin.point.x, grid.origin.point.y,
                      grid.origin.theta, grid_point.x, grid_point.y, 0);
  EXPECT_POINT2D(ans1[0], ans1[1], world_point);

  // two way grid->world->grid
  Point2D back_to_grid = world.TransformBack(world_point);
  EXPECT_POINT2D(grid_point, back_to_grid);

  // Transform point from world to grid using grid frame
  Point2D back_point_forward = grid.TransformForward(world_point);
  EXPECT_POINT2D(grid_point, back_point_forward);
}
