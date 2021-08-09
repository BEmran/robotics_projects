#include <math.h>
#include <search_scan_matching/common.h>

namespace utils {

// Point2D GridToWorld(const Pose2D& grid_coord, const Point2D& g_point) {
//   double cos_theta = std::cos(grid_coord.theta);
//   double sin_theta = std::sin(grid_coord.theta);
//   double x = cos_theta * g_point.x - sin_theta * g_point.y +
//   grid_coord.point.x; double y = sin_theta * g_point.x + cos_theta *
//   g_point.y + grid_coord.point.y; return Point2D(x, y);
// }

// Point2D WorldToGrid(const Pose2D& world_coord, const Point2D& w_point) {
//   double cos_theta = std::cos(world_coord.theta);
//   double sin_theta = std::sin(world_coord.theta);
//   double x_grid = cos_theta * (w_point.x - world_coord.point.x) +
//                   sin_theta * (w_point.y - world_coord.point.y);
//   double y_grid = -sin_theta * (w_point.x - world_coord.point.x) +
//                   cos_theta * (w_point.y - world_coord.point.y);
//   return Point2D(x_grid, y_grid);
// }

// Pose2D GridToWorld(const Pose2D& grid_coord, const Pose2D& g_pose) {
//   Point2D w_point = GridToWorld(grid_coord, g_pose.point);
//   double theta = g_pose.theta + grid_coord.theta;
//   return Pose2D(w_point, theta);
// }

// Pose2D WorldToGrid(const Pose2D& world_coord, const Pose2D& w_pose) {
//   Point2D g_point = WorldToGrid(world_coord, w_pose.point);
//   double theta = w_pose.theta - world_coord.theta;
//   return Pose2D(g_point, theta);
// }

// Point2D AToFrameB(const Pose2D& a_wrt_b, const Point2D& point_a) {
//   double cos_theta = std::cos(a_wrt_b.theta);
//   double sin_theta = std::sin(a_wrt_b.theta);
//   double x = cos_theta * point_a.x - sin_theta * point_a.y + a_wrt_b.point.x;
//   double y = sin_theta * point_a.x + cos_theta * point_a.y + a_wrt_b.point.y;
//   return Point2D(x, y);
// }

// Pose2D AToFrameB(const Pose2D& a_wrt_b, const Pose2D& point_a) {
//   Point2D point_b = GridToWorld(a_wrt_b, point_a.point);
//   double theta = point_a.theta + a_wrt_b.theta;
//   return Pose2D(point_b, theta);
// }

// Pose2D InvFrame(const Pose2D& coord) {
//   double cos_theta = std::cos(coord.theta);
//   double sin_theta = std::sin(coord.theta);
//   double x_grid = -cos_theta * coord.point.x - sin_theta * coord.point.y;
//   double y_grid = +sin_theta * coord.point.x - cos_theta * coord.point.y;
//   double theta = -coord.theta;
//   return Pose2D(x_grid, y_grid, theta);
// }
}  // namespace utils