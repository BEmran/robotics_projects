#include "search_scan_matching/range_finder.h"

#include "search_scan_matching/utils.h"

RangeFinder::RangeFinder(const double max_range, const double fov,
                         const double res)
    : max_range_(max_range), fov_(fov), res_(res) {
  // initialize range data by max_range + 1
  const size_t num_rays = fov_ / res_ + 1;
  data_ = std::vector<double>(num_rays, max_range_ + 1);

  // initialize rays angle
  rays_ = std::vector<double>(num_rays);
  const double start_ang = -fov_ / 2;
  // define rays angle
  for (size_t i = 0; i < num_rays; ++i) {
    rays_[i] = start_ang + i * res_;
  }
}

std::vector<double> RangeFinder::Execute(const std::shared_ptr<Grid2D> grid,
                                         const comm::Pose2D pose) {
  // create sensor frame to map all points from sensor frame to grid frame
  comm::Frame2D sensor_frame(pose, "sensor");

  // initialize range finder data as max range + 1
  data_ = std::vector<double>(rays_.size(), max_range_ + 1);

  // define sensor resolution as half of the grid resolution
  const double ray_res = grid->GetResolution() / 2;
  // calculate range steps
  const size_t num_steps = max_range_ / ray_res + 1;

  // loop through all sensor rays
  for (size_t i = 0; i < rays_.size(); ++i) {
    double ray = rays_[i];  // current ray angle
    double range = 0;
    // loop through all sensor range [0 , max_range]
    for (size_t j = 0; j < num_steps; ++j) {
      range = j * ray_res;  // update range
      // transform the ray's data into a Euclidean point
      const double x = range * std::cos(ray);
      const double y = range * std::sin(ray);
      // transform calculated point from sensor frame to grid frame
      auto transfeared_point = sensor_frame.TransformBack(comm::Point2D(x, y));
      // check if point is occupied
      const bool occupied = grid->IsOccupied(transfeared_point);
      // if occupied, record range and stop searching on this ray
      if (occupied) {
        data_[i] = range;
        break;
      }
    }
  }
  // return sensor data
  return data_;
}

std::vector<std::vector<uint8_t>> RangeFinder::ToGrid(
    const std::shared_ptr<Grid2D> grid, const comm::Pose2D pose) {
  // create sensor frame to map all points from sensor frame to grid frame
  comm::Frame2D sensor_frame(pose, "sensor");
  // get grid size
  const auto size = grid->GetSize();
  // create empty grid to fill it with lasser finding
  auto sensor_occupancy =
      utils::Creat2DArray(size.first, size.second, comm::FREE);
  // place sensor pose on grid
  const auto sensor_pose_cell = grid->GetCell(pose.point);
  // mark sensor pose, make sure the cell index are within the grid
  if (sensor_pose_cell.valid) {
    sensor_occupancy[sensor_pose_cell.cell.row][sensor_pose_cell.cell.col] =
        comm::SENSOR;
  }
  // loop through all sensor rays
  for (int i = 0; i < rays_.size(); ++i) {
    const double ray = rays_[i];    // current ray angle
    const double range = data_[i];  // get ray's range
    // skip ray if range is bigger than lasser mas range
    if (range > max_range_) continue;
    // transform the ray's data into a Euclidean point
    const double x = range * std::cos(ray);
    const double y = range * std::sin(ray);
    // transform calculated point from sensor frame to grid frame
    const auto transfeared_point =
        sensor_frame.TransformBack(comm::Point2D(x, y));
    // transfer calculated point to grid cell coordination
    const auto transfer_cell_info = grid->GetCell(transfeared_point);
    // make sure the cell index are within the grid
    if (transfer_cell_info.valid) {
      // register the cell as occupied
      sensor_occupancy[transfer_cell_info.cell.row]
                      [transfer_cell_info.cell.col] = comm::OCCUPIED;
    }
  }
  // return lasser occupancy
  return sensor_occupancy;
}

std::vector<double> RangeFinder::GetData() { return data_; }
