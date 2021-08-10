#include "search_scan_matching/range_finder.h"

RangeFinder::RangeFinder(const double max_range, const double fov,
                         const double res)
    : max_range_(max_range), fov_(fov), res_(res) {
  // initialize range data by max_range + 1
  num_rays_ = fov_ / res_ + 1;
  data_ = std::vector<double>(num_rays_, max_range_ + 1);

  // initialize rays angle
  rays_ = std::vector<double>(num_rays_);
  double start_ang = -fov_ / 2;
  // define rays angle
  for (int i = 0; i < num_rays_; ++i) {
    rays_[i] = start_ang + i * res_;
  }
}

std::vector<double> RangeFinder::Execute(const std::shared_ptr<Grid2D> grid,
                                         const utils::Pose2D pose) {
  // create laser frame to map all points from laser frame to grid frame
  utils::Frame2D laser_frame(pose, "laser");

  // initialize range finder data as max range + 1
  data_ = std::vector<double>(num_rays_, max_range_ + 1);

  // define laser resolution as half of the grid resolution
  double ray_res = grid->GetResolution() / 2;
  // calculate range steps
  int num_steps = max_range_ / ray_res + 1;

  // loop through all sensor rays
  for (int i = 0; i < rays_.size(); ++i) {
    double ray = rays_[i];  // current ray angle
    double range = 0;
    // loop through all laser range [0 , max_range]
    for (int j = 0; j < num_steps; ++j) {
      range = j * ray_res;  // update range
      // transform the ray's data into a Euclidean point
      double x = range * std::cos(ray);
      double y = range * std::sin(ray);
      // transform calculated point from laser frame to grid frame
      auto transfeared_point = laser_frame.TransformBack(utils::Point2D(x, y));
      // check if point is occupied
      bool occupied = grid->IsOccupied(transfeared_point);
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
    const std::shared_ptr<Grid2D> grid, const utils::Pose2D pose) {
  // create laser frame to map all points from laser frame to grid frame
  utils::Frame2D laser_frame(pose, "laser");
  // get grid size
  auto size = grid->GetSize();
  // create empty grid to fill it with lasser finding
  auto laser_occupancy =
      utils::Creat2DArray(size.first, size.second, utils::FREE);
  // place laser pose on grid
  auto laser_cell = grid->GetCell(pose.point);
  // make sure the cell index are within the grid
  if (laser_cell.valid) {
    laser_occupancy[laser_cell.cell.row][laser_cell.cell.col] = utils::SENSOR;
  }
  // loop through all sensor rays
  for (int i = 0; i < rays_.size(); ++i) {
    double ray = rays_[i];    // current ray angle
    double range = data_[i];  // get ray's range
    // skip ray if range is bigger than lasser mas range
    if (range > max_range_) continue;
    // transform the ray's data into a Euclidean point
    double x = range * std::cos(ray);
    double y = range * std::sin(ray);
    // transform calculated point from laser frame to grid frame
    auto transfeared_point = laser_frame.TransformBack(utils::Point2D(x, y));
    // transfer calculated point to grid cell coordination
    auto transfer_cell_info = grid->GetCell(transfeared_point);
    // make sure the cell index are within the grid
    if (transfer_cell_info.valid) {
      // register the cell as occupied
      laser_occupancy[transfer_cell_info.cell.row]
                     [transfer_cell_info.cell.col] = utils::OCCUPIED;
    }
  }
  // return lasser occupancy
  return laser_occupancy;
}

std::vector<double> RangeFinder::GetData() { return data_; }

void RangeFinder::Print() {
  std::cout << "Range Finder Data: [";
  // loop through data
  for (int i = 0; i < data_.size(); ++i) {
    std::cout << data_[i] << " ";
  }
  std::cout << std::endl;
}
