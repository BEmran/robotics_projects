#include <search_scan_matching/range_finder.h>

void Display(const std::vector<std::vector<uint8_t>>& occupancy) {
  for (const auto& row : occupancy) {
    std::cout << "|";
    for (const auto& c : row) {
      if (c == 0)
        std::cout << '-' << "|";
      else if (c == 255)
        std::cout << 'x' << "|";
      else if (c == 1)
        std::cout << 'o' << "|";
      else
        std::cerr << "undefined" << std::endl;
    }
    std::cout << std::endl;
  }
}

RangeFinder::RangeFinder(const double max_range, const double fov,
                         const double res)
    : max_range_(max_range), fov_(fov), res_(res) {
  // initialize range data by max_range + 1
  num_rays_ = fov_ / res_ + 1;
  data_ = std::vector<double>(num_rays_, max_range_ + 1);

  // initialize rays angle
  rays_ = std::vector<double>(num_rays_);
  // std::cout << "ang [ ";
  double start_ang = -fov_ / 2;
  for (int i = 0; i < num_rays_; ++i) {
    rays_[i] = start_ang + i * res_;
    // std::cout << rays_[i] << ", ";
  }
  // std::cout << std::endl;
}

std::vector<double> RangeFinder::Execute(const std::shared_ptr<Grid2D> grid,
                                         const utils::Pose2D pose) {
  // create laser frame to map all points from laser frame to grid frame
  utils::Frame2D laser_frame(pose, "laser");

  // initialize data as max range + 1
  data_ = std::vector<double>(num_rays_, max_range_ + 1);

  // define laser resolution as half of the grid resolution and calculate range
  // steps
  double grid_res = grid->Resolution();
  double ray_res = grid_res / 2;
  int num_steps = max_range_ / ray_res + 1;

  // loop through all sensor rays
  for (int i = 0; i < rays_.size(); ++i) {
    double ray = rays_[i];  // current ray angle
    double range = 0;
    //     grid_res / 2;  // start checking from distance half of grid
    //     resolution
    std::pair<int, int> cell;
    bool occupied = false;
    for (int j = 0; j < num_steps; ++j) {
      range = j * ray_res;  // update range
      // std::cout << "range[" << i << "]: " << range << std::endl;
      // transform the ray's data into a Euclidean point
      double x = range * std::cos(ray);
      double y = range * std::sin(ray);
      // transform calculated point from laser frame to grid frame
      auto transfeared_point = laser_frame.TransformBack(utils::Point2D(x, y));
      // check if point is occupied
      occupied = grid->IsOccupied(transfeared_point);
      // transfer calculated point to grid cell coordination
      cell = grid->GetCell(transfeared_point);
      // if occupied, record range and stop searching on this ray
      if (occupied) {
        data_[i] = range;
        // std::cout << "ray[" << i << "]: " << ray << " range: " << range
        //           << " index: [" << cell.first << ", " << cell.second
        //           << "] occupied \n";
        break;
      }
    }
    // if (!occupied) {
    //   std::cout << "ray[" << i << "]: " << ray << " range: " << range
    //             << " index: [" << cell.first << ", " << cell.second
    //             << "] free \n";
    // }
  }
  return data_;
}

std::vector<std::vector<uint8_t>> RangeFinder::ToGrid(
    const std::shared_ptr<Grid2D> grid, const utils::Pose2D pose,
    const bool display) {
  // create laser frame to map all points from laser frame to grid frame
  utils::Frame2D laser_frame(pose, "laser");
  // get grid size
  auto size = grid->GetSize();
  // create empty grid to fill it with lasser finding
  auto laser_occupancy = Creat2DArray(size.first, size.second, 0);
  // place laser pose on grid
  auto laser_cell = grid->GetCell(pose.point);
  // make sure the cell index are within the grid
  if (laser_cell.first != -1 && laser_cell.second != -1) {
    laser_occupancy[laser_cell.first][laser_cell.second] = 1;
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
    auto cell = grid->GetCell(transfeared_point);
    // make sure the cell index are within the grid
    if (cell.first != -1 && cell.second != -1) {
      // register the cell as occupied
      laser_occupancy[cell.first][cell.second] = 255;
    }
  }
  // display laser occupancy grid
  if (display) Display(laser_occupancy);
  return laser_occupancy;
}

std::vector<double> RangeFinder::GetData() { return data_; }

void RangeFinder::Print() {
  std::cout << "Laser: [";
  for (int i = 0; i < data_.size(); ++i) {
    std::cout << data_[i] << " ";
  }
  std::cout << std::endl;
}
