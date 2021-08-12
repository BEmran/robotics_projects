#include "search_scan_matching/range_finder.h"

#include "search_scan_matching/utils.h"

RangeFinder::RangeFinder(const double max_range, const double fov,
                         const double res)
    : max_range_(max_range), fov_(fov), res_(res) {
  // initialize data vector
  const size_t num_rays = fov_ / res_;
  data_ = comm::RangeData(num_rays);
}

comm::RangeData RangeFinder::Execute(const Grid2D& grid,
                                     const comm::Pose2D pose) {
  // create sensor frame to map all points from sensor frame to grid frame
  comm::Frame2D sensor_frame(pose, "sensor");
  // define sensor resolution as twice as the grid resolution
  const double distance_res = grid.GetResolution() / 2;
  // calculate range steps
  const size_t num_steps = max_range_ / distance_res + 1;
  // calculate starting angle
  const double ray0 = -fov_ / 2;
  // loop through all sensor rays
  for (double r = 0; r < data_.size(); ++r) {
    // define ray angle
    const double ray = ray0 + r * res_;
    double measurement = max_range_;  // initialize by maximum range
    // loop through sensor range [0 , max_range]
    for (int d = 0; d < num_steps; d++) {
      const double range = d * distance_res;
      // transform the measured data into a cartesian point
      const auto range_point = utils::PolarToCaretssian(ray, range);
      // transform calculated point from sensor frame to grid frame
      auto transfeared_point = sensor_frame.TransformBack(range_point);
      // check if point is occupied
      const bool occupied = grid.IsOccupied(transfeared_point);
      // if occupied stop searching
      if (occupied) {
        measurement = range;
        break;
      }
    }
    // record range and stop searching on this ray
    data_[r] = comm::RangeFinderData(ray, measurement);
  }
  // return sensor data
  return data_;
}

comm::RangeData RangeFinder::GetData() { return data_; }
