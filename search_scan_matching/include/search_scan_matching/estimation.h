#include <memory>
#include <vector>

#include "search_scan_matching/common.h"
#include "search_scan_matching/grid.h"
#include "search_scan_matching/range_finder.h"

struct EstimationInfo {
  int matching_score;
  utils::Pose2D pose;
  double closeness = 0;
  EstimationInfo(const int s, const utils::Pose2D& p)
      : matching_score(s), pose(p) {}
};

struct SearchConfig {
  double linear_tolerance = 0;
  double angular_tolerance = 0;
  double linear_resolution = 1;
  double angular_resolution = 1;
  SearchConfig();
  SearchConfig(const double lin_tol, const double ang_tol, const double lin_res,
               const double ang_res)
      : linear_tolerance(lin_tol),
        angular_tolerance(ang_tol),
        linear_resolution(lin_res),
        angular_resolution(ang_res) {}
};

int MatchingScore(std::vector<std::vector<uint8_t>> grid,
                  std::vector<std::vector<uint8_t>> laser);

class Estimation2D {
 public:
  Estimation2D(const std::shared_ptr<Grid2D> grid);
  EstimationInfo BruteSearch(const utils::Pose2D& init_pose,
                             const std::shared_ptr<RangeFinder> rf,
                             const SearchConfig& config);

 private:
  int MaximumScore();
  std::vector<EstimationInfo> MaximumEstimates();
  EstimationInfo ClosestEstimate();
  std::shared_ptr<Grid2D> grid_;
  utils::Pose2D init_pose_;
  int max_;
  std::vector<EstimationInfo> est_info_vec_;
  std::vector<EstimationInfo> max_est_info_vec_;
};
