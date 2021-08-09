#include <search_scan_matching/common.h>
#include <search_scan_matching/grid.h>
#include <search_scan_matching/range_finder.h>

#include <algorithm>
#include <iostream>

int Score(std::vector<std::vector<uint8_t>> grid,
          std::vector<std::vector<uint8_t>> laser) {
  int score = 0;
  for (int i = 0; i < grid.size(); ++i) {
    for (int j = 0; j < grid.size(); ++j) {
      if (grid[i][j] == laser[i][j]) score++;
    }
  }
  return score;
}

struct Estimation {
  utils::Pose2D pose;
  int score;
};

int main(int argc, char const* argv[]) {
  std::cout << "start ..." << std::endl;

  auto g = std::make_shared<Grid2D>(20, 40);

  utils::Pose2D laser(0.6, 0.7, 3.14);

  RangeFinder rf(1.2, 3.14 * 2, 0.1);
  // rf.Print();

  // g->Display();
  std::cout << "actual grid:" << std::endl;

  g->CreatBox(0.70, 0.20, utils::Pose2D(0.2, 1.0, 0));
  g->CreatBox(0.46, 0.46, utils::Pose2D(0.1, 0.3, 0));
  g->CreatBox(0.10, 1.20, utils::Pose2D(0.9, 0.0, 0));
  g->CreatBox(0.7, 0.05, utils::Pose2D(0.0, 0.0, 0));
  g->Display();

  rf.Execute(g, laser);
  // rf.Print();
  std::cout << "\n\n exact occupancy seen by laser" << std::endl;
  auto l1 = rf.ToGrid(g, laser, true);
  std::cout << "Perfect Score: " << Score(g->GetOccupancy(), l1) << " at "
            << laser << std::endl;

  double linear_search_tolerance = 0.21;
  double angular_search_tolerance = 0.3;
  double linear_search_resolution = 0.01;
  double angular_search_resolution = 0.02;
  double linear_steps = linear_search_tolerance / linear_search_resolution + 1;
  double angular_steps =
      angular_search_tolerance / angular_search_resolution + 1;
  double starting_x = laser.point.x - (linear_search_tolerance / 2);
  double starting_y = laser.point.y - (linear_search_tolerance / 2);
  double starting_angle = laser.theta - (angular_search_tolerance / 2);
  std::vector<Estimation> est;
  std::cout << "all possibale poses to check: " << std::endl;
  for (int i = 0; i < angular_steps; ++i) {
    double t = starting_angle + i * angular_search_resolution;
    for (int j = 0; j < linear_steps; ++j) {
      double x = starting_x + j * linear_search_resolution;
      for (int k = 0; k < linear_steps; ++k) {
        double y = starting_y + k * linear_search_resolution;
        utils::Pose2D pose(x, y, t);
        // std::cout << pose << std::endl;
        auto l = rf.ToGrid(g, pose);
        auto score = Score(g->GetOccupancy(), l);
        // std::cout << "Score: " << score << ", at " << pose << std::endl;

        est.push_back(Estimation{pose : pose, score : score});
      }
    }
  }
  auto ptr = std::max_element(est.begin(), est.end(), [](auto& p1, auto& p2) {
    return p1.score < p2.score;
  });
  std::cout << "maximum found: " << ptr->score << " at: " << ptr->pose
            << std::endl;
  // std::cout << std::endl;
  // std::cout << "\n\n exact occupancy seen by laser" << std::endl;
  // auto l1 = rf.ToGrid(g, laser);
  // std::cout << "Score: " << Score(g->GetOccupancy(), l1) << std::endl;

  // std::cout << "\n\n wrong occupancy seen by laser" << std::endl;
  // auto l2 = rf.ToGrid(g, utils::Pose2D(0.3, 0.2, 1.57));
  // std::cout << "Score: " << Score(g->GetOccupancy(), l2) << std::endl;

  // std::cout << "\n\n wrong occupancy seen by laser" << std::endl;
  // auto l3 = rf.ToGrid(g, utils::Pose2D(0.2, 0.1, 3.3));
  // std::cout << "Score: " << Score(g->GetOccupancy(), l3) << std::endl;

  // std::cout << "is occupied: " << g->IsOccupied(utils::Point2D(0.1, 0.2))
  //           << std::endl;
  return 0;
}
