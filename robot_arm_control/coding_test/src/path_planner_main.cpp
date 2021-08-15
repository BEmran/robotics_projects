#include <coding_test/PathPlanner.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  using namespace path_planner;

  rclcpp::init(argc, argv);

  {
    auto node = std::make_shared<PathPlanner>();

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Shutting down...");
  }

  rclcpp::shutdown();
  return 0;
}
