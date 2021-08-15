#include <coding_test/HighLevelController.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  using namespace high_level_controller;

  rclcpp::init(argc, argv);

  {
    auto node = std::make_shared<HighLevelController>();

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Shutting down...");
  }

  rclcpp::shutdown();
  return 0;
}
