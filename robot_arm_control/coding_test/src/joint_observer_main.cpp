#include <coding_test/JointObserver.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  using namespace joint_observer;

  rclcpp::init(argc, argv);

  {
    auto node = std::make_shared<JointObserver>();

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Shutting down...");
  }

  rclcpp::shutdown();
  return 0;
}
