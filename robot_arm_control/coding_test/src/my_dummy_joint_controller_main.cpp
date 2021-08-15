#include <coding_test/MyDummyJointController.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  using namespace my_dummy_joint_controller;

  rclcpp::init(argc, argv);

  {
    auto node = std::make_shared<MyDummyJointController>();

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Shutting down...");
  }

  rclcpp::shutdown();
  return 0;
}
