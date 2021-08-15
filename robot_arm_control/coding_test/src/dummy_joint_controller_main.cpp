#include <coding_test/DummyJointController.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  using namespace dummy_joint_controller;

  rclcpp::init(argc, argv);

  {
    auto node = std::make_shared<DummyJointController>();

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Shutting down...");
  }

  rclcpp::shutdown();
  return 0;
}
