#include <gtest/gtest.h>

#include <coding_test/DummyJointController.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "test_common.hpp"

using namespace dummy_joint_controller;

class DummyJointControllerFixture : public ::testing::Test {
 public:
  DummyJointControllerFixture()
      : controller_(
            std::make_shared<DummyJointController>(context_.node_options())),
        executor_(context_.executor_options()) {
    executor_.add_node(controller_);
  }

 protected:
  coding_test_tests::ScopedRclContext context_;
  DummyJointController::SharedPtr controller_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

constexpr auto timeout = std::chrono::seconds{2};

TEST_F(DummyJointControllerFixture, TestDoesPublishJointState) {
  bool got_joint_state = false;

  auto subscriber =
      controller_->create_subscription<DummyJointController::JointState>(
          DummyJointController::joint_state_topic, 10,
          [&got_joint_state](DummyJointController::JointState::UniquePtr msg) {
            got_joint_state = true;
            EXPECT_EQ(msg->name.size(), DummyJointController::num_joints);
            EXPECT_EQ(msg->position.size(), DummyJointController::num_joints);
            EXPECT_EQ(msg->velocity.size(), DummyJointController::num_joints);
          });

  auto until = std::chrono::steady_clock::now() + timeout;

  while (!got_joint_state && rclcpp::ok(context_.context) &&
         std::chrono::steady_clock::now() < until) {
    executor_.spin_once(timeout);
  }

  ASSERT_TRUE(got_joint_state);
}
