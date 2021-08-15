#include <gtest/gtest.h>
#include <coding_test/HighLevelController.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/create_server.hpp>
#include "test_common.hpp"

using namespace high_level_controller;

class HighLevelControllerFixture : public ::testing::Test
{
public:
  HighLevelControllerFixture()
  : controller_(std::make_shared<HighLevelController>(context_.node_options())),
    executor_(context_.executor_options())
  {
    executor_.add_node(controller_);
  }

protected:
  coding_test_tests::ScopedRclContext context_;
  HighLevelController::SharedPtr controller_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

constexpr auto timeout = std::chrono::seconds {5};

TEST_F(HighLevelControllerFixture, TestRequestsActionClient)
{
  bool got_action_request = false;

  auto server = rclcpp_action::create_server<HighLevelController::MoveToPoint>(
    controller_,
    HighLevelController::move_to_point_client_action_name,
    [&got_action_request](...) -> rclcpp_action::GoalResponse {
      got_action_request = true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](...) -> rclcpp_action::CancelResponse {return rclcpp_action::CancelResponse::ACCEPT;},
    [](...) {}
  );

  auto until = std::chrono::steady_clock::now() + timeout;

  while (!got_action_request && rclcpp::ok(context_.context) &&
    std::chrono::steady_clock::now() < until)
  {
    executor_.spin_once(timeout);
  }

  ASSERT_TRUE(got_action_request);
}
