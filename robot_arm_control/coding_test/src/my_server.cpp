
#include <coding_test_msgs/action/move_to_point.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rclcpp_components/register_node_macro.hpp"
namespace action_tutorials_cpp {
class MyServer : public rclcpp::Node {
 public:
  using MoveToPoint = coding_test_msgs::action::MoveToPoint;
  using GoalHandleMoveToPoint = rclcpp_action::ServerGoalHandle<MoveToPoint>;

  explicit MyServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("MyServer", options) {
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<MoveToPoint>(
        this, "move_to_point", std::bind(&MyServer::handle_goal, this, _1, _2),
        std::bind(&MyServer::handle_cancel, this, _1),
        std::bind(&MyServer::handle_accepted, this, _1));
  }

 private:
  rclcpp_action::Server<MoveToPoint>::SharedPtr action_server_;
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveToPoint::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d",
                goal->header.stamp.sec);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMoveToPoint> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(
      const std::shared_ptr<GoalHandleMoveToPoint> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&MyServer::execute, this, _1), goal_handle}.detach();
  }
  void execute(const std::shared_ptr<GoalHandleMoveToPoint> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToPoint::Feedback>();
    auto result = std::make_shared<MoveToPoint::Result>();

    for (int i = 1; (i < 10) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Publish feedback
      feedback->current_point.x = 1;
      feedback->current_point.x = 2;
      feedback->current_point.x = 3;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      loop_rate.sleep();
    }
    // Check if goal is done
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class MyServer
}  // namespace action_tutorials_cpp
RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::MyServer)