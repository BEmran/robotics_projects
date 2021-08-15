#ifndef CODING_TEST_HIGHLEVELCONTROLLER_HPP
#define CODING_TEST_HIGHLEVELCONTROLLER_HPP

#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>
#include <coding_test_msgs/action/move_to_point.hpp>

#include <random>

namespace high_level_controller
{

/**
 * A simple high level controller.
 *
 * every `send_goal_period`, it requests a coding_test_msgs::action::MoveToPoint action for a random point.
 */
class HighLevelController : public rclcpp::Node
{
public:
  using MoveToPoint = coding_test_msgs::action::MoveToPoint;
  using Client = rclcpp_action::Client<MoveToPoint>;

  constexpr static char move_to_point_client_action_name[] = "move_to_point";

  /// How often to (attempt to) send a `coding_test_msgs::action::MoveToPoint`
  constexpr static auto send_goal_period = std::chrono::seconds {5};

  /// Send goals with variables between negative and positive this value
  constexpr static double point_range = 2.0;

  HighLevelController(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit HighLevelController(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : HighLevelController(node_name, "", options)
  {}

  explicit HighLevelController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : HighLevelController("high_level_controller", "", options)
  {}

  ~HighLevelController() override = default;

protected:
  /// Send the goal
  void send_goal();

  /// Goal Response Callback for the action (just logs it)
  void goal_response_callback(std::shared_future<Client::GoalHandle::SharedPtr>);

  /// Goal Result Callback for the action (just logs it)
  void result_callback(const Client::GoalHandle::WrappedResult &);

  /// Goal Feedback Callback (just logs it)
  void feedback_callback(Client::GoalHandle::SharedPtr,
    std::shared_ptr<const MoveToPoint::Feedback>);

  std::mt19937 rng_;
  Client::SharedPtr move_to_point_client_;
  rclcpp::TimerBase::SharedPtr send_goal_timer_;
};

}


#endif //CODING_TEST_HIGHLEVELCONTROLLER_HPP
