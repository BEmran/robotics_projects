#ifndef CODING_TEST_TEST_COMMON_HPP
#define CODING_TEST_TEST_COMMON_HPP

#include <rclcpp/context.hpp>

namespace coding_test_tests {

class ScopedRclContext {
 public:
  ScopedRclContext() : context(std::make_shared<rclcpp::Context>()) {
    context->init(0, nullptr);
  }

  ~ScopedRclContext() { context->shutdown("test_over"); }

  rclcpp::NodeOptions node_options() {
    rclcpp::NodeOptions options{};
    options.context(context);
    return options;
  }

  rclcpp::ExecutorOptions executor_options() {
    rclcpp::ExecutorOptions options{};
    options.context = context;
    return options;
  }

  rclcpp::Context::SharedPtr context;
};

}  // namespace coding_test_tests

#endif  // CODING_TEST_TEST_COMMON_HPP
