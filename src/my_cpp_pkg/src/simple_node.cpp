#include "my_cpp_pkg/simple_node.hpp"

SimpleNode::SimpleNode() : Node("simple_node")
{
  //int unused_var = 42;  // 未使用 → warning
  publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "Hello, ROS 2!";
      publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published: '%s'", msg.data.c_str());
    });
}
