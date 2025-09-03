#ifndef MY_CPP_PKG__SIMPLE_NODE_HPP_
#define MY_CPP_PKG__SIMPLE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SimpleNode : public rclcpp::Node
{
public:
  SimpleNode();

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif
