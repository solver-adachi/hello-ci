#include "my_cpp_pkg/simple_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  int rgc = 0;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleNode>());
  rclcpp::shutdown();
  return 0;
}
