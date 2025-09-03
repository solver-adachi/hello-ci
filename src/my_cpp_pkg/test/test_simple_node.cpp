#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_cpp_pkg/simple_node.hpp"

class TestNode : public rclcpp::Node
{
public:
  TestNode() : Node("test_node"), received_(false)
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        last_msg_ = msg->data;
        received_ = true;
      });
  }

  bool received_;
  std::string last_msg_;

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

TEST(SimpleNodeTest, PublishesMessage)
{
  auto node_under_test = std::make_shared<SimpleNode>();
  auto test_node = std::make_shared<TestNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node_under_test);
  exec.add_node(test_node);

  auto start = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(3);

  while (!test_node->received_ &&
         (std::chrono::steady_clock::now() - start) < timeout)
  {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(test_node->received_);
  EXPECT_EQ(test_node->last_msg_, "Hello, ROS 2!");

  exec.remove_node(node_under_test);
  exec.remove_node(test_node);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
