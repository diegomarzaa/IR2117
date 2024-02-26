#include "rclcpp/rclcpp.hpp"          // Main ROS2
#include "std_msgs/msg/int32.hpp"
#include <iostream>

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  int input = msg->data;
  std::cout << "Received number: " << input << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("min");
  auto subscription = node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}