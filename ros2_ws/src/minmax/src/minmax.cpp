#include "rclcpp/rclcpp.hpp"          // Main ROS2
#include "std_msgs/msg/int32.hpp"
#include <iostream>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("min");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
