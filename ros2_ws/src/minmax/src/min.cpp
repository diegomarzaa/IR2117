#include "rclcpp/rclcpp.hpp"          // Main ROS2
#include "std_msgs/msg/int32.hpp"
#include <iostream>

int min_number;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> publisher;   // Publisher to topic "/min"

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {

  int input = msg->data;
  if (not min_number or input < min_number) {
    min_number = input;
  }
  std::cout << "Min number:" << min_number << std::endl;
  
  std_msgs::msg::Int32 min_to_publish;
  min_to_publish.data = min_number;
  publisher->publish(min_to_publish);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("min");
  auto subscription = node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
  publisher = node->create_publisher<std_msgs::msg::Int32>("min", 10);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
