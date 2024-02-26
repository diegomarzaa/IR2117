#include "rclcpp/rclcpp.hpp"          // Main ROS2
#include "std_msgs/msg/int32.hpp"
#include <iostream>

int max_number;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> publisher;   // Publisher to topic "/max"

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {

  int input = msg->data;
  if (not max_number or input > max_number) {
    max_number = input;
  }
  std::cout << "Max number:" << max_number << std::endl;
  
  std_msgs::msg::Int32 max_to_publish;
  max_to_publish.data = max_number;
  publisher->publish(max_to_publish);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("max");
  auto subscription = node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
  publisher = node->create_publisher<std_msgs::msg::Int32>("max", 10);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
