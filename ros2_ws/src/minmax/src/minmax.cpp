#include "rclcpp/rclcpp.hpp"          // Main ROS2
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <iostream>

int max_number;
int min_number;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>> publisher;   // Publisher to topic "/max"

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  int input = msg->data;
  if (not max_number or input > max_number) {
    max_number = input;
  }
  if (not min_number or input < min_number) {
    min_number = input;
  }
  std::cout << "Max number:" << max_number << std::endl;
  std::cout << "Min number:" << min_number << std::endl;

  std_msgs::msg::Int32MultiArray vect_minmax = std_msgs::msg::Int32MultiArray();
  vect_minmax.data.push_back(min_number);
  vect_minmax.data.push_back(max_number);

  publisher->publish(vect_minmax);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minmax");
  auto subscription = node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
  publisher = node->create_publisher<std_msgs::msg::Int32MultiArray>("minmax", 10);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
