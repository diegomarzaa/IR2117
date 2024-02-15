#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"   // Import this.
#include <iostream>

std::map<int, int> numbers_counter;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> publisher;   // Publisher to topic "/median"

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  int new_number = msg->data;
  numbers_counter[new_number]++;

  std::cout << "Nuevo número: " << new_number << std::endl;
  std::cout << "El número 3 ha aparecido " << numbers_counter[3] << " veces." << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mode");

  // SUSCRIPTOR AL TOPIC "/number"
  auto subscription = node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);

  // PUBLICADOR AL TOPIC "/mode"
  publisher = node->create_publisher<std_msgs::msg::Float32>("mode", 10);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}