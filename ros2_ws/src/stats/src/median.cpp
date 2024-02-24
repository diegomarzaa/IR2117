#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"   // Import this.
#include <iostream>

std::vector<int> numbers;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> publisher;   // Publisher to topic "/median"

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg){
  numbers.push_back(msg->data);
  // Sort the numbers
  std::sort(numbers.begin(), numbers.end());
  // Calculate the median
  int n = numbers.size();   // Number of elements in the vector
  std_msgs::msg::Float32 median_to_publish;
  // Número par: Promedio de los dos números centrales
  if (n % 2 == 0){
    median_to_publish.data = (numbers[n/2 - 1] + numbers[n/2]) / 2;
  // Número impar: El número central
  } else {
    median_to_publish.data = numbers[n/2];
  }
  // Publish the median to the topic "/median"
  publisher->publish(median_to_publish);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("median");

  // SUSCRIPTOR AL TOPIC "/number"
  auto subscription = node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);

  // PUBLICADOR AL TOPIC "/median"
  publisher = node->create_publisher<std_msgs::msg::Float32>("median", 10);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}