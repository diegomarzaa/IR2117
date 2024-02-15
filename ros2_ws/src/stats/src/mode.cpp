#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> publisher;   // Publisher to topic "/median"
std::map<int, int> numbers_counter;
int number_biggest_occurrence = 0;
int biggest_occurrence = 0;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  int new_number = msg->data;
  numbers_counter[new_number]++;

  if (numbers_counter[new_number] > biggest_occurrence) {
    number_biggest_occurrence = new_number;
    biggest_occurrence = numbers_counter[new_number];
  }

  std_msgs::msg::Int32 moda_publicar;
  moda_publicar.data = number_biggest_occurrence;
  publisher->publish(moda_publicar);
  std::cout << "Moda: " << number_biggest_occurrence << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mode");

  // SUSCRIPTOR AL TOPIC "/number"
  auto subscription = node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);

  // PUBLICADOR AL TOPIC "/mode"
  publisher = node->create_publisher<std_msgs::msg::Int32>("mode", 10);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}