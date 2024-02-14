#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"   // Import this.
#include <iostream>

std::vector<int> numbers;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg){

  numbers.push_back(msg->data);

  // Sort the numbers
  std::sort(numbers.begin(), numbers.end());
  
  // Display the numbers in the vector
  std::string numbers_str = "";
  int n = numbers.size();
  for (int i = 0; i < n; i++){
    numbers_str += std::to_string(numbers[i]);
    if (i < n-1){
      numbers_str += ", ";
    }
  }

  std::cout << "List of numbers: " << numbers_str << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("median");

  // SUSCRIPTOR AL TOPIC "/number"
  auto subscription = node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}