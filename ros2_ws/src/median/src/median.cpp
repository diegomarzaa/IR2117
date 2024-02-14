#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"   // Import this.
#include <iostream>

std::vector<int> numbers;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg){

  numbers.push_back(msg->data);

  // Sort the numbers
  std::sort(numbers.begin(), numbers.end());

  // Calculate the median
  int n = numbers.size();   // Number of elements in the vector
  float median = 0;         // Value to return

  // Número par: Promedio de los dos números centrales
  if (n % 2 == 0){
    median = (numbers[n/2 - 1] + numbers[n/2]) / 2;
    std::cout << "Número par. " << "Promedio entre " << numbers[n/2 - 1] << " y " << numbers[n/2] << " da una mediana de: " << median << std::endl;

  // Número impar: El número central
  } else {
    median = numbers[n/2];
    std::cout << "Número impar. " << "Mediana: " << median << std::endl;
  }
  
  // Display the numbers in the vector
  std::string numbers_str = "";
  for (int i = 0; i < n; i++){
    numbers_str += std::to_string(numbers[i]);
    if (i < n-1){
      numbers_str += ", ";
    }
  }

  std::cout << "List of numbers: " << numbers_str << std::endl << std::endl;
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