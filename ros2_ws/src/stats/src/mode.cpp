#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <map>
#include <iostream>

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>> publisher;   // Publisher to topic "/median"

std::map<int, int> numbers_counter;   // (número: ocurrencias)
int moda_actual = 0;      // Número máximo de ocurrencias para algun numero
auto modas = std_msgs::msg::Int32MultiArray();    // Array con las modas que se publicará

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  int new_number = msg->data;
  numbers_counter[new_number]++;
  // - Si el número actual tiene más ocurrencias que el número con más ocurrencias:
  // Sobreescribimos las modas, agregamos la nueva moda mayor
  // Actualizamos esta info
  // - Si tiene las mismas ocurrencias que la moda tras actualizar, lo agregamos a las modas
  if (numbers_counter[new_number] > moda_actual) {
    modas.data.clear();
    modas.data.push_back(new_number);
    moda_actual = numbers_counter[new_number];
  } else if (numbers_counter[new_number] == moda_actual) {
    modas.data.push_back(new_number);
  }
  publisher->publish(modas);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mode");
  // SUSCRIPTOR AL TOPIC "/number"
  auto subscription = node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
  // PUBLICADOR AL TOPIC "/mode", un array con todas las modas (enteros)
  publisher = node->create_publisher<std_msgs::msg::Int32MultiArray>("mode", 10);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}