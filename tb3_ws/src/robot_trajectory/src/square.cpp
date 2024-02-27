#include <chrono>     // Treballar en constants temporals (forma part de C++. no ros), no entra examen pero pot ser interessant
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // Canviem el tipus de fitxer al de moure motors
#include <iostream>
#include <math.h>


using namespace std::chrono_literals;   // Si no es posa aquesta linia, hauriem de posar std::chrono::milliseconds(500) en lloc de 500ms

int main(int argc, char * argv[])     // argc: nombre d'arguments, argv: punter a un array de punter a caràcters
{
  rclcpp::init(argc, argv);   // Inicialitzar el ROS
  auto node = rclcpp::Node::make_shared("publisher");     // Crear un punter compartit
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);     // El 10 es el tamany de la cua, se descartaran els primers missatges si la cua esta plena
  geometry_msgs::msg::Twist message;    // Missatge per a moure el robot
  
  // Dades
  double distance = 1.0;
  double speed = 0.1;
  rclcpp::WallRate loop_rate(10ms);
  double total_time = distance / speed;
  int i=0;
  int n = total_time / 0.01;    // iteracions

  while (rclcpp::ok() && i<n) {
    i++;
    message.linear.x = speed;
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  message.linear.x = 0.0;
  publisher->publish(message);

  double angle = 90 * M_PI / 180;           // 90 graus a radians
  double angular_speed = 9 * M_PI / 180;    // 9 graus a radians per segon
  total_time = angle / angular_speed;
  i=0;
  n = total_time / 0.01;    // iteracions

  while (rclcpp::ok() && i<n) {
    i++;
    message.angular.z = angular_speed;
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  message.angular.z = 0.0;
  publisher->publish(message);

  rclcpp::shutdown();   // Finalitzar el ROS
  return 0;
}
