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
  
  // Dades moviment
  rclcpp::WallRate loop_rate(10ms);
  int i=0;
  
  // Dades avant
  double distance = 1.0;
  double forward_speed = 0.1;
  double total_time_forward = distance / forward_speed;
  int n_avant = total_time_forward / 0.01;    // iteracions

  // Dades gir
  double angle = 90 * M_PI / 180;           // 90 graus a radians
  double angular_speed = 9 * M_PI / 180;    // 9 graus a radians per segon
  double total_time_rotation = angle / angular_speed;
  int n_gir = total_time_rotation / 0.01;    // iteracions

  // Moviment quadrat
  for(int j=0; j<4; j++)
  {
    i=0;
    
    while (rclcpp::ok() && i<n_avant) {
      i++;
      message.linear.x = forward_speed;
      publisher->publish(message);
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
    message.linear.x = 0.0;
    publisher->publish(message);
    
    i=0;

    while (rclcpp::ok() && i<n_gir) {
      i++;
      message.angular.z = angular_speed;
      publisher->publish(message);
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }

    message.angular.z = 0.0;
    publisher->publish(message);
  }

  rclcpp::shutdown();   // Finalitzar el ROS
  return 0;
}
