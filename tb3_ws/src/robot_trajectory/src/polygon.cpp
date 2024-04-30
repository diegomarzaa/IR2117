#include <chrono>     // Treballar en constants temporals (forma part de C++. no ros), no entra examen pero pot ser interessant
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // Canviem el tipus de fitxer al de moure motors
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;   // Si no es posa aquesta linia, hauriem de posar std::chrono::milliseconds(500) en lloc de 500ms

int main(int argc, char * argv[])     // argc: nombre d'arguments, argv: punter a un array de punter a caràcters
{
  rclcpp::init(argc, argv);   // Inicialitzar el ROS
  auto node = rclcpp::Node::make_shared("polygon");     // Crear un punter compartit
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);     // El 10 es el tamany de la cua, se descartaran els primers missatges si la cua esta plena
  node->declare_parameter("linear_speed", 0.8);    // Declarar un paràmetre amb el valor per defecte 0.1
  node->declare_parameter("angular_speed", 0.5);
  node->declare_parameter("segment_size", 0.5);
  node->declare_parameter("number_segments", 4);
  geometry_msgs::msg::Twist message;    // Missatge per a moure el robot
  
  // Dades moviment
  rclcpp::WallRate loop_rate(10ms);
  int i=0;
  double loop_wait = 0.01;
  double segment_size = node->get_parameter("segment_size").get_parameter_value().get<double>();
  int number_segments = node->get_parameter("number_segments").get_parameter_value().get<int>();
  
  // Dades avant
  double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();    // Obtenir el valor del paràmetre
  std::cout << "linear_speed: " << linear_speed << std::endl;
  double linear_iterations = segment_size / (loop_wait * linear_speed);

  // Dades gir
  double angle = (360/number_segments) * M_PI / 180;           // graus de la figura a radians
  double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
  std::cout << "angular_speed: " << angular_speed << std::endl;
  double angular_iterations = angle / (loop_wait * angular_speed);



  // Moviment figura
  for(int j=0; j<number_segments; j++)
  {
    i=0;
    
    std::cout << "Avant" << std::endl;
    while (rclcpp::ok() && i<linear_iterations) {
      i++;
      message.linear.x = linear_speed;
      publisher->publish(message);
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
    message.linear.x = 0.0;
    publisher->publish(message);
    
    i=0;

    std::cout << "Gir" << std::endl;
    while (rclcpp::ok() && i<angular_iterations) {
      i++;
      message.angular.z = angular_speed;
      publisher->publish(message);
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }

    message.angular.z = 0.0;
    publisher->publish(message);
  }

  message.linear.x = 0.0;
  message.angular.z = 0.0;
  publisher->publish(message);
  std::cout << "Final del programa" << std::endl;



  rclcpp::shutdown();   // Finalitzar el ROS
  return 0;
}
