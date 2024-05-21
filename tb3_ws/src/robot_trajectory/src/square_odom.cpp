#include <chrono>     // Treballar en constants temporals (forma part de C++. no ros), no entra examen pero pot ser interessant
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // Canviem el tipus de fitxer al de moure motors
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;   // Si no es posa aquesta linia, hauriem de posar std::chrono::milliseconds(500) en lloc de 500ms

double x_init = 0.0;
double y_init = 0.0;
double theta_init = 0.0;
bool actualizar_init = true;

double x = 0.0;
double y = 0.0;
double theta = 0.0;  // En radians

double distance = 0.0;
double theta_distance = 0.0;


double eulerFromQuaternion(const geometry_msgs::msg::Quaternion& quat) {
  double x = quat.x;
  double y = quat.y;
  double z = quat.z;
  double w = quat.w;

  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2.0 * (w * y - z * x);
  double pitch = std::asin(sinp);

  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  return yaw;   // El que ens interessa (angle vist desde dalt, el z)
}

void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::cout << "\nOdometry message received" << std::endl;
  
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  theta = eulerFromQuaternion(msg->pose.pose.orientation);    // En radians
  std::cout << "Position:\t\t (X: " << x << ",\tY: " << y << ",\tTheta: " << theta << ")" << std::endl;

  if (actualizar_init) {
    actualizar_init = false;
    x_init = x;
    y_init = y;
    theta_init = theta;
    std::cout << "Initial position: (X: " << x_init << ",\tY: " << y_init << ",\tTheta: " << theta_init << ")" << std::endl;
  }
  
  double x_distance = x - x_init;
  double y_distance = y - y_init;
  distance = sqrt(pow(x_distance, 2) + pow(y_distance, 2));
  theta_distance = theta - theta_init;

  std::cout << "Distance:\t\t (X: " << x_distance << ", Y: " << y_distance << ", \tTheta: " << theta_distance << ")" << std::endl;
}

int main(int argc, char * argv[])     // argc: nombre d'arguments, argv: punter a un array de punter a caràcters
{
  rclcpp::init(argc, argv);   // Inicialitzar el ROS
  auto node = rclcpp::Node::make_shared("square_odom");     // Crear un punter compartit
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);     // El 10 es el tamany de la cua, se descartaran els primers missatges si la cua esta plena
  auto subscriber_odom = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, callback_odom);     // Crear un subscriptor
  node->declare_parameter("linear_speed", 0.1);    // Declarar un paràmetre amb el valor per defecte 0.1
  node->declare_parameter("angular_speed", M_PI / 20);
  node->declare_parameter("square_length", 1.0);
  geometry_msgs::msg::Twist message;    // Missatge per a moure el robot
  
  // Dades moviment
  rclcpp::WallRate loop_rate(50ms);
  double square_length = node->get_parameter("square_length").get_parameter_value().get<double>();
  double loop_wait = 0.01;
  
  // Dades avant
  // double distance = square_length;
  double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();    // Obtenir el valor del paràmetre
  std::cout << "linear_speed: " << linear_speed << std::endl;

  // Dades gir
  double angle_a_fer = 90 * M_PI / 180;           // 90 graus a radians
  double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
  std::cout << "angular_speed: " << angular_speed << std::endl;

  // Moviment quadrat
  for(int j=0; j<4; j++)
  {
    std::cout << "Avant" << std::endl;
    actualizar_init = true;

    while (rclcpp::ok() && distance < square_length) {
      message.linear.x = linear_speed;
      publisher->publish(message);
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
    message.linear.x = 0.0;
    publisher->publish(message);

    actualizar_init = true;
    std::cout << "Gir" << std::endl;
    while (rclcpp::ok() && abs(theta_distance) < angle_a_fer) { 
      message.angular.z = angular_speed;
      publisher->publish(message);
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }

    message.angular.z = 0.0;
    publisher->publish(message);
  }
  
  // Parar robot
  message.linear.x = 0.0;
  message.angular.z = 0.0;
  publisher->publish(message);
  std::cout << "Final del programa" << std::endl;

  rclcpp::shutdown();   // Finalitzar el ROS
  return 0;
}
