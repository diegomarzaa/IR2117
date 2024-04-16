#include <chrono>     // Treballar en constants temporals (forma part de C++. no ros), no entra examen pero pot ser interessant
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" // LIDAR
#include "geometry_msgs/msg/twist.hpp"    // MOTORS

using namespace std::chrono_literals;   // Si no es posa aquesta linia, hauriem de posar std::chrono::milliseconds(500) en lloc de 500ms
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  auto moviment = geometry_msgs::msg::Twist();
  float min = msg->ranges[0];
  for (int i = 0; i <= 9; ++i) {
    if (msg->ranges[i] < min) {
      min = msg->ranges[i];
    }
  }
  for (int i = 350; i <= 359; ++i) {
    if (msg->ranges[i] < min) {
      min = msg->ranges[i];
    }
  }

  std::cout << "El valor mínim és: " << min << std::endl;
  
  if (min > 0.5) {
    std::cout << "No hi ha cap obstacle a menys de 0.5m" << std::endl;
    moviment.linear.x = 0.5;
    publisher->publish(moviment);

  } else {
    std::cout << "Hi ha un obstacle a menys de 0.5m" << std::endl;
    moviment.linear.x = 0.0;
    publisher->publish(moviment);
  }

}

int main(int argc, char * argv[])     // argc: nombre d'arguments, argv: punter a un array de punter a caràcters
{
  rclcpp::init(argc, argv);   // Inicialitzar el ROS
  auto node = rclcpp::Node::make_shared("wandering");     // Crear un punter compartit
  auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, scan_callback);
  publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  rclcpp::WallRate loop_rate(10ms);    // Frecuencia per a que el bucle es repetisca (usa chrono)

  while (rclcpp::ok()) {    // Bucle principal del programa
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();   // Finalitzar el ROS
  return 0;
}
