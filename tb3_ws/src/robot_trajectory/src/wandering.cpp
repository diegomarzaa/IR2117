#include <chrono>     // Treballar en constants temporals (forma part de C++. no ros), no entra examen pero pot ser interessant
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" // LIDAR
#include "geometry_msgs/msg/twist.hpp"    // MOTORS

using namespace std::chrono_literals;   // Si no es posa aquesta linia, hauriem de posar std::chrono::milliseconds(500) en lloc de 500ms

void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::cout << "Nou missatge de LIDAR: " << msg << std::endl;
}

int main(int argc, char * argv[])     // argc: nombre d'arguments, argv: punter a un array de punter a caràcters
{
  rclcpp::init(argc, argv);   // Inicialitzar el ROS
  auto node = rclcpp::Node::make_shared("wandering");     // Crear un punter compartit
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, scan_callback);
  rclcpp::WallRate loop_rate(10ms);    // Frecuencia per a que el bucle es repetisca (usa chrono)

  auto message = geometry_msgs::msg::Twist();
  while (rclcpp::ok()) {    // Bucle principal del programa
    message.linear.x = 0.0;
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();   // Finalitzar el ROS
  return 0;
}
