#include <chrono>     // Treballar en constants temporals (forma part de C++. no ros), no entra examen pero pot ser interessant
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // Si no es posa aquesta linia, hauriem de posar std::chrono::milliseconds(500) en lloc de 500ms

int main(int argc, char * argv[])     // argc: nombre d'arguments, argv: punter a un array de punter a caràcters
{
  rclcpp::init(argc, argv);   // Inicialitzar el ROS
  auto node = rclcpp::Node::make_shared("publisher");     // Crear un punter compartit
  auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);     // El 10 es el tamany de la cua, se descartaran els primers missatges si la cua esta plena
  std_msgs::msg::String message;
  auto publish_count = 0;
  rclcpp::WallRate loop_rate(500ms);    // Frecuencia per a que el bucle es repetisca (usa chrono)


  while (rclcpp::ok()) {    // Bucle principal del programa
    message.data = "Hello, world! " + std::to_string(publish_count++);
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();   // Finalitzar el ROS
  return 0;
}
