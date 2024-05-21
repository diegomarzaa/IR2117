#include <chrono>     // Treballar en constants temporals (forma part de C++. no ros), no entra examen pero pot ser interessant
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" // LIDAR
#include "geometry_msgs/msg/twist.hpp"    // MOTORS

using namespace std::chrono_literals;   // Si no es posa aquesta linia, hauriem de posar std::chrono::milliseconds(500) en lloc de 500ms
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

float min_esquerra;
float min_dreta;

bool turn_left = false;
bool turn_right = false;

void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  auto moviment = geometry_msgs::msg::Twist();
  min_dreta = msg->ranges[0];
  for (int i = 0; i <= 9; ++i) {
    if (msg->ranges[i] < min_dreta) {
      min_dreta = msg->ranges[i];
    }
  }
  std::cout << "El valor mínim a la dreta és: " << min_dreta << std::endl;

  min_esquerra = msg->ranges[350];
  for (int i = 350; i <= 359; ++i) {
    if (msg->ranges[i] < min_esquerra) {
      min_esquerra = msg->ranges[i];
    }
  }
  std::cout << "El valor mínim a l'esquerra és: " << min_esquerra << std::endl;
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
    auto moviment = geometry_msgs::msg::Twist();
    
    if (turn_left == false and turn_right == false) {
      moviment.linear.x = 0.5;    // Avançar
      moviment.angular.z = 0.0;

      if (min_esquerra < 0.5 or min_dreta < 0.5) {    // Si hi ha un obstacle a menys de 0.5m
        if (min_esquerra > min_dreta) {
          turn_left = true;
        } else {
          turn_right = true;
        }
      }

    } else if (turn_left == true) {
      moviment.linear.x = 0.0;
      moviment.angular.z = -0.5;    // Girar a l'esquerra
      if (min_esquerra > 0.5 and min_dreta > 0.5) {
        turn_left = false;
      }

    } else if (turn_right == true) {
      moviment.linear.x = 0.0;
      moviment.angular.z = 0.5;   // Girar a la dreta
      if (min_esquerra > 0.5 and min_dreta > 0.5) {
        turn_right = false;
      }
    }

    publisher->publish(moviment);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();   // Finalitzar el ROS
  return 0;
}

// GAZEBO: Back (180), Left (90), Front (0), Right (270)
// WEBOTS: Back (0), Left (90), Front (180), Right (270)