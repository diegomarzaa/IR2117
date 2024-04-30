#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>
#include "example_interfaces/msg/bool.hpp"
#include <cmath>

std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Bool>> publisher;

double person_angle_min;
double person_angle_max;
double person_distance_min;
double person_distance_max;

void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  example_interfaces::msg::Bool out_msg;
  out_msg.data = false;
  float angle = msg->angle_min;
  for (float range: msg->ranges) {
    if (angle > M_PI) {
      angle -= 2*M_PI;
    }
    if ((angle >= person_angle_min) and (angle <= person_angle_max)) {
      if ((person_distance_min <= range) and (range <= person_distance_max)) {
        out_msg.data = true;
      }
    }
    angle += msg->angle_increment;
  }
  publisher->publish(out_msg);
}

int main(int argc, char * argv[])
{
  // TODO: Verificar parámetros!!!!!!  OKEY
  // Verificar publisher y suscriber  okey
  // Callback bien? cambio parametros  okey

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("detector");
  auto subscription = 
    node->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, callback);
  publisher = node->create_publisher<example_interfaces::msg::Bool>("person", 10);
  node->declare_parameter("person_angle_min", -M_PI/12);
  node->declare_parameter("person_angle_max", M_PI/12);
  node->declare_parameter("person_distance_min", 0.15);
  node->declare_parameter("person_distance_max", 1.3);

  person_angle_min = node->get_parameter("person_angle_min").get_parameter_value().get<double>();
  person_angle_max = node->get_parameter("person_angle_max").get_parameter_value().get<double>();
  person_distance_min = node->get_parameter("person_distance_min").get_parameter_value().get<double>();
  person_distance_max = node->get_parameter("person_distance_max").get_parameter_value().get<double>();

  rclcpp::spin(node);   // Es queda esperant en esta linea fins que es rep un missatge
                        // Es una estructura sense bucle
  rclcpp::shutdown();
  return 0;
}