#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>
#include "example_interfaces/msg/bool.hpp"
#include <cmath>

std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Bool>> publisher;
double obs_angle_min;
double obs_angle_max;
double obs_threshold;

void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  example_interfaces::msg::Bool out_msg;
  out_msg.data = false;
  float angle = msg->angle_min;
  for (float range: msg->ranges) {
    if (angle > M_PI) {
      angle -= 2*M_PI;
    }
    if ((angle >= obs_angle_min) and (angle <= obs_angle_max)) {
      if (range <= obs_threshold) {
        out_msg.data = true;
      }
    }
    angle += msg->angle_increment;
  }

  publisher->publish(out_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("detector");
  auto subscription = 
    node->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, callback);
  publisher = node->create_publisher<example_interfaces::msg::Bool>("obstacle", 10);
  node->declare_parameter("obs_angle_min", -M_PI/8);
  node->declare_parameter("obs_angle_max", M_PI/8);
  node->declare_parameter("obs_threshold", 0.3);
  obs_angle_min = node->get_parameter("obs_angle_min").get_parameter_value().get<double>();
  obs_angle_max = node->get_parameter("obs_angle_max").get_parameter_value().get<double>();
  obs_threshold = node->get_parameter("obs_threshold").get_parameter_value().get<double>();
  rclcpp::spin(node);   // Es queda esperant en esta linea fins que es rep un missatge
                        // Es una estructura sense bucle
  rclcpp::shutdown();
  return 0;
}