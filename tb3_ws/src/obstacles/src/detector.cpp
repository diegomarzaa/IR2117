#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>
#include "example_interfaces/msg/bool.hpp"

std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Bool>> publisher;  

void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  example_interfaces::msg::Bool out_msg;
  out_msg.data = false;
  publisher->publish(out_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("detector");
  auto subscription = 
    node->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, callback);
  publisher = node->create_publisher<example_interfaces::msg::Bool>("obstacle", 10);
  rclcpp::spin(node);   // Es queda esperant en esta linea fins que es rep un missatge
                        // Es una estructura sense bucle
  rclcpp::shutdown();
  return 0;
}