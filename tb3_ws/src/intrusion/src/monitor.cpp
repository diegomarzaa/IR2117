#include <chrono>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;

bool north_ob = false;
bool northeast_ob = false;
bool east_ob = false;
bool southeast_ob = false;
bool south_ob = false;
bool southwest_ob = false;
bool west_ob = false;
bool northwest_ob = false;


void callback_north_ob(const example_interfaces::msg::Bool::SharedPtr msg)
{
  north_ob = msg->data;
}

void callback_northeast_ob(const example_interfaces::msg::Bool::SharedPtr msg)
{
  northeast_ob = msg->data;
}

void callback_east_ob(const example_interfaces::msg::Bool::SharedPtr msg)
{
  east_ob = msg->data;
}

void callback_southeast_ob(const example_interfaces::msg::Bool::SharedPtr msg)
{
  southeast_ob = msg->data;
}

void callback_south_ob(const example_interfaces::msg::Bool::SharedPtr msg)
{
  south_ob = msg->data;
}

void callback_southwest_ob(const example_interfaces::msg::Bool::SharedPtr msg)
{
  southwest_ob = msg->data;
}

void callback_west_ob(const example_interfaces::msg::Bool::SharedPtr msg)
{
  west_ob = msg->data;
}

void callback_northwest_ob(const example_interfaces::msg::Bool::SharedPtr msg)
{
  northwest_ob = msg->data;
}





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




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("avoidance");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  auto subs_north_ob = node->create_subscription<example_interfaces::msg::Bool>("/north", 10, callback_north_ob);
  auto subs_northeast_ob = node->create_subscription<example_interfaces::msg::Bool>("/north", 10, callback_northeast_ob);
  auto subs_east_ob = node->create_subscription<example_interfaces::msg::Bool>("/north", 10, callback_east_ob);
  auto subs_southeast_ob = node->create_subscription<example_interfaces::msg::Bool>("/north", 10, callback_southeast_ob);
  auto subs_south_ob = node->create_subscription<example_interfaces::msg::Bool>("/north", 10, callback_south_ob);
  auto subs_southwest_ob = node->create_subscription<example_interfaces::msg::Bool>("/north", 10, callback_southwest_ob);
  auto subs_west_ob = node->create_subscription<example_interfaces::msg::Bool>("/north", 10, callback_west_ob);
  auto subs_northwest_ob = node->create_subscription<example_interfaces::msg::Bool>("/north", 10, callback_northwest_ob);

  auto subscriber_odom = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, callback_odom);     // Crear un subscriptor

  geometry_msgs::msg::Twist message;
  rclcpp::WallRate loop_rate(50ms);

  rclcpp::shutdown();
  return 0;

}