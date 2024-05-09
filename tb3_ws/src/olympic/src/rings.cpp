#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/teleport_relative.hpp"

using namespace std::chrono_literals;

std::vector<std::vector<double>> colors = { {0, 0, 255},    // Blue
                                            {0, 0, 0},      // Black
                                            {255, 0, 0},    // Red
                                            {255, 255, 0},  // Yellow
                                            {0, 255, 0} };  // Green

double altura_alta = 5.5+0.61;
double altura_baja = 5.5-0.61;

double x_1 = 5.5-2.64;
double x_2 = 5.5;
double x_3 = 5.5+2.64;
double x_4 = 5.5-2.64/2;
double x_5 = 5.5+2.64/2;

std::vector<std::vector<double>> positions = {{x_1, altura_alta}, 
                                              {x_2, altura_alta}, 
                                              {x_3, altura_alta}, 
                                              {x_4, altura_baja}, 
                                              {x_5, altura_baja} };

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rings");
  node->declare_parameter("radius", 1.0);

  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
  geometry_msgs::msg::Twist message;
  auto publish_count = 0;
  rclcpp::WallRate loop_rate(500ms);



  // ===== CLIENT FOR SETPEN =====
  auto client_setpen = node->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
  auto request_setpen = std::make_shared<turtlesim::srv::SetPen::Request>();  

  // CHANGE PEN COLOR AND SETTINGS
  request_setpen->r = colors[4][0]; 
  request_setpen->g = colors[4][1];
  request_setpen->b = colors[4][2];
  request_setpen->width = 2;
  request_setpen->off = false;

  while (!client_setpen->wait_for_service(1s)) {   // Gives the request 1s to search for the service nodes in the network
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),      // When Ctrl+C is pressed for cancelling the request
       "Interrupted while waiting for the service.");
      return 0;
	 }
	 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
     "service not available, waiting again...");
  }

  auto result_setpen = client_setpen->async_send_request(request_setpen);

  if (rclcpp::spin_until_future_complete(node, 
       result_setpen) ==	rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
      "Pen set correctly.");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
     "Failed to call service set_pen");
  }



  // ===== CLIENT FOR TELEPORT =====

  auto client_teleport_absolute = node->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
  auto request_teleport_absolute = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();

  request_teleport_absolute->x = positions[4][0];
  request_teleport_absolute->y = positions[4][1];

  while (!client_teleport_absolute->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the teleport_absolute service.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "TeleportAbsolute service not available, waiting again...");
  }

  auto result_teleport_absolute = client_teleport_absolute->async_send_request(request_teleport_absolute);
  
  if (rclcpp::spin_until_future_complete(node, result_teleport_absolute) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Teleport set correctly.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service teleport_absolute");
  }






  while (rclcpp::ok()) {
    double radius;
    double linear_vel = 2.0;
    node->get_parameter("radius", radius);

    message.linear.x = linear_vel;
    message.angular.z = linear_vel / radius;
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
