#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/teleport_relative.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rings");
  node->declare_parameter("radius", 1.0);
  double radius = node->get_parameter("radius").get_parameter_value().get<double>();

  // ========== DATOS DE LOS ANILLOS ==========

  double altura_alta = 5.5+radius/2;
  double altura_baja = 5.5-radius/2;

  double distancia_centros_x = 1.083 * radius;

  double x_1 = 5.5 - distancia_centros_x*2;
  double x_2 = 5.5;
  double x_3 = 5.5 + distancia_centros_x*2;
  double x_4 = 5.5 - distancia_centros_x;
  double x_5 = 5.5 + distancia_centros_x;

  std::vector<std::vector<double>> colors = { {0, 0, 255},    // Blue
                                              {0, 0, 0},      // Black
                                              {255, 0, 0},    // Red
                                              {255, 255, 0},  // Yellow
                                              {0, 255, 0} };  // Green

  std::vector<std::vector<double>> positions = {{x_1, altura_alta}, 
                                                {x_2, altura_alta}, 
                                                {x_3, altura_alta}, 
                                                {x_4, altura_baja}, 
                                                {x_5, altura_baja} };

  // =========== NODO ===========

  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
  geometry_msgs::msg::Twist message;
  rclcpp::WallRate loop_rate(10ms);

  // CLIENT CREATION
  auto client_teleport_absolute = node->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
  auto client_setpen = node->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
  
  for (int r=0; r<5; r++)
  {
    // ===== CLIENT FOR SETPEN =====
    auto request_setpen = std::make_shared<turtlesim::srv::SetPen::Request>();  

    // CHANGE PEN COLOR AND SETTINGS
    request_setpen->off = true;

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
    auto request_teleport_absolute = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request_teleport_absolute->x = positions[r][0];
    request_teleport_absolute->y = positions[r][1];

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
    
    // CHANGE PEN COLOR AND SETTINGS
    request_setpen->r = colors[r][0]; 
    request_setpen->g = colors[r][1];
    request_setpen->b = colors[r][2];
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

    result_setpen = client_setpen->async_send_request(request_setpen);

    if (rclcpp::spin_until_future_complete(node, 
        result_setpen) ==	rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "Pen set correctly.");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
      "Failed to call service set_pen");
    }  

  // DRAW CIRCLE

  double loop_wait = 0.01;
  double linear_vel = 2.0;
  double distance = 2 * M_PI * radius;
  double linear_iterations = distance / (loop_wait * linear_vel);
  int i=0;

  while (rclcpp::ok() && i<linear_iterations) {
    i++;
    message.linear.x = linear_vel;
    message.angular.z = linear_vel / radius;
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  message.linear.x = 0.0;
  message.angular.z = 0.0;
  publisher->publish(message);
  }

  rclcpp::shutdown();
  return 0;
}
