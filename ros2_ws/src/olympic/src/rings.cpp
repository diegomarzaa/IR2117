#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "turtlesim/srv/set_pen.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rings");
  node->declare_parameter("radius", 1.0);

  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
  geometry_msgs::msg::Twist message;
  auto publish_count = 0;
  rclcpp::WallRate loop_rate(500ms);

  // REQUEST
  auto request_setpen = std::make_shared<turtlesim::srv::SetPen::Request>();

  // BLUE
  request->r = 0;
  request->g = 0;
  request->b = 255;
  
  // BLACK
  // request->r = 0;
  // request->g = 0;
  // request->b = 0;

  // RED
  // request->r = 255;
  // request->g = 0;
  // request->b = 0;

  // YELLOW
  // request_setpen->r = 255;
  // request_setpen->g = 255;
  // request_setpen->b = 0;

  // GREEN
  // request_setpen->r = 0;
  // request_setpen->g = 255;
  // request_setpen->b = 0;

  // GENERAL PEN
  request_setpen->width = 2;
  request_setpen->off = false;
  

  // REQUEST CLIENT
  auto client = node->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");

  while (!client->wait_for_service(1s)) {   // Gives the request 1s to search for the service nodes in the network
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),      // When Ctrl+C is pressed for cancelling the request
       "Interrupted while waiting for the service.");
      return 0;
	 }
	 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
     "service not available, waiting again...");
  }

  auto result = client->async_send_request(request_setpen);

  if (rclcpp::spin_until_future_complete(node, 
       result) ==	rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
      "Pen set correctly.");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
     "Failed to call service set_pen");
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
