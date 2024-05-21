#include <chrono>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = 
    rclcpp::Node::make_shared("turtle_tf2_frame_listener");
  auto publisher = 
    node->create_publisher<geometry_msgs::msg::Twist>(
      "turtle2/cmd_vel", 1);
  auto target_frame =
    node->declare_parameter<std::string>("target_frame", 
      "turtle1");
  auto tf_buffer =
      std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  
  rclcpp::WallRate loop_rate(1s);

  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::Twist msg;

  std::string fromFrameRel = target_frame.c_str();
  std::string toFrameRel = "turtle2";

  const double scaleRotationRate = 1.0;
  const double scaleForwardSpeed = 0.5;
  
  while (rclcpp::ok()) {
    try {
      t = tf_buffer->lookupTransform(
        toFrameRel, fromFrameRel,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        node->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    }

    msg.angular.z = scaleRotationRate * atan2(
      t.transform.translation.y,
      t.transform.translation.x);

    msg.linear.x = scaleForwardSpeed * sqrt(
      pow(t.transform.translation.x, 2) +
      pow(t.transform.translation.y, 2));

    publisher->publish(msg);              // Send velocity commands for turtle2 to reach target_frame
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

// ros2 run learning_tf2_cpp turtle_tf2_listener

// drive around the first turtle using the arrow keys (make sure your terminal window is active, not your simulator window), and you’ll see the second turtle following the first one!