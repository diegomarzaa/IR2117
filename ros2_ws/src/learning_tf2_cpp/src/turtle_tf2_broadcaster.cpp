#include <sstream>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

std::unique_ptr<tf2_ros::TransformBroadcaster>
  tf_broadcaster;
std::string turtlename;
std::shared_ptr<rclcpp::Node> node;

void handle_turtle_pose(const 
  std::shared_ptr<turtlesim::msg::Pose> msg)
{
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = node->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = turtlename.c_str();
  t.transform.translation.x = msg->x;
  t.transform.translation.y = msg->y;
  t.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_broadcaster->sendTransform(t);
  
  // Turtle only exists in 2D, thus we get x and y translation
  // coordinates from the message and set the z coordinate to 0
  // For the same reason, turtle can only rotate around one axis
  // and this why we set rotation in x and y to 0 and obtain
  // rotation in z axis from the message
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared(
           "turtle_tf2_frame_publisher");

  turtlename = node->declare_parameter<std::string>(
                 "turtlename", "turtle");

  tf_broadcaster = 
    std::make_unique<tf2_ros::TransformBroadcaster>(
      *node);
  
  std::string topic_name = "/" + turtlename +
                           "/pose";

  auto subscription = 
    node->create_subscription<turtlesim::msg::Pose>(
      topic_name, 10, handle_turtle_pose);
                   	 
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
