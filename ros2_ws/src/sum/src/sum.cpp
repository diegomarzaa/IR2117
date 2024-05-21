#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>

int sum;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  sum += msg->data;
  std_msgs::msg::Int32 out_msg;
  out_msg.data = sum;
  publisher->publish(out_msg);
}

int main(int argc, char * argv[])
{
  sum = 0;
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sum");

  auto subscription = node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
  publisher = node->create_publisher<std_msgs::msg::Int32>("sum", 10);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


// source /opt/ros/foxy/setup.bash
// source install/setup.bash
// export ROS_LOCALHOST_ONLY=1
// ros2 pkg create --build-type ament_cmake --node-name sum sum
// colcon build --packages-select sum

// ros2 run sum sum

// ros2 topic pub /number std_msgs/msg/Int32 "{data: '123'}"
// ros2 topic echo /sum

// rqt_graph