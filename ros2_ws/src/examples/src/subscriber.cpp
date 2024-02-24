#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

void topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::cout << msg->data << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("subscriber");
  auto subscription = 
    node->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
  rclcpp::spin(node);   // Es queda esperant en esta linea fins que es rep un missatge
                        // Es una estructura sense bucle
  rclcpp::shutdown();
  return 0;
}
