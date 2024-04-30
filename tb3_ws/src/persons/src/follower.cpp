#include <chrono>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/bool.hpp"

using namespace std::chrono_literals;

int state = 1;  // 0: stop, 1: forward, 2: turn right, 3: turn left
bool front_person = false;
bool left_person = false;
bool right_person = false;

// Generador de nombres aleatoris
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<> dis(0, 1);    // 0 o 1

void callback_front(const example_interfaces::msg::Bool::SharedPtr msg)
{
  front_person = msg->data;
}

void callback_left(const example_interfaces::msg::Bool::SharedPtr msg)
{
  left_person = msg->data;
}

void callback_right(const example_interfaces::msg::Bool::SharedPtr msg)
{
  right_person = msg->data;
}


int main(int argc, char * argv[])
{

  //TODO
  // - Cambio algoritmo (testear)
  // - Cambio topics bien
  // - Launch




  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("avoidance");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto subs_front = node->create_subscription<example_interfaces::msg::Bool>("/front/person", 10, callback_front);
  auto subs_left = node->create_subscription<example_interfaces::msg::Bool>("/left/person", 10, callback_left);
  auto subs_right = node->create_subscription<example_interfaces::msg::Bool>("/right/person", 10, callback_right);

  geometry_msgs::msg::Twist message;
  rclcpp::WallRate loop_rate(50ms);

  while (rclcpp::ok()) {
    // MOVIMIENTO DE MOTORES
    if (front_person) {
      state = 1;  // forward
    } else if (right_person) {
      state = 2;  // turn right
    } else if (left_person) {
      state = 3; // turn left
    } else {
      state = 0;
    }

    switch (state) {
      case 0: // stop
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        break;
      case 1: // forward
        message.linear.x = 0.4;
        message.angular.z = 0.0;
        break;
      case 2: // turn right
        message.linear.x = 0.0;
        message.angular.z = -0.7;
        break;
      case 3: // turn left
        message.linear.x = 0.0;
        message.angular.z = 0.7;
        break;
    }

    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
