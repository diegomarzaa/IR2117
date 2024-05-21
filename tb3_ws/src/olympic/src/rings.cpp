#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/teleport_relative.hpp"
#include <string>

using namespace std::chrono_literals;

    
int waitForService(auto& client)
{
  // add_compile_options(-fconcepts)       for using auto in functions
  // Sin el auto: int waitForService(rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client)

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
        "Interrupted while waiting for the service.");
      return 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
      "service not available, waiting again...");
  }
  return 0;
}

void waitForResponse(auto result, const std::shared_ptr<rclcpp::Node>& node, const std::string& message)
{
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {

    std::string message_log = message + " set correctly.";
    RCLCPP_INFO(node->get_logger(), message_log);
  } else {
    std::string message_log = "Failed to call service " + message;
    RCLCPP_ERROR(node->get_logger(), message_log);
  }
}


int main(int argc, char **argv)
{
  // This program publishes a cmd_vel topic to move the turtle and calls the services teleport and setpen

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rings");

  // ========== PARAMETER RADIUS ==========

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

  // =========== PROGRAMA ===========
  // Creación de clientes
  auto client_teleport_absolute = node->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
  auto client_setpen = node->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");

  // Creación de peticiones
  auto request_setpen = std::make_shared<turtlesim::srv::SetPen::Request>();
  auto request_teleport_absolute = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();

  
  for (int r=0; r<5; r++)
  {
    // ===== PEN TURNED OFF =====
    // Send the request to change the pen settings
    request_setpen->off = true;
    // Wait until the service is available
    if (waitForService(client_setpen) == 1) {
      return 1;
    }
    // Send the request to the service
    auto result_setpen = client_setpen->async_send_request(request_setpen);
    // Wait until a response is received
    waitForResponse(result_setpen, node, "Pen set correctly.");


    // ===== TELEPORT TO NEXT POSITION =====
    request_teleport_absolute->x = positions[r][0];
    request_teleport_absolute->y = positions[r][1];
    if (waitForService(client_teleport_absolute) == 1) {
      return 1;
    }
    // Send the request to the service
    auto result_teleport_absolute = client_teleport_absolute->async_send_request(request_teleport_absolute);
    // Wait until a response is received
    waitForResponse(result_teleport_absolute, node, "Teleport Absolute");


    // ===== PEN TURNED ON =====
    request_setpen->off = false;
    request_setpen->r = colors[r][0];
    request_setpen->g = colors[r][1];
    request_setpen->b = colors[r][2];
    request_setpen->width = 2;
    if (waitForService(client_setpen) == 1) {
      return 1;
    }
    result_setpen = client_setpen->async_send_request(request_setpen);
    waitForResponse(result_setpen, node, "Pen set correctly.");


    // ===== CIRCLE MOVEMENT =====
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



// ros2 run turtlesim turtlesim_node
// ros2 run turtlesim turtlesim_node --ros-args -p radius:=2.0

// ros2 run turtlesim turtle_teleop_key
// ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel

// rqt --force-discover

// ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

// ros2 node info /turtlesim
// - Ver servicios, acciones, subscriptores, publicadores

// ros2 param list   ->  Llistar els paràmetres
// ros2 param get /rings radius   ->  Obtenir el valor del paràmetre
// ros2 param set /rings radius 2.0   ->  Establir el valor del paràmetre
// ros2 param dump /rings   ->  Guardar en fitxer parametres actuals
// ros2 param load /rings /home/user/params.yaml   ->  Carregar els paràmetres des d'un fitxer yaml

// Parameters via launch file: tb3_ws/src/obstacles/launch/triple_detector.launch.py




// SERVICES

// ros2 service list -t   ->  Llistar els serveis actius
// ros2 service type /turtle1/set_pen   ->  Mostrar el tipus de servei

// ros2 interface show turtlesim/srv/Spawn   ->  Mostrar estructura input --- output

// ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 2, off: false}"
//                   /clear std_srvs/srv/Empty
//                   /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"




// ACTIONS

// ros2 action list -t  ->  Llistar les accions actives
// ros2 action info /turtle1/rotate_absolute
//      returns the clients and servers of the action

// ros2 interface show turtlesim/action/RotateAbsolute   ->  goal --- result --- feedback

// ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}" --feedback
