#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>

// RUNNING THE NODE
//1 terminal $ ros2 run stats mean          (nothing happens for now)
//2 terminal $ ros2 topic pub /number std_msgs/msg/Int32 "{data: '1'}"    (now in the first terminal we see "Mensaje recibido: 1" each second)
//3 terminal $ ros2 topic echo /mean        (data: 1, data: 2, data: 3...)  (this one must be executed last, while the other two are running, if not, it will show WARNING: topic [/mean] does not appear to be published yet)

float sum = 0;
int message_count = 0;
float mean_to_publish = 0;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {

  // Recibimos un entero a través del topic "/number"
  // Agregamos el nuevo valor al total, aumentamos la cantidad de mensajes, y hacemos la media
  // Luego publicamos el valor de la media al topic "/mean"

  sum += msg->data;
  message_count++;
  mean_to_publish = sum / message_count;

  std_msgs::msg::Int32 out_msg;
  out_msg.data = mean_to_publish;
  publisher->publish(out_msg);

  std::cout << "Mensaje recibido: " << msg->data << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mean");

  // SUSCRIPTOR AL TOPIC "/number"
  // Cada vez que se recibe un mensaje, se llama a la función "topic_callback", que aumenta el contador de mensajes en 1 y publica este valor.
  auto subscription = node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
  
  // PUBLICADOR AL TOPIC "/mean"
  // Se publica el valor del contador de mensajes cada vez que se recibe un mensaje.
  publisher = node->create_publisher<std_msgs::msg::Int32>("mean", 10);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
