#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <memory>
using example_interfaces::srv::AddTwoInts;    // Tipus de servei

void add(
  std::shared_ptr<AddTwoInts::Request>  request,
  std::shared_ptr<AddTwoInts::Response> response)
{
  // S'agafen els números de la request i es posa la suma en la resposta.
  // No cal publicar res, simplement al modificar el response ja es mana al client.
  response->sum = request->a + request->b;

  // Notificar a la consola usant logs
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
    "Incoming request\na: %ld" " b: %ld",
     request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
    "sending back response: [%ld]", 
     (long int)response->sum);
}


int main(int argc, char **argv)
{
  // Creació del node
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = 
    rclcpp::Node::make_shared("add_two_ints_server");

  // Creació del servei
  rclcpp::Service<AddTwoInts>::SharedPtr service =
	 node->create_service<AddTwoInts>("add_two_ints", &add);

  // Imprimeix un missatge per a indicar que el servei està actiu
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
    "Ready to add two ints.");

  rclcpp::spin(node);     // Mantindre el servei actiu
  rclcpp::shutdown();
}



// SERVICES

// MIRAR: olympic

// ros2 service list -t   ->  Llistar els serveis actius
// ros2 service type /add_two_ints   ->  Mostrar el tipus de servei

// ros2 interface show example_interfaces/srv/AddTwoInts   ->  Mostrar estructura input (param) --- output (result)
//   - int64 a
//   - int64 b
//   ---
//   - int64 sum

// ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
// - També es pot emprar el rqt  (rqt --force-discover)





