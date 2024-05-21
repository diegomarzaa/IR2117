#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"    // Estrucutra del servei
#include <chrono>
#include <cstdlib>
#include <memory>
using namespace std::chrono_literals;   // Per a poder posar 1s en lloc de std::chrono::seconds(1)

using example_interfaces::srv::AddTwoInts;    // Tipus de servei

int main(int argc, char **argv)
{
  // Programa que simplement envia una petició a un servei, mostra la resposta i finalitza

  rclcpp::init(argc, argv);
  if (argc != 3) {
  	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
     "usage: add_two_ints_client X Y");
  	return 1;
  }

  // Creació del node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");

  // Creació del client
  rclcpp::Client<AddTwoInts>::SharedPtr client = node->create_client<AddTwoInts>("add_two_ints");

  // Creació de la petició
  // Estructura definida per el archivo .srv
  // Per això ens em asegurat de que es passen 2 arguments (+1 per al nom del programa)
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = atoll(argv[1]);      // atoll: string to long long
  request->b = atoll(argv[2]);

  // ESPERAR fins que el servei estigui disponible o fins que es cancel·li
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
      "Interrupted while waiting for the service.");
      return 0;
    }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
    "service not available, waiting again...");
  }

  // Enviar la petició al servei...
  auto result = client->async_send_request(request);

  // ESPERAR fins que la resposta sigui completa. Segons el resultat, es mostra la suma o un missatge d'error
  if (rclcpp::spin_until_future_complete(node, 
      result) ==	rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
    "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
    "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
