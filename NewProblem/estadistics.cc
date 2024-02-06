// CÀLCUL D'ESTADÍSTICS A PARTIR DE VALORS

// ---------- estadístics - Versió 1
// Demanar valors fins que s'aprete "Enter"
// Mostrar llista final.

// ---------- estadístics - Versió 2
// Si no s'introdueix un valor numèric, demanar de nou.

// ---------- estadístics - Versió 3
// Calcular mitjana, moda, desviació típica, màxim, mínim. Mostrar amb bon format.
// Calcular quartils 0.25 i 0.75.
// Calcular freqüències absolutes i relatives, mostrar en format taula.

// ----------- estadístics - Versió 4
// Menú amb opcions:
// Introduir nou valor.
// Eliminar valor.
// Mostrar valors.
// Mostrar estadístics.
// Eixir.

#include <iostream>
#include <string>
#include <vector>


int main() {
  std::vector<double> lista;
  std::string input;

  while (true) {
    std::cout << "Introduce un valor (Enter para finalizar): ";
    getline(std::cin, input);

    if (input.empty()) break;

    // Si no s'introdueix un valor numèric, demanar de nou.
    try {
      double numero = std::stod(input);
      lista.push_back(numero);
    } catch (std::invalid_argument&) {
      std::cout << "Valor invàlid, no s'ha guardat." << std::endl;
    }
  }

  std::cout << "La lista es:\n[";
  for (int i = 0; i < lista.size(); i++) {
    std::cout << "\t" << lista[i] << "\t";
    if (i < lista.size() - 1) {
      std::cout << "|";
    }
  }
  std::cout << "]" << std::endl;

  return 0;
}
