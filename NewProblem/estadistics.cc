#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <fstream>

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


double calcular_mitjana(const std::vector<double>& llista_dades) {
  double total = 0;
  int tamany = llista_dades.size();

  for (double num : llista_dades) {
    total += num;
  }
  return total / tamany;
}

double calcular_moda(const std::vector<double>& llista_dades) {
  std::map<double, int> frequencies;
  for (double num : llista_dades) {
    frequencies[num]++;
  }

  double moda = 0;
  int cant_max = 0;

  for (auto parella : frequencies) {
    if (parella.second > cant_max) {
      moda = parella.first;
      cant_max = parella.second;
    }
  }

  return moda;
}


double calcular_desviacio_tipica(const std::vector<double>& llista_dades, double mitjana) {
    double total = 0;
    int tamany = llista_dades.size();

    for (double num : llista_dades) {
      total += (num - mitjana) * (num - mitjana);
    }
    return std::sqrt(total / tamany);
}

double calcular_maxim(const std::vector<double>& llista_dades) {
  double max = llista_dades[0];
  for (double num : llista_dades) {
    if (num > max) {
      max = num;
    }
  }
  return max;
}

double calcular_minim(const std::vector<double>& llista_dades) {
  double min = llista_dades[0];
  for (double num : llista_dades) {
    if (num < min) {
      min = num;
    }
  }
  return min;
}

double calcular_quartil(
  const std::vector<double>& llista_dades, double quartil) {
  
  int tamany = llista_dades.size();
  int posicio_quartil = quartil * (tamany + 1);

  // Ordenar la llista.
  std::vector<double> llista_ordenada = llista_dades;
  std::sort(llista_ordenada.begin(), llista_ordenada.end());

  // Tornar el valor corresponent al quartil.
  return llista_ordenada[posicio_quartil - 1];
}

void mostrar_llista(const std::vector<double>& llista_dades) {
  // Mostrar llista final.
  std::cout << "####### Llista de dades #######\n[\n";

  for (int i = 0; i < llista_dades.size(); i++) {
    std::cout << "\t" << llista_dades[i];
    if (i < llista_dades.size() - 1) {
      std::cout << ",\n";
    } else {
      std::cout << "\n";
    }
  }

  std::cout << "]" << std::endl << std::endl;
}

void mostrar_taula(const std::vector<double>& llista_dades) {
  std::vector<double> llista_ordenada = llista_dades;
  std::sort(llista_ordenada.begin(), llista_ordenada.end());
  std::vector<double> valors;
  std::vector<int> freq;

  double tamany = llista_dades.size();

  for (int i = 0; i < llista_ordenada.size(); i++) {
    if (i == 0 or llista_ordenada[i] != llista_ordenada[i - 1]) {
      valors.push_back(llista_ordenada[i]);
      freq.push_back(1);
    } else {
      freq[freq.size() - 1]++;
    }
  }

  std::cout << "####### Taula de freqüències #######" << std::endl;
  std::cout << "Valor\tFreq. Abs.\tFreq. Rel." << std::endl;
  for (int i = 0; i < valors.size(); i++) {
    std::cout << valors[i] << "\t" << freq[i] << "\t\t" << freq[i] / tamany << std::endl;
  }
}

void mostrar_estadistics(
  double mitjana, double moda, double desviacio_tipica, 
  double maxim, double minim, double quartil_25, double quartil_75) {

  std::cout << "####### Estadístics #######" << std::endl;
  std::cout << "Mitjana: " << mitjana << std::endl;
  std::cout << "Moda: " << moda << std::endl;
  std::cout << "Desviació típica: " << desviacio_tipica << std::endl;
  std::cout << "Màxim: " << maxim << std::endl;
  std::cout << "Mínim: " << minim << std::endl;
  std::cout << "Quartil 0.25: " << quartil_25 << std::endl;
  std::cout << "Quartil 0.75: " << quartil_75 << std::endl;
}

bool llista_buida(const std::vector<double>& llista_dades) {
  return llista_dades.size() == 0;
}

void enter_per_a_continuar() {
  std::string input;
  std::cout << "Apreta 'Enter' per a continuar" << std::endl;
  getline(std::cin, input);
}

bool guardar_dades(const std::vector<double>& llista_dades) {
  std::string nom_fitxer;
  std::cout << "Introdueix el nom del fitxer on guardar les dades: ";
  getline(std::cin, nom_fitxer);
  if (nom_fitxer.empty()) {
    std::cout << "Nom de fitxer invàlid." << std::endl;
    return false;
  }
  std::ofstream fitxer(nom_fitxer);
  for (double num : llista_dades) {
    fitxer << num << std::endl;
  }
  fitxer.close();
  return true;
}

bool carregar_dades(std::vector<double>& llista_dades) {
  std::string nom_fitxer;
  std::cout << "Introdueix el nom del fitxer d'on carregar les dades: ";
  getline(std::cin, nom_fitxer);
  std::ifstream fitxer(nom_fitxer);

  if (not fitxer.is_open()) {
    return false;
  }

  std::string linia;
  while (getline(fitxer, linia)) {
    double num = std::stod(linia);
    llista_dades.push_back(num);
  }
  fitxer.close();
  return true;
}


int main() {
  std::vector<double> llista_dades;
  std::string input;
  int opcio;

  while (true) {

    std::cout << "\n####### MENÚ #######" << std::endl;
    std::cout << "1. Introduir valor a la llista." << std::endl;
    std::cout << "2. Eliminar valor de la llista." << std::endl;
    std::cout << "3. Mostrar llista de valors." << std::endl;
    std::cout << "4. Mostrar taula de freqüències." << std::endl;
    std::cout << "5. Mostrar estadístics." << std::endl;
    std::cout << "6. Carregar dades de fitxer." << std::endl;
    std::cout << "7. Guardar dades a fitxer." << std::endl;
    std::cout << "8. Eixir." << std::endl;
    std::cout << "Introdueix una opció: ";
    getline(std::cin, input);

    if (input.empty()) {
      std::cout << "No s'ha introduït cap opció." << std::endl;
      continue;
    }

    try {
      opcio = std::stoi(input);
    } catch (std::invalid_argument&) {
      std::cout << "Opció invàlida." << std::endl;
    }

    if (opcio == 1) {
      bool introduint = true;
      while (introduint) {
        std::cout << "Introdueix un número a la llista (o apreta Enter per a acabar): ";
        getline(std::cin, input);
        if (input.empty()) {
          introduint = false;
        } else {
          try {
            double numero = std::stod(input);
            llista_dades.push_back(numero);
          } catch (std::invalid_argument&) {
            std::cout << "Valor invàlid, no s'ha guardat." << std::endl;
          }
        }
      }
      continue;

    } else if (opcio == 2 and not llista_buida(llista_dades)) {
      std::cout << "Introdueix la posició del valor a eliminar: ";      
      getline(std::cin, input);
      try {
        int posicio = std::stoi(input);
        if (posicio < 0 or posicio >= llista_dades.size()) {
          std::cout << "Posició invàlida." << std::endl;
        } else {
          llista_dades.erase(llista_dades.begin() + posicio);
          std::cout << "Valor eliminat." << std::endl;
        }
      } catch (std::invalid_argument&) {
        std::cout << "Posició invàlida." << std::endl;
      }

    } else if (opcio == 3 and not llista_buida(llista_dades)) {
      mostrar_llista(llista_dades);

    } else if (opcio == 4 and not llista_buida(llista_dades)) {
      mostrar_taula(llista_dades);

    } else if (opcio == 5 and not llista_buida(llista_dades)) {
      double mitjana = calcular_mitjana(llista_dades);
      double moda = calcular_moda(llista_dades);
      double desviacio_tipica = calcular_desviacio_tipica(llista_dades, mitjana);
      double maxim = calcular_maxim(llista_dades);
      double minim = calcular_minim(llista_dades);
      double quartil_25 = calcular_quartil(llista_dades, 0.25);
      double quartil_75 = calcular_quartil(llista_dades, 0.75);
      mostrar_estadistics(mitjana, moda, desviacio_tipica, maxim, minim, quartil_25, quartil_75);

    } else if (opcio == 6) {
      if (carregar_dades(llista_dades)) {
        std::cout << "Dades carregades correctament." << std::endl;
      } else {
        std::cout << "No s'ha pogut carregar el fitxer." << std::endl;
      }

    } else if (opcio == 7 and not llista_buida(llista_dades)) {
      if (guardar_dades(llista_dades)) {
        std::cout << "Dades guardades correctament." << std::endl;
      } else {
        std::cout << "No s'ha pogut guardar el fitxer." << std::endl;
      }

    } else if (opcio == 8) {
      break;
    
    } else {
      std::cout << "Opció invàlida. Comprova que la llista no està buida o que hages introduit una opció correcta" << std::endl;
    }

    enter_per_a_continuar();
  }
  return 0;
}
