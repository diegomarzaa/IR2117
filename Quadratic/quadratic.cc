#include <iostream>
#include <cmath>

int main() 
{
  double a, b, c;
  std::cout << "Introduce el coeficiente a: ";
  std::cin >> a;
  std::cout << "Introduce el coeficiente b: ";
  std::cin >> b;
  std::cout << "Introduce el coeficiente c: ";
  std::cin >> c;
  
  double discriminante = b*b - 4*a*c;

  if (discriminante < 0) {
    std::cout << "No hay soluciones reales" << std::endl;
    return 1;
  }

  double x1 = (-b + sqrt(discriminante))/(2*a);
  double x2 = (-b - sqrt(discriminante))/(2*a);

  std::cout << "Las soluciones son: " << x1 << " y " << x2 << std::endl;
  return 0;
}
