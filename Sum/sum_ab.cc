#include <iostream>

int main() 
{
  int a, b;
  int sum = 0;

  std::cout << "Enter a number >= 1: " << std::endl;
  std::cin >> a;
  while (a < 1)
  {
    std::cout << "Enter a number >= 1: " << std::endl;
    std::cin >> a;
  }
  std::cout << "Enter b number >= 1: " << std::endl;
  std::cin >> b;
  while (b < a) {
    std::cout << "Enter b number >= 1: " << std::endl;
    std::cin >> b;
  }
  
  for (int num_act = a; num_act<=b; num_act++) {
    sum += num_act;
  }

  std::cout << "The sum from " << a << " to " << b << " is " << sum << std::endl;
  return 0;
}
