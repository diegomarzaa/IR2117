#include <iostream>
#include <string>

int sumar(int n) 
{
  int sum = 0;
  for (int i = 1; i<=n; i++) {
    sum += i;
  }
  return sum;
}

int main() 
{
  int number;
  std::cout << "Enter a number: " << std::endl;
  std::cin >> number;

  while (number < 1) {
    std::cout << "You must enter a positive number." << std::endl;
    std::cout << "Enter a number: " << std::endl;
    std::cin >> number;
  }

  std::cout << "The sum from 1 to " << number << " is " << sumar(number) << std::endl;
  return 0;
}
