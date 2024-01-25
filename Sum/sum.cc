#include <iostream>
#include <string>

int main() {
    int number;
    int sum = 0;

    std::cout << "Enter a number: " << std::endl;
    std::cin >> number;

    for (int i = 1; i<=number; i++) {
        sum += i;
    }

    std::cout << "The sum from 1 to " << number << " is " << sum << std::endl;
    return 0;
}
