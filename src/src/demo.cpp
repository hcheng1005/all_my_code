#include <iostream>
#include <string>
#include <dirent.h>

int main() {
    const int *a;
    int *a2;
    int b = 12;
    int b2 =  100;

    std::cout << a  << ", " <<  (*a) << std::endl;
    a = &b;
    *a2 = 12;

    std::cout << a  << ", " <<  (*a) << std::endl;

    int * const cc = &b2;
    // std::cout << c  << ", " <<  (*c) << std::endl;
   return 0;
}

