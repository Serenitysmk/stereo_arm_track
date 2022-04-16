#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

int main(int argc, char* argv[]) {
  std::cout << "Hello World" << std::endl;
  int foo = 0;
  while(foo == 0){
    std::cout << "entering a number: "<< std::endl;
    std::cin >> foo;
  }
  
  return EXIT_SUCCESS;
}