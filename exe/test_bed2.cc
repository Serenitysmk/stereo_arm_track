#include <iostream>

#include <chrono>
#include <queue>
#include <unordered_map>

std::unordered_map<int, std::string*> g_test;

struct A {
  std::queue<std::unordered_map<int, std::string*>> queue;
};

int main(int argc, char* argv[]) {
  std::cout << "Hello World" << std::endl;

  auto time = std::chrono::duration<double, std::ratio<60>>(0.5);
  
  auto time_seconds = std::chrono::duration_cast<std::chrono::seconds>(time);
  std::cout << time_seconds.count() << std::endl;
  return EXIT_SUCCESS;
}