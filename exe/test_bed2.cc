#include <iostream>

#include <queue>
#include <unordered_map>

std::unordered_map<int, std::string*> g_test;

struct A {
  std::queue<std::unordered_map<int, std::string*>> queue;
};

int main(int argc, char* argv[]) {
  std::cout << "Hello World" << std::endl;

  A a;
  for (int i = 0; i < 10; i++) {
    std::string* str = new std::string(std::to_string(i) + " th frame");
    g_test.emplace(i, str);
    a.queue.push(g_test);
    g_test.clear();
  }
  
  while(!a.queue.empty()){
      std::unordered_map<int, std::string*> elm = a.queue.front();
      a.queue.pop();
      for(const auto& val: elm){
          std::cout << val.first << " , " << *val.second << std::endl;
      }
  }
  return EXIT_SUCCESS;
}