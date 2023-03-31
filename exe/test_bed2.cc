#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include <Eigen/Dense>
#include <iomanip>

int main(int argc, char* argv[]) {
  
  for (int i = 0; i < 10000; i++) {
      std::stringstream stream;
      stream << "frame_" << std::setfill('0') << std::setw(6) << i << ".png";
      std::cout << stream.str() << std::endl;
  }
  return EXIT_SUCCESS;
}