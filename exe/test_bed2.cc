#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include <Eigen/Dense>

int main(int argc, char* argv[]) {
  Eigen::Vector3d pos1(115.764, 56.6645, 291.359);
  Eigen::Vector3d pos2(127.971, 54.954, 297.063);
  Eigen::Vector3d pos3(127.45, 62.9811, 296.766);
  Eigen::Vector3d pos4(115.257, 64.7944, 291.003);  
  
  std::vector<Eigen::Vector3d> positions = {pos1, pos2, pos3, pos4};

  Eigen::Vector3d center(0.0, 0.0, 0.0);
  
  std::cout << center << std::endl;
  return EXIT_SUCCESS;
}