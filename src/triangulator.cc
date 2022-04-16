#include "src/triangulator.h"

using namespace colmap;

Triangulator::Triangulator(const std::vector<std::string>& camera_list)
    : camera_list_(camera_list) {}

bool Triangulator::Triangulate(
    const std::unordered_map<std::string, Camera>& cameras,
    const std::unordered_map<std::string, Eigen::Vector4d>& qvecs,
    const std::unordered_map<std::string, Eigen::Vector3d>& tvecs,
    Marker& marker) {
  if (marker.observations.size() < 2) {
    std::cerr << "WARNING: The marker is observed by less than 2 cameras, "
                 "can't perform triangulation"
              << std::endl;
    return false;
  }
}