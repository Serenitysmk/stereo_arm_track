#ifndef TRIANGULATOR_H_
#define TRIANGULATOR_H_

#include <base/camera.h>
#include <base/pose.h>

#include "src/marker.h"

class Triangulator {
 public:
  Triangulator(const std::vector<std::string>& camera_list);

  bool Triangulate(
      const std::unordered_map<std::string, colmap::Camera>& cameras,
      const std::unordered_map<std::string, Eigen::Vector4d>& qvecs,
      const std::unordered_map<std::string, Eigen::Vector3d>& tvecs,
      Marker& marker);

 private:
  const std::vector<std::string> camera_list_;
};

#endif