#include "src/triangulator.h"

#include <iostream>

#include <base/projection.h>
#include <base/triangulation.h>

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
  size_t num_points = 0;
  for (const auto& ob : marker.observations) {
    num_points = ob.second.size();
    break;
  }

  std::vector<Eigen::Vector3d> positions;

  for (size_t point_idx = 0; point_idx < num_points; point_idx++) {
    std::vector<Eigen::Vector2d> points;
    std::vector<Eigen::Vector4d> qvecs_data;
    std::vector<Eigen::Vector3d> tvecs_data;
    for (const std::string& sn : camera_list_) {
      if (marker.observations.find(sn) == marker.observations.end()) {
        continue;
      }
      Eigen::Vector2d point2D(marker.observations.at(sn)[point_idx].x,
                              marker.observations.at(sn)[point_idx].y);
      points.emplace_back(cameras.at(sn).ImageToWorld(point2D));
      qvecs_data.emplace_back(qvecs.at(sn));
      tvecs_data.emplace_back(tvecs.at(sn));
    }
    Eigen::Vector3d xyz;
    bool success = EstimateTriangulation(points, qvecs_data, tvecs_data, &xyz);
    if (success) {
      positions.emplace_back(xyz);
    }
  }

  if (positions.size() < 3) {
    return false;
  } else {
    Eigen::Vector3d center;
    for (const auto& pos : positions) {
      center += pos;
    }
    center /= static_cast<double>(positions.size());
    marker.positions = positions;
    marker.center = center;
    return true;
  }
}

bool Triangulator::EstimateTriangulation(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<Eigen::Vector4d>& qvecs,
    const std::vector<Eigen::Vector3d>& tvecs, Eigen::Vector3d* xyz) {
  std::vector<Eigen::Matrix3x4d> proj_matrices;
  proj_matrices.reserve(points.size());

  for (size_t i = 0; i < points.size(); i++) {
    proj_matrices.push_back(ComposeProjectionMatrix(qvecs[i], tvecs[i]));
  }
  *xyz = TriangulateMultiViewPoint(proj_matrices, points);

  for (const auto& pose : proj_matrices) {
    if (!HasPointPositiveDepth(pose, *xyz)) {
      return false;
    }
  }
  return true;
}