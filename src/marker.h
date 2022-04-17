#ifndef SRC_MARKER_H_
#define SRC_MARKER_H_

#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core.hpp>

struct Marker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Marker() {}

  // Positions.
  std::vector<Eigen::Vector3d> positions;

  // Observations.
  std::unordered_map<std::string, std::vector<cv::Point2f>> observations;

  Eigen::Vector3d center;
};

class MarkerTrack {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MarkerTrack();

  const std::vector<Marker>& Track() const;
  std::vector<Marker>& Track();

 private:
  std::vector<Marker> track_;
};

#endif