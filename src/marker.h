#ifndef SRC_MARKER_H_
#define SRC_MARKER_H_

#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

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

class MarkerTrackWriter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MarkerTrackWriter(const std::string& output_dir);

  void InsertNewMarker(const Marker& marker);

  void Close();

 private:
  void TrackWriter();

  void WriteTrack(const Marker& marker, std::string& str);

  // Output directory.
  std::string output_dir_;

  // Marker track.
  std::queue<Marker> track_;

  std::thread writer_thread_;
  bool writer_running_ = true;

  std::mutex data_mutex_;
};

#endif