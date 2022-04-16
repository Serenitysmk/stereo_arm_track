#ifndef SRC_VIEWER_H_
#define SRC_VIEWER_H_

#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include <pangolin/pangolin.h>

#include "src/marker.h"

class Viewer {
 public:
  Viewer(const std::vector<std::string>& camera_list,
         const std::unordered_map<std::string, Eigen::Vector4d>& qvecs,
         const std::unordered_map<std::string, Eigen::Vector3d>& tvecs,
         const double display_scale);

  void AddCurrentFrame(
      const std::unordered_map<std::string, cv::Mat>& current_frames,
      const Marker& current_marker);

  void Close();

 private:
  void ThreadLoop();

  cv::Mat DrawFrameImage();

  void PlotFrame(const Eigen::Vector4d& qvec, const Eigen::Vector3d& tvec, const float* color);

  const std::vector<std::string> camera_list_;

  const std::unordered_map<std::string, Eigen::Vector4d> qvecs_; 
  
  const std::unordered_map<std::string, Eigen::Vector3d> tvecs_;

  const double display_scale_;

  std::unordered_map<std::string, cv::Mat> current_frames_;

  Marker current_marker_;

  bool new_frame_arrived_ = false;

  // Viewer display.
  std::thread viewer_thread_;
  bool viewer_running_ = true;

  std::mutex viewer_data_mutex_;
};

#endif