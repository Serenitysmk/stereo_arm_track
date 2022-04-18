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
         const size_t max_track_length,
         const double world_display_scale,
         const double image_display_scale);

  void InsertCurrentFrame(
      const std::unordered_map<std::string, cv::Mat>& current_frames,
      const std::unordered_map<std::string, std::vector<cv::Point2f>>&
          current_observations);

  void InsertNewMarker(const Marker& marker);

  void Close();

 private:
  void ThreadLoop();

  cv::Mat DrawFrameImage();

  void RenderFrame(const Eigen::Vector4d& qvec, const Eigen::Vector3d& tvec,
                   const float* color);

  void RenderMarker(const Marker& marker, const float* color);
  
  void RenderMarkers();

  void RenderCoordinateAxis();

  const std::vector<std::string> camera_list_;

  const std::unordered_map<std::string, Eigen::Vector4d> qvecs_;

  const std::unordered_map<std::string, Eigen::Vector3d> tvecs_;

  const size_t max_track_length_;

  const double world_display_scale_;

  const double image_display_scale_;

  std::unordered_map<std::string, cv::Mat> current_frames_;

  std::unordered_map<std::string, std::vector<cv::Point2f>>
      current_observations_;

  std::vector<Marker> track_;

  bool new_frame_arrived_ = false;

  // Viewer display.
  std::thread viewer_thread_;
  bool viewer_running_ = true;

  std::mutex viewer_data_mutex_;
};

#endif