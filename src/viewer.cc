#include "src/viewer.h"

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

Viewer::Viewer(const std::vector<std::string>& camera_list,
               const double display_scale)
    : camera_list_(camera_list), display_scale_(display_scale) {
  viewer_thread_ = std::thread(&Viewer::ThreadLoop, this);
}

void Viewer::AddCurrentFrame(
    const std::unordered_map<std::string, cv::Mat>& current_frames,
    const Marker& current_marker) {
  std::unique_lock<std::mutex> lock(viewer_data_mutex_);
  current_frames_ = current_frames;
  current_marker_ = current_marker;
  new_frame_arrived_ = true;
}

void Viewer::Close() {
  viewer_running_ = false;
  viewer_thread_.join();
}

void Viewer::ThreadLoop() {
  while (viewer_running_) {
    std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    if (new_frame_arrived_) {
      cv::Mat img = DrawFrameImage();

      if (display_scale_ != 1.0) {
        cv::Mat img_resize;
        cv::resize(img, img_resize, cv::Size(), display_scale_, display_scale_);
        cv::imshow("Frames", img_resize);
        cv::waitKey(1);
      } else {
        cv::imshow("Frames", img);
        cv::waitKey(1);
      }
    }

    new_frame_arrived_ = false;
  }
}

cv::Mat Viewer::DrawFrameImage() {
  const size_t num_cameras = camera_list_.size();

  const size_t width = current_frames_.at(camera_list_[0]).cols;
  const size_t height = current_frames_.at(camera_list_[0]).rows;

  cv::Mat img_show;
  if (num_cameras < 4) {
    img_show = cv::Mat::zeros(cv::Size(num_cameras * width, height), CV_8UC3);
  } else {
    img_show = cv::Mat::zeros(cv::Size(2 * width, 2 * height), CV_8UC3);
  }

  for (size_t camera_idx = 0; camera_idx < num_cameras; camera_idx++) {
    cv::Mat frame = current_frames_.at(camera_list_[camera_idx]).clone();
    cv::Mat color;
    if (frame.channels() == 3) {
      color = frame;
    } else {
      cv::cvtColor(frame, color, cv::COLOR_GRAY2BGR);
    }
    if (current_marker_.observations.find(camera_list_[camera_idx]) !=
        current_marker_.observations.end()) {
      std::vector<std::vector<cv::Point2f>> marker_corners = {
          current_marker_.observations.at(camera_list_[camera_idx])};

      cv::aruco::drawDetectedMarkers(color, marker_corners, cv::noArray());
    }
    if (num_cameras < 4) {
      color.copyTo(
          img_show(cv::Range(0, height),
                   cv::Range(camera_idx * width, (camera_idx + 1) * width)));
    } else {
      int i = (int)camera_idx / 2;
      int j = (int)camera_idx % 2;
      color.copyTo(img_show(cv::Range(i * height, (i + 1) * height),
                            cv::Range(j * width, (j + 1) * width)));
    }
  }
  return img_show;
}