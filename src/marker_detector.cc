#include "marker_detector.h"

#include <util/misc.h>

#include <opencv2/opencv.hpp>

using namespace colmap;

MarkerDetector::MarkerDetector(
    const std::vector<std::string>& camera_list,
    const cv::aruco::PREDEFINED_DICTIONARY_NAME& dict_name)
    : camera_list_(camera_list), dict_name_(dict_name) {}

bool MarkerDetector::Init() {
  PrintHeading2("Initialize marker detector");

  for (const std::string& serial_number : camera_list_) {
    new_frame_arrived_.emplace(serial_number, false);
    workers_.emplace(serial_number,
                     new std::thread(&MarkerDetector::DetectorLoop, this,
                                     std::ref(serial_number)));
  }
  return true;
}

void MarkerDetector::Detect(
    const std::unordered_map<std::string, cv::Mat>& images,
    std::unordered_map<std::string, std::vector<cv::Point2f>>& results,
    std::unordered_map<std::string, bool>& is_success) {
  {
    std::unique_lock<std::mutex> lock(get_frame_mutex_);
    images_ = images;
    for (const std::string& serial_number : camera_list_) {
      new_frame_arrived_.at(serial_number) = true;
    }
  }

  // Wait for the detection process.
  {
    std::unique_lock<std::mutex> lock(write_results_mutex_);
    detect_finish_condition_.wait(
        lock, [this] { return detect_finished_ == camera_list_.size(); });
  }

  results = results_;
  is_success = is_success_;
  results_.clear();
  is_success_.clear();
  detect_finished_ = 0;
}

void MarkerDetector::Stop() {
  std::unique_lock<std::mutex> lock(request_stop_mutex_);
  stop_detector_ = true;
}

void MarkerDetector::DetectorLoop(const std::string& serial_number) {
  cv::Mat frame;
  bool success = false;
  std::vector<cv::Point2f> result;

  cv::Ptr<cv::aruco::DetectorParameters> params =
      cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dict =
      cv::aruco::getPredefinedDictionary(dict_name_);

  while (true) {
    // Check if there is a new frame.
    {
      std::unique_lock<std::mutex> lock(get_frame_mutex_);
      if (!new_frame_arrived_.at(serial_number)) {
        std::this_thread::sleep_for(std::chrono::microseconds(5));
        continue;
      } else {
        frame = images_.at(serial_number);
      }
      new_frame_arrived_.at(serial_number) = false;
    }

    // Yes, there is a new frame, now detect.
    success = DetectMarkerImpl(frame, params, dict, result);
    {
      std::unique_lock<std::mutex> lock(write_results_mutex_);
      if (success) {
        results_.emplace(serial_number, result);
      }
      is_success_.emplace(serial_number, success);
      detect_finished_++;
      detect_finish_condition_.notify_one();
    }

    // Check if stops the loop.
    {
      std::unique_lock<std::mutex> lock(request_stop_mutex_);
      if (stop_detector_) {
        break;
      }
    }
  }
}

bool MarkerDetector::DetectMarkerImpl(
    const cv::Mat& image,
    const cv::Ptr<cv::aruco::DetectorParameters>& detector_params,
    const cv::Ptr<cv::aruco::Dictionary>& dict,
    std::vector<cv::Point2f>& results) {
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;

  cv::aruco::detectMarkers(image, dict, marker_corners, marker_ids,
                           detector_params);

  if (marker_ids.size() == 0) {
    return false;
  }
  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }

  for (auto& marker : marker_corners) {
    cv::cornerSubPix(
        gray, marker, cv::Size(7, 7), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100,
                         1e-6));
  }
  results = marker_corners[0];
  return true;
}