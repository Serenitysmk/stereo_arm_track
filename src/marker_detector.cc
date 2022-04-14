#include "marker_detector.h"

#include <opencv2/opencv.hpp>

MarkerDetector::MarkerDetector(
    const std::vector<std::string>& camera_list,
    const cv::aruco::PREDEFINED_DICTIONARY_NAME& dict_name)
    : camera_list_(camera_list), dict_name_(dict_name) {}

bool MarkerDetector::Init() {
  for (const std::string& serial_number : camera_list_) {
  }
  return true;
}

void MarkerDetector::Detect(
    const std::unordered_map<std::string, cv::Mat>& images,
    std::unordered_map<std::string, std::vector<cv::Point2f>>& results,
    std::unordered_map<std::string, bool>& is_success) {}

void MarkerDetector::Stop() {}

void MarkerDetector::DetectorLoop(const std::string& serial_number) {}

bool DetectMarkerImpl(
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