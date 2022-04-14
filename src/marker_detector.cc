#include "marker_detector.h"

MarkerDetector::MarkerDetector(
    const std::vector<std::string>& camera_list,
    const cv::aruco::PREDEFINED_DICTIONARY_NAME& dict_name)
    : camera_list_(camera_list), dict_name_(dict_name) {}

bool MarkerDetector::Init() {
  for (const std::string& serial_number : camera_list_) {
    cv::Ptr<cv::aruco::DetectorParameters> params =
        cv::aruco::DetectorParameters::create();
    detector_params_.emplace(serial_number, params);
  }
  return true;
}

void Detect(const std::unordered_map<std::string, cv::Mat>& images,
            std::unordered_map<std::string, std::vector<cv::Point2f>>& results,
            std::unordered_map<std::string, bool>& is_success) {}