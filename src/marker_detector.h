#ifndef SRC_MARKER_DETECTOR_H_
#define SRC_MARKER_DETECTOR_H_

#include <unordered_map>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

class MarkerDetector {
 public:
  MarkerDetector(const std::vector<std::string>& camera_list,
                 const cv::aruco::PREDEFINED_DICTIONARY_NAME& dict_name);

  void Detect(
      const std::unordered_map<std::string, cv::Mat>& images,
      std::unordered_map<std::string, std::vector<cv::Point2f>>& results,
      std::unordered_map<std::string, bool>& is_success);

 protected:
  bool DetectMarkerImpl(
      const cv::Mat& image,
      const cv::Ptr<cv::aruco::DetectorParameters>& detector_params,
      const cv::Ptr<cv::aruco::Dictionary>& dict,
      std::vector<cv::Point2f>& results);

  const std::vector<std::string> camera_list_;

  cv::Ptr<cv::aruco::DetectorParameters> params_;
  cv::Ptr<cv::aruco::Dictionary> dict_;
};

#endif