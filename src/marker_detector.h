#ifndef SRC_MARKER_DETECTOR_H_
#define SRC_MARKER_DETECTOR_H_

#include <condition_variable>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

class MarkerDetector {
 public:
  MarkerDetector(const std::vector<std::string>& camera_list,
                 const cv::aruco::PREDEFINED_DICTIONARY_NAME& dict_name);

  bool Init();

  void Detect(
      const std::unordered_map<std::string, cv::Mat>& images,
      std::unordered_map<std::string, std::vector<cv::Point2f>>& results,
      std::unordered_map<std::string, bool>& is_success);

  void Stop();

 protected:
  void DetectorLoop(const std::string& serial_number);

  bool DetectMarkerImpl(
      const cv::Mat& image,
      const cv::Ptr<cv::aruco::DetectorParameters>& detector_params,
      const cv::Ptr<cv::aruco::Dictionary>& dict,
      std::vector<cv::Point2f>& results);

  // Aruco dictionary name.
  const cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name_;

  // Aruco detection workers.
  std::unordered_map<std::string, std::thread*> workers_;

  // Images.
  std::unordered_map<std::string, cv::Mat> images_;

  // Detection results.
  std::unordered_map<std::string, std::vector<cv::Point2f>> results_;

  // Whether the detector worked.
  std::unordered_map<std::string, bool> is_success_;

  // Mutex.
  std::mutex get_frame_mutex_;
  std::mutex write_results_mutex_;
  std::mutex request_stop_mutex_;

  // Condition variables.
  std::unordered_map<std::string, bool> new_frame_arrived_;
  int detect_finished_ = 0;
  std::condition_variable detect_finish_condition_;
  bool stop_detector_ = false;

  const std::vector<std::string> camera_list_;
};

#endif