#ifndef SRC_MARKER_DETECTOR_H_
#define SRC_MARKER_DETECTOR_H_

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
  // Aruco dictionary name.
  const cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name_;

  // Aruco detection parameters
  std::unordered_map<std::string, cv::Ptr<cv::aruco::DetectorParameters>>
      detector_params_;

  // Aruco detection workers.
  std::unordered_map<std::string, std::thread> thread_pool_;

  // Detection results.
  std::unordered_map<std::string, std::vector<cv::Point2f>> results_;

  // Whether the detector worked.
  std::unordered_map<std::string, bool> is_success_;

  // Mutex.
  std::mutex get_frame_mutex_;
  std::mutex write_results_mutex_;

  const std::vector<std::string> camera_list_;
};

#endif