#include "controller.h"

using namespace colmap;

bool ControllerOptions::Check() const {
  CHECK_OPTION_GT(num_cameras, 0);
  std::vector<std::string> _camera_list = CSVToVector<std::string>(camera_list);
  CHECK_OPTION_GT(_camera_list.size(), 0);
  CHECK_OPTION_LE(_camera_list.size(), num_cameras);
  if (input_from_videos) {
    CHECK(ExistsDir(video_path));
  }
  return true;
}

Controller::Controller(const ControllerOptions* options) : options_(options) {
  CHECK(options_->Check());

  camera_lists_ = CSVToVector<std::string>(options_->camera_list);

  PrintHeading1("Robot arm tracking system using multiple cameras");

  // Initialize frame grabber.
  grabber_ =
      new FrameGrabber(options_->num_cameras, camera_lists_,
                       options_->input_from_videos, options_->video_path);

  CHECK(grabber_->Init()) << "ERROR: Failed to initialize the frame grabber!";

  // Initialize detectror.
  detector_ = new MarkerDetector(camera_lists_, cv::aruco::DICT_6X6_1000);

}

void Controller::Run() {
  PrintHeading1("Start running ...");

  size_t frame_cnt = 0;
  while (true) {
    // Grab new frames.
    bool grab_success = true;
    std::unordered_map<std::string, cv::Mat> frames = grabber_->Next();

    for (const std::string& serial_number : camera_lists_) {
      if (frames.at(serial_number).empty()) {
        grab_success = false;
      }
    }
    if (!grab_success) {
      break;
    }

    // Grab success.
    std::cout << "frame: " << frame_cnt << std::endl;

    std::unordered_map<std::string, std::vector<cv::Point2f>> markers;
    std::unordered_map<std::string, bool> detection_success;

    detector_->Detect(frames, markers, detection_success);

    for(const std::string& serial_number: camera_lists_){
      if(detection_success.at(serial_number)){
        std::cout << serial_number << " detection success!" << std::endl;
      }else{
        std::cout << serial_number << " detection failed" << std::endl;
      }
    }
    for (const auto& marker : markers) {
      std::cout << marker.first << ": ";
      for (const auto& point : marker.second) {
        std::cout << point << " ";
      }
      std::cout << std::endl;
    }
    frame_cnt++;
  }
}

void Controller::Shutdown() {
  PrintHeading1("Finished running, shutdown the system");
  if (!grabber_->Close()) {
    std::cerr << "ERROR: Failed to close the frame grabber!" << std::endl;
    return;
  }
}
