#include "src/controller.h"

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
  detector_ = new MarkerDetector(camera_lists_, cv::aruco::DICT_4X4_1000);

  // Initialize triangulator.
  triangulator_ = new Triangulator(camera_lists_);

  // Initialize viewer.
  viewer_ = new Viewer(camera_lists_, options_->display_scale);
}

void Controller::Run() {
  std::thread running_control_thread(&Controller::StopRunningControlLoop, this);
  PrintHeading1("Start running ...");

  size_t frame_cnt = 0;
  while (!stop_running_) {
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

    std::unordered_map<std::string, std::vector<cv::Point2f>> markers_corners;
    std::unordered_map<std::string, bool> detection_success;

    detector_->Detect(frames, markers_corners, detection_success);

    for (const std::string& serial_number : camera_lists_) {
      if (detection_success.at(serial_number)) {
        std::cout << serial_number << " detection success!" << std::endl;
      } else {
        std::cout << serial_number << " detection failed" << std::endl;
      }
    }

    Marker marker;
    marker.observations = markers_corners;
    viewer_->AddCurrentFrame(frames, marker);
    frame_cnt++;
  }

  stop_running_ = true;
  running_control_thread.detach();

  viewer_->Close();
}

void Controller::Shutdown() {
  PrintHeading1("Finished running, shutdown the system");
  if (!grabber_->Close()) {
    std::cerr << "ERROR: Failed to close the frame grabber!" << std::endl;
    return;
  }
}

void Controller::StopRunningControlLoop() {
  char key;
  while (!stop_running_) {
    std::cin >> key;
    if ((int)key == 27) {
      stop_running_ = true;
      break;
    }
  }
}
