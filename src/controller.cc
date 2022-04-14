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

  PrintHeading1("Robot arm tracking program using multiple cameras"); 
  
  // Initialize frame grabber.
  grabber_ = new FrameGrabber(
      options_->num_cameras, CSVToVector<std::string>(options_->camera_list),
      options_->input_from_videos, options_->video_path);

  CHECK(grabber_->Init()) << "ERROR: Failed to initialize the frame grabber!";

  // Initialize detectror.
  //detector_ = new MarkerDetector();
}

void Controller::Run() { PrintHeading1("Running the robot arm tracker"); }