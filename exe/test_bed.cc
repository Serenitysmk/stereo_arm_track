#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/highgui.hpp>

#include <util/misc.h>

#include "src/frame_grabber.h"

using namespace colmap;

////////////////////////////////////////////////////////////////////////////////
// Define variables
////////////////////////////////////////////////////////////////////////////////

DEFINE_int32(num_cameras, 4, "Number of cameras.");
DEFINE_string(
    camera_list,
    "7L03E0EPAK00002, 7L03E0EPAK00005, 7L03E0EPAK00022, 7L03E0EPAK00026",
    "Used camera serial numbers");
DEFINE_bool(record, false, "Is recording?");
DEFINE_string(output_video_path, "../data/videos",
              "Output path of the recorded videos");

void RunTestFrameGrabber();

void RunTestVideoRecord();

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_record) {
    RunTestVideoRecord();
  } else {
    RunTestFrameGrabber();
  }

  return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

void RunTestFrameGrabber() {
  std::cout << "Testing grabbing one frame" << std::endl;

  FrameGrabber grabber(FLAGS_num_cameras,
                       CSVToVector<std::string>(FLAGS_camera_list));

  if (!grabber.Init()) {
    std::cerr << "ERROR: Failed to initialize the frame grabber!" << std::endl;
    return;
  }

  std::unordered_map<std::string, cv::Mat> frames = grabber.Next();

  bool grab_success = true;

  for (const auto& frame : frames) {
    if (frame.second.empty()) grab_success = false;
  }

  if (grab_success) {
    std::cout << "Grab success!" << std::endl;
    for (const auto& frame : frames) {
      std::stringstream stream;
      stream << "../data/Frame_" << frame.first << ".png";
      cv::imwrite(stream.str(), frame.second);
    }
    cv::waitKey(0);

  } else {
    std::cerr << "ERROR: Grab frames failed!" << std::endl;
  }

  grabber.Close();
}

void RunTestVideoRecord() {
  std::cout << "Testing recording videos" << std::endl;

  FrameGrabber grabber(FLAGS_num_cameras,
                       CSVToVector<std::string>(FLAGS_camera_list));

  if (!grabber.Init()) {
    std::cerr << "ERROR: Failed to initialize the frame grabber!" << std::endl;
    return;
  }

  grabber.Record(FLAGS_output_video_path, std::chrono::minutes(1), 25.0, false);

  grabber.Close();
}