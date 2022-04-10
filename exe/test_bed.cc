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
DEFINE_string(camera_list, "2, 3", "Used camera index");

void RunTestFrameGrabber();

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  RunTestFrameGrabber();

  return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

void RunTestFrameGrabber() {
  FrameGrabber frame_grabber(FLAGS_num_cameras,
                             CSVToVector<int>(FLAGS_camera_list));

  if (!frame_grabber.Init()) {
    std::cerr << "ERROR: Failed to initialize the frame grabber!" << std::endl;
    return;
  }

  std::unordered_map<int, cv::Mat> frames = frame_grabber.Next();

  bool grab_success = true;
  for (const auto& frame : frames) {
    if (frame.second.empty()) grab_success = false;
  }

  if (grab_success) {
    std::cout << "Grab success!" << std::endl;
    // for (const auto& frame: frames) {
    //   cv::imshow("Frame_" + std::to_string(frame.first), frame.second);
    // }
    //cv::waitKey(0);
  } else {
    std::cerr << "ERROR: Grab frames failed!" << std::endl;
  }

  frame_grabber.Close();
}