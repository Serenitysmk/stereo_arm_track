#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/highgui.hpp>

#include "src/frame_grabber.h"

////////////////////////////////////////////////////////////////////////////////
// Define variables
////////////////////////////////////////////////////////////////////////////////
DEFINE_int32(num_cameras, 4, "Number of cameras.");
DEFINE_double(frame_rate, 25.0, "Frame rate of the video stream.");

void RunTestFrameGrabber(const FrameGrabberOptions& grabber_options);

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  FrameGrabberOptions grabber_options;
  grabber_options.num_cameras = FLAGS_num_cameras;
  grabber_options.frame_rate = FLAGS_frame_rate;

  //RunTestFrameGrabber(grabber_options);

  FrameGrabber grabber(&grabber_options);
  
  grabber.Init();
  grabber.TestGrabFrameOneCamera();
  return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

void RunTestFrameGrabber(const FrameGrabberOptions& grabber_options) {
  FrameGrabber frame_grabber(&grabber_options);

  if (!frame_grabber.Init()) {
    std::cerr << "ERROR: Failed to initialize the frame grabber!" << std::endl;
    return;
  }

  std::vector<cv::Mat> frames = frame_grabber.Next();

  bool grab_success = true;
  for (const cv::Mat& frame : frames) {
    if (frame.empty()) grab_success = false;
  }

  if (grab_success) {
    std::cout << "Grab success!" << std::endl;
    for (size_t i = 0; i < frames.size(); i++) {
      cv::imshow("Frame_" + std::to_string(i), frames[i]);
    }
    cv::waitKey(0);
  } else {
    std::cerr << "ERROR: Grab frames failed!" << std::endl;
  }

  frame_grabber.Close();
}