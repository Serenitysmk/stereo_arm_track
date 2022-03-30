#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "src/frame_grabber.h"

////////////////////////////////////////////////////////////////////////////////
// Define variables
////////////////////////////////////////////////////////////////////////////////
DEFINE_int32(num_cameras, 1, "Number of cameras.");
DEFINE_double(frame_rate, 25.0, "Frame rate of the video stream.");

void RunTestFrameGrabber(const FrameGrabberOptions& grabber_options);

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  FrameGrabberOptions grabber_options;
  grabber_options.num_cameras = FLAGS_num_cameras;
  grabber_options.frame_rate = FLAGS_frame_rate;

  RunTestFrameGrabber(grabber_options);
  return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

void RunTestFrameGrabber(const FrameGrabberOptions& grabber_options) {
  FrameGrabber frame_grabber(&grabber_options);

  if (!frame_grabber.Init()) {
    std::cerr << "ERROR: failed to initialize the frame grabber!" << std::endl;
    return;
  }
}