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
  std::cout << "Number of frames: " << frames.size() << std::endl;
  for (const auto& frame : frames) {
    if (frame.second.empty()) grab_success = false;
  }

  if (grab_success) {
    std::cout << "Grab success!" << std::endl;
    for (const auto& frame: frames) {
      //cv::imshow("Frame_" + std::to_string(frame.first), frame.second);
      std::cout << "Frame: " << frame.first << " , width: " << frame.second.cols << " , height: " << frame.second.rows << " middle pixel: " << frame.second.at<cv::Vec3b>(500, 500) << std::endl;
    }
    // cv::waitKey(0);
    // cv::imshow("Frame2", frames.at(2));
    // cv::imshow("Frame3", frames.at(3));
    cv::imwrite("./frame2.png", frames.at(2));
    cv::imwrite("./frame3.png", frames.at(3));
    //cv::waitKey(0);
  } else {
    std::cerr << "ERROR: Grab frames failed!" << std::endl;
  }

  frame_grabber.Close();
}