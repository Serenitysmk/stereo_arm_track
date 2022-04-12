#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/opencv.hpp>

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

  std::vector<std::unordered_map<std::string, cv::Mat>> grabbed_frames;
  for (int i = 0; i < 10; i++) {
    std::unordered_map<std::string, cv::Mat> frames = grabber.Next();
    grabbed_frames.push_back(frames);
  }

  for (int i = 0; i < 10; i++) {
    for (const auto& frame : grabbed_frames[i]) {
      cv::Mat img;
      cv::resize(frame.second, img, cv::Size(), 0.25, 0.25);
      // cv::imshow(frame.first, img);
    }
    // cv::waitKey(1);
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

  grabber.Record(FLAGS_output_video_path, std::chrono::seconds(10), 25.0);

  while (!grabber.frames_queue_.empty()) {
    for (const std::string& serial_number : grabber.camera_list_) {
      IMV_HANDLE dev_handle = grabber.device_handles_.at(serial_number);
      // cv::Mat frame =
      //     grabber.FrameToCvMat(dev_handle,
      //     grabber.frames_queue_.front().at(dev_handle));
      IMV_Frame* frame = grabber.frames_queue_.front().at(dev_handle);
      std::cout << "Frame address: " << frame << std::endl;
    }
    grabber.frames_queue_.pop();
  }
  grabber.Close();
}