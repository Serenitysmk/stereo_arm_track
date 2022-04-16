#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <util/misc.h>

#include "src/controller.h"

using namespace colmap;

////////////////////////////////////////////////////////////////////////////////
// Define variables
////////////////////////////////////////////////////////////////////////////////

DEFINE_int32(num_cameras, 4, "Number of cameras.");
DEFINE_string(
    camera_list,
    "7L03E0EPAK00002, 7L03E0EPAK00005, 7L03E0EPAK00022, 7L03E0EPAK00026",
    "Used camera serial numbers");
// DEFINE_string(camera_list, "7L03E0EPAK00022, 7L03E0EPAK00026",
//               "Used camera serial numbers");
DEFINE_bool(record, false, "Is recording?");
DEFINE_string(video_path, "../data/videos",
              "Output path of the recorded videos");

DEFINE_bool(marker_detection, false, "Test marker detection?");

void RunTestFrameGrabber();

void RunTestVideoRecord();

void RunTestMarkerDetection();

void DetectMarker(cv::Mat& img);

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_record) {
    RunTestVideoRecord();
  } else if (FLAGS_marker_detection) {
    RunTestMarkerDetection();
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
    std::cout << "frames: " << i << std::endl;
    grabbed_frames.push_back(frames);
  }

  for (int i = 0; i < 10; i++) {
    for (const auto& frame : grabbed_frames[i]) {
      cv::Mat img;
      cv::resize(frame.second, img, cv::Size(), 0.5, 0.5);
      cv::imshow(frame.first, img);
    }

    cv::waitKey(0);
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

  grabber.Record(FLAGS_video_path, std::chrono::seconds(10), 25.0);

  grabber.Close();
}

void RunTestMarkerDetection() {
  ControllerOptions options;
  options.num_cameras = FLAGS_num_cameras;
  options.camera_list = FLAGS_camera_list;
  options.input_from_videos = false;
  options.video_path = FLAGS_video_path;

  Controller controller(&options);

  controller.Run();

  controller.Shutdown();
  return;
}

void DetectMarker(cv::Mat& img) {
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::Ptr<cv::aruco::DetectorParameters> params =
      cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);

  cv::aruco::detectMarkers(img, dict, marker_corners, marker_ids, params);

  cv::aruco::drawDetectedMarkers(img, marker_corners, marker_ids);

  cv::Mat gray;
  if (img.channels() == 3) {
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = img;
  }

  for (auto& marker : marker_corners) {
    cv::cornerSubPix(
        gray, marker, cv::Size(7, 7), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 200,
                         1e-10));
  }

  std::cout << "Detected marker ids: " << std::endl;
  std::cout << "[";
  for (const int& id : marker_ids) {
    std::cout << id << " ";
  }
  std::cout << "]" << std::endl;

  std::cout << "Detected marker corners: " << std::endl;

  std::cout << "[";
  for (const auto& marker : marker_corners) {
    for (const cv::Point2f& point : marker) {
      std::cout << point << " ";
    }
  }
  std::cout << "]" << std::endl;
}