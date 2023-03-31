#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

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
DEFINE_string(output_dir, "../data/", "Output directory to the saved frames.");

void RunSaveFrames();

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  RunSaveFrames();

  return EXIT_SUCCESS;
}

void RunSaveFrames() {
  std::cout << "Saving frames to directory" << std::endl;
  const std::vector<std::string> camera_list =
      CSVToVector<std::string>(FLAGS_camera_list);
  FrameGrabber grabber(FLAGS_num_cameras, camera_list);

  if (!grabber.Init()) {
    std::cerr << "ERROR: Failed to initialize the frame grabber!" << std::endl;
    return;
  }

  CreateDirIfNotExists(FLAGS_output_dir);

  for (const std::string& sn : grabber.CameraList()) {
    CreateDirIfNotExists(JoinPaths(FLAGS_output_dir, sn));
  }

  size_t frame_cnt = 0;
  char key;
  while (true) {
    std::unordered_map<std::string, cv::Mat> frames = grabber.Next();

    for (const std::string& sn : camera_list) {
      std::stringstream stream;
      stream << "frame_" << frame_cnt << ".png";
      const std::string output_path =
          JoinPaths(FLAGS_output_dir, sn, stream.str());

      cv::Mat frame_small;

      cv::resize(frames.at(sn), frame_small, cv::Size(), 0.25, 0.25);

      cv::imshow(sn, frame_small);
      cv::imwrite(output_path, frames.at(sn));
    }

    frame_cnt++;
    std::cout << "Saving frame: " << frame_cnt << std::endl;

    key = cv::waitKey(500);
    if ((int)key == 27) {
      break;
    }
  }

  std::cout << "Done." << std::endl;
}