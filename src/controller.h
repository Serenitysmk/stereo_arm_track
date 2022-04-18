#ifndef SRC_CONTROLLER_H_
#define SRC_CONTROLLER_H_

#include <thread>

#include <base/camera.h>
#include <base/pose.h>
#include <util/logging.h>
#include <util/misc.h>

#include "src/frame_grabber.h"
#include "src/marker_detector.h"
#include "src/triangulator.h"
#include "src/viewer.h"

struct ControllerOptions {
  // Number of cameras.
  size_t num_cameras = 4;

  // A list of cameras that are actually used (a list of serial numbers).
  std::string camera_list =
      "7L03E0EPAK00002, 7L03E0EPAK00005, 7L03E0EPAK00022, 7L03E0EPAK00026";

  // Path to the config files.
  std::string config_path = "./config";
  
  // Scale when displaying the images.
  double image_display_scale = 0.25;

  // Scale when display the world objects like cameras and trajectory.
  double world_display_scale = 0.1;

  // Maximum length to display the marker track.
  size_t max_track_length = 10000;

  // Whether the input is video.
  bool input_from_videos = true;

  // Video path if the input is video.
  std::string video_path = "";

  // Output directory for the marker track.
  std::string output_dir = "";

  bool Check() const;
};

class Controller {
 public:
  explicit Controller(const ControllerOptions* options);

  void Run();

  void Shutdown();

 private:
  void StopRunningControlLoop();

  // Load camera information, including the intrinsic parameters and extrinsic
  // parameters.
  void LoadCameraInfo(const std::string& path, colmap::Camera& camera,
                      Eigen::Vector4d& qvec, Eigen::Vector3d& tvec);

  const ControllerOptions* options_;

  std::vector<std::string> camera_lists_;

  // Cameras.
  std::unordered_map<std::string, colmap::Camera> cameras_;

  // Camera poses.
  std::unordered_map<std::string, Eigen::Vector4d> qvecs_;
  std::unordered_map<std::string, Eigen::Vector3d> tvecs_;

  // Frame grabber.
  FrameGrabber* grabber_;

  // Marker detector.
  MarkerDetector* detector_;

  // Viewer.
  Viewer* viewer_;

  // Marker triangulator.
  Triangulator* triangulator_;

  // Marker Track.
  MarkerTrackWriter* track_writer_;

  bool stop_running_ = false;
};

#endif  // STEREO_ARM_TRACK_CONTROLLER_H_