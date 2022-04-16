#ifndef SRC_CONTROLLER_H_
#define SRC_CONTROLLER_H_

#include <util/logging.h>
#include <util/misc.h>

#include "src/frame_grabber.h"
#include "src/marker_detector.h"
#include "src/viewer.h"

struct ControllerOptions {
  // Number of cameras.
  size_t num_cameras = 4;

  // A list of cameras that are actually used (a list of serial numbers).
  std::string camera_list =
      "7L03E0EPAK00002, 7L03E0EPAK00005, 7L03E0EPAK00022, 7L03E0EPAK00026";

  // Scale when displaying the images.
  double display_scale = 0.25;

  // Whether the input is video.
  bool input_from_videos = true;

  // Video path if the input is video.
  std::string video_path = "";

  bool Check() const;
};

class Controller {
 public:
  explicit Controller(const ControllerOptions* options);

  void Run();

  void Shutdown();

 private:
  const ControllerOptions* options_;

  std::vector<std::string> camera_lists_;

  // Frame grabber.
  FrameGrabber* grabber_;

  // Marker detector.
  MarkerDetector* detector_;

  // Viewer.
  Viewer* viewer_;
};

#endif  // STEREO_ARM_TRACK_CONTROLLER_H_