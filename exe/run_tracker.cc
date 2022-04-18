#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "src/controller.h"

////////////////////////////////////////////////////////////////////////////////
// Define variables
////////////////////////////////////////////////////////////////////////////////

DEFINE_int32(num_cameras, 4, "Number of cameras.");
DEFINE_string(
    camera_list,
    "7L03E0EPAK00002, 7L03E0EPAK00005, 7L03E0EPAK00022, 7L03E0EPAK00026",
    "Used camera serial numbers");
DEFINE_string(config_path, "./config/", "Path to the config files.");
DEFINE_double(image_display_scale, 0.25, "Scale for image displaying");
DEFINE_double(world_display_scale, 0.1,
              "Scale when displaying the cameras and marker track.");
DEFINE_int32(max_track_length, 10000,
             "Maximum track length to be displayed in the viewer");
DEFINE_bool(input_from_videos, false, "Grab frames from videos?");
DEFINE_string(video_path, "../data/for_marker_detection",
              "Output path of the recorded videos");
DEFINE_string(output_dir, "../result/", "Output directory for marker track.");

void RunTracker() {
  ControllerOptions options;
  options.num_cameras = FLAGS_num_cameras;
  options.camera_list = FLAGS_camera_list;
  options.config_path = FLAGS_config_path;
  options.image_display_scale = FLAGS_image_display_scale;
  options.world_display_scale = FLAGS_world_display_scale;
  options.max_track_length = FLAGS_max_track_length;
  options.input_from_videos = FLAGS_input_from_videos;
  options.video_path = FLAGS_video_path;
  options.output_dir = FLAGS_output_dir;

  Controller controller(&options);

  controller.Run();

  controller.Shutdown();

  return;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  RunTracker();

  return EXIT_SUCCESS;
}