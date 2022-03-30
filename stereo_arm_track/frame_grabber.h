#ifndef STEREO_ARM_TRACK_FRAME_GRABBER_H_
#define STEREO_ARM_TRACK_FRAME_GRABBER_H_

#include <IMVApi.h>

struct FrameGrabberOptions {
  // Number of cameras.
  int num_cameras = 2;
  // Frame rate.
  double frame_rate = 25.0;

  bool Check() const;
};

class FrameGrabber {
 public:
  FrameGrabber(const FrameGrabberOptions* options);

  bool Init();

 private:
  const FrameGrabberOptions* options_;
};

#endif  // STEREO_ARM_TRACK_FRAME_GRABBER_H_