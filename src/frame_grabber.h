#ifndef SRC_FRAME_GRABBER_H_
#define SRC_FRAME_GRABBER_H_

#include <vector>

#include <IMVApi.h>

struct FrameGrabberOptions {
  // Number of cameras.
  int num_cameras = 4;

  // Frame rate.
  double frame_rate = 25.0;

  bool Check() const;
};

class FrameGrabber {
 public:
  FrameGrabber(const FrameGrabberOptions* options);

  ~FrameGrabber();

  bool Init();

  bool TestGrabFrameOneCamera();

 private:
  int SetSoftTriggerConf(IMV_HANDLE dev_handle);

  void ExecuteSoftTrigger(IMV_HANDLE dev_handle, const size_t max_frames);

  const FrameGrabberOptions* options_;

  std::vector<IMV_HANDLE> device_handles_;
};

#endif  // STEREO_ARM_TRACK_FRAME_GRABBER_H_