#ifndef SRC_FRAME_GRABBER_H_
#define SRC_FRAME_GRABBER_H_

#include <mutex>
#include <vector>
#include <queue>

#include <opencv2/core.hpp>

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

  // Init frame grabber.
  bool Init();

  // Grab next frames (Multiple cameras).
  std::vector<cv::Mat> Next();

  // Record videos for a period of time.
  void Record(const std::string& output_dir,
              const std::chrono::minutes& time,
              const double frame_rate, const bool display);

  // Close frame grabber.
  bool Close();

  bool TestGrabFrameOneCamera();

 private:
  // Display device info in the console.
  void PrintDeviceInfo(const IMV_DeviceList& devce_info_list);

  int SetSoftTriggerConf(IMV_HANDLE dev_handle);

  int MallocConvertBuffer(IMV_HANDLE dev_handle);

  void ExecuteTriggerSoft();

  // Initialize the cameras and start grabbing,
  // but the camera won't grab a frame until it
  // revices a trigger signal.
  bool InitCameras();

  const FrameGrabberOptions* options_;

  // Device handles.
  std::vector<IMV_HANDLE> device_handles_;

  // Grabbed frames queue.
  std::queue<std::vector<IMV_Frame*>> frames_queue_;
  std::mutex frames_queue_mutex_;
};

#endif  // STEREO_ARM_TRACK_FRAME_GRABBER_H_