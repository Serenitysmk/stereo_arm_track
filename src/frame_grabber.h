#ifndef SRC_FRAME_GRABBER_H_
#define SRC_FRAME_GRABBER_H_

#include <mutex>
#include <queue>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include <IMVApi.h>

class FrameGrabber {
 public:
  FrameGrabber(const size_t num_cameras, const std::vector<std::string>& camera_list);

  ~FrameGrabber();

  const std::vector<std::string>& CameraList() const;

  // Init frame grabber.
  bool Init();

  // Grab next frames (Multiple cameras).
  std::unordered_map<std::string, cv::Mat> Next();

  // Record videos for a period of time.
  void Record(const std::string& output_dir, const std::chrono::minutes& time,
              const double frame_rate, const bool display);

  // Close frame grabber.
  bool Close();

  bool TestGrabFrameOneCamera();

 private:
  // Display device info in the console.
  void PrintDeviceInfo(const IMV_DeviceList& devce_info_list);

  int SetSoftTriggerConf(IMV_HANDLE dev_handle);

  int MallocConvertBuffer();

  void PixelFormatConversion(IMV_HANDLE dev_handle, IMV_Frame* frame);

  void ExecuteTriggerSoft();

  // Initialize the cameras and start grabbing,
  // but the camera won't grab a frame until it
  // revices a trigger signal.
  bool InitCameras();

  void NextImpl();

  size_t num_cameras_;
  const std::vector<std::string> camera_list_;

  // Device handles.
  std::unordered_map<std::string, IMV_HANDLE> device_handles_;

  std::unordered_map<IMV_HANDLE, unsigned char*> convert_buffers;

  // Grabbed frames queue.
  std::queue<std::unordered_map<std::string, IMV_Frame*>> frames_queue_;
  std::mutex frames_queue_mutex_;

};

#endif  // STEREO_ARM_TRACK_FRAME_GRABBER_H_