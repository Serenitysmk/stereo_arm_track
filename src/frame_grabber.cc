#include "frame_grabber.h"

#include <util/misc.h>

using namespace colmap;

// Data frame callback function.
void OnGetFrame(IMV_Frame* p_frame, void* p_user) {
  if (p_frame == nullptr) {
    std::cout << "WARNING: Frame pointer is NULL" << std::endl;
    return;
  }

  std::cout << "Get frame blockId = " << p_frame->frameInfo.blockId
            << std::endl;
}

bool FrameGrabberOptions::Check() const {
  CHECK_OPTION_GE(num_cameras, 1);
  CHECK_OPTION_GE(frame_rate, 1.0);
  return true;
}

FrameGrabber::FrameGrabber(const FrameGrabberOptions* options)
    : options_(options) {}

FrameGrabber::~FrameGrabber() {
  for (IMV_HANDLE dev_handle : device_handles_) {
    if (dev_handle != nullptr) IMV_DestroyHandle(dev_handle);
  }
  device_handles_.clear();
}

bool FrameGrabber::Init() {
  // Find camera devices and return if no devices are found
  // or the number of devices is different from the setting.
  int ret = IMV_OK;
  IMV_DeviceList device_info_list;
  ret = IMV_EnumDevices(&device_info_list, interfaceTypeUsb3);
  if (ret != IMV_OK) {
    std::cerr << "ERROR: failed to find camera devices! Error code " << ret
              << std::endl;
    return false;
  }

  // Found devices.
  if (device_info_list.nDevNum < 1) {
    std::cerr << "ERROR: No camera found." << std::endl;
    return false;
  } else if (device_info_list.nDevNum != options_->num_cameras) {
    std::cerr << "ERROR: found " << device_info_list.nDevNum << " cameras, but "
              << options_->num_cameras << " is expected." << std::endl;
    return false;
  }

  device_handles_.resize(device_info_list.nDevNum);
  for (size_t i = 0; i < device_info_list.nDevNum; i++) {
    ret = IMV_CreateHandle(&device_handles_[i], modeByIndex, (void*)&i);
    if (ret != IMV_OK) {
      std::cerr << "ERROR: Create device handle failed! Error code " << ret
                << std::endl;
      return false;
    }
  }

  std::cout << "Initialize success, " << device_handles_.size()
            << " cameras are founded" << std::endl;
  return true;
}

bool FrameGrabber::TestGrabFrameOneCamera() {
  std::cout << "Test grabbing frame for camera " << 0 << std::endl;

  // Open Camera.
  int ret = IMV_OK;

  IMV_HANDLE dev_handle = device_handles_[0];

  ret = IMV_Open(dev_handle);
  if (ret != IMV_OK) {
    std::cerr << "Open camera failed! Error code " << ret << std::endl;
    return false;
  }

  // Set software trigger config.
  ret = SetSoftTriggerConf(dev_handle);
  if (ret != IMV_OK) {
    return false;
  }

  // Register data frame callback function.
  ret = IMV_AttachGrabbing(dev_handle, OnGetFrame, NULL);
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Attach grabbing failed! Error code " << ret
              << std::endl;
    return false;
  }

  // Start grabbing.
  ret = IMV_StartGrabbing(dev_handle);
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Start grabbing failed! Error code " << ret
              << std::endl;
    return false;
  }

  // Stop grabbing.
  ret = IMV_StopGrabbing(dev_handle);
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Stop grabbing failed! Error code " << ret << std::endl;
    return false;
  }

  // Close camera.
  ret = IMV_Close(dev_handle);
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Close camera failed! Error code " << ret << std::endl;
    return false;
  }
}

int FrameGrabber::SetSoftTriggerConf(IMV_HANDLE dev_handle) {
  int ret = IMV_OK;

  // Set trigger source to Software.
  ret = IMV_SetEnumFeatureSymbol(dev_handle, "TriggerSource", "Software");
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Set TriggerSource value failed! Error code " << ret
              << std::endl;
    return ret;
  }

  // Set trigger selector to FrameStart.
  ret = IMV_SetEnumFeatureSymbol(dev_handle, "TriggerSelector", "FrameStart");
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Set TriggerSelector value failed! Error code " << ret
              << std::endl;
    return ret;
  }

  // Set trigger mode to On.
  ret = IMV_SetEnumFeatureSymbol(dev_handle, "TriggerMode", "On");
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Set TriggerMode valud failed! Error code " << ret
              << std::endl;
    return ret;
  }

  return ret;
}