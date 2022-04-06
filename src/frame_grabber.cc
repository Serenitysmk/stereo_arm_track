#include "frame_grabber.h"

#include <util/misc.h>

using namespace colmap;

bool FrameGrabberOptions::Check() const {
  CHECK_OPTION_GE(num_cameras, 1);
  CHECK_OPTION_GE(frame_rate, 1.0);
  return true;
}

FrameGrabber::FrameGrabber(const FrameGrabberOptions* options)
    : options_(options) {}

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

void FrameGrabber::TestGrabFrameOneCamera() {
  
}