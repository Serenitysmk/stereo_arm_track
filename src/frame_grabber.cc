#include "frame_grabber.h"

#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <util/misc.h>

using namespace colmap;
using namespace std::chrono_literals;

namespace {
bool g_is_exit_thread = false;
unsigned char* g_convert_buf = nullptr;
}  // namespace

// Data frame callback function.
static void OnGetFrame(IMV_Frame* p_frame, void* p_user) {
  if (p_frame == nullptr) {
    std::cout << "WARNING: Frame pointer is NULL" << std::endl;
    return;
  }

  std::cout << "Get frame blockId = " << p_frame->frameInfo.blockId
            << std::endl;

  int ret = IMV_OK;
  IMV_PixelConvertParam pixel_convert_params;
  unsigned char* image_data = nullptr;
  IMV_EPixelType pixel_format = gvspPixelMono8;

  // Memory allocation buffer.

  // mono8 and BGR8 raw data does not to be converted.
  if ((p_frame->frameInfo.pixelFormat != gvspPixelMono8) &&
      (p_frame->frameInfo.pixelFormat != gvspPixelBGR8)) {
    pixel_convert_params.nWidth = p_frame->frameInfo.width;
    pixel_convert_params.nHeight = p_frame->frameInfo.height;
    pixel_convert_params.ePixelFormat = p_frame->frameInfo.pixelFormat;
    pixel_convert_params.pSrcData = p_frame->pData;
    pixel_convert_params.nSrcDataLen = p_frame->frameInfo.size;
    pixel_convert_params.nPaddingX = p_frame->frameInfo.paddingX;
    pixel_convert_params.nPaddingY = p_frame->frameInfo.paddingY;
    pixel_convert_params.eBayerDemosaic = demosaicNearestNeighbor;
    pixel_convert_params.eDstPixelFormat = gvspPixelBGR8;
    pixel_convert_params.pDstBuf = g_convert_buf;
    pixel_convert_params.nDstBufSize =
        p_frame->frameInfo.width * p_frame->frameInfo.height * 3;

    ret = IMV_PixelConvert((IMV_HANDLE)p_user, &pixel_convert_params);
    if (ret != IMV_OK) {
      std::cerr << "ERROR: Image convert to BGR failed! Error code " << ret
                << std::endl;
    }
    image_data = g_convert_buf;
    pixel_format = gvspPixelBGR8;
  } else {
    image_data = p_frame->pData;
    pixel_format = p_frame->frameInfo.pixelFormat;
  }

  cv::Size image_size(p_frame->frameInfo.width, p_frame->frameInfo.height);
  cv::Mat frame(image_size, CV_8UC3, (uchar*)image_data);

  std::cout << "Width: " << frame.cols << ", height: " << frame.rows
            << std::endl;
  cv::imshow("img", frame);
  cv::waitKey(1);
  return;
}

void ExecuteSoftTrigger(IMV_HANDLE dev_handle) {
  int ret = IMV_OK;

  while (!g_is_exit_thread) {
    ret = IMV_ExecuteCommandFeature(dev_handle, "TriggerSoftware");
    if (ret != IMV_OK) {
      std::cerr << "WARNING: Execute TriggerSoftware failed! Error code " << ret
                << std::endl;
      continue;
    }
    std::this_thread::sleep_for(50ms);
  }
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

  char vendor_name_cat[11];
  char camera_name_cat[16];

  // Print title line.
  std::cout << "\nIdx Type Vendor     Model      S/N             DeviceUserID  "
               "  IP Address    \n";
  std::cout << "---------------------------------------------------------------"
               "---------------\n";

  for (unsigned int camera_index = 0; camera_index < device_info_list.nDevNum;
       camera_index++) {
    IMV_DeviceInfo* p_dev_info = &device_info_list.pDevInfo[camera_index];
    // 设备列表的相机索引  最大表示字数：3
    // Camera index in device list, display in 3 characters
    printf("%-3d", camera_index + 1);

    // 相机的设备类型（GigE，U3V，CL，PCIe）
    // Camera type
    switch (p_dev_info->nCameraType) {
      case typeGigeCamera:
        printf(" GigE");
        break;
      case typeU3vCamera:
        printf(" U3V ");
        break;
      case typeCLCamera:
        printf(" CL  ");
        break;
      case typePCIeCamera:
        printf(" PCIe");
        break;
      default:
        printf("     ");
        break;
    }

    // 制造商信息  最大表示字数：10
    // Camera vendor name, display in 10 characters
    if (strlen(p_dev_info->vendorName) > 10) {
      memcpy(vendor_name_cat, p_dev_info->vendorName, 7);
      vendor_name_cat[7] = '\0';
      strcat(vendor_name_cat, "...");
      printf(" %-10.10s", vendor_name_cat);
    } else {
      printf(" %-10.10s", p_dev_info->vendorName);
    }

    // 相机的型号信息 最大表示字数：10
    // Camera model name, display in 10 characters
    printf(" %-10.10s", p_dev_info->modelName);

    // 相机的序列号 最大表示字数：15
    // Camera serial number, display in 15 characters
    printf(" %-15.15s", p_dev_info->serialNumber);

    // 自定义用户ID 最大表示字数：15
    // Camera user id, display in 15 characters
    if (strlen(p_dev_info->cameraName) > 15) {
      memcpy(camera_name_cat, p_dev_info->cameraName, 12);
      camera_name_cat[12] = '\0';
      strcat(camera_name_cat, "...");
      printf(" %-15.15s", camera_name_cat);
    } else {
      printf(" %-15.15s", p_dev_info->cameraName);
    }

    // GigE相机时获取IP地址
    // IP address of GigE camera
    if (p_dev_info->nCameraType == typeGigeCamera) {
      printf(" %s", p_dev_info->DeviceSpecificInfo.gigeDeviceInfo.ipAddress);
    }

    printf("\n");
  }

  std::cout << "Prepare cameras to grab frames." << std::endl;

  if (!InitCameras()) {
    return false;
  }

  std::cout << "Finished initialization, cameras are ready." << std::endl;
  return true;
}

bool FrameGrabber::Close() {
  int ret = IMV_OK;
  for (size_t camera_id = 0; camera_id < device_handles_.size(); camera_id++) {
    IMV_HANDLE dev_handle = device_handles_[camera_id];
    if (dev_handle == nullptr) {
      continue;
    }
    if (IMV_IsOpen(dev_handle)) {
      if (IMV_IsGrabbing(dev_handle)) {
        // Stop grabbing.
        ret = IMV_StopGrabbing(dev_handle);
        if (ret != IMV_OK) {
          std::cerr << "ERROR: Stop grabbing failed! Error code " << ret
                    << std::endl;
          return false;
        }

        // Close camera.
        ret = IMV_Close(dev_handle);
        if (ret != IMV_OK) {
          std::cerr << "ERROR: Close camera failed! Error code " << ret
                    << std::endl;
          return false;
        }
      }
    }
  }
  return true;
}

bool FrameGrabber::TestGrabFrameOneCamera() {
  std::cout << "Test grabbing frame for camera " << 1 << std::endl;

  // Open Camera.
  int ret = IMV_OK;

  IMV_HANDLE dev_handle = device_handles_[2];

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
  ret = IMV_AttachGrabbing(dev_handle, OnGetFrame, (void*)dev_handle);
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Attach grabbing failed! Error code " << ret
              << std::endl;
    return false;
  }

  ret = MallocConvertBuffer(dev_handle);
  if (ret != IMV_OK) {
    return false;
  }

  IMV_SetIntFeatureValue(dev_handle, "ExposureTargetBrightness", 100);
  IMV_SetDoubleFeatureValue(dev_handle, "GainRaw", 4.0);
  IMV_SetDoubleFeatureValue(dev_handle, "Gamma", 0.45);

  // Start grabbing.
  ret = IMV_StartGrabbing(dev_handle);
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Start grabbing failed! Error code " << ret
              << std::endl;
    return false;
  }

  // Grab.
  std::thread grab_worker(ExecuteSoftTrigger, dev_handle);

  g_is_exit_thread = false;

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(10s);

  g_is_exit_thread = true;

  grab_worker.join();

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
  return true;
}

bool FrameGrabber::InitCameras() {
  // Open cameras.
  for (size_t camera_id = 0; camera_id < device_handles_.size(); camera_id++) {
    int ret = IMV_OK;

    IMV_HANDLE dev_handle = device_handles_[camera_id];

    ret = IMV_Open(dev_handle);
    if (ret != IMV_OK) {
      std::cerr << "ERROR: Open camera " << camera_id << " failed! Error code "
                << ret << std::endl;
      return false;
    }

    // Set software trigger config.
    ret = SetSoftTriggerConf(dev_handle);
    if (ret != IMV_OK) {
      return false;
    }

    /// TODO: Load camera config files.

    // Attach callback function.
    ret = IMV_AttachGrabbing(dev_handle, OnGetFrame, (void*)dev_handle);
    if (ret != IMV_OK) {
      std::cerr << "ERROR: Attach grabbing failed! Error code " << ret
                << std::endl;
      return false;
    }
  }
  return true;
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

int FrameGrabber::MallocConvertBuffer(IMV_HANDLE dev_handle) {
  int ret = IMV_OK;
  uint64_t pixel_format_val = 0;
  int64_t width = 0;
  int64_t height = 0;

  ret = IMV_GetEnumFeatureValue(dev_handle, "PixelFormat", &pixel_format_val);

  if (ret != IMV_OK) {
    std::cerr << "ERROR: Get PixelFormat feature value failed! Error code "
              << ret << std::endl;
    return ret;
  }

  if (pixel_format_val == (uint64_t)gvspPixelMono8 ||
      pixel_format_val == (uint64_t)gvspPixelBGR8) {
    return IMV_OK;
  }

  ret = IMV_GetIntFeatureValue(dev_handle, "Width", &width);
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Get Width feature value failed! Error code " << ret
              << std::endl;
    return ret;
  }

  ret = IMV_GetIntFeatureValue(dev_handle, "Height", &height);
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Get Height feature value failed! Error code " << ret
              << std::endl;
    return ret;
  }

  g_convert_buf = new unsigned char[(int)width * (int)height * 3];
  if (g_convert_buf == nullptr) {
    std::cerr << "ERROR: Malloc g_convert_buf failed!" << std::endl;
    return IMV_NO_MEMORY;
  }

  return IMV_OK;
}