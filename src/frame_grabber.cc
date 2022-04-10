#include "frame_grabber.h"

#include <condition_variable>
#include <thread>
#include <unordered_map>

#include <opencv2/highgui.hpp>

#include <util/misc.h>

using namespace colmap;

namespace {

bool g_is_exit_thread = false;

// Grabbed frames.
std::unordered_map<IMV_HANDLE, IMV_Frame*> g_grabbed_frames;

// Mutex to protect the grabbed frames.
std::mutex g_grab_frame_mutex;

// Condition variable for the main thread to wait for the frames to be grabbed.
std::condition_variable g_grab_finish_condition;

// Data frame callback function.
void OnFrameGrabbed(IMV_Frame* p_frame, void* p_user) {
  if (p_frame == nullptr) {
    std::cout << "WARNING: Frame pointer is NULL" << std::endl;
    return;
  }

  IMV_HANDLE dev_handle = (IMV_HANDLE)p_user;

  {
    std::unique_lock<std::mutex> lock(g_grab_frame_mutex);
    g_grabbed_frames.insert(std::make_pair(dev_handle, p_frame));
    g_grab_finish_condition.notify_one();
  }

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
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

}  // namespace

FrameGrabber::FrameGrabber(const size_t num_cameras,
                           const std::vector<std::string>& camera_list)
    : num_cameras_(num_cameras), camera_list_(camera_list) {
  CHECK_GT(camera_list_.size(), 0);
}

FrameGrabber::~FrameGrabber() {
  for (const auto& dev_handle : device_handles_) {
    if (dev_handle.second != nullptr) IMV_DestroyHandle(dev_handle.second);
  }
  device_handles_.clear();
}

const std::vector<std::string>& FrameGrabber::CameraList() const {
  return camera_list_;
}

bool FrameGrabber::Init() {
  // Find camera devices and return if no devices are found
  // or the number of devices is different from the setting.
  int ret = IMV_OK;
  IMV_DeviceList device_info_list;
  ret = IMV_EnumDevices(&device_info_list, interfaceTypeUsb3);
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Failed to find camera devices! Error code " << ret
              << std::endl;
    return false;
  }

  // Found devices.
  if (device_info_list.nDevNum < 1) {
    std::cerr << "ERROR: No camera found." << std::endl;
    return false;
  } else if (device_info_list.nDevNum != num_cameras_) {
    std::cerr << "ERROR: Found " << device_info_list.nDevNum << " cameras, but "
              << num_cameras_ << " is expected." << std::endl;
    return false;
  }

  for (size_t i = 0; i < device_info_list.nDevNum; i++) {
    IMV_HANDLE dev_handle = nullptr;
    ret = IMV_CreateHandle(&dev_handle, modeByIndex, (void*)&i);
    if (ret != IMV_OK) {
      std::cerr << "ERROR: Create device handle failed! Error code " << ret
                << std::endl;
      return false;
    }
    device_handles_.insert(
        std::make_pair(device_info_list.pDevInfo[i].serialNumber, dev_handle));
  }

  // Print device info list.
  PrintDeviceInfo(device_info_list);

  std::cout << "Prepare cameras to grab frames." << std::endl;
  std::cout << "Use camera: ";
  for (const std::string& serial_number : camera_list_) {
    std::cout << serial_number << " ";
  }
  std::cout << std::endl;

  // Initialize the cameras and start grabbing,
  // but the camera won't grab a frame until it
  // revices a trigger signal.
  if (!InitCameras()) {
    return false;
  }

  // Prepare convert buffer.
  ret = MallocConvertBuffer();
  if (ret != IMV_OK) {
    return false;
  }

  std::cout << "Finished initialization, cameras are ready." << std::endl;
  return true;
}

std::unordered_map<std::string, cv::Mat> FrameGrabber::Next() {
  int ret = IMV_OK;
  std::unordered_map<std::string, cv::Mat> grabbed_frames;

  // Execute all triggers.
  ExecuteTriggerSoft();

  {
    std::unique_lock<std::mutex> lock(g_grab_frame_mutex);
    g_grab_finish_condition.wait(lock, [this] {
      return g_grabbed_frames.size() == camera_list_.size();
    });
  }
  std::cout <<"Execute finished" << std::endl;

  for (const std::string& serial_number : camera_list_) {
    IMV_HANDLE dev_handle = device_handles_.at(serial_number);
    std::cout << "DEVICE_HANDLE: " << dev_handle << std::endl;
    {
      std::unique_lock<std::mutex> lock(g_grab_frame_mutex);
      PixelFormatConversion(dev_handle, g_grabbed_frames.at(dev_handle));
      cv::Size size(g_grabbed_frames.at(dev_handle)->frameInfo.width,
                    g_grabbed_frames.at(dev_handle)->frameInfo.height);
      grabbed_frames.insert(std::make_pair(
          serial_number,
          cv::Mat(size, CV_8UC3,
                  (uchar*)g_grabbed_frames.at(dev_handle)->pData)));
    }
  }
  {
    std::unique_lock<std::mutex> lock(g_grab_frame_mutex);
    g_grabbed_frames.clear();
  }

  return grabbed_frames;
}

void FrameGrabber::Record(const std::string& output_dir,
                          const std::chrono::minutes& time,
                          const double frame_rate, const bool display) {
  const size_t num_cameras = device_handles_.size();
  std::vector<std::string> output_paths;
  output_paths.reserve(num_cameras);

  for (size_t camera_idx = 0; camera_idx < num_cameras; camera_idx) {
    std::stringstream stream;
    stream << output_dir << "/"
           << "video_" << std::to_string(camera_idx) << ".ts";
    output_paths.emplace_back(stream.str());
  }

  // Recording loop;

  // double recorded_time = 0.0;
  // auto start = std::chrono::high_resolution_clock::now();
  // auto end = std::chrono::high_resolution_clock::now() + time;

  // while (std::chrono::high_resolution_clock::now() < end) {
  //   ExecuteTriggerSoft();

  //   std::vector<IMV_Frame*> frames;
  //   frames.reserve(device_handles_.size());
  //   // Push to the frames queue.
  //   for (IMV_HANDLE dev_handle : device_handles_) {
  //     {
  //       std::unique_lock<std::mutex> lock(g_grab_frame_mutex);
  //       frames.emplace_back(g_grabbed_frames[dev_handle]);
  //     }
  //   }
  //   {
  //     std::unique_lock<std::mutex> lock(frames_queue_mutex_);
  //     frames_queue_.push(frames);
  //   }

  //   std::cout << "Frames grabbed and pushed to the queue" << std::endl;
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  // }
  // end = std::chrono::high_resolution_clock::now();
  // recorded_time =
  //     std::chrono::duration_cast<std::chrono::minutes>(end - start).count();
  // std::cout << "Video recording stopped, time: " << recorded_time << "
  // minutes"
  //           << std::endl;
  return;
}

bool FrameGrabber::Close() {
  int ret = IMV_OK;
  for (const std::string& serial_number : camera_list_) {
    IMV_HANDLE dev_handle = device_handles_.at(serial_number);
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

  IMV_DeviceList device_info_list;
  ret = IMV_EnumDevices(&device_info_list, interfaceTypeUsb3);
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Failed to find camera devices! Error code " << ret
              << std::endl;
    return false;
  }

  // Found devices.
  if (device_info_list.nDevNum < 1) {
    std::cerr << "ERROR: No camera found." << std::endl;
    return false;
  } else if (device_info_list.nDevNum != num_cameras_) {
    std::cerr << "ERROR: Found " << device_info_list.nDevNum << " cameras, but "
              << num_cameras_ << " is expected." << std::endl;
    return false;
  }

  for (size_t i = 0; i < device_info_list.nDevNum; i++) {
    IMV_HANDLE dev_handle;
    ret = IMV_CreateHandle(&dev_handle, modeByIndex, (void*)&i);
    if (ret != IMV_OK) {
      std::cerr << "ERROR: Create device handle failed! Error code " << ret
                << std::endl;
      return false;
    }
    device_handles_.insert(
        std::make_pair(device_info_list.pDevInfo[i].serialNumber, dev_handle));
  }

  // Print device info list.
  PrintDeviceInfo(device_info_list);

  IMV_HANDLE dev_handle = device_handles_.at("7L03E0EPAK00002");

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
  ret = IMV_AttachGrabbing(dev_handle, OnFrameGrabbed, (void*)dev_handle);
  if (ret != IMV_OK) {
    std::cerr << "ERROR: Attach grabbing failed! Error code " << ret
              << std::endl;
    return false;
  }

  ret = MallocConvertBuffer();
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

void FrameGrabber::PrintDeviceInfo(const IMV_DeviceList& device_info_list) {
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

int FrameGrabber::MallocConvertBuffer() {
  int ret = IMV_OK;
  uint64_t pixel_format_val = 0;
  int64_t width = 0;
  int64_t height = 0;

  IMV_HANDLE dev_handle = device_handles_.at(*camera_list_.begin());
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

  convert_buffer = new unsigned char[(int)width * (int)height * 3];

  if (convert_buffer == nullptr) {
    std::cerr << "ERROR: Allocation memory to convert buffer failed!"
              << std::endl;
    return IMV_NO_MEMORY;
  }

  return IMV_OK;
}

void FrameGrabber::PixelFormatConversion(IMV_HANDLE dev_handle,
                                         IMV_Frame* frame) {
  int ret = IMV_OK;
  IMV_PixelConvertParam pixel_convert_params;
  std::cout << "Start pixel format conversion" << std::endl;
  // mono8 and BGR8 raw data does not to be converted.
  if ((frame->frameInfo.pixelFormat != gvspPixelMono8) &&
      (frame->frameInfo.pixelFormat != gvspPixelBGR8)) {
    pixel_convert_params.nWidth = frame->frameInfo.width;
    pixel_convert_params.nHeight = frame->frameInfo.height;
    pixel_convert_params.ePixelFormat = frame->frameInfo.pixelFormat;
    pixel_convert_params.pSrcData = frame->pData;
    pixel_convert_params.nSrcDataLen = frame->frameInfo.size;
    pixel_convert_params.nPaddingX = frame->frameInfo.paddingX;
    pixel_convert_params.nPaddingY = frame->frameInfo.paddingY;
    pixel_convert_params.eBayerDemosaic = demosaicNearestNeighbor;
    pixel_convert_params.eDstPixelFormat = gvspPixelBGR8;

    pixel_convert_params.pDstBuf = convert_buffer;

    pixel_convert_params.nDstBufSize =
        frame->frameInfo.width * frame->frameInfo.height * 3;

    ret = IMV_PixelConvert(dev_handle, &pixel_convert_params);
    if (ret != IMV_OK) {
      std::cerr << "ERROR: Image convert to BGR failed! Error code " << ret
                << std::endl;
    }
    frame->pData = convert_buffer;
    frame->frameInfo.pixelFormat = gvspPixelBGR8;
  }
}

void FrameGrabber::ExecuteTriggerSoft() {
  int ret = IMV_OK;
  for (const std::string& serial_number : camera_list_) {
    IMV_HANDLE dev_handle = device_handles_.at(serial_number);
    ret = IMV_ExecuteCommandFeature(dev_handle, "TriggerSoftware");
    if (ret != IMV_OK) {
      std::cerr << "WARNING: Execute TriggerSoftware failed! Error code " << ret
                << std::endl;
    }
  }
}

bool FrameGrabber::InitCameras() {
  // Open cameras.
  std::cout << "Init cameras" << std::endl;
  for (const std::string& serial_number : camera_list_) {
    int ret = IMV_OK;

    IMV_HANDLE dev_handle = device_handles_.at(serial_number);

    ret = IMV_Open(dev_handle);
    if (ret != IMV_OK) {
      std::cerr << "ERROR: Open camera " << serial_number
                << " failed! Error code " << ret << std::endl;
      return false;
    }
    std::cout << "Camera " << serial_number << " opened" << std::endl;

    // Set software trigger config.
    ret = SetSoftTriggerConf(dev_handle);
    if (ret != IMV_OK) {
      return false;
    }

    /// TODO: Load camera config files.

    // Attach callback function.
    ret = IMV_AttachGrabbing(dev_handle, OnFrameGrabbed, (void*)dev_handle);
    if (ret != IMV_OK) {
      std::cerr << "ERROR: Attach grabbing failed! Error code " << ret
                << std::endl;
      return false;
    }

    ret = IMV_StartGrabbing(dev_handle);
    if (ret != IMV_OK) {
      std::cerr << "ERROR: Start grabbing failed! Error code " << ret
                << std::endl;
      return false;
    }
  }
  return true;
}
