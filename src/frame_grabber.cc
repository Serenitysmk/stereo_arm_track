#include "frame_grabber.h"

#include <condition_variable>
#include <thread>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include <util/misc.h>

using namespace colmap;

namespace {

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
  PrintHeading1("Initialize cameras");
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

  // Initialize the cameras and start grabbing,
  // but the camera won't grab a frame until it
  // revices a trigger signal.
  if (!InitCameras()) {
    return false;
  }

  std::cout << "Finished initialization, cameras are ready." << std::endl;
  return true;
}

std::unordered_map<std::string, cv::Mat> FrameGrabber::Next() {
  std::unordered_map<std::string, cv::Mat> grabbed_frames;

  NextImpl();

  for (const std::string& serial_number : camera_list_) {
    IMV_HANDLE dev_handle = device_handles_.at(serial_number);
    grabbed_frames.emplace(
        serial_number,
        FrameToCvMat(dev_handle, g_grabbed_frames.at(dev_handle)));
  }

  g_grabbed_frames.clear();

  return grabbed_frames;
}

void FrameGrabber::Record(
    const std::string& output_dir,
    const std::chrono::duration<double, std::ratio<60>>& time,
    const double frame_rate) {
  // Time interval in millisecond between the last frame and the current frame.
  auto frame_interval =
      std::chrono::duration<double, std::milli>(1000.0 / frame_rate);

  CreateDirIfNotExists(output_dir);

  std::unordered_map<std::string, std::string> output_paths;

  std::unordered_map<IMV_HANDLE, IMV_RecordParam> record_params;

  for (const std::string& serial_number : camera_list_) {
    IMV_HANDLE dev_handle = device_handles_.at(serial_number);
    output_paths.insert(std::make_pair(
        serial_number, JoinPaths(output_dir, serial_number + ".avi")));
    int64_t width = 0;
    int64_t height = 0;

    IMV_GetIntFeatureValue(dev_handle, "Width", &width);

    IMV_GetIntFeatureValue(dev_handle, "Height", &height);

    IMV_RecordParam record_param;
    record_param.nWidth = (unsigned int)width;
    record_param.nHeight = (unsigned int)height;
    record_param.fFameRate = (float)frame_rate;
    record_param.nQuality = 30;
    record_param.recordFormat = typeVideoFormatAVI;
    record_param.pRecordFilePath = output_paths.at(serial_number).c_str();

    record_params.emplace(dev_handle, record_param);
  }

  for (const std::string& serial_number : camera_list_) {
    IMV_HANDLE dev_handle = device_handles_.at(serial_number);
    int ret = IMV_OK;
    ret = IMV_OpenRecord(dev_handle, &record_params.at(dev_handle));
    if (ret != IMV_OK) {
      std::cerr << "ERROR: Open record failed! Error code " << ret << std::endl;
      return;
    }
  }

  // Recording loop;
  PrintHeading1("Start video recording");

  double recorded_time = 0.0;
  double report_time = 0.0;

  auto start = std::chrono::high_resolution_clock::now();
  auto end = std::chrono::high_resolution_clock::now() + time;

  auto last_report_time = start;
  while (std::chrono::high_resolution_clock::now() <= end) {
    auto grab_start = std::chrono::high_resolution_clock::now();

    NextImpl();

    // // Push to the frames queue.
    // frames_queue_.push(frames);
    for (const std::string& serial_number : camera_list_) {
      IMV_HANDLE dev_handle = device_handles_.at(serial_number);
      IMV_RecordFrameInfoParam record_frame_param;
      IMV_Frame* frame = g_grabbed_frames.at(dev_handle);
      record_frame_param.pData = frame->pData;
      record_frame_param.nDataLen = frame->frameInfo.size;
      record_frame_param.nPaddingX = frame->frameInfo.paddingX;
      record_frame_param.nPaddingY = frame->frameInfo.paddingY;
      record_frame_param.ePixelFormat = frame->frameInfo.pixelFormat;

      // Record one frame.
      int ret = IMV_OK;
      ret = IMV_InputOneFrame(dev_handle, &record_frame_param);
      if (ret != IMV_OK) {
        std::cerr << "WARNING: Record frame failed!" << std::endl;
      }
    }
    g_grabbed_frames.clear();

    auto current_time = std::chrono::high_resolution_clock::now();

    report_time = std::chrono::duration_cast<std::chrono::seconds>(
                      current_time - last_report_time)
                      .count();
    if (report_time >= 10.0) {
      recorded_time =
          std::chrono::duration_cast<std::chrono::seconds>(current_time - start)
              .count();
      std::cout << "Recording for " << recorded_time << " seconds ..."
                << std::endl;
      last_report_time = current_time;
    }

    auto grab_end = std::chrono::high_resolution_clock::now();

    auto grab_elapsed =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            grab_end - grab_start);
    std::cout << "grab cost: " << grab_elapsed.count() << " ms" << std::endl;
    // if (frame_interval > grab_elapsed) {
    //   std::this_thread::sleep_for(frame_interval - grab_elapsed);
    // }
  }
  end = std::chrono::high_resolution_clock::now();
  recorded_time =
      std::chrono::duration_cast<std::chrono::duration<double, std::ratio<60>>>(
          end - start)
          .count();
  std::cout << "Video recording stopped, time: " << recorded_time
            << " minutes, number of frames: " << frames_queue_.size()
            << std::endl;

  for (const std::string& serial_number : camera_list_) {
    IMV_CloseRecord(device_handles_.at(serial_number));
  }

  // Write out videos.
  // VideoWriter(output_dir, frame_rate);

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
      }
      // Close camera.
      ret = IMV_Close(dev_handle);
      if (ret != IMV_OK) {
        std::cerr << "ERROR: Close camera failed! Error code " << ret
                  << std::endl;
        return false;
      }
      std::cout << "Camera " << serial_number << " closed" << std::endl;
    }
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

cv::Mat FrameGrabber::FrameToCvMat(IMV_HANDLE dev_handle, IMV_Frame* frame) {
  // PixelFormatConversion(dev_handle, frame);

  cv::Size size(frame->frameInfo.width, frame->frameInfo.height);
  cv::Mat frame_cv = cv::Mat::zeros(size, CV_8UC3);
  PixelFormatConversion(dev_handle, frame, &frame_cv);
  return frame_cv;
}

void FrameGrabber::PixelFormatConversion(IMV_HANDLE dev_handle,
                                         IMV_Frame* frame, cv::Mat* frame_cv) {
  int ret = IMV_OK;
  IMV_PixelConvertParam pixel_convert_params;

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

    pixel_convert_params.pDstBuf = frame_cv->data;

    pixel_convert_params.nDstBufSize =
        frame->frameInfo.width * frame->frameInfo.height * 3;

    ret = IMV_PixelConvert(dev_handle, &pixel_convert_params);
    if (ret != IMV_OK) {
      std::cerr << "ERROR: Image convert to BGR failed! Error code " << ret
                << std::endl;
    }
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

    // Load camera config files.
    std::stringstream stream;
    stream << "../config/" << serial_number << ".mvcfg";
    const std::string config_path = stream.str();
    IMV_ErrorList error_list;
    ret = IMV_LoadDeviceCfg(dev_handle, config_path.c_str(), &error_list);

    if (ret != IMV_OK) {
      std::cerr << "ERROR: Load camera configuration failed! Error code " << ret
                << std::endl;
      return false;
    }

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

void FrameGrabber::NextImpl() {
  int ret = IMV_OK;

  // Execute all triggers.
  ExecuteTriggerSoft();

  // Wait for the frame grabbing process.
  {
    std::unique_lock<std::mutex> lock(g_grab_frame_mutex);
    g_grab_finish_condition.wait(lock, [this] {
      return g_grabbed_frames.size() == camera_list_.size();
    });
  }
}

// void FrameGrabber::VideoWriter(const std::string& output_dir,
//                                const double frame_rate) {
//   CreateDirIfNotExists(output_dir);

//   std::unordered_map<std::string, std::string> output_paths;
//   std::unordered_map<std::string, cv::VideoWriter> video_writers;

//   for (const std::string& serial_number : camera_list_) {
//     output_paths.insert(std::make_pair(
//         serial_number, JoinPaths(output_dir, serial_number + ".ts")));
//     int64_t width = 0;
//     int64_t height = 0;

//     IMV_GetIntFeatureValue(device_handles_.at(serial_number), "Width",
//     &width);

//     IMV_GetIntFeatureValue(device_handles_.at(serial_number), "Height",
//                            &height);

//     cv::VideoWriter video_writer(output_paths.at(serial_number),
//                                  cv::VideoWriter::fourcc('M', 'P', 'E', 'G'),
//                                  frame_rate, cv::Size(width, height));

//     video_writers.insert(std::make_pair(serial_number, video_writer));
//   }

//   while (!frames_queue_.empty()) {
//     for (const std::string& serial_number : camera_list_) {
//       video_writers.at(serial_number)
//           << frames_queue_.front().at(serial_number);
//     }
//     frames_queue_.pop();
//   }
//   for (const std::string& serial_number : camera_list_) {
//     video_writers.at(serial_number).release();
//   }
// }
