#include <iostream>

#include <base/camera.h>

#include <IMVApi.h>

int main(int argc, char** argv) {
  std::cout << "Hello World" << std::endl;

  colmap::Camera camera;
  
  int ret = IMV_OK;
  unsigned int cameraIndex = 0;
  IMV_HANDLE devHandle = NULL;

  IMV_DeviceList deviceInfoList;
  ret = IMV_EnumDevices(&deviceInfoList, interfaceTypeAll);
  
  std::cout << "Number of devices: " << deviceInfoList.nDevNum << std::endl;
  return EXIT_SUCCESS;
}