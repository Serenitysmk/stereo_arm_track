cmake_minimum_required(VERSION 3.10)

project(StereoArmTrack)

################################
# Find packages
################################

# OpenCV
find_package(OpenCV 4 REQUIRED)

# COLMAP
find_package(COLMAP REQUIRED)

# Huaray SDK

set(HUARAY_SDK_PATH /opt/HuarayTech/MVviewer/)
set(HUARAY_INCLUDE_DIRS ${HUARAY_SDK_PATH}/include)
set(HUARAY_LINK_DIRS ${HUARAY_SDK_PATH}/lib)
set(HUARAY_LIBRARIES -lMVSDK)

set(COMMON_INCLUDE_DIRS
    ${OpenCV_INCLUDE_DIRS}
    ${COLMAP_INCLUDE_DIRS}
    ${HUARAY_INCLUDE_DIRS}
)

set(COMMON_LINK_LIBRARIES
    ${COLMAP_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${HUARAY_LIBRARIES}
)

# Global link directory (required by COLMAP and Huaray SDK).
set(COMMON_LINK_DIRECTORIES
    ${COLMAP_LINK_DIRS}
    ${HUARAY_LINK_DIRS}
)

add_subdirectory(stereo_arm_track)