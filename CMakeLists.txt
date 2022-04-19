cmake_minimum_required(VERSION 3.10)

project(StereoArmTrack)

################################
# Find packages
################################

# OpenCV
find_package(OpenCV REQUIRED)

# COLMAP
find_package(COLMAP REQUIRED)

# Pangolin
find_package(Pangolin REQUIRED)

# Open3D
find_package(Open3D REQUIRED)

# Huaray SDK
set(HUARAY_SDK_PATH /opt/HuarayTech/MVviewer/)
set(HUARAY_INCLUDE_DIRS ${HUARAY_SDK_PATH}/include)
set(HUARAY_LINK_DIRS ${HUARAY_SDK_PATH}/lib)
set(HUARAY_LIBRARIES -lMVSDK)

if(CMAKE_BUILD_TYPE)
    message(STATUS "Build type specified as ${CMAKE_BUILD_TYPE}")
else()
    message(STATUS "Build type not specified, using Release")
    set(CMAKE_BUILD_TYPE Release)
endif()

set(COMMON_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${COLMAP_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${HUARAY_INCLUDE_DIRS}
)

set(COMMON_LINK_LIBRARIES
    ${COLMAP_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${HUARAY_LIBRARIES}
    ${Pangolin_LIBRARIES}
    Open3D::Open3D
)

# Global link directory (required by COLMAP and Huaray SDK).
set(COMMON_LINK_DIRECTORIES
    ${COLMAP_LINK_DIRS}
    ${HUARAY_LINK_DIRS}
)

add_subdirectory(src)
add_subdirectory(exe)
