cmake_minimum_required(VERSION 3.10)

project(StereoArmTrack)

################################
# Find packages
################################
find_package(OpenCV 4 REQUIRED)

find_package(COLMAP REQUIRED)

set(COMMON_INCLUDE_DIRS
    ${OpenCV_INCLUDE_DIRS}
    ${COLMAP_INCLUDE_DIRS}
)

set(COMMON_LINK_LIBRARIES
    ${COLMAP_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# Global link directory (required by COLMAP)
link_directories(${COLMAP_LINK_DIRS})

add_subdirectory(stereo_arm_track)