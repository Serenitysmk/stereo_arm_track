# Stereo arm track library source files.
set(SRCS
frame_grabber.h frame_grabber.cc
marker.h marker.cc
marker_detector.h marker_detector.cc
triangulator.h triangulator.cc
viewer.h viewer.cc
controller.h controller.cc
)

# Stereo arm track library.
add_library(stereo_arm_track ${SRCS})
target_include_directories(stereo_arm_track PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_directories(stereo_arm_track PUBLIC ${COMMON_LINK_DIRECTORIES})
target_link_libraries(stereo_arm_track PUBLIC ${COMMON_LINK_LIBRARIES})

