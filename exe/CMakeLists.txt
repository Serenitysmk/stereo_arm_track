add_executable(main main.cc)
target_link_libraries(main PRIVATE stereo_arm_track)

add_executable(test_bed test_bed.cc)
target_link_libraries(test_bed PRIVATE stereo_arm_track)