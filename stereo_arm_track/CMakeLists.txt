add_executable(test_bed test_bed.cc)
target_include_directories(test_bed PRIVATE ${COMMON_INCLUDE_DIRS})
target_link_libraries(test_bed PRIVATE ${COMMON_LINK_LIBRARIES})