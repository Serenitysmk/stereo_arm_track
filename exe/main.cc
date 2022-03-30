#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_int32(frame_rate, 25, "Frame rate of the video stream");

int main(int argc, char* argv[]){
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    std::cout << "Frame rate: " << FLAGS_frame_rate << std::endl;

    return EXIT_SUCCESS;
}