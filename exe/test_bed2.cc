#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

int main(int argc, char* argv[]) {
  std::cout << "Hello World" << std::endl;

  const std::string vid_path1 = "../TEST_videos/videos/7L03E0EPAK00022.ts";
  const std::string vid_path2 = "../TEST_videos/videos/7L03E0EPAK00026.ts";

  cv::VideoCapture vid1(vid_path1);
  cv::VideoCapture vid2(vid_path2);

  cv::Mat frame1, frame2;
  size_t num_frames1 = 0, num_frames2 = 0;

  while(true){
    vid1 >> frame1;
    if(frame1.empty()){
      break;
    }
    num_frames1++;
  }
  while(true){
    vid2 >> frame2;
    if(frame2.empty()){
      break;
    }
    num_frames2++;
  }
  std::cout << "Num frames 1: " << num_frames1 << " , num frames 2: " << num_frames2 << std::endl;

  vid1.release();
  vid2.release();

  vid1.open(vid_path1);
  vid2.open(vid_path2);

  while (true)
  {
    vid1 >> frame1;
    vid2 >> frame2;
    if(frame1.empty() || frame2.empty()){
      break;
    }
    cv::Mat img1, img2;

    cv::imshow("frame1", frame1);
    cv::imshow("frame2", frame2);
    cv::waitKey(0);
  }
  
  
  return EXIT_SUCCESS;
}