#include "src/viewer.h"

#include <base/pose.h>
#include <base/projection.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

using namespace colmap;

Viewer::Viewer(const std::vector<std::string>& camera_list,
               const std::unordered_map<std::string, Eigen::Vector4d>& qvecs,
               const std::unordered_map<std::string, Eigen::Vector3d>& tvecs,
               const double display_scale)
    : camera_list_(camera_list),
      qvecs_(qvecs),
      tvecs_(tvecs),
      display_scale_(display_scale) {
  viewer_thread_ = std::thread(&Viewer::ThreadLoop, this);
}

void Viewer::AddCurrentFrame(
    const std::unordered_map<std::string, cv::Mat>& current_frames,
    const Marker& current_marker) {
  std::unique_lock<std::mutex> lock(viewer_data_mutex_);
  current_frames_ = current_frames;
  current_marker_ = current_marker;
  new_frame_arrived_ = true;
}

void Viewer::Close() {
  cv::destroyAllWindows();
  viewer_running_ = false;
  viewer_thread_.join();
}

void Viewer::ThreadLoop() {
  pangolin::CreateWindowAndBind("Tracker", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  pangolin::OpenGlRenderState viewer_camera(
      pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
      pangolin::ModelViewLookAt(10, -5, -10, 0, 0, 50, 0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler.
  pangolin::View& vis_display =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
          .SetHandler(new pangolin::Handler3D(viewer_camera));

  const float blue[3] = {0, 0, 1};
  const float green[3] = {0, 1, 0};

  while (!pangolin::ShouldQuit() && viewer_running_) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    vis_display.Activate(viewer_camera);

    for (const std::string& sn : camera_list_) {
      PlotFrame(qvecs_.at(sn), tvecs_.at(sn), green);
    }

    std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    if (new_frame_arrived_) {
      cv::Mat img = DrawFrameImage();
      if (display_scale_ != 1.0) {
        cv::Mat img_resize;
        cv::resize(img, img_resize, cv::Size(), display_scale_, display_scale_);
        cv::imshow("Frames", img_resize);
        cv::waitKey(1);
      } else {
        cv::imshow("Frames", img);
        cv::waitKey(1);
      }
    }

    pangolin::FinishFrame();
    new_frame_arrived_ = false;
  }
}

cv::Mat Viewer::DrawFrameImage() {
  const size_t num_cameras = camera_list_.size();

  const size_t width = current_frames_.at(camera_list_[0]).cols;
  const size_t height = current_frames_.at(camera_list_[0]).rows;

  cv::Mat img_show;
  if (num_cameras < 4) {
    img_show = cv::Mat::zeros(cv::Size(num_cameras * width, height), CV_8UC3);
  } else {
    img_show = cv::Mat::zeros(cv::Size(2 * width, 2 * height), CV_8UC3);
  }

  for (size_t camera_idx = 0; camera_idx < num_cameras; camera_idx++) {
    cv::Mat frame = current_frames_.at(camera_list_[camera_idx]).clone();
    cv::Mat color;
    if (frame.channels() == 3) {
      color = frame;
    } else {
      cv::cvtColor(frame, color, cv::COLOR_GRAY2BGR);
    }
    if (current_marker_.observations.find(camera_list_[camera_idx]) !=
        current_marker_.observations.end()) {
      std::vector<std::vector<cv::Point2f>> marker_corners = {
          current_marker_.observations.at(camera_list_[camera_idx])};

      cv::aruco::drawDetectedMarkers(color, marker_corners, cv::noArray());
    }
    if (num_cameras < 4) {
      color.copyTo(
          img_show(cv::Range(0, height),
                   cv::Range(camera_idx * width, (camera_idx + 1) * width)));
    } else {
      int i = (int)camera_idx / 2;
      int j = (int)camera_idx % 2;
      color.copyTo(img_show(cv::Range(i * height, (i + 1) * height),
                            cv::Range(j * width, (j + 1) * width)));
    }
  }
  return img_show;
}

void Viewer::PlotFrame(const Eigen::Vector4d& qvec, const Eigen::Vector3d& tvec,
                       const float* color) {
  Eigen::Vector4d qvec_c2w;
  Eigen::Vector3d tvec_c2w;
  InvertPose(qvec, tvec, &qvec_c2w, &tvec_c2w);

  const float sz = 1.0;
  const int line_width = 2.0;
  const float fx = 400;
  const float fy = 400;
  const float cx = 512;
  const float cy = 384;
  const float width = 1080;
  const float height = 768;

  glPushMatrix();

  Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();

  pose_matrix.block<3, 3>(0, 0) = QuaternionToRotationMatrix(qvec_c2w);
  pose_matrix.block<3, 1>(0, 3) = tvec_c2w / 10.0;

  Eigen::Matrix4f m = pose_matrix.template cast<float>();
  glMultMatrixf((GLfloat*)m.data());

  if (color == nullptr) {
    glColor3f(1, 0, 0);
  } else {
    glColor3f(color[0], color[1], color[2]);
  }

  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

  glEnd();
  glPopMatrix();
}