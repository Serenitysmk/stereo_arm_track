#include "src/viewer.h"

#include <base/pose.h>
#include <base/projection.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <open3d/Open3D.h>

using namespace colmap;

Viewer::Viewer(const std::vector<std::string>& camera_list,
               const std::unordered_map<std::string, Eigen::Vector4d>& qvecs,
               const std::unordered_map<std::string, Eigen::Vector3d>& tvecs,
               const size_t max_track_length, const double world_display_scale,
               const double image_display_scale)
    : camera_list_(camera_list),
      qvecs_(qvecs),
      tvecs_(tvecs),
      max_track_length_(max_track_length),
      world_display_scale_(world_display_scale),
      image_display_scale_(image_display_scale) {
  viewer_thread_ = std::thread(&Viewer::ThreadLoop2, this);
}

void Viewer::InsertCurrentFrame(
    const std::unordered_map<std::string, cv::Mat>& current_frames,
    const std::unordered_map<std::string, std::vector<cv::Point2f>>&
        current_observations) {
  std::unique_lock<std::mutex> lock(viewer_data_mutex_);
  current_frames_ = current_frames;
  current_observations_ = current_observations;
  new_frame_arrived_ = true;
}

void Viewer::InsertNewMarker(const Marker& marker) {
  std::unique_lock<std::mutex> lock(viewer_data_mutex_);
  track_.push_back(marker);
  if (track_.size() > max_track_length_) {
    track_.erase(track_.begin());
  }
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

  const float red[3] = {1, 0, 0};
  const float green[3] = {0, 1, 0};
  const float blue[3] = {0, 0, 1};

  while (!pangolin::ShouldQuit() && viewer_running_) {
    auto start = std::chrono::high_resolution_clock::now();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    vis_display.Activate(viewer_camera);

    for (const std::string& sn : camera_list_) {
      RenderFrame(qvecs_.at(sn), tvecs_.at(sn), green);
    }

    RenderCoordinateAxis();

    std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    if (new_frame_arrived_) {
      cv::Mat img = DrawFrameImage();
      if (image_display_scale_ != 1.0) {
        cv::Mat img_resize;
        cv::resize(img, img_resize, cv::Size(), image_display_scale_,
                   image_display_scale_);
        cv::imshow("Frames", img_resize);
        cv::waitKey(1);
      } else {
        cv::imshow("Frames", img);
        cv::waitKey(1);
      }
    }

    RenderMarkers();

    pangolin::FinishFrame();
    new_frame_arrived_ = false;
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed_time =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << elapsed_time.count() << " ms" << std::endl;
  }
}

void Viewer::ThreadLoop2() {
  open3d::visualization::Visualizer visualizer;
  visualizer.CreateVisualizerWindow("Tracker", 1024, 768);

  visualizer.GetRenderOption().background_color_ = Eigen::Vector3d::Zero();
  auto coordinate_axis = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1);
  visualizer.AddGeometry(coordinate_axis);

  while (viewer_running_) {
    auto start = std::chrono::high_resolution_clock::now();
    visualizer.UpdateGeometry();
    visualizer.PollEvents();
    visualizer.UpdateRender();

    std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    if (new_frame_arrived_) {
      cv::Mat img = DrawFrameImage();
      if (image_display_scale_ != 1.0) {
        cv::Mat img_resize;
        cv::resize(img, img_resize, cv::Size(), image_display_scale_,
                   image_display_scale_);
        cv::imshow("Frames", img_resize);
        cv::waitKey(1);
      } else {
        cv::imshow("Frames", img);
        cv::waitKey(1);
      }
    }
    new_frame_arrived_ = false;
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed_time =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << elapsed_time.count() << " ms" << std::endl;
  }
  return;
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
    if (current_observations_.find(camera_list_[camera_idx]) !=
        current_observations_.end()) {
      std::vector<std::vector<cv::Point2f>> marker_corners = {
          current_observations_.at(camera_list_[camera_idx])};

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

void Viewer::RenderFrame(const Eigen::Vector4d& qvec,
                         const Eigen::Vector3d& tvec, const float* color) {
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
  pose_matrix.block<3, 1>(0, 3) = tvec_c2w * world_display_scale_;

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

void Viewer::RenderMarker(const Marker& marker, const float* color) {
  if (marker.positions.size() != 4) {
    return;
  }
  std::vector<Eigen::Vector3d> positions_scaled = marker.positions;
  for (auto& point : positions_scaled) {
    point *= world_display_scale_;
  }
  const int line_width = 2.0;
  glColor3f(color[0], color[1], color[2]);

  glLineWidth(line_width);
  glBegin(GL_LINES);

  for (int i = 0; i < positions_scaled.size() - 1; i++) {
    glVertex3f(positions_scaled[i](0), positions_scaled[i](1),
               positions_scaled[i](2));
    glVertex3f(positions_scaled[i + 1](0), positions_scaled[i + 1](1),
               positions_scaled[i + 1](2));
  }
  glVertex3f(positions_scaled[0](0), positions_scaled[0](1),
             positions_scaled[0](2));
  glVertex3f(positions_scaled[3](0), positions_scaled[3](1),
             positions_scaled[3](2));
  glEnd();
}

void Viewer::RenderMarkers() {
  if (track_.size() == 0) return;

  // Render history markers in the track.
  const float blue[3] = {0, 0, 1};
  const float red[3] = {1, 0, 0};
  const float green[3] = {0, 1, 0};

  RenderMarker(track_[track_.size() - 1], green);

  if (track_.size() > 1) {
    for (size_t i = 0; i < track_.size() - 1; i++) {
      const Marker& marker = track_[i];
      RenderMarker(marker, blue);
    }

    // Render connection lines.
    const int line_width = 2.0;
    glColor3f(red[0], red[1], red[2]);
    glLineWidth(line_width);
    glBegin(GL_LINES);
    for (size_t i = 0; i < track_.size() - 1; i++) {
      Eigen::Vector3d center_i = track_[i].center;
      Eigen::Vector3d center_i_1 = track_[i + 1].center;
      center_i *= world_display_scale_;
      center_i_1 *= world_display_scale_;
      glVertex3f(center_i(0), center_i(1), center_i(2));
      glVertex3f(center_i_1(0), center_i_1(1), center_i_1(2));
    }
    glEnd();
  }
}

void Viewer::RenderCoordinateAxis() {
  const float red[3] = {1.0, 0.0, 0.0};
  const float green[3] = {0.0, 1.0, 0.0};
  const float blue[3] = {0.0, 0.0, 1.0};

  const int line_width = 6.0;
  const float sz = 3.0;

  // Render X-axis.
  glColor3f(red[0], red[1], red[2]);
  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(sz, 0, 0);
  glEnd();

  // Render Y-axis.
  glColor3f(green[0], green[1], green[2]);
  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(0, sz, 0);
  glEnd();

  // Render Z-axis.
  glColor3f(blue[0], blue[1], blue[2]);
  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, sz);
  glEnd();
}
