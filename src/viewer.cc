#include "src/viewer.h"

#include <base/pose.h>
#include <base/projection.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <open3d/Open3D.h>

using namespace colmap;

namespace {

std::shared_ptr<open3d::geometry::TriangleMesh> CreateCoordinateAxis() {
  auto axis = open3d::geometry::TriangleMesh::CreateCoordinateFrame(4);
  return axis;
}

void RenderMarker(open3d::visualization::Visualizer& visualizer,
                  const Marker& marker, const Eigen::Vector3d& color,
                  const double world_display_scale) {
  if (marker.positions.size() != 4) {
    return;
  }

  std::vector<Eigen::Vector3d> positions_scaled = marker.positions;
  for (auto& point : positions_scaled) {
    point *= world_display_scale;
  }

  std::vector<Eigen::Vector2i> lines{
      Eigen::Vector2i(0, 1),
      Eigen::Vector2i(1, 2),
      Eigen::Vector2i(2, 3),
      Eigen::Vector2i(0, 3),
  };

  auto marker_actor = std::make_shared<open3d::geometry::LineSet>();
  marker_actor->points_ = positions_scaled;
  marker_actor->lines_ = lines;
  marker_actor->PaintUniformColor(color);
  visualizer.AddGeometry(marker_actor, false);
}

void RenderMarkers(open3d::visualization::Visualizer& visualizer,
                   const std::vector<Marker>& track,
                   const double world_display_scale) {
  if (track.size() == 0) return;
  // Render history markers in the track.
  const Eigen::Vector3d blue(0, 0, 1);
  const Eigen::Vector3d red(1, 0, 0);
  const Eigen::Vector3d green(0, 1, 0);

  RenderMarker(visualizer, track[track.size() - 1], green, world_display_scale);

  if (track.size() > 1) {
    for (size_t i = 0; i < track.size() - 1; i++) {
      const Marker& marker = track[i];
      RenderMarker(visualizer, marker, blue, world_display_scale);
    }

    // Render connection lines.
    auto conn_lines = std::make_shared<open3d::geometry::LineSet>();
    conn_lines->points_.reserve(track.size());
    conn_lines->lines_.reserve(track.size() - 1);
    for (size_t i = 0; i < track.size(); i++) {
      Eigen::Vector3d center = track[i].center;
      center *= world_display_scale;
      conn_lines->points_.emplace_back(center);
    }
    for (size_t i = 0; i < track.size() - 1; i++) {
      conn_lines->lines_.emplace_back(i, i + 1);
    }
  }
}

std::shared_ptr<open3d::geometry::LineSet> CreateCamera(
    const Eigen::Vector4d& qvec, const Eigen::Vector3d& tvec,
    const Eigen::Vector3d& color, const double world_display_scale) {
  const float fx = 400;
  const float fy = 400;
  const float cx = 512;
  const float cy = 384;
  const float width = 1080;
  const float height = 768;

  Eigen::Matrix3d intrinsic;
  Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
  extrinsic.block<3, 3>(0, 0) = colmap::QuaternionToRotationMatrix(qvec);
  extrinsic.block<3, 1>(0, 3) = tvec * world_display_scale;

  intrinsic << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
  auto camera = open3d::geometry::LineSet::CreateCameraVisualization(
      width, height, intrinsic, extrinsic, 4.0);
  camera->PaintUniformColor(color);
  return camera;
}

}  // namespace

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
  viewer_thread_ = std::thread(&Viewer::ThreadLoop, this);
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
  open3d::visualization::Visualizer visualizer;
  visualizer.CreateVisualizerWindow("Tracker", 1024, 768);

  visualizer.GetRenderOption().background_color_ = Eigen::Vector3d::Zero();

  // Add coordinate axis.

  auto coordinate_axis = CreateCoordinateAxis();
  visualizer.AddGeometry(coordinate_axis);
  for (const std::string& sn : camera_list_) {
    auto camera = CreateCamera(qvecs_.at(sn), tvecs_.at(sn),
                               Eigen::Vector3d(0, 1, 0), world_display_scale_);
    visualizer.AddGeometry(camera);
  }

  Eigen::Matrix4d view_matrix = Eigen::Matrix4d::Identity();
  visualizer.GetViewControl().SetViewMatrices(view_matrix);

  while (viewer_running_) {
    auto start = std::chrono::high_resolution_clock::now();
    RenderMarkers(visualizer, track_, world_display_scale_);

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
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "cost: " << elapsed.count() << " ms" << std::endl;
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