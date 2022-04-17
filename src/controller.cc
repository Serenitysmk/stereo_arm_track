#include "src/controller.h"

using namespace colmap;

bool ControllerOptions::Check() const {
  CHECK_OPTION_GT(num_cameras, 0);
  std::vector<std::string> _camera_list = CSVToVector<std::string>(camera_list);
  CHECK_OPTION_GT(_camera_list.size(), 0);
  CHECK_OPTION_LE(_camera_list.size(), num_cameras);
  if (input_from_videos) {
    CHECK(ExistsDir(video_path));
  }
  return true;
}

Controller::Controller(const ControllerOptions* options) : options_(options) {
  CHECK(options_->Check());

  camera_lists_ = CSVToVector<std::string>(options_->camera_list);

  PrintHeading1("Robot arm tracking system using multiple cameras");

  // Initialize frame grabber.
  grabber_ =
      new FrameGrabber(options_->num_cameras, camera_lists_,
                       options_->input_from_videos, options_->video_path);

  CHECK(grabber_->Init()) << "ERROR: Failed to initialize the frame grabber!";

  // Initialize detectror.
  detector_ = new MarkerDetector(camera_lists_, cv::aruco::DICT_6X6_1000);

  // Initialize triangulator.
  triangulator_ = new Triangulator(camera_lists_);

  // Initialize cameras.
  for (const std::string& sn : camera_lists_) {
    Camera camera;
    Eigen::Vector4d qvec;
    Eigen::Vector3d tvec;
    const std::string path = JoinPaths("../config", sn + "_calibration.txt");
    LoadCameraInfo(path, camera, qvec, tvec);
    cameras_.emplace(sn, camera);
    qvecs_.emplace(sn, qvec);
    tvecs_.emplace(sn, tvec);
  }

  // Initialize viewer.
  viewer_ = new Viewer(camera_lists_, qvecs_, tvecs_, options_->display_scale);
}

void Controller::Run() {
  std::thread running_control_thread(&Controller::StopRunningControlLoop, this);
  PrintHeading1("Start running ...");

  while (!stop_running_) {
    // Grab new frames.
    bool grab_success = true;
    std::unordered_map<std::string, cv::Mat> frames = grabber_->Next();

    for (const std::string& sn : camera_lists_) {
      if (frames.at(sn).empty()) {
        grab_success = false;
      }
    }
    if (!grab_success) {
      break;
    }

    // Grab success.

    std::unordered_map<std::string, std::vector<cv::Point2f>> markers_corners;
    std::unordered_map<std::string, bool> detection_success;

    detector_->Detect(frames, markers_corners, detection_success);

    for (const std::string& sn : camera_lists_) {
      if (detection_success.at(sn)) {
        std::cout << sn << " detection success!" << std::endl;
      } else {
        std::cout << sn << " detection failed" << std::endl;
      }
    }

    Marker marker;
    marker.observations = markers_corners;

    // Triangulate marker.
    bool tri_success =
        triangulator_->Triangulate(cameras_, qvecs_, tvecs_, marker);

    std::cout << "Triangulate success! marker center: "
              << marker.center.transpose() << std::endl;
    viewer_->AddCurrentFrame(frames, marker);
  }

  stop_running_ = true;
  running_control_thread.detach();

  viewer_->Close();
}

void Controller::Shutdown() {
  PrintHeading1("Finished running, shutdown the system");
  if (!grabber_->Close()) {
    std::cerr << "ERROR: Failed to close the frame grabber!" << std::endl;
    return;
  }
}

void Controller::StopRunningControlLoop() {
  char key;
  while (!stop_running_) {
    std::cin >> key;
    if ((int)key == 27) {
      stop_running_ = true;
      break;
    }
  }
}

void Controller::LoadCameraInfo(const std::string& path, Camera& camera,
                                Eigen::Vector4d& qvec, Eigen::Vector3d& tvec) {
  std::ifstream file(path);
  CHECK(file.is_open()) << "ERROR: Cannot load camera information from: "
                        << path;

  std::string line;
  std::string item;
  std::stringstream line_stream;

  size_t width = 0;
  size_t height = 0;
  std::vector<double> params;
  params.reserve(8);

  // Skip headers.
  std::getline(file, line);
  std::getline(file, line);

  std::getline(file, line);
  line_stream = std::stringstream(line);
  std::getline(line_stream, item, ' ');
  width = std::stoi(item);
  std::getline(line_stream, item, ' ');
  height = std::stoi(item);

  // Focal lengths.
  std::getline(file, line);
  std::getline(file, line);
  line_stream = std::stringstream(line);
  for (int i = 0; i < 2; i++) {
    std::getline(line_stream, item, ' ');
    params.emplace_back(std::stold(item));
  }

  // Principal points.
  std::getline(file, line);
  std::getline(file, line);
  line_stream = std::stringstream(line);
  for (int i = 0; i < 2; i++) {
    std::getline(line_stream, item, ' ');
    params.emplace_back(std::stold(item));
  }

  // Distortion parameters [k1, k2, p1, p2].
  std::getline(file, line);
  std::getline(file, line);
  line_stream = std::stringstream(line);
  for (int i = 0; i < 4; i++) {
    std::getline(line_stream, item, ' ');
    params.emplace_back(std::stold(item));
  }

  // Rotation [QW, QX, QY, QZ].
  std::getline(file, line);
  std::getline(file, line);
  line_stream = std::stringstream(line);
  for (int i = 0; i < 4; i++) {
    std::getline(line_stream, item, ' ');
    qvec[i] = std::stold(item);
  }
  qvec.normalize();

  // Translation [TX, TY, TZ].
  std::getline(file, line);
  std::getline(file, line);
  line_stream = std::stringstream(line);
  for (int i = 0; i < 3; i++) {
    std::getline(line_stream, item, ' ');
    tvec[i] = std::stold(item);
  }

  camera.SetWidth(width);
  camera.SetHeight(height);
  camera.SetModelIdFromName("OPENCV");
  camera.SetParams(params);
}
