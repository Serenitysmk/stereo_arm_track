#include "src/marker.h"

#include <chrono>

#include <util/misc.h>

using namespace colmap;

MarkerTrackWriter::MarkerTrackWriter(const std::string& output_dir)
    : output_dir_(output_dir) {
  writer_thread_ = std::thread(&MarkerTrackWriter::TrackWriter, this);
}

void MarkerTrackWriter::InsertNewMarker(const Marker& marker) {
  std::unique_lock<std::mutex> lock(data_mutex_);
  track_.emplace(marker);
}

void MarkerTrackWriter::Close() {
  writer_running_ = false;
  writer_thread_.join();
}

void MarkerTrackWriter::TrackWriter() {
  CreateDirIfNotExists(output_dir_);
  std::stringstream filename;
  filename << "marker_track.txt";
  const std::string output_path = JoinPaths(output_dir_, filename.str());

  std::ofstream file(output_path);

  CHECK(file.is_open());

  while (writer_running_) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    if (!track_.empty()) {
      std::string line_stream;
      WriteTrack(track_.front(), line_stream);
      file << line_stream << "\n";
      track_.pop();
    }
  }
  file.close();
}

void MarkerTrackWriter::WriteTrack(const Marker& marker, std::string& str) {
  std::stringstream stream;
  // Write center.
  stream << marker.center(0) << " " << marker.center(1) << " "
         << marker.center(2) << " ";
  // 3D positions.
  for (const auto& point : marker.positions) {
    stream << point(0) << " " << point(1) << " " << point(2) << " ";
  }
  // Observations.
  for (const auto& ob : marker.observations) {
    stream << ob.first << " ";
    for (const auto& point2D : ob.second) {
      stream << point2D.x << " " << point2D.y << " ";
    }
  }
  str = stream.str();
  str = str.substr(0, str.size() - 1);
}