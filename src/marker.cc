#include "marker.h"

MarkerTrack::MarkerTrack() {}

const std::vector<Marker>& MarkerTrack::Track() const { return track_; }

std::vector<Marker>& MarkerTrack::Track() { return track_; }