#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <base/pose.h>
#include <util/misc.h>

using namespace colmap;

int main(int argc, char* argv[]) {
  Eigen::Vector4d qvec1_w2c(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d tvec1_w2c(0.0, 0.0, 0.0);

  Eigen::Matrix3d R_1to2;

  Eigen::Vector4d qvec2_w2c;
  Eigen::Vector3d tvec2_w2c;
}