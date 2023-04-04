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
  R_1to2 << 0.567736577157668, -0.042096208376089, 0.822133254526085;
  0.011927254390731, 0.999007474866713, 0.042916264552412;
  -0.823123878623378, -0.014559340671567, 0.567675176521939;
  R_1to2.transposeInPlace();
  Eigen::Vector3d tvec_1to2;
  tvec_1to2 << 2.819983203567952e+03, -7.798242318805185, 1.733115576337184e+03;
  Eigen::Vector4d qvec_1to2 = RotationMatrixToQuaternion(R_1to2);

  Eigen::Vector4d qvec2_w2c;
  Eigen::Vector3d tvec2_w2c;
  ConcatenatePoses(qvec1_w2c, tvec1_w2c, qvec_1to2, tvec_1to2, &qvec2_w2c,
                   &tvec2_w2c);
  tvec2_w2c *= 0.001;  // to meters.
  std::cout << "qvec2 w2c: " << qvec2_w2c.transpose() << std::endl;
  std::cout << "tvec2 w2c: " << tvec2_w2c.transpose() << std::endl;
}