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
  R_1to2 << 0.516440425226306, 0.003002949783359, 0.856317855404563;
  -0.022409529366309, 0.999698768090626, 0.010009299259498;
  -0.856029847718906, -0.024358886893376, 0.516352151582293;
  R_1to2.transposeInPlace();
  Eigen::Vector3d tvec_1to2;
  tvec_1to2 << 2.399095086291085e+03, -0.438003706659304, 1.697229189984155e+03;
  Eigen::Vector4d qvec_1to2 = RotationMatrixToQuaternion(R_1to2);

  Eigen::Vector4d qvec2_w2c;
  Eigen::Vector3d tvec2_w2c;
  ConcatenatePoses(qvec1_w2c, tvec1_w2c, qvec_1to2, tvec_1to2, &qvec2_w2c,
                   &tvec2_w2c);
  tvec2_w2c *= 0.001;  // to meters.
  std::cout << "qvec2 w2c: " << qvec2_w2c.transpose() << std::endl;
  std::cout << "tvec2 w2c: " << tvec2_w2c.transpose() << std::endl;
}