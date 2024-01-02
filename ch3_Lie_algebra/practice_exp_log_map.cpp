#include "sophus/se3.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
  Vector3d rotation_v(0, 2, 5);
  double theta = rotation_v.norm();
  while (theta > M_PI) {
    theta -= M_PI;
  }
  while (theta < -M_PI) {
    theta += M_PI;
  }
  cout << "theta " << theta << endl;
  Vector3d axis = rotation_v.normalized();
  cout << "axis : " << axis.transpose() << endl;

  // Covert Rotation matrix to Rotation vector
  Matrix3d R = AngleAxisd(theta, axis).toRotationMatrix();
  cout << "Eular angle of R : "
       << R.eulerAngles(2, 1, 0).transpose() * 180.0 / M_PI << endl;
  cout << "calculate theta by R " << acos((R.trace() - 1) / 2) << endl;
  Vector3d v(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  v /= 2 * sin(theta);
  cout << "coverer R to rotation matrix v " << v.transpose() << endl;
  Sophus::SO3d SO3 = R;
  cout << "SO3 from rotation matrix :\n" << SO3.matrix() << endl;
  cout << "Sophus::SO3d to so3 " << SO3.log().transpose() << endl;
  cout << "theta is " << theta << " axis is " << SO3.log().transpose() / theta
       << endl;

  // Covert Rotation vector to Rotation Matrix

  Matrix3d R_new = cos(theta) * Matrix3d::Identity() +
                   (1 - cos(theta)) * axis * axis.transpose() +
                   sin(theta) * Sophus::SO3d::hat(axis);
  cout << "R_new = \n " << R_new << endl;
}