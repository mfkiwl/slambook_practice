#include "sophus/se3.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
  cout << "SO3 operation *******************" << endl;
  Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
  Quaterniond q(R);
  Sophus::SO3d SO3_R(R);
  Sophus::SO3d SO3_q(q);

  cout << "SO3 from rotation matrix :\n" << SO3_R.matrix() << endl;
  cout << "SO3 from quaternion :\n" << SO3_q.matrix() << endl;

  if (SO3_q.matrix() == SO3_R.matrix()) {
    cout << "SO3_q = SO3_R" << endl;
  }

  Vector3d so3 = SO3_R.log();
  cout << "so3 = " << so3.transpose() << endl;
  // hat is convert vector to skew-symmetric matrix
  cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << endl;
  // inverse matrix to vector
  cout << "so3 hat vee = "
       << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

  Vector3d update_so3(1e-4, 0, 0); // a small perturbation
  Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
  cout << "SO3 update = \n" << SO3_updated.matrix() << endl;

  cout << "***********************************" << endl;

  Vector3d t(1, 0, 0);
  Sophus::SE3d SE3_Rt(
      R,
      t); // use rotation matrix and translation vector to a transform matrix.
  Sophus::SE3d SE3_qt(
      q, t); // use quaternion and translation vector to a transform matrix.
  typedef Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3_Rt.log();
  cout << "se3 = " << se3.transpose() << endl;

  cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl;
  cout << "se3 hat vee = "
       << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

  Vector6d se3_update;
  se3_update.setZero();
  se3_update(0, 0) = 1e-4;
  Sophus::SE3d SE3_updated = Sophus::SE3d::exp(se3_update) * SE3_Rt;
  cout << "SE3_updated = \n" << SE3_updated.matrix() << endl;
}
