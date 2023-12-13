#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
  Matrix3d rotation_matrix = Matrix3d::Identity();
  AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));
  cout.precision(3);
  cout << "rotation matrix = \n" << rotation_vector.matrix() << endl;
  // AngleAxisd.matrix() -> derived().toRotationMatrix();
  rotation_matrix = AngleAxisd(M_PI / 6, Vector3d(0, 1, 0)).toRotationMatrix();

  Vector3d v(1, 0, 0);
  Vector3d v_rotated = rotation_vector * v;
  cout << "(1,0,0) after rotation by Angle axis = " << v_rotated.transpose()
       << endl;

  v_rotated = rotation_matrix * v;
  cout << "(1,0,0) after rotation by rotation matrix = "
       << v_rotated.transpose() << endl;

  Isometry3d T = Isometry3d::Identity();
  T.rotate(rotation_vector);
  T.pretranslate(Vector3d(1, 2, 3));
  cout << "Transform matrix = \n" << T.matrix() << endl;
  Vector3d v_transformed = T * v;
  cout << "v transformed = " << v_transformed.transpose() << endl;

  Quaterniond q = Quaterniond(rotation_vector);
  cout << "Quaternion of  rotation vector = " << q.coeffs().transpose() << endl;
  // in Eigen Quaternion coeffs is (x,y,z,w). the real part is at the end of
  // coeffs.

  q = Quaterniond(rotation_matrix);
  cout << "Quaternion of rotation matrix = " << q.coeffs().transpose() << endl;
  // rotate vector by quaternion
  v_rotated = q * v;
  // in the math v' = qvq^{-1}
  cout << "qvq^{-1} = v' = " << v_rotated.transpose() << endl;
  // in Eigen Quaternion construster still Quaternion(w,x,y,z)
  cout << "by math definition "
       << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose()
       << endl;
}