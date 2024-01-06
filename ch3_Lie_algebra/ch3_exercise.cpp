#include "sophus/se3.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
  cout << "Verify SO(3), SE(3), and Sim(3) are groups on matrix multiplication."
       << endl;
  // to verify they are groups. they need to satisify 4 properties
  // closure, associativity, identity element, and inverse element.
  cout << "===========================================================" << endl;
  cout << "verfy SO3 " << endl;
  // closure
  Vector3d rotation_vector_1(11, 2, 3);
  double theta1 = rotation_vector_1.norm();
  Vector3d axis1(rotation_vector_1.normalized());
  Matrix3d R1 = AngleAxisd(theta1, axis1).toRotationMatrix();
  Vector3d rotation_vector_2(3, -2, -4);
  double theta2 = rotation_vector_2.norm();
  Vector3d axis2 = rotation_vector_2.normalized();
  Matrix3d R2 = AngleAxisd(theta2, axis2).toRotationMatrix();
  // R1 * R2 needs to be SO(3),  (R1R2)(R1R2)^T = I , and det(R1R2) = 1
  //(R1R2)(R1R2)^T = (R1R2)R2^TR1^T = R1 I R1^T = R1R1^T = I
  cout << "(R1*R2)*(R1*R2)^T = \n " << (R1 * R2) * (R1 * R2).transpose()
       << endl;
  // det(R1R2) = det(R1) * det(R2) = 1
  cout << "det(R1 * R2) = " << (R1 * R2).determinant()
       << ", det(R1) * det(R2) = " << R1.determinant() * R2.determinant()
       << endl;

  // associativity
  cout << "Matrix multiplication is associative " << endl;
  // Identity element
  cout << "SO(3) identity element is Identity matrix" << endl;
  // inverse element
  cout << "det(R1) = 1 , so is a invertible matrix" << endl;
}