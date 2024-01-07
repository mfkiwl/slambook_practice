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

  cout << "===========================================================" << endl;
  cout << "verfy SE3 " << endl;
  // closure
  Vector3d translation_vector_1(-5, 1, 4);
  // use R1
  Matrix4d T1 = Matrix4d::Identity();
  T1.block<3, 3>(0, 0) = R1;
  T1.block<3, 1>(0, 3) = translation_vector_1;
  //   cout << "T1 = \n " << T1 << endl;
  Vector3d translation_vector_2(7, 2, -1);
  // use R2
  Matrix4d T2 = Matrix4d::Identity();
  T2.block<3, 3>(0, 0) = R2;
  T2.block<3, 1>(0, 3) = translation_vector_2;
  //   cout << "T2 = \n " << T2 << endl;
  // T1 * T2 needs to be SE(3), the upper left 3 x 3 matrix is SO(3) , and the
  // upper right 3 x 1 matrix is a vector and bottom left is 0 , bottom right
  // is 1.
  Matrix4d T1_2 = T1 * T2;
  Matrix3d T12_R = T1_2.block<3, 3>(0, 0);
  cout << "T12_R*T12_R^T = \n " << T12_R * T12_R.transpose() << endl;
  cout << "det(T12_R) = " << (R1 * R2).determinant() << endl;
  cout << "T1_2 translation vector " << T1_2.block<3, 1>(0, 3).transpose()
       << endl;
  cout << "T1_2 bottom row " << T1_2.block<1, 4>(3, 0) << endl;

  // associativity
  cout << "Matrix multiplication is associative " << endl;
  // Identity element
  cout << "SE(3) identity element is a 4 by 4 Identity matrix"
          "vector "
       << endl;
  // // inverse element
  cout << "det(T1_2) = " << T1_2.determinant() << " , so is a invertible matrix"
       << endl;

  cout << "===========================================================" << endl;
  cout << "verfy Sim3" << endl;
  cout << "the upper right 3 by 3 matrix is a scalar with two rotation matrix "
       << endl;
  cout << "multiplication, the upper right 3 by 1 vector is rotation matrix "
          "with t2 plus t1 vector"
       << endl;
  cout << "others just same as SE(3)" << endl;
}