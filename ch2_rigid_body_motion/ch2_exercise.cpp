#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
  // question 5. assign the top left 3 by 3 blocks of a large matrix to a
  // Identity matrix
  Matrix<double, 5, 5> A = Matrix<double, 5, 5>::Zero();
  cout << "initial A : \n" << A << endl;
  A.block<3, 3>(0, 0) = Matrix<double, 3, 3>::Identity();
  cout << "A top left for Identity: \n" << A << endl;

  A = Matrix<double, 5, 5>::Identity();
  A.block<3, 3>(1, 2) = Matrix<double, 3, 3>{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  cout << "new A : \n" << A << endl;

  // question 6. solved the Ax =b  in Eigen;
  // if A is a invertible matrix
  cout << "determinant of A " << A.determinant() << endl;
  cout << "inverse of A : \n " << A.inverse() << endl;
  Matrix<double, 5, 5> b = Matrix<double, 5, 5>::Zero();
  int i = 1;
  for (int row = 0; row < 5; row++) {
    for (int col = 0; col < 5; col++) {
      b(row, col) = i++;
    }
  }
  cout << "matrix b : \n" << b << endl;
  Matrix<double, 5, 5> x = A.inverse() * b;
  cout << setprecision(3);
  cout << "matrix x : \n" << x << endl;
  cout << "Ax : \n" << A * x << endl;

  // if A is not a invertible matrix.
  cout << "make A not invertible" << endl;
  A(4, 4) = 0;
  cout << "matrix A : \n" << A << endl;
  cout << "determinant of A " << A.determinant() << endl;
  // using SVD to find the smallest solution.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  x = svd.solve(b);
  cout << "matrix x : \n" << x << endl;
  cout << "Ax : \n" << A * x << endl;
}
