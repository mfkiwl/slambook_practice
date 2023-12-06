#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
  cout << "Eigen version: " << EIGEN_WORLD_VERSION << "," << EIGEN_MAJOR_VERSION
       << "." << EIGEN_MINOR_VERSION << endl;
  // baisc declarion
  Matrix<float, 2, 3> matrixf_23;

  Vector3d v_3d = Vector3d::Zero();
  Matrix<float, 3, 1> vd_3d;

  // zero matrix.
  Matrix3d matrixd_33 = Matrix3d::Zero();
  // dynamic matrix (not recommand)
  Matrix<float, Dynamic, Dynamic> matrixf_dynamic;
  //   or
  MatrixXd matrix_x;

  Matrix4i matrixi_44;
  matrixi_44 << 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8;
  cout << "matrixi_44 : \n" << matrixi_44 << endl;

  // basic operation
  // initialzation
  matrixf_23 << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6;
  // output
  cout << "matrix 2x3 from 1 to 6: \n" << matrixf_23 << endl;
  // access elements
  cout << "matrixf_23[1,2] " << matrixf_23(1, 2) << endl;
  matrixd_33(2, 2) = 2;
  cout << "matrixd_33 :\n" << matrixd_33 << endl;

  matrixf_dynamic = matrixf_23 * matrixd_33.cast<float>();
  cout << "matrixf_dynamic \n" << matrixf_dynamic << endl;

  Matrix<double, 2, 3> result1 = matrixf_23.cast<double>() * matrixd_33;
  cout << "matrixf_23 @ matrixd_33 : \n" << result1 << endl;

  Matrix<float, 3, 2> matrixf_32 = Matrix<float, 3, 2>::Zero();
  cout << "matrixf_32 :\n" << matrixf_32 << endl;
  Matrix4f matrixf_44 = Matrix4f::Zero();
  cout << "matrixf_44 :\n" << matrixf_44 << endl;

  v_3d << 1, 2, 3;
  cout << "v_3d : \n" << v_3d << endl;

  RowVector3f row_v_3f;
  row_v_3f << 1, 2, 3;
  cout << "row_v_3f: \n" << row_v_3f << endl;

  Matrix<float, 3, 3> matrix_a{{6, 1, 4}, {4, 8, 4}, {6, 3, 5}};
  cout << "matrix_a : \n" << matrix_a << endl;
  cout << "matrix_a^T : \n" << matrix_a.transpose() << endl;
  cout << "tr(matrix_a) : \n" << matrix_a.trace() << endl;
  cout << "determinant of matrix_a : \n" << matrix_a.determinant() << endl;
  Matrix<float, 3, 3> inv_matrix_a = matrix_a.inverse();
  cout << "matrix_a inverse : \n" << matrix_a << endl;
  cout << "inv_matrix_a @ _matrix_a:\n" << matrix_a * inv_matrix_a << endl;

  Matrix<double, 3, 3> matrix_b{{4, -1, 2}, {-1, 6, 0}, {2, 0, 5}};
  cout << "matrix_b : \n" << matrix_b << endl;
  LLT<Matrix3d> lltOfA(matrix_b); // compute the Cholesky decomposition of A
  Matrix3d L = lltOfA.matrixL();  // retrieve factor L  in the decomposition
  // The previous two lines can also be written as "L = A.llt().matrixL()"

  cout << "The Cholesky factor L is" << endl << L << endl;
  cout << "To check this, let us compute L * L.transpose()" << endl;
  cout << L * L.transpose() << endl;
  cout << "This should equal the matrix_b" << endl;

  Matrix<float, 3, 4> matrix_p{
      {640., 0., 640., 2176.}, {0., 480., 480., 552.}, {0., 0., 1., 1.4}};
  cout << "matrix_p : \n" << matrix_p << endl;
  Matrix3f matrix_M = matrix_p(seq(0, 2), seq(0, 2)).inverse();
  cout << "matrix_M : \n" << matrix_M << endl;

  HouseholderQR<Matrix3f> qr = matrix_M.householderQr();
  Matrix3f q = qr.householderQ();
  cout << "matrix_q : \n" << q << endl;
  Matrix3f r = qr.matrixQR().triangularView<Eigen::Upper>();
  cout << "matrix_r : \n" << r << endl;
  cout << "K :\n" << r.inverse() << endl;
}