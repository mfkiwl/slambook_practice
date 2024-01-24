#include <iostream>
// #include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <random>

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
  double ar = 1.0, br = 2.0, cr = 1.0;  // ground truth
  double ae = 2.0, be = -1.0, ce = 5.0; // initial estimation
  int N = 100;
  double w_sigma = 1.0;
  double inv_sigma = 1.0 / w_sigma;
  std::random_device r;
  std::mt19937 generator(r());
  normal_distribution<double> distribution(0, 1.0);

  vector<double> x_data, y_data;
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) + distribution(generator));
  }

  // state Gauss - Newton iterations
  int iterations = 1000;
  double cost = 0, lastCost = 0;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for (int iter = 0; iter < iterations; iter++) {
    Matrix3d H = Matrix3d::Zero(); // Hessian = J^T W^{-1} J
    Vector3d b = Vector3d::Zero();
    cost = 0;

    for (int i = 0; i < N; i++) {
      double xi = x_data[i], yi = y_data[i];
      double estimate_value = exp(ae * xi * xi + be * xi + ce);
      double error = yi - estimate_value;
      Vector3d J;
      J[0] = -xi * xi * estimate_value;
      J[1] = -xi * estimate_value;
      J[2] = -estimate_value;

      H += inv_sigma * inv_sigma * J * J.transpose();
      b += -inv_sigma * inv_sigma * error * J;
      cost += error * error;
    }

    // solve Hx = b
    Vector3d dx = H.ldlt().solve(b);
    if (isnan(dx[0])) {
      cout << "result is nan!!!!" << endl;
      break;
    }
    if (iter > 0 && cost >= lastCost) {
      cout << "cost: " << cost << " >= lastcost: " << lastCost
           << " , we are at local mimimum, break" << endl;
      break;
    }

    ae += dx[0];
    be += dx[1];
    ce += dx[2];

    lastCost = cost;
    cout << "total cost: " << cost << ", \t\tupdate: " << dx.transpose()
         << "\t\testimated params: " << ae << "," << be << "," << ce << endl;
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  auto duration = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << duration.count() << " seconds" << endl;
  cout << "ground truth a " << ar << " b " << br << " c " << cr << endl;
  cout << "estimate a " << ae << " b " << be << " ce " << ce << endl;
  return 0;
}