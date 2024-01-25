#include <ceres/ceres.h>
#include <chrono>
#include <iostream>
#include <random>

using namespace std;

// residual cost function
struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}
  // implement operator () to compute error;
  template <typename T> bool operator()(const T *const abc, T *residual) const {
    // e = y - exp(ax^2 + bx + c)
    residual[0] =
        T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
    return true;
  }
  const double _x, _y;
};

int main(int argc, char *argv[]) {

  double ar = 1.0, br = 2.0, cr = 1.0;  // ground truth
  double ae = 2.0, be = -1.0, ce = 5.0; // initial estimation
  int N = 100;
  // double w_sigma = 1.0;
  // double inv_sigma = 1.0 / w_sigma;
  std::random_device r;
  std::mt19937 generator(r());
  normal_distribution<double> distribution(0, 1.0);

  vector<double> x_data, y_data;
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) + distribution(generator));
  }

  double abc[3] = {ae, be, ce};
  ceres::Problem problem; // construct problem (modeling problem)
  for (int i = 0; i < N; i++) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
            new CURVE_FITTING_COST(x_data[i], y_data[i])),
        nullptr, abc);
  }

  // set solver options
  ceres::Solver::Options options;
  options.linear_solver_type =
      ceres::DENSE_NORMAL_CHOLESKY; // use cholesky to solve normal equation
  options.minimizer_progress_to_stdout = true; // print to cout

  ceres::Solver::Summary summary;
  auto t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  auto t2 = chrono::steady_clock::now();
  auto duration = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << duration.count() << " seconds." << endl;

  cout << summary.BriefReport() << endl;
  cout << "estimate a,b,c = ";
  for (auto a : abc)
    cout << a << " ";
  cout << endl;
  return 0;
}