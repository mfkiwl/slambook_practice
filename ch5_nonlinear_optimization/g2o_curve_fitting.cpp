#include <Eigen/Dense>
#include <chrono>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <iostream>
#include <random>

using namespace std;

// define the Vertex
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // override the reset function
  virtual void setToOriginImpl() override { _estimate << 0, 0, 0; }

  // override the plus operator, just plain vector addition
  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(update);
  }

  // the dummy read/wirte function

  virtual bool read(istream &in) { return true; }
  virtual bool write(ostream &out) const { return true; }
};

// define edge(error): 1D error term , connected to exactly one vectex
class CurveFittingEdge
    : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}
  // define the error term computation
  virtual void computeError() override {
    const CurveFittingVertex *v =
        static_cast<const CurveFittingVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _error(0, 0) = _measurement -
                   std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
  }

  // the jacobian
  virtual void linearizeOplus() override {
    const CurveFittingVertex *v =
        static_cast<const CurveFittingVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
    _jacobianOplusXi[0] = -_x * _x * y;
    _jacobianOplusXi[1] = -_x * y;
    _jacobianOplusXi[2] = -y;
  }
  virtual bool read(istream &in) { return true; }
  virtual bool write(ostream &out) const { return true; }

public:
  double _x;
};

int main(int argc, char *argv[]) {
  double ar = 1.0, br = 2.0, cr = 1.0;  // ground truth
  double ae = 2.0, be = -1.0, ce = 5.0; // initial estimation
  int N = 100;
  double w_sigma = 1.0;
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

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;

  // choose the optimization method
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  // auto solver = new g2o::OptimizationAlgorithmLevenberg(
  //     g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  // auto solver = new g2o::OptimizationAlgorithmDogleg(
  //     g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer; // graph optimizer
  optimizer.setAlgorithm(solver); // set the alorithm
  optimizer.setVerbose(true);     // print the results;

  // add vertex
  CurveFittingVertex *v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(ae, be, ce));
  v->setId(0);
  optimizer.addVertex(v);

  // add edges
  for (int i = 0; i < N; i++) {
    CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0, v); // connect to the vertex
    edge->setMeasurement(y_data[i]);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 /
                         (w_sigma * w_sigma)); // set information matrix
    optimizer.addEdge(edge);
  }

  // carry out the optimization
  cout << "start optimization " << endl;
  auto t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  auto t2 = chrono::steady_clock::now();
  auto duration = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << duration.count() << " seconds" << endl;

  Eigen::Vector3d abc_estime = v->estimate();
  cout << "estimated mode : " << abc_estime.transpose() << endl;
  return 0;
}