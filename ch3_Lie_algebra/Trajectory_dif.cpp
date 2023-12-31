#include "sophus/se3.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

string gounrdtruth_file = "../groundtruth.txt";
string estimated_file = "../estimated.txt";

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>
    TrajectoryType;

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &estimated);

TrajectoryType ReadTrajectory(const string &path);

int main(int argc, char *argv[]) {
  TrajectoryType groundtruth = ReadTrajectory(gounrdtruth_file);
  TrajectoryType estimated = ReadTrajectory(estimated_file);

  assert(!groundtruth.empty() && !estimated.empty());
  assert(groundtruth.size() == estimated.size());

  // compute rmse
  double rmse = 0;
  for (size_t i = 0; i < estimated.size(); i++) {
    Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
    // cout << "p1 = \n" << p1.matrix() << endl;
    // cout << "p2 = \n" << p2.matrix() << endl;
    // cout << "(p1.inverse() * p2) = \n" << (p1.inverse() * p2).matrix() <<
    // endl; cout << "(p1.inverse() * p2).log() = " << (p1.inverse() *
    // p2).log().transpose() << endl;
    double error = (p1.inverse() * p2).log().norm();
    rmse += error * error;
  }
  rmse = rmse / double(estimated.size());
  rmse = sqrt(rmse);
  cout << "RMSE = " << rmse << endl;

  DrawTrajectory(groundtruth, estimated);
}

TrajectoryType ReadTrajectory(const string &path) {
  ifstream fin(path);
  TrajectoryType tra;
  if (!fin) {
    cerr << "trajectory " << path << " not found" << endl;
  }

  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    Sophus::SE3d p1(Eigen::Quaterniond(qx, qy, qz, qw),
                    Eigen::Vector3d(tx, ty, tz));
    tra.push_back(p1);
  }
  return tra;
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024.768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));
  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glLineWidth(2);
    for (size_t i = 0; i < gt.size() - 1; i++) {
      glColor3f(0.0, 0.0, 1.0); // blue
      glBegin(GL_LINES);
      auto p1 = gt[i], p2 = gt[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }

    for (size_t i = 0; i < esti.size() - 1; i++) {
      glColor3f(1.0, 0.0, 01.0); // Red
      glBegin(GL_LINES);
      auto p1 = esti[i], p2 = esti[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);
  }
}