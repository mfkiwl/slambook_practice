#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <sstream>
#include <unistd.h>

using namespace std;
using namespace Eigen;

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>
    TrajectoryType;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

void showPointCloud_6d(
    const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);

int main(int argc, char *argv[]) {
  vector<cv::Mat> color_imgs, depth_imgs;
  TrajectoryType T_wc_s;
  ifstream ifs(argv[1]);
  if (!ifs) {
    cerr << "no input pose file" << endl;
    return 1;
  }
  string line;
  while (getline(ifs, line)) {
    istringstream iss(line);
    vector<double> data;
    double value;
    while (iss >> value) {
      data.push_back(value);
    }

    T_wc_s.push_back(
        Sophus::SE3d(Quaterniond(data[6], data[3], data[4], data[5]),
                     Vector3d(data[0], data[1], data[2])));
  }
  cout << "T_wc_s .size() = " << T_wc_s.size() << endl;
  for (int i = 0; i < (int)T_wc_s.size(); i++) {
    string color_img_fn = argv[2] + to_string(i + 1) + ".png";
    string depth_img_fn = argv[3] + to_string(i + 1) + ".pgm";
    color_imgs.push_back(cv::imread(color_img_fn));
    depth_imgs.push_back(cv::imread(depth_img_fn, -1));
  }
  cout << "color_imgs .size() = " << color_imgs.size() << endl;

  double cx(325.5), cy(253.5), fx(518.0), fy(519.0);
  double depth_scale(1000.0);
  vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
  // for effiency
  pointcloud.reserve(1000000);
  for (int i = 0; i < (int)color_imgs.size(); i++) {
    cout << "converting the image" << i + 1 << endl;
    cv::Mat color = color_imgs[i];
    cv::Mat depth = depth_imgs[i];
    Sophus::SE3d T_wc = T_wc_s[i];
    for (int row = 0; row < color.rows; row++) {
      for (int col = 0; col < color.cols; col++) {
        // the pgm data structure?
        unsigned int d = depth.ptr<unsigned short>(row)[col];
        if (d == 0)
          continue; // no depth information
        Vector3d point_camera;
        point_camera[2] = double(d) / depth_scale;
        point_camera[0] = (col - cx) * point_camera[2] / fx;
        point_camera[1] = (row - cy) * point_camera[2] / fy;
        Vector3d point_world = T_wc * point_camera;

        Vector6d p;
        p.head<3>() = point_world;
        p[5] = color.data[row * color.step + col * color.channels()]; // blue
        p[4] =
            color.data[row * color.step + col * color.channels() + 1]; // green
        p[3] = color.data[row * color.step + col * color.channels() + 2]; // red
        pointcloud.push_back(p);
      }
    }
  }

  cout << "global point cloud has " << pointcloud.size() << " points." << endl;
  showPointCloud_6d(pointcloud);
  return 0;
}

void showPointCloud_6d(
    const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

  if (pointcloud.empty()) {
    cerr << "Point cloud is empty!" << endl;
    return;
  }

  pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
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
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto &p : pointcloud) {
      glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
      glVertex3d(p[0], p[1], p[2]);
    }
    glEnd();
    pangolin::FinishFrame();
    usleep(5000); // sleep 5 ms
  }
  return;
}
