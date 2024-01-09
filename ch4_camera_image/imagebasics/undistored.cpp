#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
  double k1(-0.28340811), k2(0.07395907), p1(0.00019359), p2 = (1.76187114e-05);
  // intrinsic
  double fx(458.654), fy(457.296), cx(367.215), cy(248.375);

  cv::Mat image = cv::imread(argv[1], 0);
  int rows = image.rows;
  int cols = image.cols;
  cv::Mat image_undistored = cv::Mat(rows, cols, CV_8UC1);

  for (int v = 0; v < rows; v++) {
    for (int u = 0; u < cols; u++) {
      double x((u - cx) / fx), y((v - cy) / fy);
      double x_2 = x * x;
      double y_2 = y * y;
      double xy = x * y;
      double r = sqrt(x_2 + y_2);
      double r_2 = r * r;
      double r_4 = r_2 * r_2;
      double x_distored =
          x * (1 + k1 * r_2 + k2 * r_4) + 2 * p1 * xy + p2 * (r_2 + 2 * x_2);
      double y_distored =
          y * (1 + k1 * r_2 + k2 * r_4) + p1 * (r_2 + 2 * y_2) + 2 * p2 * xy;
      double u_distored = fx * x_distored + cx;
      double v_distored = fy * y_distored + cy;
      if (u_distored >= 0 && v_distored >= 0 && u_distored < cols &&
          v_distored < rows) {
        image_undistored.at<uchar>(v, u) =
            image.at<uchar>((int)v_distored, (int)u_distored);
      } else {
        image_undistored.at<uchar>(v, u) = 0;
      }
    }
  }

  cv::imshow("distored", image);
  cv::imshow("undistored", image_undistored);
  cv::waitKey(0);
  return 0;
}