#include "feature_matches.hpp"
#include "pose_estimated.hpp"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;

inline cv::Scalar get_color(float depth) {
  float up_th = 50, low_th = 10, th_range = up_th - low_th;
  if (depth > up_th)
    depth = up_th;
  if (depth < low_th)
    depth = low_th;
  return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

int main(int argc, char *argv[]) {

  if (argc != 4) {
    cout << "usage : feature_extraction img1 img2 ratio_thresh_for_matches"
         << endl;
    return false;
  }

  Mat img1 = imread(argv[1], IMREAD_COLOR);
  Mat img2 = imread(argv[2], IMREAD_COLOR);

  Matches_data matches_data;
  cout << get_orb_match_data(img1, img2, stof(argv[3]), matches_data) << endl;

  Mat R, t;
  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  pose_estimation_by_Essential(matches_data.key_point1, matches_data.key_point2,
                               matches_data.good_matches, K, R, t);

  // recover spatial information of our keypoints.
  vector<Point3d> points;
  recover_feature_spatial_by_triangulation(
      matches_data.key_point1, matches_data.key_point2,
      matches_data.good_matches, K, R, t, points);

  Mat img1_plot = img1.clone();
  Mat img2_plot = img2.clone();
  for (int i = 0; i < (int)matches_data.good_matches.size(); i++) {

    float depth1 = points[i].z;
    cout << "depth: " << depth1 << endl;
    // Point2d pt1_cam = pixel2cam(
    //     matches_data.key_point1[matches_data.good_matches[i].queryIdx].pt,
    //     K);
    cv::circle(
        img1_plot,
        matches_data.key_point1[matches_data.good_matches[i].queryIdx].pt, 2,
        get_color(depth1), 2);

    Mat pt2_trans =
        R * (Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
    float depth2 = pt2_trans.at<double>(2, 0);
    cv::circle(
        img2_plot,
        matches_data.key_point2[matches_data.good_matches[i].trainIdx].pt, 2,
        get_color(depth2), 2);
  }
  cv::imshow("img 1", img1_plot);
  cv::imshow("img 2", img2_plot);
  cv::waitKey();

  return 0;
}