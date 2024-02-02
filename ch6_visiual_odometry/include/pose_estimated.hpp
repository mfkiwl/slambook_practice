#pragma once

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

inline Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                 (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

inline Mat vecotr_2_skew_mat(const Mat &t) {
  assert(t.rows != 3 || t.cols != 3);
  return (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
          t.at<double>(2, 0), 0, -t.at<double>(0, 0), -t.at<double>(1, 0),
          t.at<double>(0, 0), 0);
}

void pose_estimation_by_Essential(const vector<KeyPoint> &keypoint1,
                                  const vector<KeyPoint> &keypoint2,
                                  const vector<DMatch> &matches, const Mat &K,
                                  Mat &R, Mat &t);

void recover_feature_spatial_by_triangulation(
    const vector<KeyPoint> &keypoint_1, const vector<KeyPoint> &keypoint_2,
    const std::vector<DMatch> &matches, const Mat &K, const Mat &R,
    const Mat &t, vector<Point3d> &points);