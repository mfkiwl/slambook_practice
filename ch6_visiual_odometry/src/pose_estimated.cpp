#include "pose_estimated.hpp"

void pose_estimation_by_Essential(const vector<KeyPoint> &keypoint1,
                                 const vector<KeyPoint> &keypoint2,
                                 const vector<DMatch> &matches, const Mat &K,
                                 Mat &R, Mat &t) {

  vector<Point2f> point1;
  vector<Point2f> point2;
  for (int i = 0; i < (int)matches.size(); i++) {
    point1.push_back(keypoint1[matches[i].queryIdx].pt);
    point2.push_back(keypoint2[matches[i].trainIdx].pt);
  }

  Mat essential_matrix = findEssentialMat(point1, point2, K);

  recoverPose(essential_matrix, point1, point2, K, R, t);

  cout << "R is \n" << R << endl;
  cout << "t is \n" << t << endl;
  cout << "t length : "
       << sqrt(t.at<double>(0, 0) * t.at<double>(0, 0) +
               t.at<double>(1, 0) * t.at<double>(1, 0) +
               t.at<double>(2, 0) * t.at<double>(2, 0))
       << endl;
  Mat t_skew = vecotr_2_skew_mat(t);
  Mat t_skew_R = t_skew * R;
  double scalar = essential_matrix.at<double>(2, 2) / t_skew_R.at<double>(2, 2);

  cout << "scalar : " << scalar << endl;
  cout << "t^R*scalar \n" << scalar * t_skew_R << endl;

  Mat dif = scalar * t_skew_R - essential_matrix;
  cout << "Diff between E and t^R * scalar \n" << dif << endl;
}

void recover_feature_spatial_by_triangulation(
    const vector<KeyPoint> &keypoint_1, const vector<KeyPoint> &keypoint_2,
    const std::vector<DMatch> &matches, const Mat &K, const Mat &R,
    const Mat &t, vector<Point3d> &points) {
  // T1 is Idendity matrix for rigid body (the R and t is recover based on p1 =
  // K[R|t]p2 ) T2 is the extrinsic matrix of [R|t] that given any point at
  // camera2 multiply by T2 can transfer to camera1)
  Mat T1 = (Mat_<float>(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
  Mat T2 = (Mat_<float>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1),
            R.at<double>(0, 2), t.at<double>(0, 0), R.at<double>(1, 0),
            R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
            t.at<double>(2, 0));
  vector<Point2f> pts_1, pts_2;
  for (DMatch m : matches) {
    // transfer feature points in pixel to normalize coordinate system
    pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
    pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
  }

  Mat pts_4d; // this is homogeneous coordinates points
  cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
  // transfer back to 3D camera coordinate system.
  for (int i = 0; i < pts_4d.cols; i++) {
    Mat x = pts_4d.col(i);
    x /= x.at<float>(3, 0);
    Point3d p(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0));
    points.push_back(p);
  }
}
