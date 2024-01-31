#include <chrono>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

void pose_estimation_2d2d(vector<KeyPoint> keypoint1,
                          vector<KeyPoint> keypoint2, vector<DMatch> matches,
                          Mat &R, Mat &t_skew, const Mat &K) {

  vector<Point2f> point1;
  vector<Point2f> point2;
  for (int i = 0; i < (int)matches.size(); i++) {
    point1.push_back(keypoint1[matches[i].queryIdx].pt);
    point2.push_back(keypoint2[matches[i].trainIdx].pt);
  }

  // Point2d principal_point(325.1, 249.7);
  // double focal_length = 521;
  // Mat essential_matrix =
  //     findEssentialMat(point1, point2, focal_length, principal_point);
  Mat essential_matrix = findEssentialMat(point1, point2, K);

  // Mat fundamental_matrix = K.inv().t() * essential_matrix * K.inv();
  // cout << "fundamental_matrix is \n" << fundamental_matrix << endl;

  // Mat test_fundamental_matrix = findFundamentalMat(point1, point2,
  // FM_RANSAC); cout << "test_fundamental_matrix is \n" <<
  // test_fundamental_matrix << endl;
  Mat t;
  recoverPose(essential_matrix, point1, point2, K, R, t);

  cout << "R is \n" << R << endl;
  cout << "t is \n" << t << endl;
  cout << "t length : "
       << sqrt(t.at<double>(0, 0) * t.at<double>(0, 0) +
               t.at<double>(1, 0) * t.at<double>(1, 0) +
               t.at<double>(2, 0) * t.at<double>(2, 0))
       << endl;
  t_skew = (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
            t.at<double>(2, 0), 0, -t.at<double>(0, 0), -t.at<double>(1, 0),
            t.at<double>(0, 0), 0);
  Mat t_skew_R = t_skew * R;
  double scalar = essential_matrix.at<double>(2, 2) / t_skew_R.at<double>(2, 2);

  cout << "scalar : " << scalar << endl;
  cout << "t^R*scalar \n" << scalar * t_skew_R << endl;

  Mat dif = scalar * t_skew_R - essential_matrix;
  cout << "Diff between E and t^R * scalar \n" << dif << endl;
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                 (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

int main(int argc, char *argv[]) {
  if (argc != 4) {
    cout << "usage : feature_extraction img1 img2 ratio_thresh_for_matches"
         << endl;
    return 1;
  }

  //__read images
  Mat img1 = imread(argv[1], IMREAD_COLOR);
  Mat img2 = imread(argv[2], IMREAD_COLOR);
  assert(img1.data != nullptr && img2.data != nullptr);

  std::vector<KeyPoint> key_point1, key_point2;
  Mat descriptors1, descriptors2;
  Ptr<ORB> detector = ORB::create();

  auto t1 = chrono::steady_clock::now();
  detector->detectAndCompute(img1, noArray(), key_point1, descriptors1);
  detector->detectAndCompute(img2, noArray(), key_point2, descriptors2);

  auto t2 = chrono::steady_clock::now();
  auto duration = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "extract ORB cost " << duration.count() << "seconds" << endl;

  Ptr<FlannBasedMatcher> matcher = cv::makePtr<FlannBasedMatcher>(
      cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
  vector<vector<DMatch>> knn_matches;
  t1 = chrono::steady_clock::now();
  matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);
  t2 = chrono::steady_clock::now();
  duration = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "match features cost " << duration.count() << " seconds" << endl;
  cout << "knn matches size " << knn_matches.size() << endl;
  vector<DMatch> matches;
  vector<DMatch> good_matches;
  float ratio_thresh = stof(argv[3]);
  for (size_t i = 0; i < knn_matches.size(); i++) {
    if (knn_matches[i].size() < 2)
      continue;
    matches.push_back(knn_matches[i][0]);
    if (knn_matches[i][0].distance <
        ratio_thresh * knn_matches[i][1].distance) {
      good_matches.push_back(knn_matches[i][0]);
    }
  }
  cout << "matches size " << good_matches.size() << "/" << matches.size()
       << endl;
  Mat img_match;
  Mat img_goodmatch;
  drawMatches(img1, key_point1, img2, key_point2, matches, img_match);
  drawMatches(img1, key_point1, img2, key_point2, good_matches, img_goodmatch);
  Mat R, t_skew;
  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  pose_estimation_2d2d(key_point1, key_point2, good_matches, R, t_skew, K);

  for (DMatch m : matches) {
    Point2d pt1 = pixel2cam(key_point1[m.queryIdx].pt, K);
    Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
    Point2d pt2 = pixel2cam(key_point2[m.trainIdx].pt, K);
    Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
    Mat d = y2.t() * t_skew * R * y1;
    // cout << "epipolar constraint = " << d << endl;
  }
  return 0;
}