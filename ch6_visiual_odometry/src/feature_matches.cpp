#include "feature_matches.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

bool get_orb_match_data(const Mat &img1, const Mat &img2, const float &ratio_thresh,
                    Matches_data &matches_data) {
  //__read images
  assert(img1.data != nullptr && img2.data != nullptr);

  // std::vector<KeyPoint> key_point1, key_point2;
  Mat descriptors1, descriptors2;
  Ptr<ORB> detector = ORB::create();

  auto t1 = chrono::steady_clock::now();
  detector->detectAndCompute(img1, noArray(), matches_data.key_point1,
                             descriptors1);
  detector->detectAndCompute(img2, noArray(), matches_data.key_point2,
                             descriptors2);

  auto t2 = chrono::steady_clock::now();
  auto duration = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "extract ORB cost " << duration.count() << " seconds" << endl;

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
  // vector<DMatch> good_matches;
  matches_data.good_matches.resize(0);

  for (size_t i = 0; i < knn_matches.size(); i++) {
    if (knn_matches[i].size() < 2)
      continue;
    matches.push_back(knn_matches[i][0]);
    if (knn_matches[i][0].distance <
        ratio_thresh * knn_matches[i][1].distance) {
      matches_data.good_matches.push_back(knn_matches[i][0]);
    }
  }
  cout << "matches size " << matches_data.good_matches.size() << "/"
       << matches.size() << endl;
  return true;
}
