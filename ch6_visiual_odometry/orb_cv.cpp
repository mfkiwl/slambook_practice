#include <chrono>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

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
  // Ptr<DescriptorExtractor> descriptor = ORB::create();

  auto t1 = chrono::steady_clock::now();
  detector->detectAndCompute(img1, noArray(), key_point1, descriptors1);
  detector->detectAndCompute(img2, noArray(), key_point2, descriptors2);

  auto t2 = chrono::steady_clock::now();
  auto duration = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "extract ORB cost " << duration.count() << "seconds" << endl;

  Mat outimg1;
  drawKeypoints(img1, key_point1, outimg1);
  imshow("ORB features", outimg1);

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
  imshow("full match", img_match);
  imshow("good match", img_goodmatch);
  waitKey(0);

  return 0;
}