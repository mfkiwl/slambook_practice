#include <chrono>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
  if (argc != 3) {
    cout << "usage : feature_extraction img1 img2" << endl;
    return 1;
  }

  //__read images
  Mat img1 = imread(argv[1], IMREAD_COLOR);
  Mat img2 = imread(argv[2], IMREAD_COLOR);
  assert(img1.data != nullptr && img2.data != nullptr);

  std::vector<KeyPoint> key_point1, key_point2;
  Mat descriptors1, descriptors2;
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  Ptr<DescriptorMatcher> matcher =
      DescriptorMatcher::create("BruteForce-Hamming");

  auto t1 = chrono::steady_clock::now();
  detector->detect(img1, key_point1);
  detector->detect(img2, key_point2);

  descriptor->compute(img1, key_point1, descriptors1);
  descriptor->compute(img2, key_point2, descriptors2);

  auto t2 = chrono::steady_clock::now();
  auto duration = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "extract ORB cost " << duration.count() << "seconds" << endl;

  Mat outimg1;
  drawKeypoints(img1, key_point1, outimg1);
  imshow("ORB features", outimg1);

  vector<DMatch> matches;
  t1 = chrono::steady_clock::now();
  matcher->match(descriptors1, descriptors2, matches);
  t2 = chrono::steady_clock::now();
  duration = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "match features cost " << duration.count() << " seconds" << endl;

  auto min_max = minmax_element(matches.begin(), matches.end(),
                                [](const DMatch &m1, const DMatch &m2) {
                                  return m1.distance < m2.distance;
                                });
  double min_dist = min_max.first->distance;
  double max_dist = min_max.second->distance;

  cout << "Max dist " << max_dist << endl;
  cout << "Min dist " << min_dist << endl;

  vector<DMatch> good_matches;
  for (int i = 0; i < descriptors1.rows; i++) {
    if (matches[i].distance <= max(2 * min_dist, 30.0)) {
      good_matches.push_back(matches[i]);
    }
  }

  Mat img_match;
  Mat img_goodmatch;
  drawMatches(img1, key_point1, img2, key_point2, matches, img_match);
  drawMatches(img1, key_point1, img2, key_point2, good_matches, img_goodmatch);
  imshow("full match", img_match);
  imshow("good match", img_goodmatch);
  waitKey(0);

  return 0;
}