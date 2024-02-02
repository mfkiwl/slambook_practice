#pragma once

#include <iostream>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

struct Matches_data {
  /* data */
  std::vector<KeyPoint> key_point1, key_point2;
  vector<DMatch> good_matches;
};

bool get_orb_match_data(const Mat &img1, const Mat &img2,
                        const float &ratio_thresh, Matches_data &matches_data);