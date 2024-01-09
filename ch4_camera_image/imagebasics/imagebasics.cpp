#include <chrono>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;
int main(int argc, char *argv[]) {
  // Read image in argv[1]
  cv::Mat image;
  image = cv::imread(argv[1]);
  if (image.data == nullptr) {
    cerr << "file" << argv[1] << "not exist" << endl;
    return 0;
  }

  cout << "Image cols: " << image.cols << ", rows: " << image.rows
       << ", channels: " << image.channels() << endl;
  imshow("image", image);
  waitKey(0);

  if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
    cout << "image type incorrect." << endl;
    return 0;
  }

  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for (size_t y = 0; y < (size_t)image.rows; y++) {
    // get the pointer of each row
    unsigned char *row_ptr = image.ptr<unsigned char>(y);
    for (size_t x = 0; x < (size_t)image.cols; x++) {
      unsigned char *data_ptr = &row_ptr[x * image.channels()];
      for (int c = 0; c != image.channels(); c++) {
        unsigned int data = static_cast<unsigned int>(data_ptr[c]);
        cout << data;
        if (c != image.channels() - 1) {
          cout << ", ";
        }
      }
      cout << endl;
    }
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used =
      chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "time used " << time_used.count() << " seconds." << endl;

  cv::Mat image_another = image; // it is a reference;
  image_another(Rect(0, 0, 100, 100)).setTo(0);
  cv::imshow("image", image);
  cv::waitKey(0);

  cv::Mat image_clone = image.clone(); // do a real copy
  image_clone(Rect(0, 0, 100, 100)).setTo(255);
  cv::imshow("image", image);
  cv::imshow("image_clone", image_clone);
  cv::waitKey(0);

  cv::destroyAllWindows();
  return 0;
}