#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
  // given a point p[0.5,0,0.2] in coordinated system c1.
  // we have two transformation, t1w and t2w means world to system c1 and world
  // to system c2. we want to find p in coordinated system c2.

  // p1 = t1w @ pw
  // p2 = t2w @ pw
  Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
  cout.precision(3);
  cout << "q1 length = " << q1.norm() << ", q2 length = " << q2.norm() << endl;
  q1.normalize();
  q2.normalize();
  cout << "q1 length = " << q1.norm() << ", q2 length = " << q2.norm() << endl;
  Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
  Isometry3d T1w(q1), T2w(q2);
  cout << "T1w with q1 \n" << T1w.matrix() << endl;
  cout << "T2w with q2 \n" << T2w.matrix() << endl;
  T1w.pretranslate(t1);
  T2w.pretranslate(t2);
  cout << "T1w with q1 & t1 \n" << T1w.matrix() << endl;
  cout << "T2w with q2 & t2 \n" << T2w.matrix() << endl;

  // to get the p in system c2.
  // p2 = t2w @ pw = t2w @ t1w.inverse @ p1

  Vector3d p2 = T2w * T1w.inverse() * Vector3d(0.5, 0, 0.2);
  cout << "p2 = " << p2.transpose() << endl;
}