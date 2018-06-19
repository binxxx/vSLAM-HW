#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char** argv)
{
    Eigen::Quaterniond q1 = Eigen::Quaterniond(0.55, 0.3, 0.2, 0.2);
    q1.normalize();
//    Eigen::Quaterniond q1_inverse = q1.inverse();
    Eigen::Vector3d t1 = Eigen::Vector3d(0.7, 1.1, 0.2);
    Eigen::Quaterniond q2 = Eigen::Quaterniond(-0.1, 0.3, -0.7, 0.2);
    q2.normalize();
    Eigen::Vector3d t2 = Eigen::Vector3d(-0.1, 0.4, 0.8);

    Eigen::Vector3d p1 = Eigen::Vector3d(0.5, -0.1, 0.2);
    Eigen::Vector3d p2;
//    Eigen::Vector3d p;
//    Eigen::Vector3d tmp1, tmp2;
//
//    tmp1 = p1 - t1;
//
//    p = q1.inverse() * tmp1;
//    p2 = q2 * p + t2;

    p2 = q2 * (q1.inverse() * p1 - q1.inverse() * t1) + t2;


    cout << "p2 = \n" << p2 << endl;


//    cout << "normalized quaternion q1 = \n" << q1.coeffs() << endl;
//    cout << "normalized quaternion q1 = \n" << q1.inverse().coeffs() << endl;

    return 0;
}