//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE

    ifstream p3dfile(p3d_file);
    if (p3dfile.is_open()) {
        string line;
        while (getline(p3dfile, line)) {
            double d1, d2, d3;
            istringstream in(line);
            in >> d1 >> d2 >> d3;
            Eigen::Vector3d p3d_vec3d(d1, d2, d3);
            p3d.push_back(p3d_vec3d);
        }
        p3dfile.close();
    }
    cout << p3d.size() << endl;

    ifstream p2dfile(p2d_file);
    if (p2dfile.is_open()) {
        string line;
        while (getline(p2dfile, line)) {
            double d1, d2;
            istringstream in(line);
            in >> d1 >> d2;
            Eigen::Vector2d p2d_vec2d(d1, d2);
            p2d.push_back(p2d_vec2d);
        }
    }
    cout << p2d.size() << endl;

    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE
            Matrix<double,4,1> P;
            P << p3d[i], 1.0;
            Vector4d Pp;
            Pp = T_esti.matrix() * P;
            double xp, yp, zp;
            xp = Pp(0,0);
            yp = Pp(1,0);
            zp = Pp(2,0);

            Vector3d error;
            error = K * Pp.block<3,1>(0,0);
            Vector2d e;
            e(0,0) = p2d[i](0,0) - error(0,0) / error(2,0);
            e(1,0) = p2d[i](1,0) - error(1,0) / error(2,0);

            cost += (e(0,0)*e(0,0) + e(1,0)*e(1,0))*0.5;

	    // END YOUR CODE HERE

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE

            J << -fx/zp, 0, fx*xp/(zp*zp), fx*xp*yp/(zp*zp), -(fx+fx*xp*xp/(zp*zp)), fx*yp/zp,
                    0, -fy/zp, fy*yp/(zp*zp), fy+fy*yp*yp/(zp*zp), -fy*xp*yp/(zp*zp), -fy*xp/zp;

	    // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	// solve dx 
        Vector6d dx;

        // START YOUR CODE HERE

        dx = H.ldlt().solve(b);

        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE

        T_esti = Sophus::SE3::exp(dx) * T_esti;

        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
