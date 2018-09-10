//
// Created by binx on 9/10/18.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <Eigen/Core>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

inline float GetPixelValue(const cv::Mat& img, float x, float y) {
    uchar* data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}

int main(int argc, char** argv) {
    string left_file = "../left.png";
    string right_file = "../right.png";
    string disparity_file = "../disparity.png";

    cv::Mat left_image = cv::imread(left_file, 0);
    cv::Mat right_image = cv::imread(right_file, 0);
    cv::Mat disparity_image = cv::imread(disparity_file, 0);

    int nPoints = 100;
    vector<cv::KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(nPoints, 0.01, 20);
    detector->detect(left_image, kp1);

    vector<Point2f> pt1, pt2;
    for (auto& kp : kp1) {
        pt1.push_back(kp.pt);
    }
    vector<uchar> status;
    vector<float> error;
    cv:calcOpticalFlowPyrLK(left_image, right_image, pt1, pt2, status, error, cv::Size(8, 8));

    double total_error = 0.0;
    double count = 0.0;

    cv::Mat img2_CV;
    cv::cvtColor(left_image, img2_CV, CV_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pt1[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
            total_error += (double)GetPixelValue(disparity_image, pt1[i].x, pt1[i].y) -
                    (double)(pt1[i].x - pt2[i].x);
            count += 1;
        }
    }
    cout << "average error: " << total_error / count << endl;

    cv::imshow("opencv", img2_CV);
    cv::waitKey(0);

    return 0;
}

