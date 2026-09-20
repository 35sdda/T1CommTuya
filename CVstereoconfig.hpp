// stereoconfig.h
#ifndef CVSTEREOCONFIG_H
#define CVSTEREOCONFIG_H

#include <opencv2/opencv.hpp>

class StereoCamera {
public:
    cv::Mat cam_matrix_left;
    cv::Mat cam_matrix_right;
    cv::Mat distortion_l;
    cv::Mat distortion_r;
    cv::Mat R;
    cv::Mat T;

    double doffs;
    bool isRectified;

    StereoCamera();
    void setMiddleBurryParams();
    void loadParametersFromFile(const std::string& filename);
};

cv::Mat rodrigues(const cv::Mat& rot_vector);

#endif
