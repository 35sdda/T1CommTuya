#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include "CVstereoconfig.hpp"

StereoCamera::StereoCamera() {
    // Initialize with default parameters
    cam_matrix_left = (cv::Mat_<double>(3, 3) <<
        343.08137932672, 0, 322.85915399716,
        0, 346.33407617662, 179.98677724702,
        0, 0, 1);
    cam_matrix_right = (cv::Mat_<double>(3, 3) <<
        341.08322800107, 0, 319.31709270004,
        0, 344.64095231921, 183.97491447331,
        0, 0, 1);
    distortion_l = (cv::Mat_<double>(1, 5) <<
        0.009575955880318718, -0.02692131066403678, 0.006446429415722862, -0.0005385024233826699, -0.002365091941747838);
    distortion_r = (cv::Mat_<double>(1, 5) <<
        -0.005419592453167523, 0.05019692681278288, 0.006477964228343043, -0.004549628271537758, -0.142577045951517);
    R = (cv::Mat_<double>(3, 3) <<
        0.9999544551686244, 0.001223783108950137, 0.00946519642277867,
        -0.001160874948859106, 0.9999772219811565, -0.006648901277804161,
        -0.009473117637433052, 0.006637610545303391, 0.9999330988463559);
    T = (cv::Mat_<double>(3, 1) << -59.22414148310872, 0.03679882872157407, -2.813894634419487);
    doffs = 0.0;
    isRectified = false;
    
    // Load parameters from StereoParam.upt file
    loadParametersFromFile("upt/StereoParam.upt");
}

void StereoCamera::loadParametersFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    std::string line;
    // Read left camera intrinsic matrix
    while (std::getline(file, line)) {
        if (line.find("Left Camera Intrinsic Matrix") != std::string::npos) {
            break;
        }
    }

    // Read K1 matrix (3x3)
    double k1_data[9];
    char dummy;
    // Read first line
    std::getline(file, line);
    std::istringstream k1_stream1(line);
    k1_stream1 >> dummy; // [
    k1_stream1 >> k1_data[0];
    k1_stream1 >> dummy; // ,
    k1_stream1 >> k1_data[1];
    k1_stream1 >> dummy; // ,
    k1_stream1 >> k1_data[2];
    k1_stream1 >> dummy; // ;
    // Read second line
    std::getline(file, line);
    std::istringstream k1_stream2(line);
    k1_stream2 >> k1_data[3];
    k1_stream2 >> dummy; // ,
    k1_stream2 >> k1_data[4];
    k1_stream2 >> dummy; // ,
    k1_stream2 >> k1_data[5];
    k1_stream2 >> dummy; // ;
    // Read third line
    std::getline(file, line);
    std::istringstream k1_stream3(line);
    k1_stream3 >> k1_data[6];
    k1_stream3 >> dummy; // ,
    k1_stream3 >> k1_data[7];
    k1_stream3 >> dummy; // ,
    k1_stream3 >> k1_data[8];
    k1_stream3 >> dummy; // ]

    // Read left camera distortion coefficients
    while (std::getline(file, line)) {
        if (line.find("Left Camera Distortion Coefficients") != std::string::npos) {
            break;
        }
    }

    // Read D1 coefficients (5x1)
    double d1_data[5];
    // Read first line
    std::getline(file, line);
    std::istringstream d1_stream1(line);
    d1_stream1 >> dummy; // [
    d1_stream1 >> d1_data[0];
    d1_stream1 >> dummy; // ;
    // Read second line
    std::getline(file, line);
    std::istringstream d1_stream2(line);
    d1_stream2 >> d1_data[1];
    d1_stream2 >> dummy; // ;
    // Read third line
    std::getline(file, line);
    std::istringstream d1_stream3(line);
    d1_stream3 >> d1_data[2];
    d1_stream3 >> dummy; // ;
    // Read fourth line
    std::getline(file, line);
    std::istringstream d1_stream4(line);
    d1_stream4 >> d1_data[3];
    d1_stream4 >> dummy; // ;
    // Read fifth line
    std::getline(file, line);
    std::istringstream d1_stream5(line);
    d1_stream5 >> d1_data[4];
    d1_stream5 >> dummy; // ]

    // Read right camera intrinsic matrix
    while (std::getline(file, line)) {
        if (line.find("Right Camera Intrinsic Matrix") != std::string::npos) {
            break;
        }
    }

    // Read K2 matrix (3x3)
    double k2_data[9];
    // Read first line
    std::getline(file, line);
    std::istringstream k2_stream1(line);
    k2_stream1 >> dummy; // [
    k2_stream1 >> k2_data[0];
    k2_stream1 >> dummy; // ,
    k2_stream1 >> k2_data[1];
    k2_stream1 >> dummy; // ,
    k2_stream1 >> k2_data[2];
    k2_stream1 >> dummy; // ;
    // Read second line
    std::getline(file, line);
    std::istringstream k2_stream2(line);
    k2_stream2 >> k2_data[3];
    k2_stream2 >> dummy; // ,
    k2_stream2 >> k2_data[4];
    k2_stream2 >> dummy; // ,
    k2_stream2 >> k2_data[5];
    k2_stream2 >> dummy; // ;
    // Read third line
    std::getline(file, line);
    std::istringstream k2_stream3(line);
    k2_stream3 >> k2_data[6];
    k2_stream3 >> dummy; // ,
    k2_stream3 >> k2_data[7];
    k2_stream3 >> dummy; // ,
    k2_stream3 >> k2_data[8];
    k2_stream3 >> dummy; // ]

    // Read right camera distortion coefficients
    while (std::getline(file, line)) {
        if (line.find("Right Camera Distortion Coefficients") != std::string::npos) {
            break;
        }
    }

    // Read D2 coefficients (5x1)
    double d2_data[5];
    // Read first line
    std::getline(file, line);
    std::istringstream d2_stream1(line);
    d2_stream1 >> dummy; // [
    d2_stream1 >> d2_data[0];
    d2_stream1 >> dummy; // ;
    // Read second line
    std::getline(file, line);
    std::istringstream d2_stream2(line);
    d2_stream2 >> d2_data[1];
    d2_stream2 >> dummy; // ;
    // Read third line
    std::getline(file, line);
    std::istringstream d2_stream3(line);
    d2_stream3 >> d2_data[2];
    d2_stream3 >> dummy; // ;
    // Read fourth line
    std::getline(file, line);
    std::istringstream d2_stream4(line);
    d2_stream4 >> d2_data[3];
    d2_stream4 >> dummy; // ;
    // Read fifth line
    std::getline(file, line);
    std::istringstream d2_stream5(line);
    d2_stream5 >> d2_data[4];
    d2_stream5 >> dummy; // ]

    // Read rotation matrix
    while (std::getline(file, line)) {
        if (line.find("Stereo Extrinsic Rotation Matrix") != std::string::npos) {
            break;
        }
    }

    // Read R matrix (3x3)
    double r_data[9];
    // Read first line
    std::getline(file, line);
    std::istringstream r_stream1(line);
    r_stream1 >> dummy; // [
    r_stream1 >> r_data[0];
    r_stream1 >> dummy; // ,
    r_stream1 >> r_data[1];
    r_stream1 >> dummy; // ,
    r_stream1 >> r_data[2];
    r_stream1 >> dummy; // ;
    // Read second line
    std::getline(file, line);
    std::istringstream r_stream2(line);
    r_stream2 >> r_data[3];
    r_stream2 >> dummy; // ,
    r_stream2 >> r_data[4];
    r_stream2 >> dummy; // ,
    r_stream2 >> r_data[5];
    r_stream2 >> dummy; // ;
    // Read third line
    std::getline(file, line);
    std::istringstream r_stream3(line);
    r_stream3 >> r_data[6];
    r_stream3 >> dummy; // ,
    r_stream3 >> r_data[7];
    r_stream3 >> dummy; // ,
    r_stream3 >> r_data[8];
    r_stream3 >> dummy; // ]

    // Read translation vector
    while (std::getline(file, line)) {
        if (line.find("Stereo Extrinsic Translation Vector") != std::string::npos) {
            break;
        }
    }

    // Read T vector (3x1)
    double t_data[3];
    // Read first line
    std::getline(file, line);
    std::istringstream t_stream1(line);
    t_stream1 >> dummy; // [
    t_stream1 >> t_data[0];
    t_stream1 >> dummy; // ;
    // Read second line
    std::getline(file, line);
    std::istringstream t_stream2(line);
    t_stream2 >> t_data[1];
    t_stream2 >> dummy; // ;
    // Read third line
    std::getline(file, line);
    std::istringstream t_stream3(line);
    t_stream3 >> t_data[2];
    t_stream3 >> dummy; // ]

    file.close();

    // Convert 1920x1080 parameters to 640x360 (scale factor 1/3)
    const double scale = 1.0 / 3.0;
    
    // Left camera intrinsic matrix
    cam_matrix_left = (cv::Mat_<double>(3, 3) <<
        k1_data[0] * scale, 0, k1_data[2] * scale,
        0, k1_data[4] * scale, k1_data[5] * scale,
        0, 0, 1);
    
    // Right camera intrinsic matrix
    cam_matrix_right = (cv::Mat_<double>(3, 3) <<
        k2_data[0] * scale, 0, k2_data[2] * scale,
        0, k2_data[4] * scale, k2_data[5] * scale,
        0, 0, 1);

    // Distortion coefficients (unchanged with resolution)
    distortion_l = (cv::Mat_<double>(1, 5) <<
        d1_data[0], d1_data[1], d1_data[2], d1_data[3], d1_data[4]);

    distortion_r = (cv::Mat_<double>(1, 5) <<
        d2_data[0], d2_data[1], d2_data[2], d2_data[3], d2_data[4]);

    // Rotation matrix (unchanged with resolution)
    R = (cv::Mat_<double>(3, 3) <<
        r_data[0], r_data[1], r_data[2],
        r_data[3], r_data[4], r_data[5],
        r_data[6], r_data[7], r_data[8]);

    // Translation vector (convert from meters to millimeters)
    T = (cv::Mat_<double>(3, 1) <<
        t_data[0] * 1000.0, t_data[1] * 1000.0, t_data[2] * 1000.0);

    doffs = 0.0;
    isRectified = false;
}

void StereoCamera::setMiddleBurryParams() {
    cam_matrix_left = (cv::Mat_<double>(3, 3) <<
        3997.684, 0, 225.0,
        0, 3997.684, 187.5,
        0, 0, 1);

    cam_matrix_right = (cv::Mat_<double>(3, 3) <<
        3997.684, 0, 225.0,
        0, 3997.684, 187.5,
        0, 0, 1);

    distortion_l = cv::Mat::zeros(1, 5, CV_64F);
    distortion_r = cv::Mat::zeros(1, 5, CV_64F);

    R = cv::Mat::eye(3, 3, CV_64F);
    T = (cv::Mat_<double>(3, 1) << -193.001, 0.0, 0.0);

    doffs = 131.111;
    isRectified = true;
}

cv::Mat rodrigues(const cv::Mat& rot_vector) {
    double theta = cv::norm(rot_vector);
    cv::Mat r = rot_vector / theta;

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
         0,       -r.at<double>(2),  r.at<double>(1),
         r.at<double>(2),  0,       -r.at<double>(0),
        -r.at<double>(1),  r.at<double>(0), 0);

    cv::Mat I = cv::Mat::eye(3, 3, CV_64F);

    return std::cos(theta) * I + (1 - std::cos(theta)) * r * r.t() + std::sin(theta) * K;
}
