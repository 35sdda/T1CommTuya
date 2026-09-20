#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include "CVstereoconfig.hpp"

using namespace cv;
using namespace std;

void getRectifyTransform(
    int height,
    int width,
    const StereoCamera& config,
    cv::Mat& map1x,
    cv::Mat& map1y,
    cv::Mat& map2x,
    cv::Mat& map2y,
    cv::Mat& Q)
{
    // Get camera extrinsic parameters and distortion coefficients
    const cv::Mat& R = config.R;
    const cv::Mat& T = config.T;
    const cv::Mat& left_distortion = config.distortion_l;
    const cv::Mat& right_distortion = config.distortion_r;
    
    const cv::Mat& left_K = config.cam_matrix_left;
    const cv::Mat& right_K = config.cam_matrix_right;

    // Rectification output parameters
    cv::Mat R1, R2, P1, P2;
    cv::Rect roi1, roi2;

    // Perform stereo rectification
    cv::stereoRectify(
        left_K, left_distortion,
        right_K, right_distortion,
        cv::Size(width, height),
        R, T,
        R1, R2, P1, P2, Q,
        cv::CALIB_ZERO_DISPARITY, 0, cv::Size(), &roi1, &roi2
    );

    // Generate left and right mapping tables
    cv::initUndistortRectifyMap(
        left_K, left_distortion, R1, P1,
        cv::Size(width, height),
        CV_32FC1, map1x, map1y
    );

    cv::initUndistortRectifyMap(
        right_K, right_distortion, R2, P2,
        cv::Size(width, height),
        CV_32FC1, map2x, map2y
    );
}

void rectifyImage(const Mat& img1, const Mat& img2,
                  const Mat& map1x, const Mat& map1y,
                  const Mat& map2x, const Mat& map2y,
                  Mat& out1, Mat& out2) {
    cv::remap(img1, out1, map1x, map1y, INTER_AREA);
    cv::remap(img2, out2, map2x, map2y, INTER_AREA);
}

cv::Mat draw_line(const cv::Mat& image1, const cv::Mat& image2) {
    // Get output image size
    int height = std::max(image1.rows, image2.rows);
    int width = image1.cols + image2.cols;

    // Create black output image (3 channels)
    cv::Mat output(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    // Copy images to left and right positions of output image
    image1.copyTo(output(cv::Rect(0, 0, image1.cols, image1.rows)));
    image2.copyTo(output(cv::Rect(image1.cols, 0, image2.cols, image2.rows)));

    // Draw green horizontal lines
    int line_interval = 50;
    for (int k = 1; k * line_interval < height; ++k) {
        int y = k * line_interval;
        cv::line(output, cv::Point(0, y), cv::Point(width, y),
                 cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
    return output;
}

/* 2025.01.01: creat by bupt
 */
void stereoMatchSGBM(const Mat& left_image, const Mat& right_image,
                     Mat& disp_left, Mat& disp_right,
                     bool down_scale = false) {
    // Set SGBM matching parameters
    int img_channels = left_image.channels();
    int blockSize = 3;
    Ptr<StereoSGBM> left_matcher = cv::StereoSGBM::create(
        0, 32, blockSize,
        8 * img_channels * blockSize * blockSize,
        32 * img_channels * blockSize * blockSize,
        1, 63, 15, 100, 1,
        StereoSGBM::MODE_SGBM_3WAY
    );
    // Create SGBM object
    Ptr<StereoSGBM> right_matcher = cv::StereoSGBM::create(
        -32, 32, blockSize,
        8 * img_channels * blockSize * blockSize,
        32 * img_channels * blockSize * blockSize,
        1, 63, 15, 100, 1,
        StereoSGBM::MODE_SGBM_3WAY
    );
    // Calculate disparity map
    Size size = left_image.size();
    if (!down_scale) {
        left_matcher->compute(left_image, right_image, disp_left);
        right_matcher->compute(right_image, left_image, disp_right);
    } else {
        Mat left_down, right_down;
        pyrDown(left_image, left_down);
        pyrDown(right_image, right_down);
        double scale = static_cast<double>(left_image.cols) / left_down.cols;

        Mat disp_left_half, disp_right_half;
        left_matcher->compute(left_down, right_down, disp_left_half);
        right_matcher->compute(right_down, left_down, disp_right_half);

        resize(disp_left_half, disp_left, size, 0, 0, INTER_AREA);
        resize(disp_right_half, disp_right, size, 0, 0, INTER_AREA);

        disp_left *= scale;
        disp_right *= scale;
    }

    disp_left.convertTo(disp_left, CV_32F, 1.0 / 16.0);
    disp_right.convertTo(disp_right, CV_32F, 1.0 / 16.0);
}
/* 2026.03.29: creat by gemini
 * may be more suitable on lawn
 * but not suitable indoor
 *
void stereoMatchSGBM(const Mat& left_image, const Mat& right_image,
                     Mat& disp_left, Mat& disp_right,
                     bool down_scale = false) {
    int img_channels = left_image.channels();
    
    // Increased range to see closer objects (30-40cm)
    int numDisp = 64; 
    int blockSize = 3;

    // Left Matcher configuration
    Ptr<StereoSGBM> left_matcher = cv::StereoSGBM::create(
        0,              // minDisparity
        numDisp,        // numDisparities (must be multiple of 16)
        blockSize,      // blockSize
        8 * img_channels * blockSize * blockSize,   // P1
        32 * img_channels * blockSize * blockSize,  // P2
        1,              // disp12MaxDiff
        63,             // preFilterCap
        10,             // uniquenessRatio (5-10 for better lawn texture)
        100,            // speckleWindowSize
        1,              // speckleRange
        StereoSGBM::MODE_SGBM_3WAY // Faster performance for X3MD
    );

    // Right Matcher configuration
    Ptr<StereoSGBM> right_matcher = cv::StereoSGBM::create(
        -numDisp,       // minDisparity must be negative for right-to-left
        numDisp, 
        blockSize,
        8 * img_channels * blockSize * blockSize,
        32 * img_channels * blockSize * blockSize,
        1, 63, 10, 100, 1,
        StereoSGBM::MODE_SGBM_3WAY
    );

    Size size = left_image.size();
    if (!down_scale) {
        left_matcher->compute(left_image, right_image, disp_left);
        right_matcher->compute(right_image, left_image, disp_right);
    } else {
        Mat left_down, right_down;
        pyrDown(left_image, left_down);
        pyrDown(right_image, right_down);

        Mat disp_l_half, disp_r_half;
        left_matcher->compute(left_down, right_down, disp_l_half);
        right_matcher->compute(right_down, left_down, disp_r_half);

        resize(disp_l_half, disp_left, size, 0, 0, INTER_LINEAR);
        resize(disp_r_half, disp_right, size, 0, 0, INTER_LINEAR);

        double scale = static_cast<double>(left_image.cols) / left_down.cols;
        disp_left *= scale;
        disp_right *= scale;
    }

    disp_left.convertTo(disp_left, CV_32F, 1.0 / 16.0);
    disp_right.convertTo(disp_right, CV_32F, 1.0 / 16.0);
}
*/

// Calculate depth map using opencv function
Mat getDepthMapWithQ(const Mat& disparityMap, const Mat& Q) {
    Mat depthMap;
    reprojectImageTo3D(disparityMap, depthMap, Q);
    for (int y = 0; y < depthMap.rows; ++y) {
        for (int x = 0; x < depthMap.cols; ++x) {
            Vec3f& point = depthMap.at<Vec3f>(y, x);
            if (point[2] < 0.0f || point[2] > 65535.0f) {
                point = Vec3f(0, 0, 0);
            }
        }
    }
    return depthMap;
}

Mat getDepthMapWithConfig(const Mat& disparityMap, const StereoCamera& config) {
    // Focal length fx with 640x360 resolution
    double fx = config.cam_matrix_left.at<double>(0,0);
    // Baseline B is the horizontal distance between two cameras
    double baseline = -config.T.at<double>(0, 0);
    // Create single-channel float depth map with the same size as disparity map
    Mat depthMap(disparityMap.size(), CV_32F);
    // Traverse each pixel
    for (int y = 0; y < disparityMap.rows; ++y) {
        for (int x = 0; x < disparityMap.cols; ++x) {
            // Read disparity value of current pixel
            float disp = disparityMap.at<float>(y, x);
            // Calculate depth
            float depth = (disp > 0) ? (fx * baseline / (disp + config.doffs)) : 0;
            if (depth < 0 || depth > 65535) depth = 0;
            depthMap.at<float>(y, x) = depth;
        }
    }

    return depthMap;
}

double getDistanceWithDepthMap(int x, int y, const Mat& depthMap) {
    Vec3f point = depthMap.at<Vec3f>(y, x);
    return sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]) / 1000.0;
}
