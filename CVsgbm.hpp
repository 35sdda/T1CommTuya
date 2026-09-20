#ifndef CVSGBM_H
#define CVSGBM_H

#include <opencv2/opencv.hpp>
#include "CVstereoconfig.hpp"

// 生成矫正映射和Q矩阵
void getRectifyTransform(
    int height,
    int width,
    const StereoCamera& config,
    cv::Mat& map1x,
    cv::Mat& map1y,
    cv::Mat& map2x,
    cv::Mat& map2y,
    cv::Mat& Q);

// 应用映射进行图像矫正
void rectifyImage(const cv::Mat& img1, const cv::Mat& img2,
                  const cv::Mat& map1x, const cv::Mat& map1y,
                  const cv::Mat& map2x, const cv::Mat& map2y,
                  cv::Mat& out1, cv::Mat& out2);

// 使用 SGBM 匹配计算视差图
void stereoMatchSGBM(const cv::Mat& left_image, const cv::Mat& right_image,
                     cv::Mat& disp_left, cv::Mat& disp_right,
                     bool down_scale = false);

// 用 Q 矩阵重建深度图
cv::Mat getDepthMapWithQ(const cv::Mat& disparityMap, const cv::Mat& Q);

// 用相机内参和 T 向量手动计算深度图
/*cv::Mat getDepthMapWithConfig(const cv::Mat& disparityMap,
                              const cv::Mat& cam_matrix_left,
                              const cv::Mat& T, double doffs);*/
cv::Mat getDepthMapWithConfig(const cv::Mat& disparityMap, const StereoCamera& config);

// 从深度图中获取指定点的空间距离（单位米）
double getDistanceWithDepthMap(int x, int y, const cv::Mat& depthMap);

#endif // SGBM_H
