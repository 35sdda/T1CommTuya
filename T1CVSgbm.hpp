#ifndef T1CVSgbm_HPP
#define T1CVSgbm_HPP
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <chrono>
#include <string>
#include <sstream>
#include <iostream>
#include <ctime>
#include <vector>
#include <cstdio>
#include "CVstereoconfig.hpp"
#include "CVsgbm.hpp"
#include "CVYolo26.hpp"
#include "T1ObsDisCalcu.hpp"

class T1CVSgbm{
public:
    T1CVSgbm();
    ~T1CVSgbm();
    void initialize(int width, int height);
    void start(cv::Mat left_image, cv::Mat right_image);
    cv::Mat meDepthMap;

private:
    void simpleNormalize(cv::Mat, cv::Mat);
    void ColorDepth(const cv::Mat& depth_map, cv::Mat& color_map, float min_mm = 500.0f, float max_mm = 4000.0f);
    StereoCamera myStereoConfig;
    
    // Cached rectification transformation maps and Q matrix
    cv::Mat meMap1x, meMap1y, meMap2x, meMap2y, meQ;
    bool meIsInitialized = false;
};

#endif // T1CVSgbm_HPP
