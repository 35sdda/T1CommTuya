#ifndef T1_GRASS_ANALYZER_HPP
#define T1_GRASS_ANALYZER_HPP

#include <opencv2/opencv.hpp>

class T1GrassAnalyzer {
public:
    T1GrassAnalyzer();
    ~T1GrassAnalyzer();

    // Analyze fixed ROI (50,300,280x60) and return: 1 if grass ratio <= 10%, 0 if > 10%
    int start(const cv::Mat& binaryMask);


private:
    // Calculate grass ratio in fixed ROI
    double calcGrassRatio(const cv::Mat& mask);
};

#endif // T1_GRASS_ANALYZER_HPP
