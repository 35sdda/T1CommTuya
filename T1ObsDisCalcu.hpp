#ifndef T1OBSDISCALCU_HPP
#define T1OBSDISCALCU_HPP

#include "T1Common.hpp"
#include "CVstereoconfig.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

struct DetectedObject {
    std::string class_name;
    float confidence;
    cv::Rect2d bounding_box;
    double distance;
};

class T1ObsDisCalcu {
public:
    T1ObsDisCalcu();
    ~T1ObsDisCalcu();
    
    std::vector<DetectedObject> calcuObsDist(
        const cv::Mat& depth_map,
        const std::vector<std::string>& class_names,
        const std::vector<cv::Rect2d>& bounding_boxes,
        const std::vector<float>& confidences);
    
    double getAvgDistInRegion(const cv::Mat& depth_map, const cv::Rect2d& region);
    double getPointDist(const cv::Mat& depth_map, int x, int y);
    double calcObjAvgDepth(const cv::Mat& depth_map, const std::vector<cv::Rect2d>& bounding_boxes, const std::vector<std::string>& class_names, const std::string& target_class);
    int calcHist(const cv::Mat& depth_map, int x, int y, int w, int h, int bin_cm, int avoidThresh, int avoidFreq);
    Wall calcHistNew(const cv::Mat& depth_map, int x, int y, int w, int h, int bin_cm, int avoidThresh, int avoidFreq);
    void calcHistDraw(const std::vector<int>& histogram, int bin_cm, int x, int y, int w, int h);
    
private:
    StereoCamera myStereoConfig;
};

#endif // OBJECTDISTANCECALCULATOR_HPP
