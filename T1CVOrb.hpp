#ifndef T1CVORB_HPP
#define T1CVORB_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class T1CVOrb {
public:
    T1CVOrb(
        int nFeatures = 2000,
        float scaleFactor = 1.1f,
        int nLevels = 8,
        int edgeThreshold = 15,
        int firstLevel = 0,
        int WTA_K = 2,
        cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE,
        int patchSize = 31,
        int fastThreshold = 10
    );

    ~T1CVOrb() = default;

    bool setTargetImage(const std::string& target_path);
    bool detect_obj(const std::string& test_path, cv::Mat& result_img, std::vector<cv::Point2f>& target_corners);
    bool detect_obj(const cv::Mat& test_img, cv::Mat& result_img, std::vector<cv::Point2f>& target_corners);
    bool save_res(const std::string& save_path, const cv::Mat& result_img);
    void setRANSACParams(float reproj_threshold, int max_iter, float confidence);
    void set_match_thresh(float strict_threshold, float relaxed_threshold);
    int start(const cv::Mat& test_img, const std::string& obj_path);
    
    float calculateTargetAngle(const std::vector<cv::Point2f>& target_corners);
    float calculateTargetDistance(const std::vector<cv::Point2f>& target_corners);
    float getOppositeSide() const;
    float getAdjacentSide() const;
    float getTargetAngle() const;

private:
    cv::Ptr<cv::ORB> orb_detector_;
    cv::Mat target_gray_;
    cv::Mat target_color_;
    std::vector<cv::KeyPoint> target_keypoints_;
    cv::Mat target_descriptors_;

    float ransac_reproj_threshold_;
    int ransac_max_iterations_;
    float ransac_confidence_;
    float match_ratio_strict_;
    float match_ratio_relaxed_;
    int max_good_matches_;

    float opposite_side;
    float adjacent_side;
    float angle_deg;
};

#endif  // T1CVORB_HPP