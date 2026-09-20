#ifndef T1CVSIFT_HPP
#define T1CVSIFT_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// Must add namespaces to avoid compilation errors
using namespace cv;
using namespace std;

class T1CVSift {
public:
    // SIFT constructor (using core module parameters)
    T1CVSift(
        int nFeatures = 0,
        int nOctaveLayers = 3,
        double contrastThreshold = 0.04,
        double edgeThreshold = 10,
        double sigma = 1.6
    );

    ~T1CVSift() = default;

    // Core function
    bool setTargetImage(const string& target_path);
    bool detect_obj(const string& test_path, Mat& result_img, vector<Point2f>& target_corners);
    bool detect_obj(const Mat& test_img, Mat& result_img, vector<Point2f>& target_corners);
    bool save_res(const string& save_path, const Mat& result_img);
    void setRANSACParams(float reproj_threshold, int max_iter, float confidence);
    void set_match_thresh(float strict_threshold, float relaxed_threshold);
    int start(const Mat& test_img, const string& obj_path);
    float calculateTargetAngle(const vector<Point2f>& target_corners);
    float calculateTargetDistance(const vector<Point2f>& target_corners);
    float getOppositeSide() const;
    float getAdjacentSide() const;
    float getTargetAngle() const;

private:
    // Key: Use SIFT from core module (no need for xfeatures2d)
    Ptr<SIFT> sift_detector_;  
    Mat target_gray_;         // Target image (grayscale)
    Mat target_color_;        // Target image (color)
    vector<KeyPoint> target_keypoints_;  // Target key points
    Mat target_descriptors_;  // Target descriptors

    // Algorithm parameters
    float ransac_reproj_threshold_;    // RANSAC reprojection threshold
    int ransac_max_iterations_;        // RANSAC maximum iterations
    float ransac_confidence_;          // RANSAC confidence
    float match_ratio_strict_;         // Strict matching threshold
    float match_ratio_relaxed_;        // Relaxed matching threshold
    int max_good_matches_;             // Maximum number of good matches
    
    float opposite_side;
    float adjacent_side;
    float angle_deg;
};

#endif  // T1CVSIFT_HPP