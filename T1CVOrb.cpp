#include "T1CVOrb.hpp"
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

T1CVOrb::T1CVOrb(
    int nFeatures, float scaleFactor, int nLevels, int edgeThreshold,
    int firstLevel, int WTA_K, ORB::ScoreType scoreType, int patchSize, int fastThreshold
) {
    orb_detector_ = ORB::create(
        nFeatures, scaleFactor, nLevels, edgeThreshold,
        firstLevel, WTA_K, scoreType, patchSize, fastThreshold
    );

    ransac_reproj_threshold_ = 5.0f;
    ransac_max_iterations_ = 2000;
    ransac_confidence_ = 0.995f;
    match_ratio_strict_ = 0.75f;
    match_ratio_relaxed_ = 0.85f;
    max_good_matches_ = 80;

    opposite_side = 0.0f;
    adjacent_side = 0.0f;
    angle_deg = 0.0f;
}

bool T1CVOrb::setTargetImage(const string& target_path) {
    target_color_ = imread(target_path);
    if (target_color_.empty()) {
        cerr << "[T1CVOrb Error] Failed to read target image! Path: " << target_path << endl;
        return false;
    }

    cvtColor(target_color_, target_gray_, COLOR_BGR2GRAY);
    equalizeHist(target_gray_, target_gray_);

    orb_detector_->detectAndCompute(target_gray_, noArray(), target_keypoints_, target_descriptors_);

    if (target_keypoints_.empty()) {
        cerr << "[T1CVOrb Error] No keypoints detected in target image! Use an image with richer texture." << endl;
        return false;
    }

    cout << "[T1CVOrb Info] Target image keypoints extracted: " << target_keypoints_.size() << endl;
    return true;
}

bool T1CVOrb::detect_obj(const string& test_path, Mat& result_img, vector<Point2f>& target_corners) {
    Mat test_color = imread(test_path);
    if (test_color.empty()) {
        cerr << "[T1CVOrb Error] Failed to read test image! Path: " << test_path << endl;
        return false;
    }

    return detect_obj(test_color, result_img, target_corners);
}

bool T1CVOrb::detect_obj(const Mat& test_img, Mat& result_img, vector<Point2f>& target_corners) {
    if (target_gray_.empty() || target_descriptors_.empty()) {
        cerr << "[T1CVOrb Error] Call setTargetImage() first to initialize target!" << endl;
        return false;
    }

    Mat test_gray;
    cvtColor(test_img, test_gray, COLOR_BGR2GRAY);
    equalizeHist(test_gray, test_gray);

    vector<KeyPoint> test_keypoints;
    Mat test_descriptors;
    orb_detector_->detectAndCompute(test_gray, noArray(), test_keypoints, test_descriptors);

    if (test_keypoints.empty()) {
        cerr << "[T1CVOrb Error] No keypoints detected in test image!" << endl;
        return false;
    }
    cout << "[T1CVOrb Info] Test image keypoints extracted: " << test_keypoints.size() << endl;

    BFMatcher matcher(NORM_HAMMING);
    vector<vector<DMatch>> knn_matches;
    matcher.knnMatch(target_descriptors_, test_descriptors, knn_matches, 2);

    vector<DMatch> good_matches;
    for (const auto& match_pair : knn_matches) {
        if (match_pair.size() == 2 && match_pair[0].distance < match_ratio_strict_ * match_pair[1].distance) {
            good_matches.push_back(match_pair[0]);
        }
    }
    cout << "[T1CVOrb Info] Valid matches after strict threshold (" << match_ratio_strict_ << "): " << good_matches.size() << endl;

    if (good_matches.size() < 6) {
        cout << "[T1CVOrb Warning] Insufficient valid matches. Relaxing threshold to " << match_ratio_relaxed_ << "..." << endl;
        good_matches.clear();
        for (const auto& match_pair : knn_matches) {
            if (match_pair.size() == 2 && match_pair[0].distance < match_ratio_relaxed_ * match_pair[1].distance) {
                good_matches.push_back(match_pair[0]);
            }
        }
        if (good_matches.size() < 4) {
            cerr << "[T1CVOrb Error] Still insufficient valid matches after relaxing threshold. Target detection failed!" << endl;
            return false;
        }
        cout << "[T1CVOrb Info] Valid matches after relaxed threshold: " << good_matches.size() << endl;
    }

    if (good_matches.size() > max_good_matches_) {
        sort(good_matches.begin(), good_matches.end());
        good_matches.resize(max_good_matches_);
        cout << "[T1CVOrb Info] Keeping top " << max_good_matches_ << " best matches for localization" << endl;
    }

    vector<Point2f> target_points, test_points;
    for (const auto& match : good_matches) {
        target_points.push_back(target_keypoints_[match.queryIdx].pt);
        test_points.push_back(test_keypoints[match.trainIdx].pt);
    }

    Mat homography_matrix, inlier_mask;
    homography_matrix = findHomography(
        target_points, test_points,
        RANSAC,
        ransac_reproj_threshold_,
        inlier_mask,
        ransac_max_iterations_,
        ransac_confidence_
    );

    if (homography_matrix.empty()) {
        cerr << "[T1CVOrb Error] Failed to compute homography matrix! Too many outliers." << endl;
        return false;
    }

    int inlier_count = countNonZero(inlier_mask);
    cout << "[T1CVOrb Info] Inliers after RANSAC filtering: " << inlier_count << endl;
    if (inlier_count < 4) {
        cerr << "[T1CVOrb Error] Insufficient inliers for accurate localization!" << endl;
        return false;
    }

    vector<Point2f> target_corners_original = {
        Point2f(0, 0),
        Point2f(target_gray_.cols, 0),
        Point2f(target_gray_.cols, target_gray_.rows),
        Point2f(0, target_gray_.rows)
    };
    perspectiveTransform(target_corners_original, target_corners, homography_matrix);

    drawMatches(
        target_color_, target_keypoints_,
        test_img, test_keypoints,
        good_matches, result_img,
        Scalar::all(-1), Scalar::all(-1),
        inlier_mask, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
    );

    int target_img_width = target_color_.cols;
    for (int i = 0; i < 4; i++) {
        line(
            result_img,
            target_corners[i] + Point2f(target_img_width, 0),
            target_corners[(i + 1) % 4] + Point2f(target_img_width, 0),
            Scalar(0, 0, 255),
            2
        );
    }

    cout << "\n[T1CVOrb Result] Target corners in test image (clockwise order):" << endl;
    for (int i = 0; i < 4; i++) {
        cout << "Corner " << i + 1 << ": (" << target_corners[i].x << ", " << target_corners[i].y << ")" << endl;
    }
    
    float angle = calculateTargetAngle(target_corners);
    float distance = calculateTargetDistance(target_corners);
    
    cout << "[T1CVOrb Result] Target angle: " << angle << " degrees" << endl;
    cout << "[T1CVOrb Result] Target distance: " << distance << " cm" << endl;
    
    string angle_text = "Angle: " + to_string(angle) + " deg";
    string distance_text = "Distance: " + to_string(distance) + " cm";
    
    Point text_pos(10, 30);
    putText(result_img, angle_text, text_pos, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
    putText(result_img, distance_text, Point(text_pos.x, text_pos.y + 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);

    return true;
}

bool T1CVOrb::save_res(const string& save_path, const Mat& result_img) {
    if (result_img.empty()) {
        cerr << "[T1CVOrb Error] Result image is empty! Cannot save." << endl;
        return false;
    }

    if (imwrite(save_path, result_img)) {
        cout << "[T1CVOrb Info] Result image saved to: " << save_path << endl;
        return true;
    } else {
        cerr << "[T1CVOrb Error] Failed to save result image! Path: " << save_path << endl;
        return false;
    }
}

void T1CVOrb::setRANSACParams(float reproj_threshold, int max_iter, float confidence) {
    ransac_reproj_threshold_ = reproj_threshold;
    ransac_max_iterations_ = max_iter;
    ransac_confidence_ = confidence;
    cout << "[T1CVOrb Info] RANSAC params updated: reproj_threshold=" << reproj_threshold 
         << ", max_iter=" << max_iter << ", confidence=" << confidence << endl;
}

void T1CVOrb::set_match_thresh(float strict_threshold, float relaxed_threshold) {
    match_ratio_strict_ = strict_threshold;
    match_ratio_relaxed_ = relaxed_threshold;
    cout << "[T1CVOrb Info] Match threshold updated: strict=" << strict_threshold 
         << ", relaxed=" << relaxed_threshold << endl;
}

float T1CVOrb::calculateTargetAngle(const vector<Point2f>& target_corners) {
    if (target_corners.size() < 4) {
        cerr << "[T1CVOrb Error] Insufficient corners for angle calculation!" << endl;
        return 0.0f;
    }
    
    float x1 = target_corners[0].x;  
    float x2 = target_corners[1].x;  
    
    float center_x = (x1 + x2) / 2.0f;
    opposite_side = (center_x - 960.0f) * 21.0f / (x2 - x1);
    adjacent_side = calculateTargetDistance(target_corners);

    if (adjacent_side <= 0) {
        cerr << "[T1CVOrb Error] Invalid distance for angle calculation!" << endl;
        return 0.0f;
    }
    
    float angle_rad = std::atan2(opposite_side, adjacent_side);
    angle_deg = angle_rad * 180.0f / CV_PI;
    return angle_deg;
}


float T1CVOrb::calculateTargetDistance(const vector<Point2f>& target_corners) {
    if (target_corners.size() < 4) {
        cerr << "[T1CVOrb Error] Insufficient corners for distance calculation!" << endl;
        return 0.0f;
    }
    
    float x1 = target_corners[0].x;  
    float x2 = target_corners[1].x;  
    float y1 = target_corners[0].y;  
    float y4 = target_corners[3].y;  
    
    float width = x2 - x1;
    float height = y4 - y1;
    if (width <= 0 || height <= 0) {
        cerr << "[T1CVOrb Error] Invalid target dimensions for distance calculation!" << endl;
        return 0.0f;
    }

    float reference_area = 220.0f * 240.0f;
    float actual_area = width * height;
    float calibration_factor = 100.0f; 
    
    float distance = calibration_factor * sqrt(reference_area / actual_area);
    
    adjacent_side = distance; 
    return distance;
}

int T1CVOrb::start(const cv::Mat& test_img, const std::string& obj_path){
    if (!setTargetImage(obj_path)) {
        cerr << "[T1CVOrb Error] Target image initialization failed. Path: " << obj_path << endl;
        return -1;
    }
    Mat result_image;
    vector<Point2f> target_corners;

    if (!detect_obj(test_img, result_image, target_corners)) {
        cerr << "[T1CVOrb Error] Object detection failed!" << endl;
        return -1;
    }
    
    save_res("orb_detection_result.jpg", result_image);

    return 0;
}

float T1CVOrb::getOppositeSide() const {
    return opposite_side;
}

float T1CVOrb::getAdjacentSide() const {
    return adjacent_side;
}

float T1CVOrb::getTargetAngle() const {
    return angle_deg;
}