#include "T1CVSift.hpp"
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

T1CVSift::T1CVSift(
    int nFeatures, int nOctaveLayers, double contrastThreshold,
    double edgeThreshold, double sigma
) {
    sift_detector_ = SIFT::create(
        nFeatures, nOctaveLayers, contrastThreshold,
        edgeThreshold, sigma
    );

    ransac_reproj_threshold_ = 5.0f;
    ransac_max_iterations_ = 2000;
    ransac_confidence_ = 0.995f;
    match_ratio_strict_ = 0.75f; 
    match_ratio_relaxed_ = 0.80f;
    max_good_matches_ = 80;
    
    opposite_side = 0.0f;
    adjacent_side = 0.0f;
    angle_deg = 0.0f;
}

bool T1CVSift::setTargetImage(const string& target_path) {
    target_color_ = imread(target_path);
    if (target_color_.empty()) {
        cerr << "[T1CVSift Error] Unable to read target image! Path:" << target_path << endl;
        return false;
    }

    cvtColor(target_color_, target_gray_, COLOR_BGR2GRAY);
    sift_detector_->detectAndCompute(target_gray_, noArray(), target_keypoints_, target_descriptors_);

    if (target_keypoints_.empty()) {
        cerr << "[T1CVSift Error] No key points detected in the target image! Please use an image with richer texture." << endl;
        return false;
    }

    cout << "[T1CVSift Info] Number of key points in the target image:" << target_keypoints_.size() << endl;
    return true;
}

bool T1CVSift::detect_obj(const string& test_path, Mat& result_img, vector<Point2f>& target_corners) {
    Mat test_color = imread(test_path);
    if (test_color.empty()) {
        cerr << "[T1CVSift Error] Unable to read test image! Path:" << test_path << endl;
        return false;
    }
    return detect_obj(test_color, result_img, target_corners);
}

bool T1CVSift::detect_obj(const Mat& test_img, Mat& result_img, vector<Point2f>& target_corners) {
    if (target_gray_.empty() || target_descriptors_.empty()) {
        cerr << "[T1CVSift Error] Please call setTargetImage() to initialize the target image first!" << endl;
        return false;
    }

    // Test image preprocessing
    Mat test_gray;
    cvtColor(test_img, test_gray, COLOR_BGR2GRAY);

    // Extract SIFT features from test image
    vector<KeyPoint> test_keypoints;
    Mat test_descriptors;
    sift_detector_->detectAndCompute(test_gray, noArray(), test_keypoints, test_descriptors);

    if (test_keypoints.empty()) {
        cerr << "[T1CVSift Error] No key points detected in the test image!" << endl;
        return false;
    }
    cout << "[T1CVSift Info] Number of key points in test image: " << test_keypoints.size() << endl;

    // SIFT descriptor matching (using Euclidean distance NORM_L2)
    BFMatcher matcher(NORM_L2);
    vector<vector<DMatch>> knn_matches;
    matcher.knnMatch(target_descriptors_, test_descriptors, knn_matches, 2);  // k=2 nearest neighbor matching

    // Filter good matches (strict threshold)
    vector<DMatch> good_matches;
    for (const auto& match_pair : knn_matches) {
        if (match_pair.size() == 2 && match_pair[0].distance < match_ratio_strict_ * match_pair[1].distance) {
            good_matches.push_back(match_pair[0]);
        }
    }
    cout << "[T1CVSift Info] Number of matches with strict threshold (" << match_ratio_strict_ << "): " << good_matches.size() << endl;

    // Use relaxed threshold if not enough matches
    if (good_matches.size() < 6) {
        cout << "[T1CVSift Warning] Insufficient matches, using relaxed threshold (" << match_ratio_relaxed_ << ")..." << endl;
        good_matches.clear();
        for (const auto& match_pair : knn_matches) {
            if (match_pair.size() == 2 && match_pair[0].distance < match_ratio_relaxed_ * match_pair[1].distance) {
                good_matches.push_back(match_pair[0]);
            }
        }
        if (good_matches.size() < 4) {
            cerr << "[T1CVSift Error] Still less than 4 matches after relaxed threshold, detection failed!" << endl;
            return false;
        }
        cout << "[T1CVSift Info] Number of matches with relaxed threshold: " << good_matches.size() << endl;
    }

    // Keep optimal matches (avoid too many matches affecting speed)
    if (good_matches.size() > max_good_matches_) {
        sort(good_matches.begin(), good_matches.end());  // Sort by distance (ascending)
        good_matches.resize(max_good_matches_);
        cout << "[T1CVSift Info] Keeping top " << max_good_matches_ << " optimal matches" << endl;
    }

    // Prepare matched point pairs (target points + test points)
    vector<Point2f> target_points, test_points;
    for (const auto& match : good_matches) {
        target_points.push_back(target_keypoints_[match.queryIdx].pt);  // Target image points
        test_points.push_back(test_keypoints[match.trainIdx].pt);      // Test image points
    }

    // Calculate homography matrix (RANSAC to filter outliers)
    Mat homography_matrix, inlier_mask;
    homography_matrix = findHomography(
        target_points, test_points,
        RANSAC,                  // RANSAC algorithm
        ransac_reproj_threshold_,// Reprojection error threshold
        inlier_mask,             // Inlier mask
        ransac_max_iterations_,  // Maximum iterations
        ransac_confidence_       // Confidence
    );

    if (homography_matrix.empty()) {
        cerr << "[T1CVSift Error] Homography matrix calculation failed! Too many outliers." << endl;
        return false;
    }

    // Count inlier number
    int inlier_count = countNonZero(inlier_mask);
    cout << "[T1CVSift Info] Number of inliers after RANSAC filtering: " << inlier_count << endl;
    if (inlier_count < 4) {
        cerr << "[T1CVSift Error] Insufficient inliers, inaccurate localization!" << endl;
        return false;
    }

    // Calculate target corners in test image (perspective transformation)
    vector<Point2f> target_corners_original = {
        Point2f(0, 0),                      // Top-left
        Point2f(target_gray_.cols, 0),      // Top-right
        Point2f(target_gray_.cols, target_gray_.rows),  // Bottom-right
        Point2f(0, target_gray_.rows)       // Bottom-left
    };
    perspectiveTransform(target_corners_original, target_corners, homography_matrix);

    // Draw matching results (horizontal concatenation: target image + test image + match lines)
    drawMatches(
        target_color_, target_keypoints_,
        test_img, test_keypoints,
        good_matches, result_img,
        Scalar::all(-1),  // Match line color (auto-assign)
        Scalar::all(-1),  // Unmatched points color (not drawn)
        inlier_mask,      // Only draw inlier matches
        DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS  // Do not draw single points
    );

    // Draw target bounding box (red solid line, line width 2)
    int target_img_width = target_color_.cols;  // Target image width (offset for test image in concatenation)
    for (int i = 0; i < 4; i++) {
        line(
            result_img,
            target_corners[i] + Point2f(target_img_width, 0),  // Corners in test image (offset by target width)
            target_corners[(i + 1) % 4] + Point2f(target_img_width, 0),
            Scalar(0, 0, 255),  // Red color
            2                   // Line width
        );
    }

    // Calculate target angle and distance
    float angle = calculateTargetAngle(target_corners);
    float distance = calculateTargetDistance(target_corners);
    
    // Output result information
    cout << "\n[T1CVSift Result] Target corners in test image (clockwise order):" << endl;
    for (int i = 0; i < 4; i++) {
        cout << "Corner " << i + 1 << ": (" << target_corners[i].x << ", " << target_corners[i].y << ")" << endl;
    }
    cout << "[T1CVSift Result] Target angle: " << angle << " degrees" << endl;
    cout << "[T1CVSift Result] Target distance: " << distance << " cm" << endl;

    // Draw angle and distance on result image (green text)
    string angle_text = "Angle: " + to_string(angle) + " deg";
    string distance_text = "Distance: " + to_string(distance) + " cm";
    Point text_pos(10, 30);  // Text position (top-left)
    putText(result_img, angle_text, text_pos, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
    putText(result_img, distance_text, Point(text_pos.x, text_pos.y + 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);

    return true;
}

// Save result image
bool T1CVSift::save_res(const string& save_path, const Mat& result_img) {
    if (result_img.empty()) {
        cerr << "[T1CVSift Error] Result image is empty, cannot save!" << endl;
        return false;
    }

    if (imwrite(save_path, result_img)) {
        cout << "[T1CVSift Info] Result image saved to: " << save_path << endl;
        return true;
    } else {
        cerr << "[T1CVSift Error] Failed to save result image! Path: " << save_path << endl;
        return false;
    }
}

// Set RANSAC parameters
void T1CVSift::setRANSACParams(float reproj_threshold, int max_iter, float confidence) {
    ransac_reproj_threshold_ = reproj_threshold;
    ransac_max_iterations_ = max_iter;
    ransac_confidence_ = confidence;
    cout << "[T1CVSift Info] RANSAC parameters updated: reproj_threshold=" << reproj_threshold 
         << ", max_iter=" << max_iter << ", confidence=" << confidence << endl;
}

// Set matching thresholds
void T1CVSift::set_match_thresh(float strict_threshold, float relaxed_threshold) {
    match_ratio_strict_ = strict_threshold;
    match_ratio_relaxed_ = relaxed_threshold;
    cout << "[T1CVSift Info] Matching thresholds updated: strict=" << strict_threshold 
         << ", relaxed=" << relaxed_threshold << endl;
}

// Calculate target angle (degrees)
float T1CVSift::calculateTargetAngle(const vector<Point2f>& target_corners) {
    if (target_corners.size() < 4) {
        cerr << "[T1CVSift Error] Insufficient corner points, cannot calculate angle!" << endl;
        return 0.0f;
    }
    
    float x1 = target_corners[0].x;  // Top-left x
    float x2 = target_corners[1].x;  // Top-right x
    float center_x = (x1 + x2) / 2.0f;  // Target center x
    opposite_side = (center_x - 960.0f) * 21.0f / (x2 - x1);  // Opposite side length (assuming image width 1920)
    adjacent_side = calculateTargetDistance(target_corners);  // Adjacent side length (distance)
    
    if (adjacent_side <= 0) {
        cerr << "[T1CVSift Error] Invalid distance, cannot calculate angle!" << endl;
        return 0.0f;
    }
    
    float angle_rad = atan2(opposite_side, adjacent_side);  // Radians
    angle_deg = angle_rad * 180.0f / CV_PI;  // Convert to degrees
    return angle_deg;
}

// Calculate target distance (cm)
float T1CVSift::calculateTargetDistance(const vector<Point2f>& target_corners) {
    if (target_corners.size() < 4) {
        cerr << "[T1CVSift Error] Insufficient corner points, cannot calculate distance!" << endl;
        return 0.0f;
    }
    
    float x1 = target_corners[0].x;  // Top-left x
    float x2 = target_corners[1].x;  // Top-right x
    float y1 = target_corners[0].y;  // Top-left y
    float y4 = target_corners[3].y;  // Bottom-left y
    
    float width = x2 - x1;  // Target width (pixels)
    float height = y4 - y1; // Target height (pixels)
    if (width <= 0 || height <= 0) {
        cerr << "[T1CVSift Error] Invalid target size, cannot calculate distance!" << endl;
        return 0.0f;
    }
    
    // Distance formula: calibration factor * sqrt(reference area / actual pixel area)
    float reference_area = 220.0f * 240.0f;  // Reference area (assuming target actual size 220x240mm)
    float actual_area = width * height;      // Actual pixel area
    float calibration_factor = 100.0f;       // Calibration factor (adjust according to actual scenario)
    
    float distance = calibration_factor * sqrt(reference_area / actual_area);
    adjacent_side = distance;  
    return distance;
}

// Start detection process
int T1CVSift::start(const Mat& test_img, const string& obj_path) {
    // Initialize target image
    if (!setTargetImage(obj_path)) {
        cerr << "[T1CVSift Error] Target image initialization failed! Path: " << obj_path << endl;
        return -1;
    }

    Mat result_image;
    vector<Point2f> target_corners;

    // Execute detection
    if (!detect_obj(test_img, result_image, target_corners)) {
        cerr << "[T1CVSift Error] Target detection failed!" << endl;
        return -1;
    }
    
    // Save result
    save_res("sift_detection_result.jpg", result_image);
    return 0;
}

float T1CVSift::getOppositeSide() const {
    return opposite_side;
}

float T1CVSift::getAdjacentSide() const {
    return adjacent_side;
}

float T1CVSift::getTargetAngle() const {
    return angle_deg;
}
