#include "T1CVSift2.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

using namespace cv;
using namespace std;

T1CVSift2::T1CVSift2(
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

bool T1CVSift2::setTargetImage(const string& target_path) {
    target_color_ = imread(target_path);
    if (target_color_.empty()) {
        cerr << "[T1CVSift2 Error] Unable to read target image! Path:" << target_path << endl;
        return false;
    }

    cvtColor(target_color_, target_gray_, COLOR_BGR2GRAY);
    sift_detector_->detectAndCompute(target_gray_, noArray(), target_keypoints_, target_descriptors_);

    if (target_keypoints_.empty()) {
        cerr << "[T1CVSift2 Error] No key points detected in the target image! Please use an image with richer texture." << endl;
        return false;
    }

    cout << "[T1CVSift2 Info] Number of key points in the target image:" << target_keypoints_.size() << endl;
    return true;
}

bool T1CVSift2::setTargetImageYUV(const string& target_path, int test_width, int test_height) {
    // Open YUV file
    ifstream yuv_file(target_path, ios::binary);
    if (!yuv_file.is_open()) {
        cerr << "[T1CVSift2 Error] Unable to open YUV target image! Path:" << target_path << endl;
        return false;
    }

    // Get file size
    yuv_file.seekg(0, ios::end);
    streampos file_size = yuv_file.tellg();
    yuv_file.seekg(0, ios::beg);
    cerr << "[T1CVSift2 Debug] YUV file size: " << file_size << " bytes" << endl;

    // Calculate the number of pixels (NV12 format: 1.5 bytes per pixel)
    long long pixels = static_cast<long long>(file_size) * 2 / 3;
    if (pixels <= 0) {
        cerr << "[T1CVSift2 Error] Invalid YUV file size!" << endl;
        yuv_file.close();
        return false;
    }

    // Try to find appropriate width and height based on pixel count
    // This is a simple approach, you may need to adjust it based on your actual YUV format
    int width = static_cast<int>(sqrt(pixels));
    int height = static_cast<int>(pixels / width);
    
    // Adjust if needed to get exact pixel count
    while (width * height != pixels) {
        if (width * height < pixels) {
            width++;
        } else {
            height--;
        }
    }
    
    cerr << "[T1CVSift2 Debug] Automatically calculated YUV template size: " << width << "x" << height << endl;

    // Calculate YUV data size
    int data_size = width * height * 3 / 2;
    vector<uint8_t> yuv_data(data_size);

    // Read YUV data
    yuv_file.read(reinterpret_cast<char*>(yuv_data.data()), data_size);
    if (!yuv_file) {
        cerr << "[T1CVSift2 Error] Failed to read YUV data from file! Path:" << target_path << endl;
        cerr << "[T1CVSift2 Debug] Bytes read: " << yuv_file.gcount() << endl;
        yuv_file.close();
        return false;
    }
    yuv_file.close();

    // Create NV12 format Mat
    Mat nv12_mat(height * 3 / 2, width, CV_8UC1, yuv_data.data());

    // Extract Y channel as grayscale
    target_gray_ = Mat(height, width, CV_8UC1);
    for (int i = 0; i < height; i++) {
        memcpy(target_gray_.ptr(i), nv12_mat.ptr(i), width);
    }

    // Convert NV12 to BGR for visualization
    Mat yuv_mat(height * 3 / 2, width, CV_8UC1, yuv_data.data());
    cvtColor(yuv_mat, target_color_, COLOR_YUV2BGR_NV12);

    // Detect SIFT features
    sift_detector_->detectAndCompute(target_gray_, noArray(), target_keypoints_, target_descriptors_);

    if (target_keypoints_.empty()) {
        cerr << "[T1CVSift2 Error] No key points detected in the YUV target image! Please use an image with richer texture." << endl;
        return false;
    }

    cout << "[T1CVSift2 Info] Number of key points in the YUV target image:" << target_keypoints_.size() << endl;
    return true;
}

bool T1CVSift2::detect_obj(const string& test_path, Mat& result_img, vector<Point2f>& target_corners) {
    Mat test_color = imread(test_path);
    if (test_color.empty()) {
        cerr << "[T1CVSift2 Error] Unable to read test image! Path:" << test_path << endl;
        return false;
    }
    return detect_obj(test_color, result_img, target_corners);
}

bool T1CVSift2::detect_obj(const Mat& test_img, Mat& result_img, vector<Point2f>& target_corners) {
    if (target_gray_.empty() || target_descriptors_.empty()) {
        cerr << "[T1CVSift2 Error] Please call setTargetImage() to initialize the target image first!" << endl;
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
        cerr << "[T1CVSift2 Error] No key points detected in the test image!" << endl;
        return false;
    }
    cout << "[T1CVSift2 Info] Number of key points in test image: " << test_keypoints.size() << endl;

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
    cout << "[T1CVSift2 Info] Number of matches with strict threshold (" << match_ratio_strict_ << "): " << good_matches.size() << endl;

    // Use relaxed threshold if not enough matches
    if (good_matches.size() < 6) {
        cout << "[T1CVSift2 Warning] Insufficient matches, using relaxed threshold (" << match_ratio_relaxed_ << ")..." << endl;
        good_matches.clear();
        for (const auto& match_pair : knn_matches) {
            if (match_pair.size() == 2 && match_pair[0].distance < match_ratio_relaxed_ * match_pair[1].distance) {
                good_matches.push_back(match_pair[0]);
            }
        }
        if (good_matches.size() < 4) {
            cerr << "[T1CVSift2 Error] Still less than 4 matches after relaxed threshold, detection failed!" << endl;
            return false;
        }
        cout << "[T1CVSift2 Info] Number of matches with relaxed threshold: " << good_matches.size() << endl;
    }

    // Keep optimal matches (avoid too many matches affecting speed)
    if (good_matches.size() > max_good_matches_) {
        sort(good_matches.begin(), good_matches.end());  // Sort by distance (ascending)
        good_matches.resize(max_good_matches_);
        cout << "[T1CVSift2 Info] Keeping top " << max_good_matches_ << " optimal matches" << endl;
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
        cerr << "[T1CVSift2 Error] Homography matrix calculation failed! Too many outliers." << endl;
        return false;
    }

    // Count inlier number
    int inlier_count = countNonZero(inlier_mask);
    cout << "[T1CVSift2 Info] Number of inliers after RANSAC filtering: " << inlier_count << endl;
    if (inlier_count < 4) {
        cerr << "[T1CVSift2 Error] Insufficient inliers, inaccurate localization!" << endl;
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
    cout << "\n[T1CVSift2 Result] Target corners in test image (clockwise order):" << endl;
    for (int i = 0; i < 4; i++) {
        cout << "Corner " << i + 1 << ": (" << target_corners[i].x << ", " << target_corners[i].y << ")" << endl;
    }
    cout << "[T1CVSift2 Result] Target angle: " << angle << " degrees" << endl;
    cout << "[T1CVSift2 Result] Target distance: " << distance << " cm" << endl;

    // Draw angle and distance on result image (green text)
    string angle_text = "Angle: " + to_string(angle) + " deg";
    string distance_text = "Distance: " + to_string(distance) + " cm";
    Point text_pos(10, 30);  // Text position (top-left)
    putText(result_img, angle_text, text_pos, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
    putText(result_img, distance_text, Point(text_pos.x, text_pos.y + 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);

    return true;
}

// Extract Y channel from NV12 image as grayscale
Mat T1CVSift2::extractYChannelFromNV12(const Mat& nv12_img, int width, int height) {
    // Create a new grayscale image for Y channel
    Mat y_channel(height, width, CV_8UC1);
    
    // Copy Y channel directly from NV12 image (first part of NV12 data)
    for (int i = 0; i < height; i++) {
        memcpy(y_channel.ptr(i), nv12_img.ptr(i), width);
    }
    
    return y_channel;
}

// NV12 format detection function
bool T1CVSift2::detect_obj_nv12(const Mat& nv12_img, int width, int height, Mat& result_img, vector<Point2f>& target_corners) {
    if (target_gray_.empty() || target_descriptors_.empty()) {
        cerr << "[T1CVSift2 Error] Please call setTargetImage() to initialize the target image first!" << endl;
        return false;
    }

    // Check NV12 image size
    int expected_size = width * height * 3 / 2;  // NV12 format size is 1.5 bytes per pixel
    if (nv12_img.total() < expected_size) {
        cerr << "[T1CVSift2 Error] Invalid NV12 image size! Expected: " << expected_size << ", Got: " << nv12_img.total() << endl;
        return false;
    }

    // Extract Y channel as grayscale (direct copy without conversion)
    Mat test_gray = extractYChannelFromNV12(nv12_img, width, height);
    
    // For visualization, convert to BGR
    Mat test_bgr;
    cvtColor(nv12_img, test_bgr, COLOR_YUV2BGR_NV12);

    // Extract SIFT features from test image
    vector<KeyPoint> test_keypoints;
    Mat test_descriptors;
    sift_detector_->detectAndCompute(test_gray, noArray(), test_keypoints, test_descriptors);

    if (test_keypoints.empty()) {
        cerr << "[T1CVSift2 Error] No key points detected in the test image!" << endl;
        return false;
    }
    cout << "[T1CVSift2 Info] Number of key points in test image: " << test_keypoints.size() << endl;

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
    cout << "[T1CVSift2 Info] Number of matches with strict threshold (" << match_ratio_strict_ << "): " << good_matches.size() << endl;

    // Use relaxed threshold if not enough matches
    if (good_matches.size() < 6) {
        cout << "[T1CVSift2 Warning] Insufficient matches, using relaxed threshold (" << match_ratio_relaxed_ << ")..." << endl;
        good_matches.clear();
        for (const auto& match_pair : knn_matches) {
            if (match_pair.size() == 2 && match_pair[0].distance < match_ratio_relaxed_ * match_pair[1].distance) {
                good_matches.push_back(match_pair[0]);
            }
        }
        if (good_matches.size() < 4) {
            cerr << "[T1CVSift2 Error] Still less than 4 matches after relaxed threshold, detection failed!" << endl;
            return false;
        }
        cout << "[T1CVSift2 Info] Number of matches with relaxed threshold: " << good_matches.size() << endl;
    }

    // Keep optimal matches (avoid too many matches affecting speed)
    if (good_matches.size() > max_good_matches_) {
        sort(good_matches.begin(), good_matches.end());  // Sort by distance (ascending)
        good_matches.resize(max_good_matches_);
        cout << "[T1CVSift2 Info] Keeping top " << max_good_matches_ << " optimal matches" << endl;
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
        cerr << "[T1CVSift2 Error] Homography matrix calculation failed! Too many outliers." << endl;
        return false;
    }

    // Count inlier number
    int inlier_count = countNonZero(inlier_mask);
    cout << "[T1CVSift2 Info] Number of inliers after RANSAC filtering: " << inlier_count << endl;
    if (inlier_count < 4) {
        cerr << "[T1CVSift2 Error] Insufficient inliers, inaccurate localization!" << endl;
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
        test_bgr, test_keypoints,
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
    float distanceX = calculateTargetDistX(target_corners);
    float distanceY = calculateTargetDistance(target_corners);
    
    // Output result information
    cout << "\n[T1CVSift2 Result] Target corners in test image (clockwise order):" << endl;
    for (int i = 0; i < 4; i++) {
        cout << "Corner " << i + 1 << ": (" << target_corners[i].x << ", " << target_corners[i].y << ")" << endl;
    }
    cout << "[T1CVSift2 Result] Target angle: " << distanceX << " cm" << endl;
    cout << "[T1CVSift2 Result] Target distance: " << distanceY << " cm" << endl;

    // Draw angle and distance on result image (green text)
    string angle_text = "Angle: " + to_string(distanceX) + " cm";
    string distance_text = "Distance: " + to_string(distanceY) + " cm";
    Point text_pos(10, 30);  // Text position (top-left)
    putText(result_img, angle_text, text_pos, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
    putText(result_img, distance_text, Point(text_pos.x, text_pos.y + 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);

    return true;
}

// Save result image
bool T1CVSift2::save_res(const string& save_path, const Mat& result_img) {
    if (result_img.empty()) {
        cerr << "[T1CVSift2 Error] Result image is empty, cannot save!" << endl;
        return false;
    }

    // Get current time
    time_t now = time(0);
    struct tm* tstruct;
    char time_str[80];
    tstruct = localtime(&now);
    strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", tstruct);

    // Extract filename and extension
    size_t last_dot = save_path.find_last_of('.');
    string filename = save_path.substr(save_path.find_last_of('/') + 1, last_dot - save_path.find_last_of('/') - 1);
    string extension = save_path.substr(last_dot);

    // Create new filename with timestamp
    string save_dir = "../image";
    string new_filename = filename + "_" + time_str + extension;
    string full_path = save_dir + "/" + new_filename;

    if (imwrite(full_path, result_img)) {
        cout << "[T1CVSift2 Info] Result image saved to: " << full_path << endl;
        return true;
    } else {
        cerr << "[T1CVSift2 Error] Failed to save result image! Path: " << full_path << endl;
        return false;
    }
}

// Set RANSAC parameters
void T1CVSift2::setRANSACParams(float reproj_threshold, int max_iter, float confidence) {
    ransac_reproj_threshold_ = reproj_threshold;
    ransac_max_iterations_ = max_iter;
    ransac_confidence_ = confidence;
    cout << "[T1CVSift2 Info] RANSAC parameters updated: reproj_threshold=" << reproj_threshold 
         << ", max_iter=" << max_iter << ", confidence=" << confidence << endl;
}

// Set matching thresholds
void T1CVSift2::set_match_thresh(float strict_threshold, float relaxed_threshold) {
    match_ratio_strict_ = strict_threshold;
    match_ratio_relaxed_ = relaxed_threshold;
    cout << "[T1CVSift2 Info] Matching thresholds updated: strict=" << strict_threshold 
         << ", relaxed=" << relaxed_threshold << endl;
}

// Calculate target angle (degrees)
float T1CVSift2::calculateTargetAngle(const vector<Point2f>& target_corners) {
    if (target_corners.size() < 4) {
        cerr << "[T1CVSift2 Error] Insufficient corner points, cannot calculate angle!" << endl;
        return 0.0f;
    }
    
    float x1 = target_corners[0].x;  // Top-left x
    float x2 = target_corners[1].x;  // Top-right x
    float center_x = (x1 + x2) / 2.0f;  // Target center x
    opposite_side = (center_x - 960.0f) * 21.0f / (x2 - x1);  // Opposite side length (assuming image width 1920)
    adjacent_side = calculateTargetDistance(target_corners);  // Adjacent side length (distance)
    
    if (adjacent_side <= 0) {
        cerr << "[T1CVSift2 Error] Invalid distance, cannot calculate angle!" << endl;
        return 0.0f;
    }
    
    float angle_rad = atan2(opposite_side, adjacent_side);  // Radians
    angle_deg = angle_rad * 180.0f / CV_PI;  // Convert to degrees
    return angle_deg;
}

float T1CVSift2::calculateTargetDistX(const vector<Point2f>& target_corners) {
    if (target_corners.size() < 4) {
        cerr << "[T1CVSift2 Error] Insufficient corner points, cannot calculate angle!" << endl;
        return 0.0f;
    }
    
    float x1 = target_corners[0].x;  // Top-left x
    float x2 = target_corners[1].x;  // Top-right x
    float center_x = (x1 + x2) / 2.0f;  // Target center x
    opposite_side = (center_x - 960.0f) * 21.0f / (x2 - x1);  // Opposite side length (assuming image width 1920)

    return opposite_side;
}

// Calculate target distance (cm)
float T1CVSift2::calculateTargetDistance(const vector<Point2f>& target_corners) {
    if (target_corners.size() < 4) {
        cerr << "[T1CVSift2 Error] Insufficient corner points, cannot calculate distance!" << endl;
        return 0.0f;
    }
    
    float x1 = target_corners[0].x;  // Top-left x
    float x2 = target_corners[1].x;  // Top-right x
    float y1 = target_corners[0].y;  // Top-left y
    float y4 = target_corners[3].y;  // Bottom-left y
    
    float width = x2 - x1;  // Target width (pixels)
    float height = y4 - y1; // Target height (pixels)
    if (width <= 0 || height <= 0) {
        cerr << "[T1CVSift2 Error] Invalid target size, cannot calculate distance!" << endl;
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
int T1CVSift2::start(const Mat& test_img, const string& obj_path) {
    // Initialize target image
    if (!setTargetImage(obj_path)) {
        cerr << "[T1CVSift2 Error] Target image initialization failed! Path: " << obj_path << endl;
        return -1;
    }

    Mat result_image;
    vector<Point2f> target_corners;

    // Execute detection
    if (!detect_obj(test_img, result_image, target_corners)) {
        cerr << "[T1CVSift2 Error] Target detection failed!" << endl;
        return -1;
    }
    
    // Save result
    save_res("sift2_detection_result.jpg", result_image);
    return 0;
}

// NV12 format start function
int T1CVSift2::start_nv12(const Mat& nv12_img, int width, int height, const string& obj_path) {
    // Initialize target image using YUV format
    if (!setTargetImageYUV(obj_path, width, height)) {
        cerr << "[T1CVSift2 Error] YUV target image initialization failed! Path: " << obj_path << endl;
        return -1;
    }

    Mat result_image;
    vector<Point2f> target_corners;

    // Execute NV12 detection
    if (!detect_obj_nv12(nv12_img, width, height, result_image, target_corners)) {
        cerr << "[T1CVSift2 Error] Target detection failed!" << endl;
        return -1;
    }
    
    // Save result
    save_res("sift2_detection_result_nv12.jpg", result_image);
    return 0;
}

// NV12 format target recognition function
int T1CVSift2::Recognize(const Mat& nv12_img, int width, int height, const string& obj_path) {
    cv::Mat nv12_img_crop;
    // ========== Core Crop Configuration - Crop ROI (480,520) with size 960x540 ==========
    const int crop_x = 480;    // Crop start X coordinate
    const int crop_y = 0;    // Crop start Y coordinate (your correct value)
    const int crop_w = 960;    // Cropped image width
    const int crop_h = 540;    // Cropped image height

    // Automatically calculate the maximum safe crop Y coordinate to avoid out of bounds
    int max_safe_crop_y = height - crop_h;
    cerr << "[T1CVSift2 Info] Max safe crop Y coordinate: " << max_safe_crop_y << endl;

    // Mandatory safety check: Verify crop ROI is within the original image boundary
    bool is_out_of_bounds = false;
    if(crop_x < 0 || crop_y < 0 || (crop_x + crop_w) > width || (crop_y + crop_h) > height) {
        is_out_of_bounds = true;
        cerr << "[T1CVSift2 Warn] Crop params out of bounds! Src image size(" << width << "," << height 
             << "), Crop area(" << crop_x << "," << crop_y << "," << crop_w << "," << crop_h << ")" << endl;
        cerr << "[T1CVSift2 Warn] Reason: crop coordinate + crop size exceeds the original image resolution" << endl;
        return -1; // Terminate function immediately if out of bounds, prevent program crash
    }

    // Correct NV12 lossless crop method: Crop Y and UV channel synchronously, avoid color distortion
    // NV12 format specification: Single-channel YUV420SP, Y channel first (w*h), UV channel follows (w*h/2)
    // Requirement: Input must be 1920x1080 NV12 single-channel Mat
    if (!nv12_img.empty() && nv12_img.channels() == 1 && width == 1920 && height == 1080) {
        // Step 1: Crop Y channel (Luminance) - direct crop by original coordinate
        cv::Rect rect_Y(crop_x, crop_y, crop_w, crop_h);
        cv::Mat mat_Y_crop = nv12_img(rect_Y);

        // Step 2: Crop UV channel (Chrominance) - UV channel starts at height row of source image
        // UV channel resolution is 1/2 of Y channel, so crop_y need to divide by 2 for equal scaling
        cv::Rect rect_UV(crop_x, height + (crop_y/2), crop_w, crop_h/2);
        cv::Mat mat_UV_crop = nv12_img(rect_UV);

        // Step 3: Merge cropped Y and UV channels to generate final NV12 format image
        nv12_img_crop.create(crop_h * 3 / 2, crop_w, nv12_img.type());
        mat_Y_crop.copyTo(nv12_img_crop(cv::Rect(0, 0, crop_w, crop_h)));
        mat_UV_crop.copyTo(nv12_img_crop(cv::Rect(0, crop_h, crop_w, crop_h/2)));
    } else {
        cerr << "[T1CVSift2 Error] Invalid input image! Require 1920x1080 NV12 single-channel Mat only!" << endl;
        return -1;
    }

    // Initialize target template image with YUV format, pass cropped image size
    if (!setTargetImageYUV(obj_path, crop_w, crop_h)) {
        cerr << "[T1CVSift2 Error] YUV target image initialization failed! File path: " << obj_path << endl;
        return -1;
    }

    Mat result_image;
    vector<Point2f> target_corners;

    // Execute target detection on the cropped NV12 image
    if (!detect_obj_nv12(nv12_img_crop, crop_w, crop_h, result_image, target_corners)) {
        cerr << "[T1CVSift2 Error] Target detection failed in cropped NV12 image!" << endl;
        return -1;
    }

    // Calculate aspect ratio filter based on template size (220x210)
    const int template_width = 220;
    const int template_height = 210;
    const double template_aspect_ratio = static_cast<double>(template_width) / template_height;
    const double aspect_ratio_tolerance = 0.10; // 10% tolerance for aspect ratio

    // Calculate bounding box from detected corners
    float min_x = target_corners[0].x, max_x = target_corners[0].x;
    float min_y = target_corners[0].y, max_y = target_corners[0].y;
    for (int i = 1; i < 4; i++) {
        min_x = min(min_x, target_corners[i].x);
        max_x = max(max_x, target_corners[i].x);
        min_y = min(min_y, target_corners[i].y);
        max_y = max(max_y, target_corners[i].y);
    }

    float detected_width = max_x - min_x;
    float detected_height = max_y - min_y;
    double detected_aspect_ratio = detected_width / detected_height;

    cout << "[T1CVSift2 Info] Template aspect ratio: " << template_aspect_ratio 
         << " (size: " << template_width << "x" << template_height << ")" << endl;
    cout << "[T1CVSift2 Info] Detected aspect ratio: " << detected_aspect_ratio 
         << " (size: " << detected_width << "x" << detected_height << ")" << endl;

    // Check if aspect ratio is within acceptable range
    double aspect_ratio_diff = abs(detected_aspect_ratio - template_aspect_ratio) / template_aspect_ratio;
    if (aspect_ratio_diff > aspect_ratio_tolerance) {
        cerr << "[T1CVSift2 Error] Aspect ratio mismatch! Difference: " << (aspect_ratio_diff * 100) 
             << "% exceeds tolerance " << (aspect_ratio_tolerance * 100) << "%" << endl;
        cerr << "[T1CVSift2 Error] Expected aspect ratio: " << template_aspect_ratio 
             << ", Detected: " << detected_aspect_ratio << endl;
        return -1;
    }

    cout << "[T1CVSift2 Info] Aspect ratio check passed! Difference: " << (aspect_ratio_diff * 100) << "%" << endl;
    
    // Save the final detection result image
    save_res("sift2_Recognition_result_nv12.jpg", result_image); 
    return 0;
}

float T1CVSift2::getOppositeSide() const {
    return opposite_side;
}

float T1CVSift2::getAdjacentSide() const {
    return adjacent_side;
}

float T1CVSift2::getTargetAngle() const {
    return angle_deg;
}
