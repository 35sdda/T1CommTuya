#include "T1ObsDisCalcu.hpp"
#include <iostream>
#include <algorithm>

int debugObs=0;

T1ObsDisCalcu::T1ObsDisCalcu() {
}

T1ObsDisCalcu::~T1ObsDisCalcu() {
}

//use CV_32FC3 and Calculate the distance of all detected objects in the depth map
std::vector<DetectedObject> T1ObsDisCalcu::calcuObsDist(
    const cv::Mat& depth_map,
    const std::vector<std::string>& class_names,
    const std::vector<cv::Rect2d>& bounding_boxes,
    const std::vector<float>& confidences) {
    
    std::vector<DetectedObject> results;
    
    if (depth_map.empty() || class_names.empty() || bounding_boxes.empty() || 
        class_names.size() != bounding_boxes.size() ||  
        bounding_boxes.size() != confidences.size()) {
        std::cerr << "invariant" << std::endl;
        return results;
    }
    
    if (depth_map.type() != CV_32F && depth_map.type() != CV_16U && depth_map.type() != CV_32FC3) {
        std::cerr << "error,CV_32F.CV_16U or CV_32FC3" << std::endl;
        return results;
    }
    for (size_t i = 0; i < bounding_boxes.size(); ++i) {
        DetectedObject obj;
        obj.class_name = class_names[i];
        obj.confidence = confidences[i];
        obj.bounding_box = bounding_boxes[i];
        
        obj.distance = getAvgDistInRegion(depth_map, bounding_boxes[i]);
        
        results.push_back(obj);
    }
    
    return results;
}

// Calculate the distance of all detected objects in the depth map
double T1ObsDisCalcu::getAvgDistInRegion(const cv::Mat& depth_map, const cv::Rect2d& region) {
    std::cout << "type: " << depth_map.type() << ", size: " << depth_map.cols << "x" << depth_map.rows << std::endl;
    std::cout << "region: x=" << region.x << ", y=" << region.y << ", width=" << region.width << ", height=" << region.height << std::endl;
    
    cv::Rect region_int(static_cast<int>(region.x), static_cast<int>(region.y),
                        static_cast<int>(region.width), static_cast<int>(region.height));
    cv::Rect valid_region = region_int & cv::Rect(0, 0, depth_map.cols, depth_map.rows);
    if (valid_region.empty()) {
        if (debugObs) std::cout << "warnning:no valid region" << std::endl;
        return 0.0;
    }
    
    double total_distance = 0.0;
    int valid_count = 0;
    
    for (int y = valid_region.y; y < valid_region.y + valid_region.height; ++y) {
        for (int x = valid_region.x; x < valid_region.x + valid_region.width; ++x) {
            double distance = getPointDist(depth_map, x, y);
            if (distance > 0.0 && std::isfinite(distance)) {
                total_distance += distance;
                valid_count++;
                
                if (x % 10 == 0 && y % 10 == 0) {
                    if (debugObs) std::cout << "(" << x << ", " << y << "): " << distance << "m (valid)" << std::endl;
                }
            } else if (x % 10 == 0 && y % 10 == 0) {
                if (debugObs) std::cout << "(" << x << ", " << y << "): " << distance << "m (filtered)" << std::endl;
            }
        }
    }
    
    std::cout << "valid pixels: " << valid_count << std::endl;
    
    if (valid_count > 0) {
        double avg_distance = total_distance / valid_count;
        if (debugObs) std::cout << "average distance: " << avg_distance << "m" << std::endl;
        return avg_distance;
    } else {
        if (debugObs) std::cout << "warning: no valid pixels in distance range" << std::endl;
        return 0.0; 
    }
}

//Calculate the distance of all detected objects in the depth map
double T1ObsDisCalcu::getPointDist(const cv::Mat& depth_map, int x, int y) {
    if (x < 0 || x >= depth_map.cols || y < 0 || y >= depth_map.rows) {
        return 0.0;
    }
    
    if (depth_map.type() == CV_32F) {
        float depth = depth_map.at<float>(y, x);
        if (depth > 0.0) { // If depth is valid
            return depth / 1000.0; // Convert to meters (assuming depth is in millimeters)
        }
        return 0.0;
    } else if (depth_map.type() == CV_16U) {
        return static_cast<double>(depth_map.at<unsigned short>(y, x)) / 1000.0;
    } else if (depth_map.type() == CV_32FC3) {
        cv::Vec3f point = depth_map.at<cv::Vec3f>(y, x);
        if (point[2] > 0.0) { // If depth is valid
            return point[2] / 1000.0; // Convert to meters (assuming depth is in millimeters)
        }
        return 0.0;
    }
    
    return 0.0;
}


// Calculate the average depth for specified object ROI regions
double T1ObsDisCalcu::calcObjAvgDepth(const cv::Mat& depth_map, const std::vector<cv::Rect2d>& bounding_boxes, const std::vector<std::string>& class_names, const std::string& target_class) {
    double total_distance = 0.0;
    int valid_count = 0;
    
    for (size_t i = 0; i < bounding_boxes.size(); ++i) {
        if (class_names[i] == target_class) {
            double avg_dist = getAvgDistInRegion(depth_map, bounding_boxes[i]);
            if (avg_dist > 0.0) {
                total_distance += avg_dist;
                valid_count++;
            }
        }
    }
    
    if (valid_count > 0) {
        double avg_distance = total_distance / valid_count;
        std::cout << "Average depth for " << target_class << ": " << avg_distance << "m" << std::endl;
        return avg_distance;
    } else {
        std::cout << "No valid " << target_class << " detected" << std::endl;
        return 0.0;
    }
}

// Draw depth histogram
void T1ObsDisCalcu::calcHistDraw(const std::vector<int>& histogram, int bin_cm, int x, int y, int w, int h) {
    int bin_count = histogram.size();
    if (bin_count == 0) {
        return;
    }
    
    // Create histogram image and display with floating axes
    int margin = 80; // Floating margin for axes
    int hist_w = 1024;
    int hist_h = 600;
    int plot_w = hist_w - margin * 2; // Plot area width
    int plot_h = hist_h - margin * 2; // Plot area height
    int bin_w = cvRound(static_cast<double>(plot_w) / bin_count);
    if (bin_w < 1) bin_w = 1; // Ensure minimum width
    
    cv::Mat hist_image(hist_h, hist_w, CV_8UC3, cv::Scalar(20, 20, 20));
    
    // Find max value for normalization
    int max_count = *std::max_element(histogram.begin(), histogram.end());
    
    // Draw floating axes
    cv::line(hist_image, cv::Point(margin, margin), cv::Point(margin, hist_h - margin), cv::Scalar(200, 200, 200), 2); // Y axis
    cv::line(hist_image, cv::Point(margin, hist_h - margin), cv::Point(hist_w - margin, hist_h - margin), cv::Scalar(200, 200, 200), 2); // X axis
    
    // Draw histogram bars (skip empty bins, merge adjacent non-empty bins if too thin)
    int merge_bins = (bin_w < 15) ? 2 : 1; // Merge bins if too thin
    int display_bin_w = bin_w * merge_bins;
    
    for (int i = 0; i < bin_count; i += merge_bins) {
        // Calculate merged height
        int merged_count = 0;
        for (int j = 0; j < merge_bins && (i + j) < bin_count; ++j) {
            merged_count += histogram[i + j];
        }
        
        if (merged_count > 0) {
            int bin_height = cvRound(static_cast<double>(merged_count) / max_count * (plot_h - 20));
            int x_start = margin + i * bin_w;
            int y_start = hist_h - margin - bin_height;
            
            // Draw thicker bar
            cv::rectangle(hist_image, 
                          cv::Point(x_start + 2, y_start),
                          cv::Point(x_start + display_bin_w - 2, hist_h - margin),
                          cv::Scalar(0, 255, 0), 
                          cv::FILLED);
            
            // Draw frequency label on top of bar
            std::string freq = std::to_string(merged_count);
            cv::putText(hist_image, freq, cv::Point(x_start + 4, y_start - 5), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
        }
    }
    
    // Draw X-axis tick marks and labels
    int tick_interval = (bin_count > 20) ? 4 : 2;
    for (int i = 0; i < bin_count; i += tick_interval) {
        int x_pos = margin + i * bin_w;
        // Tick mark
        cv::line(hist_image, cv::Point(x_pos, hist_h - margin), cv::Point(x_pos, hist_h - margin + 5), cv::Scalar(200, 200, 200), 1);
        // Label
        int start_cm = i * bin_cm;
        std::string label = std::to_string(start_cm);
        cv::putText(hist_image, label, cv::Point(x_pos - 10, hist_h - margin + 25), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
    
    // Draw Y-axis tick marks and labels
    int y_ticks = 5;
    for (int i = 0; i <= y_ticks; ++i) {
        int y_pos = hist_h - margin - (plot_h * i / y_ticks);
        // Tick mark
        cv::line(hist_image, cv::Point(margin - 5, y_pos), cv::Point(margin, y_pos), cv::Scalar(200, 200, 200), 1);
        // Label
        int freq_val = max_count * i / y_ticks;
        std::string label = std::to_string(freq_val);
        cv::putText(hist_image, label, cv::Point(margin - 50, y_pos + 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
    
    // Title
    std::string title = "Depth Histogram (" + std::to_string(bin_cm) + "cm/bin, Region: " + std::to_string(x) + "," + std::to_string(y) + " " + std::to_string(w) + "x" + std::to_string(h) + ")";
    cv::putText(hist_image, title, cv::Point(hist_w/2 - 200, 40), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    // Axis labels
    cv::putText(hist_image, "Distance (cm)", cv::Point(hist_w/2 - 50, hist_h - 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
    cv::putText(hist_image, "Count", cv::Point(20, hist_h/2), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
    
    // Show histogram window
    cv::imshow("Depth Histogram", hist_image);
    cv::moveWindow("Depth Histogram", 680, 0);
}

// Calculate depth histogram for specified region with bin_cm per segment, display in real-time
int T1ObsDisCalcu::calcHist(const cv::Mat& depth_map, int x, int y, int w, int h, int bin_cm, int avoidThresh, int avoidFreq) {
    std::vector<int> histogram;
    
    // Ensure region is within image bounds
    cv::Rect region(x, y, w, h);
    cv::Rect valid_region = region & cv::Rect(0, 0, depth_map.cols, depth_map.rows);
    if (valid_region.empty()) {
        std::cerr << "T1ObsDisCalcu-calcHist Invalid, region for histogram" << std::endl;
        return 0;
    }
    
    // Collect all valid depth values (convert to cm)
    std::vector<float> depths_cm;
    float max_depth_cm = 0;
    
    for (int row = valid_region.y; row < valid_region.y + valid_region.height; ++row) {
        for (int col = valid_region.x; col < valid_region.x + valid_region.width; ++col) {
            double dist_m = getPointDist(depth_map, col, row);
            if (dist_m > 0.0 && std::isfinite(dist_m)) {
                float dist_cm = dist_m * 100.0f;
                depths_cm.push_back(dist_cm);
                if (dist_cm > max_depth_cm) {
                    max_depth_cm = dist_cm;
                }
            }
        }
    }
    
    if (depths_cm.empty()) {
        //std::cout << "T1ObsDisCalcu-calcHist Invalid, No valid depth data in region" << std::endl;
        return 0;
    }
    
    // Calculate number of bins for histogram
    int bin_count = static_cast<int>(max_depth_cm / bin_cm) + 1;
    histogram.resize(bin_count, 0);
    
    // Count pixels in each bin
    int freq_above_threshold = 0;
    for (float depth : depths_cm) {
        int bin_idx = static_cast<int>(depth / bin_cm);
        if (bin_idx < bin_count) {
            histogram[bin_idx]++;
        }
        // Count frequencies below or equal to the threshold
        if (depth <= avoidThresh) {
            freq_above_threshold++;
        }
    }
    
    // Call the draw function
    //calcHistDraw(histogram, bin_cm, x, y, w, h);
    
    // Check if frequency above threshold exceeds avoidFreq
    if (freq_above_threshold > avoidFreq) {
        return 1;
    } else {
        return 0;
    }
}

/*
 * calcHistNew - Calculate depth histogram and detect wall/obstacle edges
 * Return: Wall structure with left/right edge depth and horizontal offset from image center
 *   leftDepth: Depth of left edge (cm)
 *   leftOffset: Horizontal offset of left edge from image center (cm), negative=left, positive=right
 *   rightDepth: Depth of right edge (cm)
 *   rightOffset: Horizontal offset of right edge from image center (cm), negative=left, positive=right
 * Edge detection: Uses depth jump (>5cm) between adjacent columns to identify true edges
 * 2026.05.22: created by BY
 */
Wall T1ObsDisCalcu::calcHistNew(const cv::Mat& depth_map, int x, int y, int w, int h, int bin_cm, int avoidThresh, int avoidFreq) {
    Wall wall = {0};
    
    // Get focal length from camera intrinsic matrix
    float focalLength = static_cast<float>(myStereoConfig.cam_matrix_left.at<double>(0, 0));
    
    // Ensure region is within image bounds
    cv::Rect region(x, y, w, h);
    cv::Rect valid_region = region & cv::Rect(0, 0, depth_map.cols, depth_map.rows);
    if (valid_region.empty()) {
        return wall;
    }
    
    // Build depth map for the region: store depth per pixel
    std::vector<float> depths_cm;
    float max_depth_cm = 0;
    
    for (int row = valid_region.y; row < valid_region.y + valid_region.height; ++row) {
        for (int col = valid_region.x; col < valid_region.x + valid_region.width; ++col) {
            double dist_m = getPointDist(depth_map, col, row);
            if (dist_m > 0.0 && std::isfinite(dist_m)) {
                float dist_cm = dist_m * 100.0f;
                depths_cm.push_back(dist_cm);
                if (dist_cm > max_depth_cm) {
                    max_depth_cm = dist_cm;
                }
            }
        }
    }
    
    if (depths_cm.empty()) {
        return wall;
    }
    
    // Count frequencies below or equal to the threshold
    int freq_above_threshold = 0;
    for (float depth : depths_cm) {
        if (depth <= avoidThresh) {
            freq_above_threshold++;
        }
    }
    
    // Check if obstacle detected
    if (freq_above_threshold > avoidFreq) {
        wall.leftDepth = 0;
        wall.rightDepth = 0;
        wall.leftOffset = 0;
        wall.rightOffset = 0;
        
        // Build per-column depth data: for each column, collect valid depths
        int centerX = x + w / 2;
        std::vector<std::vector<float>> colDepths(valid_region.width);
        
        for (int row = valid_region.y; row < valid_region.y + valid_region.height; ++row) {
            for (int col = valid_region.x; col < valid_region.x + valid_region.width; ++col) {
                double dist_m = getPointDist(depth_map, col, row);
                if (dist_m > 0.0 && std::isfinite(dist_m)) {
                    float dist_cm = dist_m * 100.0f;
                    colDepths[col - valid_region.x].push_back(dist_cm);
                }
            }
        }
        
        // Calculate column median depth and mark valid columns
        float minValidDepth = avoidThresh * 0.2f;
        int minPixelCount = std::max(10, valid_region.height / 6); // At least 10 or 1/6 of height
        std::vector<float> colMedian(valid_region.width, -1);
        std::vector<bool> colValid(valid_region.width, false);
        
        for (int col = 0; col < valid_region.width; ++col) {
            if (colDepths[col].size() >= minPixelCount) {
                // Calculate median (more robust than average)
                std::vector<float> sorted = colDepths[col];
                std::sort(sorted.begin(), sorted.end());
                float median = sorted[sorted.size() / 2];
                
                if (median >= minValidDepth && median <= avoidThresh) {
                    colMedian[col] = median;
                    colValid[col] = true;
                }
            }
        }
        
        // Find left edge: scan from left, find first valid column with depth jump
        int leftEdgeCol = -1;
        float leftEdgeDepth = 0;
        for (int col = 0; col < valid_region.width; ++col) {
            if (!colValid[col]) continue;
            
            // Check if this is a depth jump edge
            bool isEdge = false;
            if (col == 0) {
                isEdge = true; // First column is edge
            } else {
                // Compare with previous columns
                for (int prev = std::max(0, col - 3); prev < col; ++prev) {
                    if (!colValid[prev]) {
                        isEdge = true; // Previous was invalid, this is edge
                        break;
                    }
                    float diff = std::abs(colMedian[col] - colMedian[prev]);
                    if (diff > 5.0f) { // Depth jump > 5cm
                        isEdge = true;
                        break;
                    }
                }
            }
            
            if (isEdge) {
                leftEdgeCol = col;
                leftEdgeDepth = colMedian[col];
                break;
            }
        }
        
        // Find right edge: scan from right, find first valid column with depth jump
        int rightEdgeCol = -1;
        float rightEdgeDepth = 0;
        for (int col = valid_region.width - 1; col >= 0; --col) {
            if (!colValid[col]) continue;
            
            bool isEdge = false;
            if (col == valid_region.width - 1) {
                isEdge = true;
            } else {
                for (int next = col + 1; next <= std::min(valid_region.width - 1, col + 3); ++next) {
                    if (!colValid[next]) {
                        isEdge = true;
                        break;
                    }
                    float diff = std::abs(colMedian[col] - colMedian[next]);
                    if (diff > 5.0f) {
                        isEdge = true;
                        break;
                    }
                }
            }
            
            if (isEdge) {
                rightEdgeCol = col;
                rightEdgeDepth = colMedian[col];
                break;
            }
        }
        
        // Set wall values with unified offset sign: left of center = negative, right of center = positive
        // Convert pixel offset to real-world horizontal distance (cm): horizontal = pixel_offset * depth / focalLength
        if (leftEdgeCol >= 0) {
            int leftPixelX = valid_region.x + leftEdgeCol;
            int pixelOffset = leftPixelX - centerX;
            wall.leftDepth = leftEdgeDepth;
            wall.leftOffset = (focalLength > 0) ? (pixelOffset * leftEdgeDepth / focalLength) : static_cast<float>(pixelOffset);
        }
        
        if (rightEdgeCol >= 0) {
            int rightPixelX = valid_region.x + rightEdgeCol;
            int pixelOffset = rightPixelX - centerX;
            wall.rightDepth = rightEdgeDepth;
            wall.rightOffset = (focalLength > 0) ? (pixelOffset * rightEdgeDepth / focalLength) : static_cast<float>(pixelOffset);
        }
    }
    
    return wall;
}
