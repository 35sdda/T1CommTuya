#include "T1CVSgbm.hpp"
#include <numeric>

T1CVSgbm::T1CVSgbm() {
}

T1CVSgbm::~T1CVSgbm() {
}

/**
 * Main entry point for processing stereo images and generating depth maps.
 * Note: initialize() must be called before this function.
 */
void T1CVSgbm::start(cv::Mat left_image, cv::Mat right_image) {
    auto t2 = std::chrono::high_resolution_clock::now();
    auto t1 = t2;
    
    t1 = t2; t2 = std::chrono::high_resolution_clock::now();
    //std::cout << "T1Vision::processStereoImages converter.createMatFromMemory time cost: " << std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()/1000.0 << "ms" << std::endl;

    // 1. Check if input images are valid
    if (left_image.empty() || right_image.empty()) return;
    
    // 2. Check if initialized
    if (!meIsInitialized) {
        // Auto-initialize if not done yet (fallback)
        initialize(left_image.cols, left_image.rows);
    }

    // 3. Apply rectification to align both camera views
    // Using cached transformation maps from initialize()
    cv::Mat rectified_left, rectified_right;
    rectifyImage(left_image, right_image, meMap1x, meMap1y, meMap2x, meMap2y, rectified_left, rectified_right);
    
    // 4. Compute disparity using Semi-Global Block Matching (SGBM)
    cv::Mat disparity_left, disparity_right;
    stereoMatchSGBM(rectified_left, rectified_right, disparity_left, disparity_right, true);
    
    // 4.1 WLS filter is commented out to reduce processing time
    // WLS (Weighted Least Squares) filter improves disparity map quality
    // but adds significant computational overhead
    // Uncomment the following block if you need higher quality results
    /*
    cv::Mat filtered_disparity;
    cv::Mat disp_f;
    
    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(0, 32, 3);
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
    wls_filter->setLambda(8000.0);
    wls_filter->setSigmaColor(1.5);
    wls_filter->filter(disparity_left, rectified_left, filtered_disparity, disparity_right);

    // 5. Filter small noise blocks (speckles) from the filtered disparity
    if (filtered_disparity.type() == CV_16SC1) {
        cv::filterSpeckles(filtered_disparity, 0, 400, 16);
    }
    */
    
    // Use raw disparity directly 
    cv::Mat filtered_disparity = disparity_left;
    cv::Mat disp_f;

    // 6. Convert fixed-point disparity to floating-point and smooth it
    filtered_disparity.convertTo(disp_f, CV_32F, 1.0);
    cv::medianBlur(disp_f, disp_f, 3);

    // 7. Reconstruct 3D depth from disparity using the Q matrix
    //cv::Mat depth_map = getDepthMapWithQ(disp_f, Q);
    cv::Mat depth_map = getDepthMapWithConfig(disp_f, myStereoConfig);
    meDepthMap = depth_map;

    // 8. Create an 8-bit image for visual output and apply custom normalization
    //cv::Mat depth_8u = cv::Mat::zeros(depth_map.size(), CV_8U);
    //simpleNormalize(depth_map, depth_8u);

    // 9. Generate a high-contrast grayscale image for debugging
    cv::Mat disparity_normalized;
    cv::normalize(disp_f, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

    // 10. Apply JET colormap with custom depth range (0.5m = nearest/red, 4m = farthest/blue)
    cv::Mat depth_color;
    ColorDepth(meDepthMap, depth_color, 500.0f, 4000.0f);

    /*// 11. Draw red border on depth maps
    cv::Point topLeft(230, 210);
    cv::Point bottomRight(230 + 200, 210 + 50);
    cv::Scalar redColor(0, 0, 255); // BGR format
    int thickness = 2;
    cv::rectangle(depth_color, topLeft, bottomRight, redColor, thickness);
    cv::rectangle(disparity_normalized, topLeft, bottomRight, redColor, thickness);*/

    // 12. Display visual results in windows
    cv::imshow("depthm", depth_color);
    cv::moveWindow("depthm", 0, 680);
    //cv::imshow("depthm-gray", disparity_normalized);
    //cv::moveWindow("depthm-gray", depth_color.cols, 680);

    // 13. Refresh GUI windows
    cv::waitKey(1);
    //std::cout << "T1Vision::processStereoImages cv::waitKey(1) time cost: " << std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()/1000.0 << "ms" << std::endl;
    //std::cout << "*******end********" << std::endl;
}

/**
 * Manually map physical distance to 8-bit pixel values.
 */
void T1CVSgbm::simpleNormalize(const cv::Mat depth_map, cv::Mat depth_8u) {
// Safety: Ensure depth_8u is the same size as the input
    if (depth_8u.rows != depth_map.rows || depth_8u.cols != depth_map.cols) {
        printf("T1CVSgbm::simpleNormalize begin error, size not equal.\n");
        return; 
    }
    // Iterate through every pixel using row pointers for performance
    for (int y = 0; y < depth_map.rows; y++) {
        const float* depth_ptr = depth_map.ptr<float>(y);
        uchar* output_ptr = depth_8u.ptr<uchar>(y);
        for (int x = 0; x < depth_map.cols; x++) {
            float z_mm = depth_ptr[x];
            // Filter out non-numeric values and points outside the search range (0-5m)
            if (std::isinf(z_mm) || std::isnan(z_mm) || z_mm <= 0.0f || z_mm > 5000.0f) {
                output_ptr[x] = 0; // Set to black
                continue; 
            }
            // 2. Map distance to pixel (2cm per unit)
            int z_cm = static_cast<int>(z_mm / 10.0f);
            int pixel_val = 255 - (z_cm / 2);
            // 3. Final clamp to ensure we stay in the 0-255 range
            output_ptr[x] = static_cast<uchar>(std::max(0, pixel_val));
        }
    }
}

/**
 * Apply JET colormap to depth image with custom range
 * @param depth_map: Input depth map (CV_32F, values in mm)
 * @param color_map: Output color image (CV_8UC3)
 * @param min_mm: Minimum depth in mm (default 500mm = 0.5m, mapped to nearest/red)
 * @param max_mm: Maximum depth in mm (default 4000mm = 4m, mapped to farthest/blue)
 * 2026.05.18: created by BY
 */
void T1CVSgbm::ColorDepth(const cv::Mat& depth_map, cv::Mat& color_map, float min_mm, float max_mm) {
    // Create normalized 8-bit depth image
    cv::Mat depth_8u = cv::Mat::zeros(depth_map.size(), CV_8U);
    
    for (int y = 0; y < depth_map.rows; y++) {
        const float* depth_ptr = depth_map.ptr<float>(y);
        uchar* output_ptr = depth_8u.ptr<uchar>(y);
        
        for (int x = 0; x < depth_map.cols; x++) {
            float depth = depth_ptr[x];
            
            // Filter invalid values
            if (depth <= 0.0f || std::isinf(depth) || std::isnan(depth)) {
                output_ptr[x] = 0;  // Black for invalid depth
                continue;
            }
            
            // Clamp to range and normalize to [0, 255]
            float clamped = std::max(min_mm, std::min(max_mm, depth));
            float normalized = 1.0 - (clamped - min_mm) / (max_mm - min_mm);
            output_ptr[x] = static_cast<uchar>(normalized * 255.0f);
        }
    }
    
    // Apply JET colormap
    cv::applyColorMap(depth_8u, color_map, cv::COLORMAP_JET);
}

/** initialize redundant operations
 * 2026.05.09: created by BY
 */
void T1CVSgbm::initialize(int width, int height) {
    // Obtain rectification transformation maps and Q matrix
    // These are computed once and cached since they depend only on:
    // - Camera intrinsic parameters (fixed after calibration)
    // - Camera extrinsic parameters (fixed after calibration)
    // - Image resolution (fixed unless camera settings change)
    getRectifyTransform(height, width, myStereoConfig, meMap1x, meMap1y, meMap2x, meMap2y, meQ);
    meIsInitialized = true;
}

