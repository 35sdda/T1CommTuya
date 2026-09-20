#include "T1GrassAnalyzer.hpp"

T1GrassAnalyzer::T1GrassAnalyzer() {
}

T1GrassAnalyzer::~T1GrassAnalyzer() {
}

/**
 * Analyze fixed ROI region for grass detection
 * @param binaryMask: Input binary mask image from UNet segmentation
 * @return: 1 if grass ratio <= 10% (no grass), 0 if grass ratio > 10% (has grass), -1 if input empty
 * 
 * ROI Region: Fixed rectangle starting at (50, 300) with width=280, height=60
 * This region is used to detect whether there is grass in the robot's front area
 * 
 * 2026.05.18: modified by BY
 */
int T1GrassAnalyzer::start(const cv::Mat& binaryMask) {
    if (binaryMask.empty()) {
        return -1;
    }

    double grassRatio = calcGrassRatio(binaryMask);

    /*printf("T1GrassAnalyzer-start: grassRatio=%.3f, mask_size=(%d,%d)\n", 
           grassRatio, binaryMask.cols, binaryMask.rows);*/

    if (grassRatio <= 0.9f) {
        return 1;  // Grass ratio <= 10%, no grass detected
    } else {
        return 0;  // Grass ratio > 10%, grass detected
    }
}

/**
 * Calculate grass pixel ratio in fixed ROI region
 * @param mask: Input binary mask image (grass pixels are white/non-zero)
 * @return: Ratio of grass pixels in the ROI region [0.0, 1.0]
 * 
 * Fixed ROI: Rectangle at (50, 300) with size 280x60 pixels
 * The ratio is calculated as: grass_pixels / total_roi_pixels
 * 
 * 2026.05.18: created by BY
 */
double T1GrassAnalyzer::calcGrassRatio(const cv::Mat& mask) {
    // Fixed ROI: start at (50, 228), width=280, height=60
    // Note: Mask size is 512x288 (from UNet model output), not 512x360
    cv::Rect roi_rect(50, 228, 280, 60);

    // Ensure ROI is within image bounds to prevent out-of-range access
    cv::Rect valid_roi = roi_rect & cv::Rect(0, 0, mask.cols, mask.rows);
    if (valid_roi.empty()) {
        printf("T1GrassAnalyzer-calcGrassRatio: ROI out of bounds!\n");
        return 0.0;
    }

    // Extract ROI region from the mask
    cv::Mat roi_mask = mask(valid_roi);

    // Calculate total pixels and grass pixels in ROI
    double roi_total = static_cast<double>(roi_mask.rows * roi_mask.cols);
    double roi_grass = cv::countNonZero(roi_mask);

    /*printf("T1GrassAnalyzer-calcGrassRatio: ROI=(%d,%d,%d,%d), valid=(%d,%d,%d,%d), grass=%.0f/%.0f\n",
           roi_rect.x, roi_rect.y, roi_rect.width, roi_rect.height,
           valid_roi.x, valid_roi.y, valid_roi.width, valid_roi.height,
           roi_grass, roi_total);*/

    // Return grass ratio
    return roi_grass / roi_total;
}
