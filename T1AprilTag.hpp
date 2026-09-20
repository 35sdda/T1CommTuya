#ifndef T1APRILTAG_HPP
#define T1APRILTAG_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

typedef struct AprilTagResult {
    int tagId = -1000;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double rotation[9] = {0.0};
    double relativeLateral = 0.0;
    double relativeVertical = 0.0;
    double normalDistance = 0.0;
    double yawError = 0.0;
    double reprojectionErrorPx = 0.0;
    bool valid = false;
} AprilTagResult;

class T1AprilTag {
public:
    T1AprilTag();
    
    void process_image(cv::Mat& image, float tag_size, float scale_factor);
    void StartNV12(const cv::Mat& nv12_img, int width, int height, float tag_size);
    
    // Getter methods for distance values
    double getVerticalDistance() const;
    double getHorizontalDistance() const;
    double getYawAngle() const;
    int getTagId() const;
    AprilTagResult getResult() const;

private:
    // Helper methods
    cv::Mat nv12ToGray(const unsigned char* nv12_data, int width, int height);
    void saveResultImage(cv::Mat& image, const std::string& save_path);
    void loadCameraParameters();
    // Calculate camera position in charger coordinate system
    //cv::Point2f getCameraPosition(float tag_angle, float tag_x, float tag_z) const;

    cv::Mat meCameraMatrix;
    cv::Mat meDistortionCoeffs;
    double meVerticalDistance;
    double meHorizontalDistance;
    double meYawAngle;
    int meDetectedTagId;
    AprilTagResult meResult;
};

#endif // T1APRILTAG_HPP
