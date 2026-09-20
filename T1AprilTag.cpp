#define _USE_MATH_DEFINES
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <chrono>
#include "T1AprilTag.hpp"
#include "T1Utils.hpp"

using namespace cv;
using namespace std;

namespace {

// Project the Tag corners back to the image and return the RMS pixel error.
// A smaller value means that the estimated pose matches the detected corners better.
double calculateReprojectionRms(const vector<Point3f>& objectPoints,
                                const vector<Point2f>& imagePoints,
                                const Mat& rotationVector,
                                const Mat& translationVector,
                                const Mat& cameraMatrix,
                                const Mat& distortionCoeffs) {
    vector<Point2f> projectedPoints;
    projectPoints(objectPoints, rotationVector, translationVector,
                  cameraMatrix, distortionCoeffs, projectedPoints);
    if (projectedPoints.size() != imagePoints.size() || imagePoints.empty()) return INFINITY;

    double errorSquareSum = 0.0;
    for (size_t i = 0; i < imagePoints.size(); ++i) {
        const double dx = projectedPoints[i].x-imagePoints[i].x;
        const double dy = projectedPoints[i].y-imagePoints[i].y;
        errorSquareSum += dx*dx+dy*dy;
    }
    return sqrt(errorSquareSum/imagePoints.size());
}

// Reject non-finite, back-facing, or behind-camera pose solutions.
bool isPoseValid(const vector<Point3f>& objectPoints, const Mat& rotationVector,
                 const Mat& translationVector) {
    if (rotationVector.empty() || translationVector.empty() ||
        !checkRange(rotationVector) || !checkRange(translationVector)) return false;

    Mat rotationMatrix;
    Rodrigues(rotationVector, rotationMatrix);
    const double facingDot = -(rotationMatrix.at<double>(0,2)*translationVector.at<double>(0)+
                               rotationMatrix.at<double>(1,2)*translationVector.at<double>(1)+
                               rotationMatrix.at<double>(2,2)*translationVector.at<double>(2));
    if (!std::isfinite(facingDot) || facingDot<=0.0) return false;
    for (const Point3f& point : objectPoints) {
        Mat pointTag = (Mat_<double>(3,1) << point.x, point.y, point.z);
        Mat pointCamera = rotationMatrix*pointTag+translationVector;
        if (!std::isfinite(pointCamera.at<double>(2)) || pointCamera.at<double>(2)<=0.0) return false;
    }
    return true;
}

struct PoseSolution {
    Mat rotationVector;
    Mat translationVector;
    double candidateErrors[2] = {INFINITY, INFINITY};
    double finalError = INFINITY;
    bool valid = false;
};

vector<Point3f> createSquareObjectPoints(float tagSize) {
    const float halfSize = tagSize/2.0f;
    return {
        Point3f(-halfSize, halfSize, 0),
        Point3f( halfSize, halfSize, 0),
        Point3f( halfSize,-halfSize, 0),
        Point3f(-halfSize,-halfSize, 0)
    };
}

PoseSolution estimateSquarePose(const vector<Point3f>& objectPoints,
                                const vector<Point2f>& corners,
                                const Mat& cameraMatrix,
                                const Mat& distortionCoeffs) {
    PoseSolution solution;
    vector<Mat> rotationVectors, translationVectors;
    const int poseCount = solvePnPGeneric(
        objectPoints,corners,cameraMatrix,distortionCoeffs,
        rotationVectors,translationVectors,false,SOLVEPNP_IPPE_SQUARE);

    int bestIndex = -1;
    double bestError = INFINITY;
    for (int index=0; index<poseCount; ++index) {
        if (!isPoseValid(objectPoints,rotationVectors[index],translationVectors[index])) continue;
        const double error = calculateReprojectionRms(
            objectPoints,corners,rotationVectors[index],translationVectors[index],
            cameraMatrix,distortionCoeffs);
        if (index<2) solution.candidateErrors[index] = error;
        if (std::isfinite(error) && error<bestError) {
            bestError = error;
            bestIndex = index;
        }
    }
    if (bestIndex<0) return solution;

    solution.rotationVector = rotationVectors[bestIndex].clone();
    solution.translationVector = translationVectors[bestIndex].clone();
    solvePnPRefineLM(objectPoints,corners,cameraMatrix,distortionCoeffs,
                     solution.rotationVector,solution.translationVector,
                     TermCriteria(TermCriteria::EPS+TermCriteria::COUNT,20,1e-6));
    solution.finalError = calculateReprojectionRms(
        objectPoints,corners,solution.rotationVector,solution.translationVector,
        cameraMatrix,distortionCoeffs);
    solution.valid = isPoseValid(objectPoints,solution.rotationVector,solution.translationVector) &&
                     std::isfinite(solution.finalError) && solution.finalError<=3.0;
    return solution;
}

AprilTagResult createRelativePose(int tagId, const PoseSolution& solution,
                                float scaleFactor) {
    Mat rotationMatrix;
    Rodrigues(solution.rotationVector,rotationMatrix);

    // Correct the 180-degree mounting rotation before creating control values.
    const Mat tagMountRotation = (Mat_<double>(3,3) <<
        -1,0,0, 0,-1,0, 0,0,1);
    const Mat controlRotation = rotationMatrix*tagMountRotation;

    AprilTagResult pose;
    pose.tagId = tagId;
    pose.tx = solution.translationVector.at<double>(0)*scaleFactor*100.0;
    pose.ty = solution.translationVector.at<double>(1)*scaleFactor*100.0;
    pose.tz = solution.translationVector.at<double>(2)*scaleFactor*100.0;
    pose.reprojectionErrorPx = solution.finalError;
    pose.valid = true;
    for (int row=0; row<3; ++row)
        for (int col=0; col<3; ++col)
            pose.rotation[row*3+col] = controlRotation.at<double>(row,col);

    // Invert Tag-in-camera to camera-in-Tag: cameraInTag = -R^T*t.
    const double translationCm[3] = {pose.tx,pose.ty,pose.tz};
    double cameraInTag[3] = {0.0,0.0,0.0};
    for (int row=0; row<3; ++row)
        for (int col=0; col<3; ++col)
            cameraInTag[row] -= pose.rotation[col*3+row]*translationCm[col];

    pose.relativeLateral = -cameraInTag[0];
    pose.relativeVertical = cameraInTag[1];
    pose.normalDistance = cameraInTag[2];
    pose.yawError = atan2(-pose.rotation[6],-pose.rotation[8])*180.0/M_PI;
    while (pose.yawError>180.0) pose.yawError -= 360.0;
    while (pose.yawError<=-180.0) pose.yawError += 360.0;
    return pose;
}

void calculatePitchRoll(const Mat& rotationVector, double& pitch, double& roll) {
    Mat rotationMatrix;
    Rodrigues(rotationVector,rotationMatrix);
    const Mat pitchFlip = (Mat_<double>(3,3) << 1,0,0, 0,-1,0, 0,0,-1);
    const Mat corrected = rotationMatrix*pitchFlip;
    const double sy = sqrt(pow(corrected.at<double>(0,0),2)+
                           pow(corrected.at<double>(1,0),2));
    if (sy>=1e-6) {
        pitch = atan2(corrected.at<double>(2,1),corrected.at<double>(2,2))*180.0/M_PI;
        roll = atan2(corrected.at<double>(1,0),corrected.at<double>(0,0))*180.0/M_PI;
    } else {
        pitch = atan2(-corrected.at<double>(1,2),corrected.at<double>(1,1))*180.0/M_PI;
        roll = 0.0;
    }
}

void drawPoseResult(Mat& image, const vector<Point2f>& corners,
                    const AprilTagResult& pose, double pitch, double roll) {
    for (size_t index=0; index<corners.size(); ++index)
        line(image,corners[index],corners[(index+1)%corners.size()],Scalar(255),2);
    putText(image,to_string(pose.tagId),corners[0],FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
    circle(image,(corners[0]+corners[2])*0.5f,4,Scalar(255),-1);

    char normal[32], lateral[32], yaw[32], pitchText[32], rollText[32];
    snprintf(normal,sizeof(normal),"%.2f",pose.normalDistance);
    snprintf(lateral,sizeof(lateral),"%.2f",pose.relativeLateral);
    snprintf(yaw,sizeof(yaw),"%.1f",pose.yawError);
    snprintf(pitchText,sizeof(pitchText),"%.1f",pitch);
    snprintf(rollText,sizeof(rollText),"%.1f",roll);
    const vector<string> textLines = {
        "Tag ID: "+to_string(pose.tagId),
        "Normal Distance: "+string(normal)+" cm",
        "Relative Lateral: "+string(lateral)+" cm",
        "Yaw Error: "+string(yaw)+" deg",
        "Pitch: "+string(pitchText)+" deg",
        "Roll: "+string(rollText)+" deg"
    };
    int y = static_cast<int>(image.rows*0.04);
    for (const string& textLine : textLines) {
        putText(image,textLine,Point(static_cast<int>(image.cols*0.04),y),
                FONT_HERSHEY_SIMPLEX,0.6,Scalar(255),2);
        y += static_cast<int>(image.rows*0.06);
    }
}

} // namespace

// ===================== T1AprilTag Class Implementation =====================

T1AprilTag::T1AprilTag() {
    meVerticalDistance = -10;
    meHorizontalDistance = -10;
    meYawAngle = 0;
    meDetectedTagId = -1000;
    meResult = AprilTagResult();
    loadCameraParameters();
}

void T1AprilTag::loadCameraParameters() {
    std::ifstream file("upt/StereoParam.upt");
    if (file.is_open()) {
        std::cout << "T1AprilTag-loadCameraParameters() Loading parameters from upt/StereoParam.upt for T1AprilTag" << std::endl;

        std::string line;
        while (std::getline(file, line)) {
            if (line.find("Left Camera Intrinsic Matrix") != std::string::npos) {
                break;
            }
        }

        double k1_data[9];
        char dummy;
        std::getline(file, line);
        std::istringstream k1_stream1(line);
        k1_stream1 >> dummy;
        k1_stream1 >> k1_data[0];
        k1_stream1 >> dummy;
        k1_stream1 >> k1_data[1];
        k1_stream1 >> dummy;
        k1_stream1 >> k1_data[2];
        k1_stream1 >> dummy;
        
        std::getline(file, line);
        std::istringstream k1_stream2(line);
        k1_stream2 >> k1_data[3];
        k1_stream2 >> dummy;
        k1_stream2 >> k1_data[4];
        k1_stream2 >> dummy;
        k1_stream2 >> k1_data[5];
        k1_stream2 >> dummy;
        
        std::getline(file, line);
        std::istringstream k1_stream3(line);
        k1_stream3 >> k1_data[6];
        k1_stream3 >> dummy;
        k1_stream3 >> k1_data[7];
        k1_stream3 >> dummy;
        k1_stream3 >> k1_data[8];
        k1_stream3 >> dummy;

        while (std::getline(file, line)) {
            if (line.find("Left Camera Distortion Coefficients") != std::string::npos) {
                break;
            }
        }

        double d1_data[5];
        std::getline(file, line);
        std::istringstream d1_stream1(line);
        d1_stream1 >> dummy;
        d1_stream1 >> d1_data[0];
        d1_stream1 >> dummy;
        
        std::getline(file, line);
        std::istringstream d1_stream2(line);
        d1_stream2 >> d1_data[1];
        d1_stream2 >> dummy;
        
        std::getline(file, line);
        std::istringstream d1_stream3(line);
        d1_stream3 >> d1_data[2];
        d1_stream3 >> dummy;
        
        std::getline(file, line);
        std::istringstream d1_stream4(line);
        d1_stream4 >> d1_data[3];
        d1_stream4 >> dummy;
        
        std::getline(file, line);
        std::istringstream d1_stream5(line);
        d1_stream5 >> d1_data[4];
        d1_stream5 >> dummy;

        file.close();

        meCameraMatrix = (cv::Mat_<double>(3, 3) <<
            k1_data[0], 0, k1_data[2],
            0, k1_data[4], k1_data[5],
            0, 0, 1);
        
        meDistortionCoeffs = (cv::Mat_<double>(1, 5) <<
            d1_data[0], d1_data[1], d1_data[2], d1_data[3], d1_data[4]);
    } else {
        std::cerr << "Error: Could not open file upt/StereoParam.upt, using default parameters" << std::endl;
        meCameraMatrix = (cv::Mat_<double>(3, 3) <<
            1035.130879634413, 0, 988.4520149329596,
            0, 1038.589274648906, 521.1443864552878,
            0, 0, 1);
        
        meDistortionCoeffs = (cv::Mat_<double>(1, 5) <<
            0.02205551102109187, -0.06749484350512436, -0.001160221277195326, 0.004541173257076944, 0.05123668475443666);
    }
}

void T1AprilTag::StartNV12(const cv::Mat& nv12_img, int width, int height, float tag_size) {
    int debugAprilTag = 0; 

    if(debugAprilTag){
        cout << "[T1AprilTag::StartNV12] Starting AprilTag detection from NV12 image..." << endl;
        cout << "Image size: " << width << "x" << height << " tag_size: " << tag_size << endl;
    }
    
    Mat gray_image = nv12ToGray(nv12_img.data, width, height);

    process_image(gray_image, tag_size, 1.0f);
  
    if(debugAprilTag){
        cout << "[T1AprilTag::StartNV12] Detection completed!" << endl;
        cout << "Tag Normal Distance: " << getVerticalDistance() << " cm" << endl;
        cout << "Tag Relative Lateral: " << getHorizontalDistance() << " cm" << endl;
        cout << "Tag Relative Yaw Error: " << getYawAngle() << " degrees" << endl;
    }
}

void T1AprilTag::process_image(cv::Mat& image, float tag_size, float scale_factor) {
    int debugAprilTag = 0; 

    // 1. Detect AprilTag corners in the preprocessed grayscale image.
    Ptr<aruco::Dictionary> tag_dictionary = aruco::getPredefinedDictionary(aruco::DICT_APRILTAG_36h11);
    Ptr<aruco::DetectorParameters> detect_params = aruco::DetectorParameters::create();
    detect_params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    detect_params->minMarkerPerimeterRate = 0.02;
    detect_params->maxMarkerPerimeterRate = 4.0;

    Mat processed_frame;
    equalizeHist(image, processed_frame);// histogram equalization ：Enhance the contrast between black and white

    GaussianBlur(processed_frame, processed_frame, Size(3, 3), 0);// Gaussian filter ： eliminate noise

    vector<vector<Point2f>> marker_corners, rejected_corners;
    vector<int> marker_ids;
    aruco::detectMarkers(processed_frame, tag_dictionary, marker_corners, marker_ids, detect_params, rejected_corners);

    meVerticalDistance = -10; meHorizontalDistance = -10;
    meYawAngle = 0;
    meDetectedTagId = -1000;
    meResult = AprilTagResult();

    // 2. Estimate every detected Tag and retain the lowest-error pose.
    double bestFrameError = INFINITY;

    for (size_t i = 0; i < marker_ids.size(); ++i) {
        const vector<Point2f>& corners = marker_corners[i];
        const int tag_id = marker_ids[i];

        const vector<Point3f> object_points = createSquareObjectPoints(tag_size);

        const PoseSolution solution = estimateSquarePose(
            object_points,corners,meCameraMatrix,meDistortionCoeffs);
        if (!solution.valid) continue;

        const double finalError = solution.finalError;
        if (finalError>=bestFrameError) {
            if (debugAprilTag) {
                cout << "[T1AprilTag::process_image] rejected tag " << tag_id
                     << ", candidate RMS=" << solution.candidateErrors[0] << "/" << solution.candidateErrors[1]
                     << ", final RMS=" << finalError << " px" << endl;
            }
            continue;
        }

        // 3. Convert the selected PnP result into control-facing X, Y, and yaw.
        const AprilTagResult pose = createRelativePose(tag_id,solution,scale_factor);
        double pitch_angle = 0.0;
        double roll_angle = 0.0;
        calculatePitchRoll(solution.rotationVector,pitch_angle,roll_angle);
        bestFrameError = finalError;
        meDetectedTagId = tag_id;
        meVerticalDistance = pose.normalDistance;
        meHorizontalDistance = pose.relativeLateral;
        meYawAngle = pose.yawError;
        meResult = pose;
        // printf("T1AprilTag-IPPE: id=%d, candidateRMS=(%.3f,%.3f)px, finalRMS=%.3fpx, cameraRaw=(tx=%.1f,ty=%.1f,tz=%.1f)cm, tagRelative=(x=%.1f,vertical=%.1f,normal=%.1f,yawError=%.1f)\n",
        //        tag_id,solution.candidateErrors[0],solution.candidateErrors[1],finalError,
        //        pose.tx,pose.ty,pose.tz,
        //        pose.relativeLateral,pose.relativeVertical,
        //        pose.normalDistance,pose.yawError);
        // 4. Keep visualization separate from pose estimation and control data.
        drawPoseResult(image,corners,pose,pitch_angle,roll_angle);

        if(debugAprilTag){
            cout << "\n[T1AprilTag::process_image] === Detection Result ===" << endl;
            cout << "Tag ID: " << tag_id << endl;
            cout << "Tag Normal Distance: " << pose.normalDistance << " cm" << endl;
            cout << "Tag Relative Lateral: " << pose.relativeLateral << " cm" << endl;
            cout << "YawError/Pitch/Roll: " << pose.yawError << "/" << pitch_angle << "/" << roll_angle << " degrees" << endl;
            cout << "IPPE candidate RMS: " << solution.candidateErrors[0] << "/" << solution.candidateErrors[1]
                 << " px, final RMS: " << finalError << " px" << endl;
        }
    }

    //saveResultImage(image, "../image/detection_result.jpg");
}

cv::Mat T1AprilTag::nv12ToGray(const unsigned char* nv12_data, int width, int height) {
    Mat gray_frame(height, width, CV_8UC1);
    memcpy(gray_frame.data, nv12_data, static_cast<size_t>(width) * height);
    return gray_frame;
}

void T1AprilTag::saveResultImage(cv::Mat& image, const std::string& save_path) {
    int debugAprilTag = 0; // 

    Mat bgr_image;
    if (image.channels() == 1) {
        cvtColor(image, bgr_image, COLOR_GRAY2BGR);
    } else {
        bgr_image = image.clone();
    }
    
    if (imwrite(save_path, bgr_image)) {
        if(debugAprilTag){
            cout << "[T1AprilTag::saveResultImage] Success: Result saved to - " << save_path << endl;
        }
    } else {
        if(debugAprilTag){
            cerr << "[T1AprilTag::saveResultImage] Error: Failed to save result - " << save_path << endl;
        }
    }
}

double T1AprilTag::getVerticalDistance() const {
    return meVerticalDistance;
}

double T1AprilTag::getHorizontalDistance() const {
    return meHorizontalDistance;
}

double T1AprilTag::getYawAngle() const {
    return meYawAngle;
}

int T1AprilTag::getTagId() const {
    return meDetectedTagId;
}

AprilTagResult T1AprilTag::getResult() const {
    return meResult;
}

/*cv::Point2f T1AprilTag::getCameraPosition(float tag_angle, float tag_x, float tag_z) const {
    char tmpstr[200];
    int debug = 0; 
    const double DEG2RAD = M_PI / 180.0;
    
    float tag_angle1 = 90 - tag_angle;
    if ( tag_angle1<0 ) tag_angle1+=360;

    double tag_yaw   = tag_angle1;
    double yaw       = meYawAngle;
    double vertical  = meVerticalDistance;
    double horizontal= meHorizontalDistance;

    double angle1 = tag_yaw + yaw;
    double rad1 = angle1 * DEG2RAD;
    double dx1 = vertical * cos(rad1);
    double dz1 = vertical * sin(rad1);

    double angle2 = (horizontal > 0) ? (angle1 + 90.0) : (angle1 - 90.0);
    double h_dist = fabs(horizontal);
    double rad2 = angle2 * DEG2RAD;
    double dx2 = h_dist * cos(rad2);
    double dz2 = h_dist * sin(rad2);

    double cam_x_m = (tag_x / 100.0) + dx1 + dx2;
    double cam_z_m = (tag_z / 100.0) + dz1 + dz2;
    
    float cam_x = static_cast<float>(cam_x_m * 100.0);
    float cam_z = static_cast<float>(cam_z_m * 100.0);

    if (debug) {
        cout << fixed << setprecision(2)
             << "[T1AprilTag::getCameraPosition] Tag(" << tag_x << "," << tag_z << ")"
             << " TagHeading:" << tag_angle
             << " CameraYaw:" << yaw
             << " Camera(X,Z):(" << cam_x << "," << cam_z << ")cm" << endl;
    }
    sprintf(tmpstr,"T1AprilTag-getCameraPosition end, tagAXZ=(%.1f->%.1f, %.1f,%.1f), TagPos=(%.1f,%.1f,%.1f), camXZ=(%.1f,%.1f)",
        tag_angle, tag_angle1, tag_x, tag_z, horizontal, vertical, yaw, cam_x, cam_z);
    T1Utils::LogBug(tmpstr);

    return cv::Point2f(cam_x, cam_z);
}*/
