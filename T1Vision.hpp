#ifndef T1VISION_HPP
#define T1VISION_HPP
#include <opencv2/opencv.hpp>
#include <thread>
#include <stdio.h>
#include "CVsgbm.hpp"
#include "CVMobileNetUNet.hpp"
#include "CVMobileNetUNet2.hpp"
#include "CVstereoconfig.hpp"
#include "T1CVSgbm.hpp"
#include "CVYolo26.hpp"
#include "T1CVOrb.hpp"
#include "T1CVSift.hpp"
#include "T1CVSift2.hpp"
#include "T1AprilTag.hpp"
#include "T1Video.hpp"
#include <chrono>
#include "T1GrassAnalyzer.hpp"
#include "T1Common.hpp"

#define WIDTH 1920  // Image width (1920*1080)
#define HEIGHT 1080  // Image height (1920*1080)
#define WIDTH_SMALL 640  // Small image width (640*360)
#define HEIGHT_SMALL 360  // Small image height (640*360)
#define BUF_SIZE (8*1024*1024)  // 8M buffer, more than enough for NV12 data
#define DATA_SIZE_LARGE 3110400  // NV12 size (1920*1080*1.5)
#define DATA_SIZE_SMALL 691200  // BGR size (640*360*3)
/*#define FIFO_BUF_SIZE  (4 * 1024 * 1024)  // 4M buffer, more than 3110400

#ifndef F_SETPIPE_SZ
#define F_SETPIPE_SZ 1031
#endif
*/
#define FIFO_CMD "/tmp/T1_fifo_cmd"
#define FIFO_DATA0 "/tmp/T1_fifo_data0"
#define FIFO_DATA1 "/tmp/T1_fifo_data1"

using namespace cv;

class T1Vision {
public:
    T1Vision();
    ~T1Vision();
    int Detect();
    void SetStatus(char, int);
    void SetAprilTagSize(float size) { meApriltagSize = size; }
    void SetAvoidThresh(int thresh) { meAvoidThresh = thresh; }
    void SetAvoidFreq(int freq) { meAvoidFreq = freq; }
    double getClosestDistance();

    int GetVisSignal(char);

    PointRobo GetPoint(char);
    AprilTagPose GetAprilTagPose() const;
    float getVerticalDis();
    float getHorizontalDis();
    float getYawAngle();
    int getTagId();
    Wall GetWall();
    cv::Point2f getCameraPosition (float, float, float) const;
    void ResetWall();
    //cv::Point2f getCameraPosition(float tag_angle, float tag_x, float tag_y) const { return myAprilTag.getCameraPosition(tag_angle, tag_x, tag_y); };

private:
    void createFifo(const std::string& );
    void Cleanup();

    int ImageSaveHighRes();
    int ImageSave(int);
    int ImageConvert();
    int ImageDisplay();
    int ImageRecive(int);

    double calcObjAvgDepth(const std::string& class_name);
    int AprilTag(Mat& image);

    int timeCalculate(int, uint64_t *, uint64_t *, const char *);

private:
    pid_t pid = -1;

    int fd_cmd = -1;
    int fd_data0 = -1;
    int fd_data1 = -1;
    int file_count = 0;
    char * image_data0 = NULL;
    char * image_data1 = NULL;
    
    int flagSave = 0;
    bool flagAprilTag = 0;
    int Init();
    int meCountHighLowRes = 9; // Count of high resolution images per low resolution image
    float meApriltagSize = 0.1f;
    int meAvoidThresh = 50; // Threshold of obstacle avoidance, cm
    int meAvoidFreq = 1000; // Frequency of obstacle avoidance
    PointRobo myPointVA;
    AprilTagPose myAprilTagPose;
    PointRobo myPointVB;
    Wall myDetectWall;
   
    StereoCamera myStereoConfig;
    CVMobileNetUNet unet;
    CVMobileNetUNet2 myUnet;
    CVYolo26 myYolo26;
    T1CVSgbm mySgbm;
    T1CVOrb myOrb;
    T1CVSift mySift;
    T1CVSift2 mySift2;
    T1AprilTag myAprilTag;
    T1Video myVideo;
    T1ObsDisCalcu myObsDisCalcu;
    T1GrassAnalyzer myGrassAnalyzer;
    cv::Mat meImageLeft;
    cv::Mat meImageRight;
    cv::Mat meImageCroppedLeft;
};

#endif // T1VISION_HPP
