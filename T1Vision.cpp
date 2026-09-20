#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <chrono>
#include <ctime>
#include <cstring>
#include <cmath>

#include "T1Vision.hpp"
#include "T1Utils.hpp"


T1Vision::T1Vision() {
    myVideo.startRecording();
    Init();
    printf("T1Vision: Visual Sensor initilized.\n\n\n\n\n\n");
}

T1Vision::~T1Vision() {
    Cleanup();
}

void T1Vision::Cleanup() { 
    printf("T1Vision-Cleanup, begin.\n");
    if ( image_data0!=NULL ) delete[] image_data0;
    if ( image_data1!=NULL ) delete[] image_data1;
    printf("T1Vision-Cleanup, step 1.\n");
    close(fd_cmd);
    close(fd_data0);
    close(fd_data1);
    printf("T1Vision-Cleanup, step 2.\n");
    waitpid(pid, nullptr, 0);
    printf("T1Vision-Cleanup, step 3.\n");
    unlink(FIFO_CMD);
    unlink(FIFO_DATA0);
    unlink(FIFO_DATA1);
    printf("T1Vision-Cleanup, end.\n");
    myVideo.stopRecording();
    //return 1;   
}

int T1Vision::Init() {
    // creat command and two camera datas' FIFO
    printf("T1Vision-Init, begin.\n");
    createFifo(FIFO_CMD);
    createFifo(FIFO_DATA0);
    createFifo(FIFO_DATA1);
    printf("T1Vision-Init, FIFOs have been created .\n");
 
    // load child process
    pid = fork();
    if ( pid==-1 ) {
        printf("T1Vision-Init, paretn process(%d) load child failed", getpid());
        exit(EXIT_FAILURE);
    }
    if ( pid==0 ) {
        printf("T1Vision-Init, child process(%d) t1GetIsp was loading......\n", getpid());
        execl("../t1GetIsp", "t1GetIsp", nullptr);
        perror("T1Vision-Init, chile t1GetIsp exited.");
        exit(EXIT_FAILURE);
    }
    
    printf("T1Vision-Init, parent process(%d) loaded child.\n", getpid());

    // open FIFO
    fd_cmd = open(FIFO_CMD, O_WRONLY);
    //printf("T1Vision-Init, open FIFO_CMD.\nb\nb\nb\nb\nb\n");
    fd_data0 = open(FIFO_DATA0, O_RDONLY);
    //printf("T1Vision-Init, open FIFO_DATA1.\nc\nc\nc\nc\nc\n");
    fd_data1 = open(FIFO_DATA1, O_RDONLY);
    //printf("T1Vision-Init, open FIFO_CMD.\nd\nd\nd\nd\nd\n");
    if ( fd_cmd==-1 ) {//|| fd_data0 == -1 || fd_data1 == -1) {
        perror("T1Vision-Init, FIFO open failed.\n");
        exit(EXIT_FAILURE);
    } else
        printf("T1Vision-Init, FIFOs are opened.\n");

    // maxinum the buffers
    fcntl(fd_data0, F_SETPIPE_SZ, BUF_SIZE);
    fcntl(fd_data1, F_SETPIPE_SZ, BUF_SIZE);

    image_data0 = new char[DATA_SIZE_LARGE];
    image_data1 = new char[DATA_SIZE_LARGE];

    printf("T1Vision-Init, end.\n");
    return 1;
}

void T1Vision::createFifo(const std::string& path) {
    // delete old FIFO firstly??otherwith the filemode will be wrong
    if ( access(path.c_str(),F_OK)!=-1 ) {
        unlink(path.c_str());
    }
    // creat FIFO and set filemode to 0666
    if ( mkfifo(path.c_str(),0666)==-1 ) {
        perror("T1Vision-createFifo, mkfifo failed");
        exit(EXIT_FAILURE);
    }
    if ( chmod(path.c_str(),0666)==-1 ) {
        perror("T1Vision-createFifo, chmod failed");
        exit(EXIT_FAILURE);
    }
}

void T1Vision::SetStatus(char type, int val) {
    switch (type) {
    /*case 'S': // save YUV files or not
        flagSave = val;
        break;*/
    case 'A'://AprilTag
        flagAprilTag = val;
        break;
    case 'C':
        Cleanup();
        break;
    case 'V':
        meCountHighLowRes = val;
        if ( meCountHighLowRes < 0 ) meCountHighLowRes = 0;
        if ( meCountHighLowRes > 9 ) meCountHighLowRes = 9;
        break;
    default:
        printf("T1Vision-SetStatus, unknown status type.\n");
        break;
    }
}

int T1Vision::GetVisSignal(char type)
{
    switch (type) {
    case 'A': // A: AprilTag detection
        return flagAprilTag;
        break;
    default:
        return -1;
        break;
    }
}

/*cv::Point2f T1Vision::getCameraPosition(float tag_angle, float tag_x, float tag_y) const {
    return myAprilTag.getCameraPosition(tag_angle, tag_x, tag_y);
};*/

int T1Vision::ImageConvert() {
    return 0;
}

int T1Vision::ImageDisplay() {
    return 0;
}

int T1Vision::ImageSave(int highRes) {
    //printf("T1Vision::ImageSave image type is %d\n", type);
    if ( flagSave==0 ) return 0;
    if ( highRes==1 ) return ImageSaveHighRes();
    
    // compile the filename
    std::string file0 = "../Yuv/camera_L0_" + std::to_string(file_count) + ".yuv";
    std::string file1 = "../Yuv/camera_R1_" + std::to_string(file_count) + ".yuv";
    // write file in binary
    std::ofstream f0(file0, std::ios::binary);
    std::ofstream f1(file1, std::ios::binary);
    if ( f0.is_open() && f1.is_open() ) {
        f0.write(image_data0, DATA_SIZE_SMALL);
        f1.write(image_data1, DATA_SIZE_SMALL);
        //std::cout << "T1Vision-Detect, file writed: " << file0 << " and " << file1 << " (each size: " << DATA_SIZE << " bytes)" << std::endl;
        //std::cout << "T1Vision-Detect, file writed: " << file_count " (each size: " << DATA_SIZE << " bytes)" << std::endl;
        file_count++;
    } else {
        perror("T1Vision--ImageSave, open to write yuv file failed");
        return -1;
    }
    return 0;
}

int T1Vision::ImageSaveHighRes() {
    // Use unified T1Utils time functions
    std::string dateStr = T1Utils::getCurrentYYYYMMDD();
    std::string timeStr = T1Utils::getCurrentHHMMSS();
    uint64_t timeMs = T1Utils::getCurrentTimeMs();
    
    std::stringstream ss;
    ss << dateStr << "_" << timeStr << "_" << std::setw(3) << std::setfill('0') << (timeMs % 1000);
    std::string timestamp = ss.str();
    
    std::string yuv_file0 = "../image/camera_L0_" + timestamp + ".yuv";
    std::string yuv_file1 = "../image/camera_R1_" + timestamp + ".yuv";
    
    std::ofstream f0(yuv_file0, std::ios::binary);
    std::ofstream f1(yuv_file1, std::ios::binary);
    
    if (f0.is_open() && f1.is_open()) {
        f0.write(image_data0, DATA_SIZE_LARGE);
        f1.write(image_data1, DATA_SIZE_LARGE);
        f0.close();
        f1.close();
        return 0;
    } else {
        perror("T1Vision-ImageSaveHighRes, open to write Ver image failed");
        return -1;
    }
}

/** receive image from T1GetIsp
 * return: 1 success, -1 failed
 * 2026.05.14: created by Leo
 */ 
int T1Vision::ImageRecive(int highRes) {
    char cmd;
    int current_data_size;

    if ( highRes==1 ) {
        cmd = 'f';
        current_data_size = DATA_SIZE_LARGE;
        //printf("T1Vision-Detect, using high resolution (1920x1080) NV12 format\n");
    } else {
        cmd = 's';
        current_data_size = DATA_SIZE_SMALL;
        //printf("T1Vision-Detect, using low resolution (640x360) BGR format\n");
    }
    
    if ( write(fd_cmd,&cmd,1)!=1 ) {
        perror("T1Vision-ImageRecive, write FIFO cmd failed");
        return -1;
    }
    //printf("T1Vision-Detect, send command [%c]??then waiiting for 3M yuv data......\n", cmd);
    //return 0;

    // reading datas from FIFO (loops to make sure all datas readed)
    ssize_t total_read0 = 0, total_read1 = 0;
    while (total_read0 < current_data_size) {
        ssize_t len = read(fd_data0, image_data0 + total_read0, current_data_size - total_read0);
        if ( len<=0 ) {
            perror("T1Vision-ImageRecive, parent read image_data1 failed");
            return -1;
        }
        total_read0 += len;
    }
    while (total_read1 < current_data_size) {
        ssize_t len = read(fd_data1, image_data1 + total_read1, current_data_size - total_read1);
        if (len <= 0) {
            perror("T1Vision-ImageRecive, parent read data2 failed");
            return -1;
        }
        total_read1 += len;
    }
    
    return 1;
}

/** the main for all vision process
 * return: >= 0 success, -1 failed
 * 2025.10.20: created by Leo
 * 2026.05.14: modified by Leo
 */ 
int T1Vision::Detect() {
    int debug=0; // 2:printf timecost,  0: not printf

    int ret=-1;
    char tmpstr[256];
    uint64_t timenow = T1Utils::getCurrentTimeUs();
    uint64_t time0 = timenow;
    uint64_t time1 = timenow;
    uint64_t timerecv = timenow;
    static unsigned int countDetect = 0;
    int flagHighRes = 0;
    //int ret=0;
    //printf("T1Vision-Detect, begin......\n");
    countDetect++;
    // CountHighLowRes is the count of high resolution images per low resolution image, default is 10, means 1 high res image after 9 low res image
    if ( countDetect%(meCountHighLowRes+1) == 0 ) {
        flagHighRes = 1;  // high resolution
    } else {
        flagHighRes = 0;  // low resolution
    }
    if ( flagAprilTag==1 ) { // this will be deleted sooner or later
        flagHighRes = 1;  // High resolution for AprilTag detection
    }

    if ( ImageRecive(flagHighRes)==-1 ) return -1;
    timeCalculate(debug, &time1, &timenow, "recive image");
    timerecv = timenow;

    //ret = ImageSave(flagSave);
    if ( ImageSave(flagHighRes)==-1 ) return -1;
    timeCalculate(debug, &time1, &timenow, "image save");
        //ImageSaveHighRes();

    //cv::Mat image = ImageConvert();
    int image = ImageConvert();
    timeCalculate(debug, &time1, &timenow, "image convert");

    ImageDisplay();
    timeCalculate(debug, &time1, &timenow, "image display");

    if ( flagHighRes==1 ) {
        // NV12 format (Y + UV planes, 1.5 bytes per pixel)
        meImageLeft = cv::Mat(HEIGHT + HEIGHT/2, WIDTH, CV_8UC1, image_data0);
        meImageRight = cv::Mat(HEIGHT + HEIGHT/2, WIDTH, CV_8UC1, image_data1);
        if (debug>3) printf("T1Vision-Detect, Count:%d/%d, High resolution(1920x1080) image processed.\n", countDetect, meCountHighLowRes);
        AprilTag(meImageLeft);
        if ( myAprilTag.getTagId() >= 0 && myAprilTag.getTagId() <=100 ) {
            ret = myAprilTag.getTagId();
            // Keep the full matrix pose for map conversion and mirror the three
            // control-facing values into the legacy PointRobo interface.
            const AprilTagResult tagResult = myAprilTag.getResult();
            myAprilTagPose = AprilTagPose();
            myAprilTagPose.tagId = tagResult.tagId;
            myAprilTagPose.tx = tagResult.tx;
            myAprilTagPose.ty = tagResult.ty;
            myAprilTagPose.tz = tagResult.tz;
            for (int i=0; i<9; ++i) myAprilTagPose.rotation[i] = tagResult.rotation[i];
            myAprilTagPose.relativeLateral = tagResult.relativeLateral;
            myAprilTagPose.relativeVertical = tagResult.relativeVertical;
            myAprilTagPose.normalDistance = tagResult.normalDistance;
            myAprilTagPose.yawError = tagResult.yawError;
            myAprilTagPose.reprojectionErrorPx = tagResult.reprojectionErrorPx;
            myAprilTagPose.valid = tagResult.valid;

            // myPointVA.fx = myAprilTagPose.tx;
            // myPointVA.fy = myAprilTagPose.tz;
            myPointVA.fx = myAprilTagPose.relativeLateral-3.0f ;
            myPointVA.fy = myAprilTagPose.normalDistance;
            myPointVA.yaw = (int)std::lround(myAprilTagPose.yawError);
            myPointVA.flagPosSetBy = 'A';
            myPointVA.timeSet = T1Utils::getCurrentTimeMs();
            // Both objects must carry the same frame timestamp.
            myAprilTagPose.timeSet = myPointVA.timeSet;
            // printf("\033[32mT1Vision-Detect, AprilTag detected! tagId=%d, cameraRaw=(tx=%.1f,ty=%.1f,tz=%.1f)cm, tagRelative=(x=%.1f,vertical=%.1f,normal=%.1f,yawError=%.1f), RMS=%.3fpx\033[0m\n",
            //     ret,
            //     myAprilTagPose.tx,myAprilTagPose.ty,myAprilTagPose.tz,
            //     myPointVA.fx,myAprilTagPose.relativeVertical,myPointVA.fy,myAprilTagPose.yawError,
            //     myAprilTagPose.reprojectionErrorPx);
            // printf("\033[32mT1Vision-Detect, AprilTag detected! tagId=%d, cameraRaw=(tx=%.1f,ty=%.1f,tz=%.1f)cm, tagRelative=(x=%.1f,vertical=%.1f,normal=%.1f,yawError=%.1f), RMS=%.3fpx\033[0m\n",
            //     ret,myAprilTagPose.tx,myAprilTagPose.ty,myAprilTagPose.tz,
            //     myAprilTagPose.relativeLateral,myAprilTagPose.relativeVertical,
            //     myAprilTagPose.normalDistance,myAprilTagPose.yawError,
            //     myAprilTagPose.reprojectionErrorPx);
            sprintf(tmpstr,"T1Vision-Detect, AprilTag timeSet updated: %ld\n", myPointVA.timeSet);
            //T1Utils::LogRecord(tmpstr, "timelock");

        } else {
            //printf("[DEBUG-Vision] AprilTag NOT detected, timeSet NOT updated (still: %ld)\n", myPointVA.timeSet);
        }
        flagAprilTag = 0;
        timeCalculate(debug, &time1, &timenow, "AprilTag");
    } else {
        meImageLeft = cv::Mat(HEIGHT_SMALL, WIDTH_SMALL, CV_8UC3, image_data0);
        meImageRight = cv::Mat(HEIGHT_SMALL, WIDTH_SMALL, CV_8UC3, image_data1);
        if (debug>3) printf("T1Vision-Detect, Count:%d/%d, Low resolution(640x360) image processed.\n", countDetect, meCountHighLowRes);
        
        myVideo.processFrame(meImageLeft, meImageRight);
        timeCalculate(debug, &time1, &timenow, "Video.processVideoFrame");
        
        meImageCroppedLeft = meImageLeft(cv::Rect(128,72,512,288));
        myUnet.start(meImageCroppedLeft);
        timeCalculate(debug, &time1, &timenow, "Unet.start");

        myYolo26.start(meImageLeft);
        timeCalculate(debug, &time1, &timenow, "Yolo26.start");
        
        mySgbm.start(meImageLeft,meImageRight);
        myDetectWall = myObsDisCalcu.calcHistNew(mySgbm.meDepthMap, 220, 210, 200, 50, 5, meAvoidThresh, meAvoidFreq);
        if (debug>2) printf("T1Vision-Detect, myDetectWall: leftDepth=%.1f, leftOffset=%.1f, rightDepth=%.1f, rightOffset=%.1f\n", myDetectWall.leftDepth, myDetectWall.leftOffset, myDetectWall.rightDepth, myDetectWall.rightOffset);
        if ( myDetectWall.leftDepth != 0 || myDetectWall.rightDepth != 0 ) {
            myPointVB.flagPosSetBy = 'B';
            myPointVB.timeSet = T1Utils::getCurrentTimeMs();
        }
        if ( debug>3 ) printf("T1Vision-Detect, calcHist wall: leftDepth=%.1f, leftOffset=%.1f, rightDepth=%.1f, rightOffset=%.1f\n", myDetectWall.leftDepth, myDetectWall.leftOffset, myDetectWall.rightDepth, myDetectWall.rightOffset);
        timeCalculate(debug, &time1, &timenow, "Sgbm.start");
        
        ret = 101;
    }

    timeCalculate(debug, &time0, &timenow, "0");
    return ret;
}

PointRobo T1Vision::GetPoint(char type) {
    PointRobo robopoint;
    switch (type) {
    case 'A': // AprilTag detection results
        robopoint.fx = myPointVA.fx;
        robopoint.fy = myPointVA.fy;
        robopoint.yaw = myPointVA.yaw;
        robopoint.roll = myPointVA.roll;
        robopoint.pitch = myPointVA.pitch;
        robopoint.timeSet = myPointVA.timeSet;
        robopoint.flagPosSetBy = myPointVA.flagPosSetBy;
        myPointVA.flagPosSetBy = '='; // reset flag immediately after reading
        return robopoint;
        break;
    case 'B': // Unet detection results
        robopoint.fx = myPointVB.fx;
        robopoint.fy = myPointVB.fy;
        robopoint.yaw = myPointVB.yaw;
        robopoint.roll = myPointVB.roll;
        robopoint.pitch = myPointVB.pitch;
        robopoint.timeSet = myPointVB.timeSet;
        robopoint.flagPosSetBy = myPointVB.flagPosSetBy;
        myPointVB.flagPosSetBy = '='; // reset flag immediately after reading
        return robopoint;
        break;
    default:
        printf("T1Vision-GetPoint, unknown point type.\n"); 
        return robopoint;
    break;
    }
    return robopoint;
}

AprilTagPose T1Vision::GetAprilTagPose() const {
    return myAprilTagPose;
}

int T1Vision::AprilTag(Mat& image) {
    myAprilTag.StartNV12(image,1920,1080, meApriltagSize);
    return 0;
}

float T1Vision::getVerticalDis() {
    return myAprilTag.getVerticalDistance();
}

float T1Vision::getHorizontalDis() {
    return myAprilTag.getHorizontalDistance();
}

float T1Vision::getYawAngle() {
    return myAprilTag.getYawAngle();
}

int T1Vision::getTagId() {
    return myAprilTag.getTagId();
}

// display the time cost of each part
int T1Vision::timeCalculate(int type, uint64_t *time1, uint64_t *timenow, const char *str) {
    static int seg_count=0;
    long long timecost=0;
    *timenow = T1Utils::getCurrentTimeUs();
    timecost = *timenow - *time1;
    if ( str[0]=='0' ) {
        seg_count = 0;
        if ( type>1 )
            printf("T1Vision-Detect total:%8.3fms, usleep:%dms\n\n", timecost/1000.0, 100 - (int)(timecost/1000));
    } else {
        if ( type>2 )
            printf("T1Vision-Detect, time cost:%8.3fms,   segment %d: %s\n", timecost/1000.0, seg_count, str);
    }
    *time1 = *timenow;
    seg_count++;
    return (int)(timecost/1000);
}

// Calculate the average depth for specified object ROI regions
double T1Vision::calcObjAvgDepth(const std::string& class_name) {
    // Get YOLO detection results
    std::vector<DetectionResult> detectionResults = myYolo26.getDetectionResults();
    
    // Prepare detection result data
    std::vector<std::string> classNames;
    std::vector<cv::Rect2d> boundingBoxes;
    
    for (const auto& result : detectionResults) {
        classNames.push_back(result.class_name);
        boundingBoxes.push_back(result.bounding_box);
    }
    
    // Calculate average depth for specified object class
    double avgdepth = myObsDisCalcu.calcObjAvgDepth(mySgbm.meDepthMap, boundingBoxes, classNames, class_name);
    
    return avgdepth;
}

Wall T1Vision::GetWall(){
    return myDetectWall;
}
