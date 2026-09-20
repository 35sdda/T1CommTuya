#ifndef T1VIDEO_HPP
#define T1VIDEO_HPP

#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include <vector>

class T1Video {
public:
    T1Video();
    ~T1Video();
    
    void startRecording();
    void stopRecording();
    void processFrame(cv::Mat left_image, cv::Mat right_image);
    
private:
    void rotateVideoFile(); // Rotate to next video file
    void scanExistingVideos(); // Scan video files in the folder
    void deleteOldestVideo(); // Delete the oldest video file
    
    cv::VideoWriter videoWriter;
    std::string videoPath;
    bool isRecording;
    std::chrono::system_clock::time_point currentFileStartTime; // Current file start time
    std::string basePath; // Base path for video files
    std::vector<std::string> videoFilePaths; // List of video file paths for rotation
};

#endif // T1VIDEO_HPP
