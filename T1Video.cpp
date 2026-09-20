#include "T1Video.hpp"
#include "T1Utils.hpp"
#include <iostream>
#include <sstream>
#include <ctime>
#include <cstdio>
#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

T1Video::T1Video() : isRecording(false) {
}

T1Video::~T1Video() {
    if (isRecording && videoWriter.isOpened()) {
        videoWriter.release();
        isRecording = false;
    }
}

// Check if directory exists
bool directoryExists(const std::string& path) {
    struct stat info;
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

// Create directory recursively
bool createDirectory(const std::string& path) {
    size_t pos = 0;
    std::string dir;
    int ret = 0;
    
    if (path[path.size() - 1] != '/') {
        dir = path + "/";
    } else {
        dir = path;
    }
    
    while ((pos = dir.find_first_of("/", pos + 1)) != std::string::npos) {
        std::string subdir = dir.substr(0, pos);
        if (!directoryExists(subdir)) {
            ret = mkdir(subdir.c_str(), 0755);
            if (ret != 0 && errno != EEXIST) {
                return false;
            }
        }
    }
    return true;
}

// Scan video files in the folder
void T1Video::scanExistingVideos() {
    videoFilePaths.clear();
    
    if (!directoryExists(basePath)) {
        createDirectory(basePath);
        return;
    }
    
    DIR* dir = opendir(basePath.c_str());
    if (dir == nullptr) {
        return;
    }
    
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;
        // Check if it is a video file (video_*.avi)
        if (filename.find("video_") == 0 && filename.find(".avi") != std::string::npos) {
            videoFilePaths.push_back(basePath + filename);
        }
    }
    
    closedir(dir);
    
    // Sort by filename (chronological order)
    std::sort(videoFilePaths.begin(), videoFilePaths.end());
    
    std::cout << "Found " << videoFilePaths.size() << " existing video files" << std::endl;
}

// Delete the oldest video file
void T1Video::deleteOldestVideo() {
    if (videoFilePaths.empty()) return;
    
    std::string oldestFile = videoFilePaths.front();
    videoFilePaths.erase(videoFilePaths.begin());
    
    if (std::remove(oldestFile.c_str()) == 0) {
        std::cout << "deleted oldest file: " << oldestFile << std::endl;
    } else {
        std::cerr << "failed to delete file: " << oldestFile << std::endl;
    }
}

void T1Video::startRecording() {
    basePath = "../Yuv/";
    
    // Scan existing video files
    scanExistingVideos();
    
    // If there are 100 or more videos, delete the oldest until only 99 remain
    while (videoFilePaths.size() >= 100) {
        deleteOldestVideo();
    }
    
    // Create first file with timestamp using unified T1Utils time functions
    std::string dateStr = T1Utils::getCurrentYYYYMMDD();
    std::string timeStr = T1Utils::getCurrentHHMMSS();
    
    std::stringstream ss;
    ss << basePath << "video_" << dateStr << "_" << timeStr << ".avi";
    videoPath = ss.str();
    
    int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D'); 
    double fps = 15; 
    cv::Size frameSize(1280, 360); 
    
    videoWriter.open(videoPath, fourcc, fps, frameSize);
    
    if (videoWriter.isOpened()) {
        isRecording = true;
        currentFileStartTime = std::chrono::system_clock::now();
        videoFilePaths.push_back(videoPath);
        std::cout << "start to record " << videoPath << " (total files: " << videoFilePaths.size() << ")" << std::endl;
    } else {
        std::cerr << "failed to record " << videoPath << std::endl;
    }
}

void T1Video::stopRecording() {
    if (isRecording && videoWriter.isOpened()) {
        videoWriter.release();
        isRecording = false;
        std::cout << "save" << videoPath << std::endl;
    }
}

void T1Video::rotateVideoFile() {
    // Close current file
    if (videoWriter.isOpened()) {
        videoWriter.release();
        std::cout << "save " << videoPath << std::endl;
    }
    
    // If there are 100 or more videos, delete the oldest until only 99 remain
    while (videoFilePaths.size() >= 100) {
        deleteOldestVideo();
    }
    
    // Create new file with timestamp using unified T1Utils time functions
    std::string dateStr = T1Utils::getCurrentYYYYMMDD();
    std::string timeStr = T1Utils::getCurrentHHMMSS();
    
    std::stringstream ss;
    ss << basePath << "video_" << dateStr << "_" << timeStr << ".avi";
    videoPath = ss.str();
    
    int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D'); 
    double fps = 15; 
    cv::Size frameSize(1280, 360); 
    
    videoWriter.open(videoPath, fourcc, fps, frameSize);
    
    if (videoWriter.isOpened()) {
        currentFileStartTime = std::chrono::system_clock::now();
        videoFilePaths.push_back(videoPath);
        std::cout << "start to record " << videoPath << " (total files: " << videoFilePaths.size() << ")" << std::endl;
    } else {
        std::cerr << "failed to record " << videoPath << std::endl;
    }
}

void T1Video::processFrame(cv::Mat left_image, cv::Mat right_image) {
    if (left_image.empty() || right_image.empty()) {
        return;
    }

    cv::Mat combined_image;
    cv::hconcat(left_image, right_image, combined_image);

    if (isRecording && videoWriter.isOpened()) {
        // Check if 60 seconds (1 minute) has elapsed before writing, rotate to next file if so
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - currentFileStartTime);
        if (elapsed.count() >= 60) {
            rotateVideoFile();
        }

        // Write frame to current file
        if (videoWriter.isOpened()) {
            videoWriter.write(combined_image);
        }
    }
}