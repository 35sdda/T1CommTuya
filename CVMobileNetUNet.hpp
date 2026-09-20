#ifndef CVMOBILENET_UNET_HPP
#define CVMOBILENET_UNET_HPP

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <limits>
#include <fstream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"
#include <unistd.h>
#include <sys/stat.h>

#define LINE_THICKNESS 1
#define NUM_CLASSES 19

#define RDK_CHECK_SUCCESS(value, errmsg)                        \
    do                                                          \
    {                                                           \
        auto ret_code = value;                                  \
        if (ret_code != 0)                                      \
        {                                                       \
            std::cout << errmsg << ", error code:" << ret_code; \
            return ret_code;                                    \
        }                                                       \
    } while (0);

class CVMobileNetUNet {
private:
    // 私有成员变量（包含路径）
    std::string m_modelPath;       // 模型路径（类内部初始化）
    std::string m_imgPath;         // 图片路径（类内部初始化）
    hbPackedDNNHandle_t m_packedDnnHandle = nullptr;
    hbDNNHandle_t m_dnnHandle = nullptr;
    hbDNNTensor m_inputTensor;
    hbDNNTensor* m_outputTensors = nullptr;
    int32_t m_outputCount = 0;
    cv::Mat m_originImg;
    cv::Mat m_nv12Img;
    int32_t m_inputH = 0;
    int32_t m_inputW = 0;
    bool m_isModelInitialized = false;
    // 资源管理相关
    bool m_areInferResourcesAllocated = false;
    // 内存池相关
    void* m_inputMem = nullptr;
    size_t m_inputMemSize = 0;
    // 预分配的输入缓冲区，避免重复创建
    cv::Mat m_preallocatedInputBuffer;
    cv::Mat m_preallocatedNv12Img;
    cv::Mat m_preallocatedResizedImg;
    cv::Mat m_preallocatedYuvI420Img;

    // 私有辅助函数
    int checkModelIO();
    int allocateInferResources();   // 分配推理相关资源
    void releaseInferResources();   // 释放推理相关资源

public:
    // 核心修改：默认构造函数（内部初始化路径）
    CVMobileNetUNet();
    // 析构函数
    ~CVMobileNetUNet();

    // 内部步骤函数
    int initModel();
    int preprocess();           // 从文件加载图像的预处理
    int preprocessFromMemory(); // 从内存加载图像的预处理
    int infer();
    int postprocess(std::vector<std::vector<int>>& outClassMatrix);
    int plotResult(const std::vector<std::vector<int>>& outClassMatrix);
    int visualizeAndDisplay(const std::vector<std::vector<int>>& outClassMatrix); // 结果可视化与显示

    int start(const cv::Mat& img);
    static void saveOutToFile(const std::vector<std::vector<int>>& out, const std::string& filename = "output.txt");
};

#endif // MOBILENET_UNET_HPP
