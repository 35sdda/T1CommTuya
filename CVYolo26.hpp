#ifndef CVYOLO26_HPP
#define CVYOLO26_HPP

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "dnn/hb_dnn.h"
#include "dnn/hb_dnn_ext.h"

struct DetectionResult {
    std::string class_name;
    float confidence;
    cv::Rect2d bounding_box;
};

// 宏定义（错误检查）
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

class CVYolo26 {
private:
    // 配置参数（私有成员）
    const std::string MODEL_PATH = "Yolo.bin";
    const int PREPROCESS_TYPE = 1; // LETTERBOX_TYPE
    const int CLASSES_NUM = 30;
    const std::vector<float> ANCHORS = {10.0, 13.0, 16.0, 30.0, 33.0, 23.0, 30.0, 61.0, 62.0, 45.0, 59.0, 119.0, 116.0, 90.0, 156.0, 198.0, 373.0, 326.0};
    const float NMS_THRESHOLD = 0.45;
    const float SCORE_THRESHOLD = 0.25;
    const int NMS_TOP_K = 300;
    const float MYAPP_FONT_SIZE = 1.0;
    const float FONT_THICKNESS = 1.0;
    const float LINE_SIZE = 2.0;

    // 类别名称
std::vector<std::string> object_names = { 
    "person", "bicycle", "car", "motorcycle", "fire hydrant",
    "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "backpack", "umbrella", "handbag", "suitcase", "frisbee",
    "sports ball", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "chair", "fence",
    "shoe", "charging station", "football", "tree" };

    // 模型相关句柄（私有成员）
    hbPackedDNNHandle_t packed_dnn_handle = nullptr;
    hbDNNHandle_t dnn_handle = nullptr;
    hbDNNTensor input;
    hbDNNTensor* output = nullptr;
    int output_count = 0;
    int input_H, input_W;
    int order[3] = {0, 1, 2};

    // 预处理相关参数（私有成员）
    float y_scale = 1.0;
    float x_scale = 1.0;
    int x_shift = 0;
    int y_shift = 0;

    // 检测结果存储
    std::vector<DetectionResult> detection_results;
    // 保存原始图像用于后续重新渲染
    cv::Mat original_image;
    // 标记模型是否已加载
    bool isModelLoaded = false;
    // 标记推理资源是否已分配
    bool areInferResourcesAllocated = false;
    // 内存池相关
    hbSysMem input_mem;
    size_t input_mem_size = 0;
    // 预分配的输入缓冲区，避免重复创建
    cv::Mat preallocated_input_buffer;
    cv::Mat preallocated_processed_img;

    // 私有成员函数（需要用 Yolo26:: 限定）
    int loadModel();                  // 加载模型（替代原 initModel）
    int preprocess(const cv::Mat& img, cv::Mat& processed_img);  // 预处理
    int infer(const cv::Mat& processed_img);                     // 推理
    int postprocessAndRender(cv::Mat& img);                      // 后处理+渲染
    void releaseResources();                                      // 释放所有资源
    void releaseInferResources();                                 // 释放推理相关资源
    int allocateInferResources();                                 // 分配推理相关资源
	
public:
    CVYolo26();               			// 构造函数
    ~CVYolo26();                        // 析构函数（自动释放资源）
    //int start();                       // 对外接口：启动推理流程
    int start(const cv::Mat& img);  
    // 获取检测结果
    std::vector<DetectionResult> getDetectionResults();
};

#endif // YOLO26_HPP
