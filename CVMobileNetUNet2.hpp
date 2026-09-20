#ifndef CV_MOBILENET_UNET2_HPP
#define CV_MOBILENET_UNET2_HPP

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <opencv2/opencv.hpp>

#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"

struct TensorShapeInfo {
    int n = 1;
    int c = 1;
    int h = 0;
    int w = 0;
    bool is_nchw = true;
};

class CVMobileNetUNet2 {
public:
    CVMobileNetUNet2();
    ~CVMobileNetUNet2();

    int start(const cv::Mat& img, const std::string& model_path = "Unet.bin", float threshold = 0.75f);
    void release();

    const cv::Mat& getBinaryMask() const;

private:
    int initModel();
    int allocateInferResources();
    int preprocessFromMemory(const cv::Mat& bgr);
    int infer();
    int postprocess(std::vector<std::vector<int>>& out);
    int visualizeAndDisplay(const std::vector<std::vector<int>>& out);
    void freeInputTensor();
    void freeInferResources();

    static TensorShapeInfo parseShape(const hbDNNTensorProperties& prop);
    static cv::Mat bgrToNV12(const cv::Mat& bgr);
    static inline float sigmoid(float x);

private:
    hbPackedDNNHandle_t packed_dnn_handle_;
    hbDNNHandle_t dnn_handle_;
    hbDNNTensorProperties input_properties_;
    hbDNNTensorProperties output_properties_;
    TensorShapeInfo input_shape_;
    TensorShapeInfo output_shape_;
    int output_count_;
    float threshold_;
    std::string model_path_;
    std::string model_name_;

    cv::Mat m_originImg;
    cv::Mat m_binaryMask;
    hbDNNTensor* m_input_tensor_;
    hbDNNTensor* m_output_tensors_;
    hbDNNTaskHandle_t m_task_handle_;
    bool m_initialized_;
    bool m_resources_allocated_;
};

#endif // CV_MOBILENET_UNET2_HPP
