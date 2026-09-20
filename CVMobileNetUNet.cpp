#include "CVMobileNetUNet.hpp"
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

// 核心修改：默认构造函数（初始化路径，适配build同级结构）
CVMobileNetUNet::CVMobileNetUNet() {
    // 路径配置：可执行程序在build文件夹下，模型/图片与build同级
    m_modelPath = "./mobilenet_unet_1024x2048_nv12.bin";
    m_imgPath = "./segmentation.png";
    m_isModelInitialized = false; // 添加初始化标记
    std::cout << "[CVMobileNetUNet] Object created. Model path: " << m_modelPath 
              << ", Img path: " << m_imgPath << std::endl;
}

// 析构函数（释放资源）
CVMobileNetUNet::~CVMobileNetUNet() {
    releaseInferResources();
    if (m_packedDnnHandle != nullptr) {
        hbDNNRelease(m_packedDnnHandle);
        m_packedDnnHandle = nullptr;
    }
    std::cout << "[CVMobileNetUNet] Object destroyed. Resources released." << std::endl;
}

// 启动函数：接收已创建的cv::Mat图像并处理，直接显示结果
int CVMobileNetUNet::start(const cv::Mat& img) {
	//std::cout << "\n Unet Process" << std::endl;
    // 打印OpenCV版本
    //std::cout << "[OpenCV] Version: " << CV_VERSION << std::endl;
    //std::cout << "[CVMobileNetUNet] Starting process with provided image..." << std::endl;
    
    if (img.empty()) {
        std::cerr << "[Error] Invalid image data" << std::endl;
        return -1;
    }
    
    // 直接使用传入的图像
    m_originImg = img;
    
    //std::cout << "Image received successfully" << std::endl;
    
    // 调用独立的处理函数，使start函数更简洁
    int ret = 0;
    ret = initModel();      if (ret != 0) return ret;
    ret = allocateInferResources(); if (ret != 0) return ret;
    ret = preprocessFromMemory(); if (ret != 0) return ret;
    ret = infer();          if (ret != 0) return ret;
    
    std::vector<std::vector<int>> out;
    ret = postprocess(out); if (ret != 0) return ret;
    ret = visualizeAndDisplay(out); if (ret != 0) return ret;
    
    //std::cout << "\n Unet Process completed!" << std::endl;
    return 0;
}

// 初始化模型（补充模型名称打印）
int CVMobileNetUNet::initModel() {
    //std::cout << "\n[Step 1] Initializing model..." << std::endl;
    
    // 检查模型是否已经初始化，如果已经初始化则直接返回成功
    if (m_isModelInitialized) {
        //std::cout << "[CVMobileNetUNet] Model already initialized, skipping reloading." << std::endl;
        return 0;
    }
    
    auto beginTime = std::chrono::system_clock::now();

    const char* modelPathPtr = m_modelPath.c_str();
    RDK_CHECK_SUCCESS(
        hbDNNInitializeFromFiles(&m_packedDnnHandle, &modelPathPtr, 1),
        "Load model failed"
    );

    const char**modelNameList = nullptr;
    int32_t modelCount = 0;
    RDK_CHECK_SUCCESS(
        hbDNNGetModelNameList(&modelNameList, &modelCount, m_packedDnnHandle),
        "Get model name list failed"
    );
    // 补充：打印模型名称
    const char* modelName = modelNameList[0];
    std::cout << "[Model name]: " << modelName << std::endl;

    RDK_CHECK_SUCCESS(
        hbDNNGetModelHandle(&m_dnnHandle, m_packedDnnHandle, modelName),
        "Get model handle failed"
    );

    RDK_CHECK_SUCCESS(checkModelIO(), "Check model IO failed");

    // 设置模型已初始化标志
    m_isModelInitialized = true;
    
    auto loadTime = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - beginTime
    ).count() / 1000.0;
    std::cout << "\033[31mModel load time: " << loadTime << " ms\033[0m" << std::endl;
    return 0;
}

// 分配推理资源
int CVMobileNetUNet::allocateInferResources() {
    if (m_areInferResourcesAllocated) {
        // 释放旧资源后重新分配
        releaseInferResources();
    }

    // 获取输入张量属性
    hbDNNGetInputTensorProperties(&m_inputTensor.properties, m_dnnHandle, 0);
    m_inputMemSize = m_inputH * m_inputW * 3 / 2;

    // 分配输入内存
    RDK_CHECK_SUCCESS(
        hbSysAllocCachedMem(&m_inputTensor.sysMem[0], m_inputMemSize),
        "Allocate input mem failed"
    );
    m_inputMem = m_inputTensor.sysMem[0].virAddr;

    // 预分配处理图像的缓冲区
    m_preallocatedResizedImg.create(cv::Size(m_inputW, m_inputH), CV_8UC3);
    m_preallocatedYuvI420Img.create(cv::Size(m_inputW, m_inputH * 3 / 2), CV_8UC1);

    // 创建输出张量数组
    m_outputTensors = new hbDNNTensor[m_outputCount];
    for (int i = 0; i < m_outputCount; ++i) {
        hbDNNGetOutputTensorProperties(&m_outputTensors[i].properties, m_dnnHandle, i);
        int outSize = m_outputTensors[i].properties.alignedByteSize;
        RDK_CHECK_SUCCESS(
            hbSysAllocCachedMem(&m_outputTensors[i].sysMem[0], outSize),
            "Allocate output mem failed"
        );
    }

    m_areInferResourcesAllocated = true;
    return 0;
}

// 释放推理资源
void CVMobileNetUNet::releaseInferResources() {
    if (!m_areInferResourcesAllocated) {
        return;
    }

    // 释放输入内存
    if (m_inputTensor.sysMem[0].virAddr != nullptr) {
        hbSysFreeMem(&m_inputTensor.sysMem[0]);
        m_inputTensor.sysMem[0].virAddr = nullptr;
        m_inputMem = nullptr;
    }

    // 释放输出张量内存
    if (m_outputTensors != nullptr) {
        for (int i = 0; i < m_outputCount; ++i) {
            if (m_outputTensors[i].sysMem[0].virAddr != nullptr) {
                hbSysFreeMem(&m_outputTensors[i].sysMem[0]);
            }
        }
        delete[] m_outputTensors;
        m_outputTensors = nullptr;
    }

    // 重置标记
    m_areInferResourcesAllocated = false;
}

// 私有函数：检查模型输入输出（补充完整校验）
int CVMobileNetUNet::checkModelIO() {
    int32_t inputCount = 0;
    RDK_CHECK_SUCCESS(hbDNNGetInputCount(&inputCount, m_dnnHandle), "Get input count failed");
    if (inputCount > 1) {
        std::cerr << "Model has more than 1 input (expected 1)" << std::endl;
        return -1;
    }

    hbDNNTensorProperties inputProp;
    RDK_CHECK_SUCCESS(hbDNNGetInputTensorProperties(&inputProp, m_dnnHandle, 0), "Get input prop failed");
    
    // 补充：检查输入维度是否为4（NV12格式特征）
    if (inputProp.validShape.numDimensions != 4) {
        std::cerr << "Input tensor is not NV12 (expected 4 dimensions)" << std::endl;
        return -1;
    }

    // 补充：检查输入布局是否为NCHW
    if (inputProp.tensorType != 1) {
        std::cerr << "Input tensor layout is not HB_DNN_LAYOUT_NCHW, please check!" << std::endl;
        return -1;
    }
    std::cout << "Input tensor layout: HB_DNN_LAYOUT_NCHW" << std::endl;

    // 补充：打印完整输入shape
    m_inputH = inputProp.validShape.dimensionSize[2];
    m_inputW = inputProp.validShape.dimensionSize[3];
    std::cout << "Input tensor valid shape: (" 
              << inputProp.validShape.dimensionSize[0] << ", "  // N
              << inputProp.validShape.dimensionSize[1] << ", "  // C
              << m_inputH << ", "  // H
              << m_inputW << ")" << std::endl;  // W

    // 补充：输出校验与完整shape打印
    RDK_CHECK_SUCCESS(hbDNNGetOutputCount(&m_outputCount, m_dnnHandle), "Get output count failed");
    if (m_outputCount != 1) {
        std::cerr << "Model has more than 1 output (expected 1)" << std::endl;
        return -1;
    }
    hbDNNTensorProperties outputProp;
    hbDNNGetOutputTensorProperties(&outputProp, m_dnnHandle, 0);
    std::cout << "Output tensor valid shape: (" 
              << outputProp.validShape.dimensionSize[0] << ", " 
              << outputProp.validShape.dimensionSize[1] << ", " 
              << outputProp.validShape.dimensionSize[2] << ", " 
              << outputProp.validShape.dimensionSize[3] << ")" << std::endl;

    return 0;
}

// 预处理（补充时间统计）
int CVMobileNetUNet::preprocess() {
    //std::cout << "\n[Step 2] Preprocessing..." << std::endl;
    m_originImg = cv::imread(m_imgPath);
    if (m_originImg.empty()) {
        std::cerr << "Read image failed" << std::endl;
        return -1;
    }
    // 补充：打印原图信息
    std::cout << "Original image (rows, cols, channels): (" 
              << m_originImg.rows << ", " << m_originImg.cols << ", " << m_originImg.channels() << ")" << std::endl;

    // 补充：Resize时间统计
    auto beginTime = std::chrono::system_clock::now();
    cv::Mat resizedImg;
    cv::resize(m_originImg, resizedImg, cv::Size(m_inputW, m_inputH));
    auto resizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - beginTime
    ).count() / 1000.0;
    //std::cout << "\033[31mResize time: " << resizeTime << " ms\033[0m" << std::endl;

    // 补充：格式转换时间统计
    beginTime = std::chrono::system_clock::now();
    cv::Mat yuvI420;
    cv::cvtColor(resizedImg, yuvI420, cv::COLOR_BGR2YUV_I420);
    m_nv12Img.create(m_inputH * 3 / 2, m_inputW, CV_8UC1);
    uint8_t* yuvData = yuvI420.ptr<uint8_t>();
    uint8_t* nv12Data = m_nv12Img.ptr<uint8_t>();
    int ySize = m_inputH * m_inputW;
    memcpy(nv12Data, yuvData, ySize);
    uint8_t* uData = yuvData + ySize;
    uint8_t* vData = uData + ySize / 4;
    uint8_t* nv12UV = nv12Data + ySize;
    for (int i = 0; i < ySize / 4; ++i) {
        *nv12UV++ = *uData++;
        *nv12UV++ = *vData++;
    }
    auto convertTime = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - beginTime
    ).count() / 1000.0;
    std::cout << "\033[31mBGR to NV12 time: " << convertTime << " ms\033[0m" << std::endl;

    return 0;
}

// 推理（补充时间统计）
int CVMobileNetUNet::infer() {
    //std::cout << "\n[Step 3] Running inference..." << std::endl;
    auto beginTime = std::chrono::system_clock::now();  // 补充：推理总时间统计

    // 检查资源是否已经分配
    if (!m_areInferResourcesAllocated) {
        RDK_CHECK_SUCCESS(allocateInferResources(), "Allocate infer resources failed");
    }

    // 复制NV12图像数据到预分配的输入内存
    memcpy(m_inputMem, m_nv12Img.ptr<uint8_t>(), m_inputMemSize);
    hbSysFlushMem(&m_inputTensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

    hbDNNTaskHandle_t taskHandle = nullptr;
    hbDNNInferCtrlParam inferCtrl;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&inferCtrl);
    RDK_CHECK_SUCCESS(hbDNNInfer(&taskHandle, &m_outputTensors, &m_inputTensor, m_dnnHandle, &inferCtrl), "Infer failed");
    hbDNNWaitTaskDone(taskHandle, 0);
    
    // 补充：推理时间统计
    auto inferTime = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - beginTime
    ).count() / 1000.0;
    //std::cout << "\033[31mInference time: " << inferTime << " ms\033[0m" << std::endl;

    // 释放资源
    hbDNNReleaseTask(taskHandle);
    
    return 0;
}

// 后处理（修复维度索引 + 补充时间统计）
int CVMobileNetUNet::postprocess(std::vector<std::vector<int>>& outClassMatrix) {
    //std::cout << "\n[Step 4] Postprocessing..." << std::endl;
    auto beginTime = std::chrono::system_clock::now();  // 补充：后处理时间统计

    if (m_outputTensors == nullptr) {
        std::cerr << "[Error] m_outputTensors is nullptr in postprocess()" << std::endl;
        return -1;
    }
    
    hbDNNTensor& outTensor = m_outputTensors[0];
    hbSysFlushMem(&outTensor.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    float* data = reinterpret_cast<float*>(outTensor.sysMem[0].virAddr);

    // 修复：维度索引与原始代码一致（N, H, W, C）
    int outH = outTensor.properties.validShape.dimensionSize[1];  // 原错误：[2] → 修复为[1]
    int outW = outTensor.properties.validShape.dimensionSize[2];  // 原错误：[3] → 修复为[2]
    int outC = outTensor.properties.validShape.dimensionSize[3];  // 原错误：[1] → 修复为[3]
    outClassMatrix.resize(outH, std::vector<int>(outW, 0));

    for (int i = 0; i < outH; ++i) {
        for (int j = 0; j < outW; ++j) {
            int base = (i * outW + j) * outC;
            float maxVal = std::numeric_limits<float>::lowest();
            int maxClass = 0;
            for (int c = 0; c < outC; ++c) {
                if (data[base + c] > maxVal) {
                    maxVal = data[base + c];
                    maxClass = c;
                }
            }
            outClassMatrix[i][j] = (maxClass < NUM_CLASSES) ? maxClass : 0;
        }
    }

    // 补充：后处理时间统计
    auto postTime = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - beginTime
    ).count() / 1000.0;
    //std::cout << "\033[31mPostprocessing time: " << postTime << " ms\033[0m" << std::endl;

    return 0;
}

// 绘制结果（修复维度索引 + 补充时间统计）
int CVMobileNetUNet::plotResult(const std::vector<std::vector<int>>& outClassMatrix) {
    std::cout << "\n[Step 5] Plotting result..." << std::endl;
    if (outClassMatrix.empty()) return -1;
    auto beginTime = std::chrono::system_clock::now();  // 补充：绘图时间统计

    int outH = outClassMatrix.size();
    int outW = outClassMatrix[0].size();

    std::vector<unsigned char> palette = {
        128,  64, 128, 244,  35, 232,  70,  70,  70, 102, 102, 156,
        190, 153, 153, 153, 153, 153, 250, 170,  30, 220, 220,   0,
        107, 142,  35, 152, 251, 152,   0, 130, 180, 220,  20,  60,
        255,   0,   0,   0,   0, 142,   0,   0,  70,   0,  60, 100,
          0,  80, 100,   0,   0, 230, 119,  11,  32
    };

    // 补充：掩码值越界检查与统计
    cv::Mat grayMask(outH, outW, CV_8UC1);
    int invalidCount = 0;
    for (int i = 0; i < outH; ++i) {
        for (int j = 0; j < outW; ++j) {
            int val = outClassMatrix[i][j];
            if (val < 0 || val >= NUM_CLASSES) {
                val = 0;  // 越界值设为默认类别
                invalidCount++;
            }
            grayMask.at<unsigned char>(i, j) = val;
        }
    }
    if (invalidCount > 0) {
        std::cerr << "Warning: " << invalidCount 
                  << " out-of-range values (expected 0 to " << NUM_CLASSES - 1 << "), set to 0.\n";
    }

    // 补充：打印掩码值范围
    double minVal, maxVal;
    cv::minMaxLoc(grayMask, &minVal, &maxVal);
    std::cout << "Mask value range: [" << minVal << ", " << maxVal << "]\n";

    cv::Mat resizedMask;
    cv::resize(grayMask, resizedMask, m_originImg.size(), 0, 0, cv::INTER_NEAREST);
    cv::Mat colorMask(m_originImg.size(), CV_8UC3);
    for (int i = 0; i < colorMask.rows; ++i) {
        for (int j = 0; j < colorMask.cols; ++j) {
            int idx = resizedMask.at<unsigned char>(i, j);
            // 补充：二次越界检查
            if (idx < 0 || idx >= NUM_CLASSES) idx = 0;
            int base = idx * 3;
            colorMask.at<cv::Vec3b>(i, j) = cv::Vec3b(palette[base+2], palette[base+1], palette[base]);
        }
    }
    std::string outputDir="./output";
    if(access(outputDir.c_str(),0) == -1 ){
		mkdir(outputDir.c_str(),0755);
	}
    
    cv::imwrite(outputDir+"/output_mask.png", resizedMask);
    cv::imwrite(outputDir+"/output_color.png", colorMask);

    cv::Mat result;
    cv::addWeighted(m_originImg, 0.4, colorMask, 0.6, 0, result);
    cv::imwrite(outputDir+"/segment_result.png", result);
    std::cout << "Segment Result saved as: segment_result.png" << std::endl;

    // 补充：绘图时间统计
    auto plotTime = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - beginTime
    ).count() / 1000.0;
    std::cout << "\033[31mPlotting time: " << plotTime << " ms\033[0m" << std::endl;

    return 0;
}

// 从内存加载图像的预处理
int CVMobileNetUNet::preprocessFromMemory() {
    //std::cout << "\n[Step 2] Preprocessing from memory..." << std::endl;
    auto beginTime = std::chrono::system_clock::now();
    
    // 检查资源是否已经分配
    if (!m_areInferResourcesAllocated) {
        RDK_CHECK_SUCCESS(allocateInferResources(), "Allocate infer resources failed");
    }
    
    // 调整图像大小到预分配的缓冲区
    cv::resize(m_originImg, m_preallocatedResizedImg, cv::Size(m_inputW, m_inputH));
    
    // 转换为NV12格式，使用预分配的缓冲区
    cv::cvtColor(m_preallocatedResizedImg, m_preallocatedYuvI420Img, cv::COLOR_BGR2YUV_I420);
    
    // 确保m_nv12Img已经创建
    if (m_nv12Img.empty()) {
        m_nv12Img.create(m_inputH * 3 / 2, m_inputW, CV_8UC1);
    }
    
    uint8_t* yuvData = m_preallocatedYuvI420Img.ptr<uint8_t>();
    uint8_t* nv12Data = m_nv12Img.ptr<uint8_t>();
    
    // 计算Y通道大小
    int ySize = m_inputH * m_inputW;
    
    // 复制Y通道
    memcpy(nv12Data, yuvData, ySize);
    
    // 合并UV通道
    uint8_t* uData = yuvData + ySize;
    uint8_t* vData = uData + ySize / 4;
    uint8_t* nv12UV = nv12Data + ySize;
    for (int i = 0; i < ySize / 4; ++i) {
        *nv12UV++ = *uData++;
        *nv12UV++ = *vData++;
    }
    
    auto preprocessTime = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - beginTime
    ).count() / 1000.0;
    //std::cout << "\033[31mPreprocessing time: " << preprocessTime << " ms\033[0m" << std::endl;
    
    return 0;
}

// 结果可视化与显示
int CVMobileNetUNet::visualizeAndDisplay(const std::vector<std::vector<int>>& outClassMatrix) {
    //std::cout << "\n[Step 5] Generating and displaying results..." << std::endl;
    auto beginTime = std::chrono::system_clock::now();
    
    if (outClassMatrix.empty()) {
        std::cerr << "Output class matrix is empty" << std::endl;
        return -1;
    }
    
    int outH = outClassMatrix.size();
    int outW = outClassMatrix[0].size();
    
    // 定义调色板
    std::vector<unsigned char> palette = {
        128,  64, 128, 244,  35, 232,  70,  70,  70, 102, 102, 156,
        190, 153, 153, 153, 153, 153, 250, 170,  30, 220, 220,   0,
        107, 142,  35, 152, 251, 152,   0, 130, 180, 220,  20,  60,
        255,   0,   0,   0,   0, 142,   0,   0,  70,   0,  60, 100,
          0,  80, 100,   0,   0, 230, 119,  11,  32
    };
    
    // 创建灰度掩码
    cv::Mat grayMask(outH, outW, CV_8UC1);
    int invalidCount = 0;
    for (int i = 0; i < outH; ++i) {
        for (int j = 0; j < outW; ++j) {
            int val = outClassMatrix[i][j];
            if (val < 0 || val >= NUM_CLASSES) {
                val = 0;  // 越界值设为默认类别
                invalidCount++;
            }
            grayMask.at<unsigned char>(i, j) = val;
        }
    }
    
    // 调整掩码大小与原图匹配
    cv::Mat resizedMask;
    cv::resize(grayMask, resizedMask, m_originImg.size(), 0, 0, cv::INTER_NEAREST);
    
    // 创建彩色掩码
    cv::Mat colorMask(m_originImg.size(), CV_8UC3);
    for (int i = 0; i < colorMask.rows; ++i) {
        for (int j = 0; j < colorMask.cols; ++j) {
            int idx = resizedMask.at<unsigned char>(i, j);
            if (idx < 0 || idx >= NUM_CLASSES) idx = 0;
            int base = idx * 3;
            colorMask.at<cv::Vec3b>(i, j) = cv::Vec3b(palette[base+2], palette[base+1], palette[base]);
        }
    }
    
    // 创建融合结果
    cv::Mat result;
    cv::addWeighted(m_originImg, 0.4, colorMask, 0.6, 0, result);
    cv::resize(result, result, cv::Size(640, 360));
    
    // 显示三张结果图像
    //cv::namedWindow("Original Image", cv::WINDOW_NORMAL);
    //cv::namedWindow("Segmentation Mask", cv::WINDOW_NORMAL);
    cv::namedWindow("Segmentation Result", cv::WINDOW_NORMAL);
    
    //cv::imshow("Original Image", m_originImg);
    //cv::imshow("Segmentation Mask", colorMask);
    cv::imshow("Segmentation Result", result);
    
    // 添加窗口调整和延迟，确保图像显示
    //cv::resizeWindow("Original Image", 640, 360);
    //cv::resizeWindow("Segmentation Mask", 640, 360);
    cv::resizeWindow("Segmentation Result", 640, 360);
    cv::moveWindow("Segmentation Result", 1260, 0);
    
    cv::waitKey(1); // 非阻塞显示
    
    auto displayTime = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - beginTime
    ).count() / 1000.0;
    //std::cout << "\033[31mDisplay time: " << displayTime << " ms\033[0m" << std::endl;
    
    return 0;
}

// 静态函数：保存结果（保持不变）
void CVMobileNetUNet::saveOutToFile(const std::vector<std::vector<int>>& out, const std::string& filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return;
    }
    for (const auto& row : out) {
        for (size_t j = 0; j < row.size(); ++j) {
            ofs << row[j] << (j+1 < row.size() ? " " : "\n");
        }
    }
    std::cout << "Output class matrix saved to: " << filename << std::endl;
}
