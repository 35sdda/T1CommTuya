#include "CVMobileNetUNet2.hpp"

CVMobileNetUNet2::CVMobileNetUNet2()
    : packed_dnn_handle_(nullptr),
      dnn_handle_(nullptr),
      output_count_(0),
      threshold_(0.5f),
      m_input_tensor_(nullptr),
      m_output_tensors_(nullptr),
      m_task_handle_(nullptr),
      m_initialized_(false),
      m_resources_allocated_(false) {
}

CVMobileNetUNet2::~CVMobileNetUNet2() {
    release();
}

int CVMobileNetUNet2::start(const cv::Mat& img, const std::string& model_path, float threshold) {
    if (!m_initialized_) {
        threshold_ = threshold;
        model_path_ = model_path;
        int ret = initModel();
        if (ret != 0) return ret;

        ret = allocateInferResources();
        if (ret != 0) return ret;
    }

    if (img.empty()) {
        std::cerr << "CVMobileNetUNet2::start [Error] Invalid image data" << std::endl;
        return -1;
    }

    m_originImg = img;

    int ret = 0;
    ret = preprocessFromMemory(img);
    if (ret != 0) {
        freeInferResources();
        m_resources_allocated_ = false;
        return ret;
    }

    ret = infer();
    if (ret != 0) {
        freeInferResources();
        m_resources_allocated_ = false;
        return ret;
    }

    std::vector<std::vector<int>> out;
    ret = postprocess(out);
    if (ret != 0) {
        freeInferResources();
        m_resources_allocated_ = false;
        return ret;
    }

    ret = visualizeAndDisplay(out);
    if (ret != 0) {
        freeInferResources();
        m_resources_allocated_ = false;
        return ret;
    }

    freeInputTensor();

    return 0;
}

void CVMobileNetUNet2::release() {
    freeInferResources();

    if (packed_dnn_handle_ != nullptr) {
        hbDNNRelease(packed_dnn_handle_);
        packed_dnn_handle_ = nullptr;
        dnn_handle_ = nullptr;
    }

    m_initialized_ = false;
    m_resources_allocated_ = false;
    m_originImg.release();
    m_binaryMask.release();
}

int CVMobileNetUNet2::initModel() {
    if (m_initialized_) {
        return 0;
    }

    const char* model_file = model_path_.c_str();

    int ret = hbDNNInitializeFromFiles(&packed_dnn_handle_, &model_file, 1);
    if (ret != 0) {
        std::cerr << "CVMobileNetUNet2::initModel [Error] hbDNNInitializeFromFiles failed, ret=" << ret << std::endl;
        return ret;
    }

    const char** model_name_list = nullptr;
    int model_count = 0;
    ret = hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle_);
    if (ret != 0 || model_count <= 0) {
        std::cerr << "CVMobileNetUNet2::initModel [Error] hbDNNGetModelNameList failed, ret=" << ret << std::endl;
        return ret;
    }

    model_name_ = model_name_list[0];
    std::cout << "CVMobileNetUNet2::initModel [Model] name: " << model_name_ << std::endl;

    ret = hbDNNGetModelHandle(&dnn_handle_, packed_dnn_handle_, model_name_.c_str());
    if (ret != 0) {
        std::cerr << "CVMobileNetUNet2::initModel [Error] hbDNNGetModelHandle failed, ret=" << ret << std::endl;
        return ret;
    }

    int input_count = 0;
    ret = hbDNNGetInputCount(&input_count, dnn_handle_);
    if (ret != 0) {
        std::cerr << "CVMobileNetUNet2::initModel [Error] hbDNNGetInputCount failed, ret=" << ret << std::endl;
        return ret;
    }
    if (input_count != 1) {
        std::cerr << "CVMobileNetUNet2::initModel [Error] This framework only supports single-input model, got "
                  << input_count << std::endl;
        return -1;
    }

    ret = hbDNNGetInputTensorProperties(&input_properties_, dnn_handle_, 0);
    if (ret != 0) {
        std::cerr << "CVMobileNetUNet2::initModel [Error] hbDNNGetInputTensorProperties failed, ret=" << ret << std::endl;
        return ret;
    }

    input_shape_ = parseShape(input_properties_);
    std::cout << "CVMobileNetUNet2::initModel [Input] shape = ("
              << input_shape_.n << ", "
              << input_shape_.c << ", "
              << input_shape_.h << ", "
              << input_shape_.w << "), layout = "
              << (input_shape_.is_nchw ? "NCHW" : "NHWC")
              << std::endl;

    ret = hbDNNGetOutputCount(&output_count_, dnn_handle_);
    if (ret != 0) {
        std::cerr << "CVMobileNetUNet2::initModel [Error] hbDNNGetOutputCount failed, ret=" << ret << std::endl;
        return ret;
    }
    if (output_count_ != 1) {
        std::cerr << "CVMobileNetUNet2::initModel [Error] This framework only supports single-output model, got "
                  << output_count_ << std::endl;
        return -1;
    }

    ret = hbDNNGetOutputTensorProperties(&output_properties_, dnn_handle_, 0);
    if (ret != 0) {
        std::cerr << "CVMobileNetUNet2::initModel [Error] hbDNNGetOutputTensorProperties failed, ret=" << ret << std::endl;
        return ret;
    }

    output_shape_ = parseShape(output_properties_);
    std::cout << "CVMobileNetUNet2::initModel [Output] shape = ("
              << output_shape_.n << ", "
              << output_shape_.c << ", "
              << output_shape_.h << ", "
              << output_shape_.w << "), layout = "
              << (output_shape_.is_nchw ? "NCHW" : "NHWC")
              << std::endl;

    m_initialized_ = true;
    return 0;
}

int CVMobileNetUNet2::allocateInferResources() {
    if (m_resources_allocated_) {
        return 0;
    }

    m_output_tensors_ = new hbDNNTensor[output_count_];
    std::memset(m_output_tensors_, 0, sizeof(hbDNNTensor) * output_count_);

    for (int i = 0; i < output_count_; ++i) {
        int ret = hbDNNGetOutputTensorProperties(&m_output_tensors_[i].properties, dnn_handle_, i);
        if (ret != 0) {
            std::cerr << "CVMobileNetUNet2::allocateInferResources [Error] hbDNNGetOutputTensorProperties failed at " << i
                      << ", ret=" << ret << std::endl;
            delete[] m_output_tensors_;
            m_output_tensors_ = nullptr;
            return ret;
        }

        int alloc_size = m_output_tensors_[i].properties.alignedByteSize;
        ret = hbSysAllocCachedMem(&m_output_tensors_[i].sysMem[0], alloc_size);
        if (ret != 0) {
            std::cerr << "CVMobileNetUNet2::allocateInferResources [Error] hbSysAllocCachedMem for output failed at " << i
                      << ", ret=" << ret << std::endl;
            for (int j = 0; j < i; ++j) {
                hbSysFreeMem(&m_output_tensors_[j].sysMem[0]);
            }
            delete[] m_output_tensors_;
            m_output_tensors_ = nullptr;
            return ret;
        }
    }

    m_resources_allocated_ = true;
    return 0;
}

int CVMobileNetUNet2::preprocessFromMemory(const cv::Mat& bgr) {
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(input_shape_.w, input_shape_.h));

    cv::Mat nv12 = bgrToNV12(resized);

    m_input_tensor_ = new hbDNNTensor;
    std::memset(m_input_tensor_, 0, sizeof(hbDNNTensor));
    m_input_tensor_->properties = input_properties_;

    int input_bytes = static_cast<int>(nv12.total() * nv12.elemSize());
    int input_alloc_size = input_properties_.alignedByteSize > 0
                         ? input_properties_.alignedByteSize
                         : input_bytes;

    int ret = hbSysAllocCachedMem(&m_input_tensor_->sysMem[0], input_alloc_size);
    if (ret != 0) {
        std::cerr << "CVMobileNetUNet2::preprocessFromMemory [Error] hbSysAllocCachedMem for input failed, ret=" << ret << std::endl;
        delete m_input_tensor_;
        m_input_tensor_ = nullptr;
        return ret;
    }

    std::memset(m_input_tensor_->sysMem[0].virAddr, 0, input_alloc_size);
    std::memcpy(m_input_tensor_->sysMem[0].virAddr, nv12.data, input_bytes);
    hbSysFlushMem(&m_input_tensor_->sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

    return 0;
}

int CVMobileNetUNet2::infer() {
    if (m_task_handle_ != nullptr) {
        hbDNNReleaseTask(m_task_handle_);
        m_task_handle_ = nullptr;
    }

    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);

    int ret = hbDNNInfer(&m_task_handle_, &m_output_tensors_, m_input_tensor_, dnn_handle_, &infer_ctrl_param);
    if (ret != 0) {
        std::cerr << "CVMobileNetUNet2::infer [Error] hbDNNInfer failed, ret=" << ret << std::endl;
        return ret;
    }

    ret = hbDNNWaitTaskDone(m_task_handle_, 0);
    if (ret != 0) {
        std::cerr << "CVMobileNetUNet2::infer [Error] hbDNNWaitTaskDone failed, ret=" << ret << std::endl;
        hbDNNReleaseTask(m_task_handle_);
        m_task_handle_ = nullptr;
        return ret;
    }

    return 0;
}

int CVMobileNetUNet2::postprocess(std::vector<std::vector<int>>& out) {
    hbSysFlushMem(&m_output_tensors_[0].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);

    const float* data = reinterpret_cast<const float*>(m_output_tensors_[0].sysMem[0].virAddr);

    TensorShapeInfo s = parseShape(m_output_tensors_[0].properties);

    if (s.n != 1) {
        std::cerr << "CVMobileNetUNet2::postprocess [Error] Only batch=1 is supported." << std::endl;
        return -1;
    }
    if (s.c != 1) {
        std::cerr << "CVMobileNetUNet2::postprocess [Error] This framework expects binary segmentation output with C=1, got C="
                  << s.c << std::endl;
        return -1;
    }

    m_binaryMask.create(s.h, s.w, CV_8UC1);

    if (s.is_nchw) {
        for (int y = 0; y < s.h; ++y) {
            for (int x = 0; x < s.w; ++x) {
                int idx = y * s.w + x;
                float prob = sigmoid(data[idx]);
                m_binaryMask.at<uint8_t>(y, x) = (prob > threshold_) ? 255 : 0;
            }
        }
    } else {
        for (int y = 0; y < s.h; ++y) {
            for (int x = 0; x < s.w; ++x) {
                int idx = (y * s.w + x) * s.c;
                float prob = sigmoid(data[idx]);
                m_binaryMask.at<uint8_t>(y, x) = (prob > threshold_) ? 255 : 0;
            }
        }
    }

    out.push_back(std::vector<int>(m_binaryMask.data, m_binaryMask.data + m_binaryMask.total()));
    
    // Step 1: Strict color verification - reject non-grass early
    if (cv::countNonZero(m_binaryMask) > 0) {
        cv::Mat mask_resized;
        cv::resize(m_binaryMask, mask_resized, m_originImg.size(), 0, 0, cv::INTER_NEAREST);
        
        // Check 1: Area ratio - grass should occupy reasonable portion
        float areaRatio = (float)cv::countNonZero(mask_resized) / (m_originImg.rows * m_originImg.cols);
        if (areaRatio < 0.02f || areaRatio > 0.8f) {
            m_binaryMask.setTo(0);
            return 0;
        }
        
        // Check 2: Convert to HSV for better color analysis
        cv::Mat hsvImg;
        cv::cvtColor(m_originImg, hsvImg, cv::COLOR_BGR2HSV);
        
        cv::Scalar meanHsv = cv::mean(hsvImg, mask_resized);
        float meanH = meanHsv[0];  // Hue: grass typically 35-85
        float meanS = meanHsv[1];  // Saturation: grass should be somewhat saturated
        float meanV = meanHsv[2];  // Value: not too dark
        
        // Grass hue range check (green is around 60 in OpenCV HSV)
        bool hueOk = (meanH > 25 && meanH < 95);
        // Saturation should be moderate (not gray)
        bool satOk = (meanS > 20);
        // Not too dark
        bool brightOk = (meanV > 30);
        
        if (!hueOk || !satOk || !brightOk) {
            m_binaryMask.setTo(0);
            return 0;
        }
        
        // Check 3: RGB analysis - green must dominate significantly
        cv::Scalar meanColor = cv::mean(m_originImg, mask_resized);
        float b = meanColor[0], g = meanColor[1], r = meanColor[2];
        
        // Green must be clearly dominant (not just slightly larger)
        bool greenDominant = (g > r + 15) && (g > b + 15);
        // Green should be reasonably bright
        bool greenBright = (g > 50);
        // Color should not be too gray (low saturation in RGB terms)
        float maxC = std::max({r, g, b});
        float minC = std::min({r, g, b});
        bool notGray = ((maxC - minC) > 20);
        
        if (!greenDominant || !greenBright || !notGray) {
            m_binaryMask.setTo(0);
            return 0;
        }
    } else {
        return 0;
    }
    
    // Step 2: Retain only the bottom-most connected component (grass is usually at bottom)
    cv::Mat labels, stats, centroids;
    int numLabels = cv::connectedComponentsWithStats(m_binaryMask, labels, stats, centroids, 8, CV_32S);
    if (numLabels > 1) {
        int bottomLabel = 1;
        int maxY = 0;
        for (int i = 1; i < numLabels; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area > 50) {  // Ignore tiny noise
                int y = stats.at<int>(i, cv::CC_STAT_TOP) + stats.at<int>(i, cv::CC_STAT_HEIGHT);
                if (y > maxY) {
                    maxY = y;
                    bottomLabel = i;
                }
            }
        }
        m_binaryMask.setTo(0);
        m_binaryMask.setTo(255, labels == bottomLabel);
    }
    
    // Step 3: Region growing to refine edges
    cv::Mat seedMask;
    cv::resize(m_binaryMask, seedMask, m_originImg.size(), 0, 0, cv::INTER_NEAREST);
    
    cv::Scalar grassMean = cv::mean(m_originImg, seedMask);
    float refB = grassMean[0], refG = grassMean[1], refR = grassMean[2];
    
    cv::Mat grownMask = cv::Mat::zeros(m_originImg.size(), CV_8UC1);
    seedMask.copyTo(grownMask, seedMask);
    
    cv::Mat dilatedSeed;
    cv::dilate(seedMask, dilatedSeed, cv::Mat(), cv::Point(-1,-1), 2);
    cv::Mat boundary = dilatedSeed - seedMask;
    
    float colorThreshold = 45.0f;
    int maxIterations = 20;
    
    for (int iter = 0; iter < maxIterations; iter++) {
        cv::Mat newBoundary;
        cv::dilate(boundary, newBoundary, cv::Mat(), cv::Point(-1,-1), 1);
        newBoundary = newBoundary - grownMask;
        
        if (cv::countNonZero(newBoundary) == 0) break;
        
        cv::Mat nextBoundary = cv::Mat::zeros(m_originImg.size(), CV_8UC1);
        
        for (int y = 0; y < m_originImg.rows; y++) {
            for (int x = 0; x < m_originImg.cols; x++) {
                if (newBoundary.at<uint8_t>(y, x) > 0) {
                    cv::Vec3b pixel = m_originImg.at<cv::Vec3b>(y, x);
                    float dB = std::abs(pixel[0] - refB);
                    float dG = std::abs(pixel[1] - refG);
                    float dR = std::abs(pixel[2] - refR);
                    
                    if (dB < colorThreshold && dG < colorThreshold && dR < colorThreshold) {
                        if (pixel[1] > pixel[2] + 10 && pixel[1] > pixel[0] + 10) {
                            grownMask.at<uint8_t>(y, x) = 255;
                            nextBoundary.at<uint8_t>(y, x) = 255;
                        }
                    }
                }
            }
        }
        
        boundary = nextBoundary;
        if (cv::countNonZero(boundary) == 0) break;
    }
    
    cv::resize(grownMask, m_binaryMask, m_binaryMask.size(), 0, 0, cv::INTER_NEAREST);
    
    // Step 4: Final morphological processing to smooth result
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(m_binaryMask, m_binaryMask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
    cv::morphologyEx(m_binaryMask, m_binaryMask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 1);

    return 0;
}

int CVMobileNetUNet2::visualizeAndDisplay(const std::vector<std::vector<int>>& out) {
    cv::Mat mask_resized;
    cv::resize(m_binaryMask, mask_resized, m_originImg.size(), 0, 0, cv::INTER_NEAREST);

    cv::Mat color_mask = cv::Mat::zeros(m_originImg.size(), CV_8UC3);
    color_mask.setTo(cv::Scalar(0, 255, 0), mask_resized);

    cv::Mat result;
    cv::addWeighted(m_originImg, 0.7, color_mask, 0.3, 0.0, result);

    cv::imshow("Segmentation Result", result);
    cv::moveWindow("Segmentation Result", 1260, 0);
    cv::waitKey(1);

    return 0;
}

void CVMobileNetUNet2::freeInputTensor() {
    if (m_input_tensor_ != nullptr) {
        hbSysFreeMem(&m_input_tensor_->sysMem[0]);
        delete m_input_tensor_;
        m_input_tensor_ = nullptr;
    }
}

void CVMobileNetUNet2::freeInferResources() {
    if (m_task_handle_ != nullptr) {
        hbDNNReleaseTask(m_task_handle_);
        m_task_handle_ = nullptr;
    }

    freeInputTensor();

    if (m_output_tensors_ != nullptr) {
        for (int i = 0; i < output_count_; ++i) {
            hbSysFreeMem(&m_output_tensors_[i].sysMem[0]);
        }
        delete[] m_output_tensors_;
        m_output_tensors_ = nullptr;
    }

    m_binaryMask.release();
    m_resources_allocated_ = false;
}

TensorShapeInfo CVMobileNetUNet2::parseShape(const hbDNNTensorProperties& prop) {
    TensorShapeInfo s;
    if (prop.validShape.numDimensions != 4) {
        throw std::runtime_error("CVMobileNetUNet2::parseShape [Error] Only 4D tensor is supported in this framework.");
    }

    if (prop.tensorLayout == HB_DNN_LAYOUT_NCHW) {
        s.is_nchw = true;
        s.n = prop.validShape.dimensionSize[0];
        s.c = prop.validShape.dimensionSize[1];
        s.h = prop.validShape.dimensionSize[2];
        s.w = prop.validShape.dimensionSize[3];
    } else if (prop.tensorLayout == HB_DNN_LAYOUT_NHWC) {
        s.is_nchw = false;
        s.n = prop.validShape.dimensionSize[0];
        s.h = prop.validShape.dimensionSize[1];
        s.w = prop.validShape.dimensionSize[2];
        s.c = prop.validShape.dimensionSize[3];
    } else {
        s.is_nchw = true;
        s.n = prop.validShape.dimensionSize[0];
        s.c = prop.validShape.dimensionSize[1];
        s.h = prop.validShape.dimensionSize[2];
        s.w = prop.validShape.dimensionSize[3];
        std::cerr << "CVMobileNetUNet2::parseShape [Warn] Unknown tensor layout, fallback to NCHW interpretation." << std::endl;
    }
    return s;
}

cv::Mat CVMobileNetUNet2::bgrToNV12(const cv::Mat& bgr) {
    int h = bgr.rows;
    int w = bgr.cols;
    int y_size = h * w;
    int uv_size = y_size / 4;

    cv::Mat yuv_i420;
    cv::cvtColor(bgr, yuv_i420, cv::COLOR_BGR2YUV_I420);

    cv::Mat nv12(h * 3 / 2, w, CV_8UC1);
    uint8_t* yuv = yuv_i420.ptr<uint8_t>();
    uint8_t* dst = nv12.ptr<uint8_t>();

    std::memcpy(dst, yuv, y_size);

    uint8_t* u_data = yuv + y_size;
    uint8_t* v_data = u_data + uv_size;
    uint8_t* uv_dst = dst + y_size;

    for (int i = 0; i < uv_size; ++i) {
        *uv_dst++ = *u_data++;
        *uv_dst++ = *v_data++;
    }

    return nv12;
}

inline float CVMobileNetUNet2::sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

const cv::Mat& CVMobileNetUNet2::getBinaryMask() const {
    return m_binaryMask;
}
