#include "CVYolo26.hpp"
#include <algorithm>
#include <chrono>
#include <opencv2/dnn/dnn.hpp>
#include "dnn/plugin/hb_dnn_layer.h"
#include "dnn/plugin/hb_dnn_plugin.h"
#include "dnn/hb_sys.h"

int debugYolo26=0;

CVYolo26::CVYolo26(){
	int ret = loadModel();
	if(ret != 0){
		std::cerr << "CVYolo26 - failed to load model" << std::endl;
	}
}

CVYolo26::~CVYolo26() {
    releaseResources();
}

int CVYolo26::loadModel() {
    std::cout << "[OpenCV] Version: " << CV_VERSION << std::endl;

    auto begin_time = std::chrono::system_clock::now();
    const char* model_file_name = MODEL_PATH.c_str();
    RDK_CHECK_SUCCESS(
        hbDNNInitializeFromFiles(&packed_dnn_handle, &model_file_name, 1),
        "hbDNNInitializeFromFiles failed");
    std::cout << "\033[31m Load D-Robotics Quantize model time = " 
              << std::fixed << std::setprecision(2) 
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     std::chrono::system_clock::now() - begin_time).count() / 1000.0 
              << " ms\033[0m" << std::endl;

    const char**model_name_list;
    int model_count = 0;
    RDK_CHECK_SUCCESS(
        hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle),
        "hbDNNGetModelNameList failed");

    if (model_count > 1) {
        std::cout << "This model file have more than 1 model, only use model 0." << std::endl;
    }
    const char* model_name = model_name_list[0];
    std::cout << "[model name]: " << model_name << std::endl;

    RDK_CHECK_SUCCESS(
        hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name),
        "hbDNNGetModelHandle failed");

    int32_t input_count = 0;
    RDK_CHECK_SUCCESS(
        hbDNNGetInputCount(&input_count, dnn_handle),
        "hbDNNGetInputCount failed");

    hbDNNTensorProperties input_properties;
    RDK_CHECK_SUCCESS(
        hbDNNGetInputTensorProperties(&input_properties, dnn_handle, 0),
        "hbDNNGetInputTensorProperties failed");

    if (input_count > 1) {
        std::cout << "Your Model have more than 1 input, please check!" << std::endl;
        return -1;
    }

    if (input_properties.validShape.numDimensions != 4) {
        std::cout << "input tensor type is not HB_DNN_IMG_TYPE_NV12, please check!" << std::endl;
        return -1;
    }

    if (input_properties.tensorType != 1) {
        std::cout << "input tensor layout is not HB_DNN_LAYOUT_NCHW, please check!" << std::endl;
        return -1;
    }

    input_H = input_properties.validShape.dimensionSize[2];
    input_W = input_properties.validShape.dimensionSize[3];
    std::cout << "input tensor valid shape: (" << input_properties.validShape.dimensionSize[0]
              << ", " << input_properties.validShape.dimensionSize[1]
              << ", " << input_H << ", " << input_W << ")" << std::endl;

    RDK_CHECK_SUCCESS(
        hbDNNGetOutputCount(&output_count, dnn_handle),
        "hbDNNGetOutputCount failed");

    if (output_count != 3) {
        std::cout << "Your Model's outputs num is not 3, please check!" << std::endl;
        return -1;
    }

    for (int i = 0; i < 3; i++) {
        hbDNNTensorProperties output_properties;
        RDK_CHECK_SUCCESS(
            hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i),
            "hbDNNGetOutputTensorProperties failed");
        std::cout << "output[" << i << "] ";
        std::cout << "valid shape: (" << output_properties.validShape.dimensionSize[0]
                  << ", " << output_properties.validShape.dimensionSize[1]
                  << ", " << output_properties.validShape.dimensionSize[2]
                  << ", " << output_properties.validShape.dimensionSize[3] << "), ";
        if (output_properties.quantiType == SHIFT)
            std::cout << "QuantiType: SHIFT" << std::endl;
        if (output_properties.quantiType == SCALE)
            std::cout << "QuantiType: SCALE" << std::endl;
        if (output_properties.quantiType == NONE)
            std::cout << "QuantiType: NONE" << std::endl;
    }

    int32_t H_8 = input_H / 8;
    int32_t H_16 = input_H / 16;
    int32_t H_32 = input_H / 32;
    int32_t W_8 = input_W / 8;
    int32_t W_16 = input_W / 16;
    int32_t W_32 = input_W / 32;
    int32_t order_we_want[6][3] = {
        {H_8, W_8, 3 * (5 + CLASSES_NUM)},
        {H_16, W_16, 3 * (5 + CLASSES_NUM)},
        {H_32, W_32, 3 * (5 + CLASSES_NUM)},
    };

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            hbDNNTensorProperties output_properties;
            RDK_CHECK_SUCCESS(
                hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, j),
                "hbDNNGetOutputTensorProperties failed");
            int32_t h = output_properties.validShape.dimensionSize[1];
            int32_t w = output_properties.validShape.dimensionSize[2];
            int32_t c = output_properties.validShape.dimensionSize[3];
            if (h == order_we_want[i][0] && w == order_we_want[i][1] && c == order_we_want[i][2]) {
                order[i] = j;
                break;
            }
        }
    }

    if (order[0] + order[1] + order[2] == 0 + 1 + 2) {
        std::cout << "Outputs order check SUCCESS, continue." << std::endl;
        std::cout << "order = {" << order[0] << ", " << order[1] << ", " << order[2] << "}" << std::endl;
    } else {
        std::cout << "Outputs order check FAILED, use default" << std::endl;
        for (int i = 0; i < 3; i++)
            order[i] = i;
    }

    isModelLoaded = true;

    return 0;
}

int CVYolo26::preprocess(const cv::Mat& img, cv::Mat& processed_img) {
    //std::cout << "img (rows, cols, channels): (" << img.rows << ", " << img.cols << ", " << img.channels() << ")" << std::endl;

    auto begin_time = std::chrono::system_clock::now();
    
    // 确保推理资源已分配，这样预分配的缓冲区才可用
    if (!areInferResourcesAllocated) {
        if (allocateInferResources() != 0) {
            std::cerr << "Failed to allocate infer resources" << std::endl;
            return -1;
        }
    }
    
    // 使用预分配的缓冲区
    cv::Mat resize_img;
    if (preallocated_input_buffer.empty()) {
        resize_img.create(input_H, input_W, CV_8UC3);
    } else {
        // 重新调整预分配缓冲区的大小以适应resize_img的需求
        resize_img = preallocated_input_buffer(cv::Rect(0, 0, input_W, input_H)).clone();
        resize_img.convertTo(resize_img, CV_8UC3);
    }

    if (PREPROCESS_TYPE == 1) { // LETTERBOX_TYPE
        x_scale = std::min(1.0 * input_H / img.rows, 1.0 * input_W / img.cols);
        y_scale = x_scale;
        if (x_scale <= 0 || y_scale <= 0) {
            std::cout << "Invalid scale factor." << std::endl;
            return -1;
        }

        int new_w = img.cols * x_scale;
        x_shift = (input_W - new_w) / 2;
        int x_other = input_W - new_w - x_shift;

        int new_h = img.rows * y_scale;
        y_shift = (input_H - new_h) / 2;
        int y_other = input_H - new_h - y_shift;

        cv::Size targetSize(new_w, new_h);
        cv::resize(img, resize_img, targetSize);
        cv::copyMakeBorder(resize_img, resize_img, y_shift, y_other, x_shift, x_other, cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127));
        //std::cout << "\033[31m pre process (LetterBox) time = " 
                  //<< std::fixed << std::setprecision(2) 
                  //<< std::chrono::duration_cast<std::chrono::microseconds>(
                         //std::chrono::system_clock::now() - begin_time).count() / 1000.0 
                  //<< " ms\033[0m" << std::endl;
    } else { // RESIZE_TYPE
        cv::Size targetSize(input_W, input_H);
        cv::resize(img, resize_img, targetSize);
        y_scale = 1.0 * input_H / img.rows;
        x_scale = 1.0 * input_W / img.cols;
        y_shift = 0;
        x_shift = 0;
        std::cout << "\033[31m pre process (Resize) time = " 
                  << std::fixed << std::setprecision(2) 
                  << std::chrono::duration_cast<std::chrono::microseconds>(
                         std::chrono::system_clock::now() - begin_time).count() / 1000.0 
                  << " ms\033[0m" << std::endl;
    }

    //std::cout << "y_scale = " << y_scale << ", x_scale = " << x_scale << std::endl;
    //std::cout << "y_shift = " << y_shift << ", x_shift = " << x_shift << std::endl;

    begin_time = std::chrono::system_clock::now();
    
    // 使用预分配的缓冲区进行颜色转换
    cv::Mat yuv_mat;
    if (preallocated_input_buffer.empty()) {
        yuv_mat.create(input_H * 3 / 2, input_W, CV_8UC1);
    } else {
        yuv_mat = preallocated_input_buffer;
    }
    
    cv::cvtColor(resize_img, yuv_mat, cv::COLOR_BGR2YUV_I420);
    uint8_t* yuv = yuv_mat.ptr<uint8_t>();
    
    // 使用预分配的processed_img缓冲区
    if (preallocated_processed_img.empty()) {
        processed_img = cv::Mat(input_H * 3 / 2, input_W, CV_8UC1);
    } else {
        processed_img = preallocated_processed_img;
    }
    
    uint8_t* ynv12 = processed_img.ptr<uint8_t>();
    int uv_height = input_H / 2;
    int uv_width = input_W / 2;
    int y_size = input_H * input_W;
    memcpy(ynv12, yuv, y_size);
    
    uint8_t* nv12 = ynv12 + y_size;
    uint8_t* u_data = yuv + y_size;
    uint8_t* v_data = u_data + uv_height * uv_width;
    for (int i = 0; i < uv_width * uv_height; i++) {
        *nv12++ = *u_data++;
        *nv12++ = *v_data++;
    }
    
    //std::cout << "\033[31m bgr8 to nv12 time = " 
              //<< std::fixed << std::setprecision(2) 
              //<< std::chrono::duration_cast<std::chrono::microseconds>(
                     //std::chrono::system_clock::now() - begin_time).count() / 1000.0 
              //<< " ms\033[0m" << std::endl;

    return 0;
}

int CVYolo26::allocateInferResources() {
    if (areInferResourcesAllocated) {
        return 0; // 资源已经分配
    }

    // 分配输入内存
    hbDNNTensorProperties input_properties;
    hbDNNGetInputTensorProperties(&input_properties, dnn_handle, 0);
    input_mem_size = int(3 * input_H * input_W / 2);
    if (hbSysAllocCachedMem(&input_mem, input_mem_size) != 0) {
        std::cerr << "Failed to allocate input memory" << std::endl;
        return -1;
    }

    // 分配输出张量
    output = new (std::nothrow) hbDNNTensor[output_count];
    if (output == nullptr) {
        std::cerr << "Failed to allocate output tensors" << std::endl;
        hbSysFreeMem(&input_mem);
        return -1;
    }

    for (int i = 0; i < output_count; i++) {
        hbDNNTensorProperties& output_properties = output[i].properties;
        hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i);
        int out_aligned_size = output_properties.alignedByteSize;
        hbSysMem& mem = output[i].sysMem[0];
        if (hbSysAllocCachedMem(&mem, out_aligned_size) != 0) {
            std::cerr << "Failed to allocate output memory for tensor " << i << std::endl;
            // 释放已分配的资源
            for (int j = 0; j < i; j++) {
                hbSysFreeMem(&(output[j].sysMem[0]));
            }
            delete[] output;
            output = nullptr;
            hbSysFreeMem(&input_mem);
            return -1;
        }
    }

    // 预分配输入缓冲区
    preallocated_input_buffer.create(input_H * 3 / 2, input_W, CV_8UC1);
    preallocated_processed_img.create(input_H * 3 / 2, input_W, CV_8UC1);

    areInferResourcesAllocated = true;
    return 0;
}

int CVYolo26::infer(const cv::Mat& processed_img) {
    auto begin_time = std::chrono::system_clock::now();

    // 确保推理资源已分配
    if (!areInferResourcesAllocated) {
        if (allocateInferResources() != 0) {
            std::cerr << "Failed to allocate infer resources" << std::endl;
            return -1;
        }
    }

    // 设置输入张量属性
    hbDNNTensorProperties input_properties;
    hbDNNGetInputTensorProperties(&input_properties, dnn_handle, 0);
    input.properties = input_properties;
    
    // 使用预分配的输入内存
    input.sysMem[0] = input_mem;
    memcpy(input.sysMem[0].virAddr, processed_img.ptr<uint8_t>(), input_mem_size);
    hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
    
    int ret = hbDNNInfer(&task_handle, &output, &input, dnn_handle, &infer_ctrl_param);
    if (ret != 0) {
        std::cerr << "hbDNNInfer failed, error code: " << ret << std::endl;
        return ret;
    }

    hbDNNWaitTaskDone(task_handle, 0);
    
    ret = hbDNNReleaseTask(task_handle);
    if (ret != 0) {
        std::cerr << "hbDNNReleaseTask failed, error code: " << ret << std::endl;
        return ret;
    }

    return 0;
}

int CVYolo26::postprocessAndRender(cv::Mat& img) {
    float CONF_THRES_RAW = -log(1 / SCORE_THRESHOLD - 1);
    std::vector<std::vector<cv::Rect2d>> bboxes(CLASSES_NUM);
    std::vector<std::vector<float>> scores(CLASSES_NUM);

    if (ANCHORS.size() != 18) {
        std::cout << "Anchors size is not 18, please check!" << std::endl;
        return -1;
    }
    
    /*std::cout << "anchors: ";
    for (float anchor : ANCHORS) {
        std::cout << anchor << "  ";
    }
    std::cout << std::endl;*/

    std::vector<std::pair<double, double>> s_anchors = {
        {ANCHORS[0], ANCHORS[1]},
        {ANCHORS[2], ANCHORS[3]},
        {ANCHORS[4], ANCHORS[5]}
    };
    std::vector<std::pair<double, double>> m_anchors = {
        {ANCHORS[6], ANCHORS[7]},
        {ANCHORS[8], ANCHORS[9]},
        {ANCHORS[10], ANCHORS[11]}
    };
    std::vector<std::pair<double, double>> l_anchors = {
        {ANCHORS[12], ANCHORS[13]},
        {ANCHORS[14], ANCHORS[15]},
        {ANCHORS[16], ANCHORS[17]}
    };

    auto begin_time = std::chrono::system_clock::now();

    int H_8 = input_H / 8;
    int W_8 = input_W / 8;
    if (output[order[0]].properties.quantiType != NONE) {
        std::cout << "output[order[0]] QuantiType should be NONE, please check!" << std::endl;
        return -1;
    }
    hbSysFlushMem(&(output[order[0]].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    auto* s_raw = reinterpret_cast<float*>(output[order[0]].sysMem[0].virAddr);
    
    for (int h = 0; h < H_8; h++) {
        for (int w = 0; w < W_8; w++) {
            for (const auto& anchor : s_anchors) {
                float* cur_s_raw = s_raw;
                s_raw += (5 + CLASSES_NUM);

                if (cur_s_raw[4] < CONF_THRES_RAW)
                    continue;

                int cls_id = 5;
                int end = CLASSES_NUM + 5;
                for (int i = 6; i < end; i++) {
                    if (cur_s_raw[i] > cur_s_raw[cls_id]) {
                        cls_id = i;
                    }
                }
                
                float score = 1.0 / (1.0 + std::exp(-cur_s_raw[4])) * 
                             1.0 / (1.0 + std::exp(-cur_s_raw[cls_id]));
                
                if (score < SCORE_THRESHOLD)
                    continue;
                cls_id -= 5;

                float center_x = ((1.0 / (1.0 + std::exp(-cur_s_raw[0]))) * 2 - 0.5 + w) * 8;
                float center_y = ((1.0 / (1.0 + std::exp(-cur_s_raw[1]))) * 2 - 0.5 + h) * 8;
                float bbox_w = std::pow((1.0 / (1.0 + std::exp(-cur_s_raw[2]))) * 2, 2) * anchor.first;
                float bbox_h = std::pow((1.0 / (1.0 + std::exp(-cur_s_raw[3]))) * 2, 2) * anchor.second;
                float bbox_x = center_x - bbox_w / 2.0;
                float bbox_y = center_y - bbox_h / 2.0;

                bboxes[cls_id].push_back(cv::Rect2d(bbox_x, bbox_y, bbox_w, bbox_h));
                scores[cls_id].push_back(score);
            }
        }
    }

    int H_16 = input_H / 16;
    int W_16 = input_W / 16;
    if (output[order[1]].properties.quantiType != NONE) {
        std::cout << "output[order[1]] QuantiType should be NONE, please check!" << std::endl;
        return -1;
    }
    hbSysFlushMem(&(output[order[1]].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    auto* m_raw = reinterpret_cast<float*>(output[order[1]].sysMem[0].virAddr);
    
    for (int h = 0; h < H_16; h++) {
        for (int w = 0; w < W_16; w++) {
            for (const auto& anchor : m_anchors) {
                float* cur_m_raw = m_raw;
                m_raw += (5 + CLASSES_NUM);

                if (cur_m_raw[4] < CONF_THRES_RAW)
                    continue;

                int cls_id = 5;
                int end = CLASSES_NUM + 5;
                for (int i = 6; i < end; i++) {
                    if (cur_m_raw[i] > cur_m_raw[cls_id]) {
                        cls_id = i;
                    }
                }
                
                float score = 1.0 / (1.0 + std::exp(-cur_m_raw[4])) * 
                             1.0 / (1.0 + std::exp(-cur_m_raw[cls_id]));
                
                if (score < SCORE_THRESHOLD)
                    continue;
                cls_id -= 5;

                float center_x = ((1.0 / (1.0 + std::exp(-cur_m_raw[0]))) * 2 - 0.5 + w) * 16;
                float center_y = ((1.0 / (1.0 + std::exp(-cur_m_raw[1]))) * 2 - 0.5 + h) * 16;
                float bbox_w = std::pow((1.0 / (1.0 + std::exp(-cur_m_raw[2]))) * 2, 2) * anchor.first;
                float bbox_h = std::pow((1.0 / (1.0 + std::exp(-cur_m_raw[3]))) * 2, 2) * anchor.second;
                float bbox_x = center_x - bbox_w / 2.0;
                float bbox_y = center_y - bbox_h / 2.0;

                bboxes[cls_id].push_back(cv::Rect2d(bbox_x, bbox_y, bbox_w, bbox_h));
                scores[cls_id].push_back(score);
            }
        }
    }

    int H_32 = input_H / 32;
    int W_32 = input_W / 32;
    if (output[order[2]].properties.quantiType != NONE) {
        std::cout << "output[order[2]] QuantiType should be NONE, please check!" << std::endl;
        return -1;
    }
    hbSysFlushMem(&(output[order[2]].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    auto* l_raw = reinterpret_cast<float*>(output[order[2]].sysMem[0].virAddr);
    
    for (int h = 0; h < H_32; h++) {
        for (int w = 0; w < W_32; w++) {
            for (const auto& anchor : l_anchors) {
                float* cur_l_raw = l_raw;
                l_raw += (5 + CLASSES_NUM);

                if (cur_l_raw[4] < CONF_THRES_RAW)
                    continue;

                int cls_id = 5;
                int end = CLASSES_NUM + 5;
                for (int i = 6; i < end; i++) {
                    if (cur_l_raw[i] > cur_l_raw[cls_id]) {
                        cls_id = i;
                    }
                }
                
                float score = 1.0 / (1.0 + std::exp(-cur_l_raw[4])) * 
                             1.0 / (1.0 + std::exp(-cur_l_raw[cls_id]));
                
                if (score < SCORE_THRESHOLD)
                    continue;
                cls_id -= 5;

                float center_x = ((1.0 / (1.0 + std::exp(-cur_l_raw[0]))) * 2 - 0.5 + w) * 32;
                float center_y = ((1.0 / (1.0 + std::exp(-cur_l_raw[1]))) * 2 - 0.5 + h) * 32;
                float bbox_w = std::pow((1.0 / (1.0 + std::exp(-cur_l_raw[2]))) * 2, 2) * anchor.first;
                float bbox_h = std::pow((1.0 / (1.0 + std::exp(-cur_l_raw[3]))) * 2, 2) * anchor.second;
                float bbox_x = center_x - bbox_w / 2.0;
                float bbox_y = center_y - bbox_h / 2.0;

                bboxes[cls_id].push_back(cv::Rect2d(bbox_x, bbox_y, bbox_w, bbox_h));
                scores[cls_id].push_back(score);
            }
        }
    }

    std::vector<std::vector<int>> indices(CLASSES_NUM);
    for (int i = 0; i < CLASSES_NUM; i++) {
        cv::dnn::NMSBoxes(bboxes[i], scores[i], SCORE_THRESHOLD, NMS_THRESHOLD, indices[i], 1.f, NMS_TOP_K);
    }
    
    //std::cout << "\033[31m Post Process time = " 
              //<< std::fixed << std::setprecision(2) 
              //<< std::chrono::duration_cast<std::chrono::microseconds>(
                     //std::chrono::system_clock::now() - begin_time).count() / 1000.0 
              //<< " ms\033[0m" << std::endl;

    begin_time = std::chrono::system_clock::now();
    for (int cls_id = 0; cls_id < CLASSES_NUM; cls_id++) {
        for (int idx : indices[cls_id]) {
            float x1 = (bboxes[cls_id][idx].x - x_shift) / x_scale;
            float y1 = (bboxes[cls_id][idx].y - y_shift) / y_scale;
            float x2 = x1 + (bboxes[cls_id][idx].width) / x_scale;
            float y2 = y1 + (bboxes[cls_id][idx].height) / y_scale;
            float score = scores[cls_id][idx];
            std::string name = object_names[cls_id % CLASSES_NUM];

            cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), LINE_SIZE);

            DetectionResult result;
            result.class_name = name;
            result.confidence = score;
            result.bounding_box = cv::Rect2d(x1, y1, x2 - x1, y2 - y1);
            detection_results.push_back(result);

            std::string text = name + ": " + std::to_string(static_cast<int>(score * 100)) + "%";

            for (const auto& result : detection_results) {
                if (result.class_name == name && 
                    std::abs(result.bounding_box.x - x1) < 1.0 && 
                    std::abs(result.bounding_box.y - y1) < 1.0) {
                    break;
                }
            }

            cv::putText(img, text, cv::Point(x1, y1 - 5), cv::FONT_HERSHEY_SIMPLEX, 
                       MYAPP_FONT_SIZE, cv::Scalar(0, 0, 255), FONT_THICKNESS, cv::LINE_AA);

            if (debugYolo26) std::cout << "(" << x1 << " " << y1 << " " << x2 << " " << y2 << "): \t" << text << std::endl;
        }
    }
    
    //std::cout << "\033[31m Draw Result time = " 
             // << std::fixed << std::setprecision(2) 
              //<< std::chrono::duration_cast<std::chrono::microseconds>(
                 //    std::chrono::system_clock::now() - begin_time).count() / 1000.0 
             // << " ms\033[0m" << std::endl;

    return 0;
}

void CVYolo26::releaseInferResources() {
    if (areInferResourcesAllocated) {
        // 释放输出张量内存
        if (output != nullptr) {
            for (int i = 0; i < output_count; i++) {
                hbSysFreeMem(&(output[i].sysMem[0]));
            }
            delete[] output;
            output = nullptr;
        }

        // 释放输入内存
        hbSysFreeMem(&input_mem);
        input_mem_size = 0;

        // 清空预分配的缓冲区
        preallocated_input_buffer.release();
        preallocated_processed_img.release();

        areInferResourcesAllocated = false;
    }

    // 移除兼容旧代码的部分，避免重复释放内存
    // input.sysMem[0] 的内存已经通过 input_mem 释放
}

void CVYolo26::releaseResources() {
    // 释放推理相关资源
    releaseInferResources();

    // 释放模型相关资源
    if (packed_dnn_handle != nullptr) {
        hbDNNRelease(packed_dnn_handle);
        packed_dnn_handle = nullptr;
    }

    // 重置模型加载状态
    isModelLoaded = false;
}

int CVYolo26::start(const cv::Mat& img){
    if(packed_dnn_handle == nullptr ){
        std::cerr << "Model is not loaded" << std::endl;
        return -1;
    }

    if(img.empty()){ 
        std::cout << "Input image is empty" << std::endl;
        return -1;
    }

    detection_results.clear();
    cv::Mat processed_img;
    int ret = preprocess(img,processed_img);
    if(ret!=0) return ret;
    
    ret = infer(processed_img);
    if(ret!=0){
        releaseInferResources();
        return ret;
    }

    original_image = img.clone();

    cv::Mat result_img = original_image.clone();
    ret = postprocessAndRender(result_img);
    if(ret != 0){
        releaseInferResources(); 
        return ret;
    }

    cv::namedWindow("YOLO Detection Result",cv::WINDOW_NORMAL);
    cv::imshow("YOLO Detection Result",result_img);
    cv::resizeWindow("YOLO Detection Result", 640, 360);
    cv::moveWindow("YOLO Detection Result", 0, 0);
    cv::waitKey(1);
    
    releaseInferResources();
    
    return 0;
}

std::vector<DetectionResult> CVYolo26::getDetectionResults() {
    return detection_results;
}
