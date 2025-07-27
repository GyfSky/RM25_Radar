#include "../include/Inference.h"
namespace TRTInferV1
{

    void TRTInfer::nms(std::vector<DetectionObj> &input_boxes, float &nms_threshold)
    {
        // 按置信度从高到低排序
        std::sort(input_boxes.begin(), input_boxes.end(), [](DetectionObj a, DetectionObj b)
                  { return a.confidence > b.confidence; });
        // 计算每个检测框的面积
        std::vector<float> vArea(input_boxes.size());
        for (int i = 0; i < int(input_boxes.size()); ++i)
        {
            vArea[i] = (input_boxes.at(i).x2 - input_boxes.at(i).x1 + 1) * (input_boxes.at(i).y2 - input_boxes.at(i).y1 + 1);
        }

        // 标记哪些检测框被抑制
        std::vector<bool> isSuppressed(input_boxes.size(), false);
        for (int i = 0; i < int(input_boxes.size()); ++i)
        {
            if (isSuppressed[i])
            {
                continue;
            }
            for (int j = i + 1; j < int(input_boxes.size()); ++j)
            {
                if (isSuppressed[j])
                {
                    continue;
                }
                // 计算交集区域
                float xx1 = (std::max)(input_boxes[i].x1, input_boxes[j].x1);
                float yy1 = (std::max)(input_boxes[i].y1, input_boxes[j].y1);
                float xx2 = (std::min)(input_boxes[i].x2, input_boxes[j].x2);
                float yy2 = (std::min)(input_boxes[i].y2, input_boxes[j].y2);

                float w = (std::max)(float(0), xx2 - xx1 + 1);
                float h = (std::max)(float(0), yy2 - yy1 + 1);
                float inter = w * h;
                float ovr = inter / (vArea[i] + vArea[j] - inter);

                // 如果重叠率大于阈值，则抑制该检测框
                if (ovr >= nms_threshold)
                {
                    isSuppressed[j] = true;
                }
            }
        }
        // return post_nms;
        // 移除被抑制的检测框
        int idx_t = 0;
        input_boxes.erase(std::remove_if(input_boxes.begin(), input_boxes.end(), [&idx_t, &isSuppressed](const DetectionObj &f)
                                         { return isSuppressed[idx_t++]; }),
                          input_boxes.end());
    }
      void TRTInfer::decodeout(std::vector<DetectionObj> &res, cv::Mat &frame, float *pdata, float &confidence_threshold)
    {
        int padh = 0, padw = 0;
        float r = std::min(this->input_dims.d[3] / (frame.cols * 1.0), this->input_dims.d[2] / (frame.rows * 1.0));
        int unpad_w = r * frame.cols;
        int unpad_h = r * frame.rows;
        float ratioh = (float)frame.rows / unpad_h, ratiow = (float)frame.cols / unpad_w;
        padw = this->input_dims.d[3] - unpad_w;
        padh = this->input_dims.d[2] - unpad_h;
        padw /= 2;
        padh /= 2;
        int out1 = output_dims.d[1];
        for (int i=0;i<out1;i++)
        {
            float max=0;
            int id=0;
            for (int j=0;j<num_classes+4;j++)
            {
                // std::cout<<pdata[i*(num_classes+12)+j]<<"\n";
                if (j>3&&pdata[i*(num_classes+4)+j]>max)
                {
                    id=j-4;
                    max = pdata[i*(num_classes+4)+j];
                }
            }
            if (max>confidence_threshold)
            {
                // std::cout<<id<<std::endl;
                const float x = pdata[i*(num_classes+4)];
                const float y = pdata[i*(num_classes+4)+1];
                const float w = pdata[i*(num_classes+4)+2];
                const float h = pdata[i*(num_classes+4)+3];
                const int cls = id;
                float xmin = (x-w/2-padw)*ratiow;
                float ymin = (y-h/2-padh)*ratioh;
                float xmax = (x+w/2-padw)*ratiow;
                float ymax = (y+h/2-padh)*ratioh;
                // std::vector<std::pair<float,float>> keypoints;
                // for (int k=0;k<7;k+=2)
                // {
                //     float x1 = (pdata[i*(num_classes+12)+num_classes+4+k]-padw)*ratiow;
                //     float y1 = (pdata[i*(num_classes+12)+num_classes+4+k+1]-padh)*ratioh;
                //     keypoints.emplace_back(x1, y1);
                // }
                if (xmin >= 0. && ymin >= 0. && xmax <= float(frame.cols) && ymax <= float(frame.rows) && int(xmax-xmin) != 0 && int(ymax-ymin) != 0)
                    res.emplace_back(DetectionObj{cls,max,xmin,ymin,xmax,ymax});
            }
        }
    }
    void TRTInfer::decode_output(std::vector<DetectionObj> &res, cv::Mat &frame, float *pdata, float &obj_threshold, float &confidence_threshold, float &nms_threshold)
    {
        //初始化填充高度和宽度为0。
        //计算缩放比例 r，确保输入图像在缩放到模型输入尺寸时保持宽高比不变。
        int padh = 0, padw = 0;
        float r = std::min(this->input_dims.d[3] / (frame.cols * 1.0), this->input_dims.d[2] / (frame.rows * 1.0));
        //计算未填充的宽度和高度 unpad_w 和 unpad_h。
        //计算原始图像与缩放后图像的高度和宽度比例 ratioh 和 ratiow
        int unpad_w = r * frame.cols;
        int unpad_h = r * frame.rows;
        float ratioh = (float)frame.rows / unpad_h, ratiow = (float)frame.cols / unpad_w;
        //计算填充宽度和高度，并将其均分到左右和上下两侧。
        padw = this->input_dims.d[3] - unpad_w;
        padh = this->input_dims.d[2] - unpad_h;
        padw /= 2;
        padh /= 2;
        int n = 0, q = 0, i = 0, j = 0, row_ind = 0, k = 0; /// xmin,ymin,xamx,ymax,box_score, class_score
        for (n = 0; n < this->num_stride; ++n)
        {
            //遍历不同尺度的特征图，计算每个特征图的步长 stride。
            //计算每个特征图的网格数量 num_grid_x 和 num_grid_y
            const float stride = pow(2, n + 3);
            int num_grid_x = (int)ceil(this->input_dims.d[3] / stride);
            int num_grid_y = (int)ceil(this->input_dims.d[2] / stride);
            //遍历每个特征图中的每个网格单元。
            //获取预测框的置信度 box_score
            //如果置信度大于对象阈值 obj_threshold，则继续处理
            //遍历所有类别，找到最大类别得分 max_class_score 及其索引 max_ind，并将最大类别得分乘以置信度
            for (q = 0; q < 3; ++q)
            {
                for (i = 0; i < num_grid_y; ++i)
                {
                    for (j = 0; j < num_grid_x; ++j)
                    {
                        //pdata 0:cx  1:cy  2:w  3:h  4:框的置信度  4+k:对象置信度
                        float box_score = pdata[4];
                        if (box_score > obj_threshold)
                        {
                            int max_ind = 0;
                            float max_class_score = 0;
                            for (k = 0; k < this->num_classes; ++k)
                            {
                                if (pdata[k + 5] > max_class_score)
                                {
                                    max_class_score = pdata[k + 5];
                                    max_ind = k;
                                }
                            }
                            max_class_score *= box_score;
                            //如果最大类别得分大于置信度阈值 confidence_threshold，则继续处理。
                            //获取预测框的中心坐标 cx, cy 和宽度 w, 高度 h
                            if (max_class_score > confidence_threshold)
                            {
                                float cx = pdata[0]; /// cx
                                float cy = pdata[1]; /// cy
                                float w = pdata[2];  /// w
                                float h = pdata[3];  /// h
                                //计算预测框在原始图像中的边界框坐标 xmin, ymin, xmax, ymax。
                                float xmin = (cx - padw - 0.5 * w) * ratiow;
                                float ymin = (cy - padh - 0.5 * h) * ratioh;
                                float xmax = (cx - padw + 0.5 * w) * ratiow;
                                float ymax = (cy - padh + 0.5 * h) * ratioh;
                                //检查边界框是否在图像范围内，如果是，则将检测结果添加到 res 中。
                                //更新 row_ind 和 pdata 指针，以便处理下一个网格单元
                                if (xmin >= 0. && ymin >= 0. && xmax <= float(frame.cols) && ymax <= float(frame.rows))
                                    //图像坐标系为左上角，所以为左上角的点和右下角的点
                                    res.emplace_back(DetectionObj{max_ind, max_class_score, xmin, ymin, xmax, ymax});
                            }
                        }
                        ++row_ind;
                        pdata += this->output_dims.d[2];


                    }
                }
            }
        }
        //应用非极大值抑制（NMS），去除重叠的检测框。
        this->nms(res, nms_threshold);
    }

    void TRTInfer::postprocess(std::vector<std::vector<DetectionObj>> &batch_res, std::vector<cv::Mat> &frames, float &obj_threshold, float &confidence_threshold, float &nms_threshold,int flag)
    {
        for (int b = 0; b < int(frames.size()); ++b)
        {
            auto &res = batch_res[b];
            if (flag) {
                this->decodeout(res, frames[b], &this->output[b * this->output_size], confidence_threshold);
                this->nms(res, nms_threshold);
            }//yolov8

            else
                this->decode_output(res, frames[b], &this->output[b * this->output_size],obj_threshold, confidence_threshold,nms_threshold);
                //yolov5
        }
    }

    TRTInfer::TRTInfer(const int device)
    {
        cudaSetDevice(device);
    }

    TRTInfer::TRTInfer(){
    }

    void TRTInfer::getDevice(const int device) {
        cudaSetDevice(device);
    }

    TRTInfer::~TRTInfer()
    {
    }

    bool TRTInfer::initModule(const std::string engine_file_path, const int batch_size, const int num_classes)
    {
        assert(batch_size > 0 && num_classes > 0);
        this->batch_size = batch_size;
        this->num_classes = num_classes;
        char *trtModelStream{nullptr};
        size_t size{0};
        std::ifstream file(engine_file_path, std::ios::binary);
        if (file.good())
        {
            file.seekg(0, file.end);
            size = file.tellg();
            file.seekg(0, file.beg);
            trtModelStream = new char[size];
            assert(trtModelStream);
            file.read(trtModelStream, size);
            file.close();
        }
        else
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Engine bad file");
            return false;
        }
        this->runtime = createInferRuntime(this->gLogger);
        assert(runtime != nullptr);
        this->engine = this->runtime->deserializeCudaEngine(trtModelStream, size);
        assert(this->engine != nullptr);
        this->context = this->engine->createExecutionContext();
        assert(context != nullptr);
        delete trtModelStream;
        this->input_dims = this->engine->getTensorShape(INPUT_BLOB_NAME);
        this->input_dims.d[0] = batch_size;

        if (this->input_dims.d[2] == 1280)
        {
            this->num_stride = this->num_stride_1280;
            this->anchors = (float *)this->anchors_1280;
        }
        else
        {
            this->num_stride = this->num_stride_640;
//            this->anchors = (float *)this->anchors_640;
        }

        this->output_dims = this->engine->getTensorShape(OUTPUT_BLOB_NAME);
        this->context->setInputShape(INPUT_BLOB_NAME, input_dims);
        this->output_size = output_dims.d[1] * output_dims.d[2];
        int IOtensorsNum = engine->getNbIOTensors();
        assert(IOtensorsNum == 2);
        for (int i = 0; i < IOtensorsNum; ++i)
        {
            if (strcmp(this->engine->getIOTensorName(i), INPUT_BLOB_NAME))
            {
                this->inputIndex = i;
                assert(this->engine->getTensorDataType(INPUT_BLOB_NAME) == nvinfer1::DataType::kFLOAT);
            }
            else if (strcmp(this->engine->getIOTensorName(i), OUTPUT_BLOB_NAME))
            {
                this->outputIndex = i;
                assert(this->engine->getTensorDataType(OUTPUT_BLOB_NAME) == nvinfer1::DataType::kFLOAT);
            }
        }
        if (this->inputIndex == -1 || this->outputIndex == -1)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Uncorrect Input/Output tensor name");
            delete context;
            delete engine;
            return false;
        }
        CHECK(cudaMalloc(&buffers[inputIndex], batch_size * this->input_dims.d[1] * this->input_dims.d[2] * this->input_dims.d[3] * sizeof(float)));
        CHECK(cudaMalloc(&buffers[outputIndex], batch_size * this->output_size * sizeof(float)));
        CHECK(cudaMallocHost((void **)&this->img_host, MAX_IMAGE_INPUT_SIZE_THRESH * 3 * sizeof(float)));
        CHECK(cudaMalloc((void **)&this->img_device, MAX_IMAGE_INPUT_SIZE_THRESH * 3 * sizeof(float)));
        this->output = (float *)malloc(batch_size * this->output_size * sizeof(float));
        this->_is_inited = true;
        return true;
    }

    void TRTInfer::unInitModule()
    {
        this->_is_inited = false;
        delete this->context;
        delete this->runtime;
        delete this->engine;
        CHECK(cudaFree(img_device));
        CHECK(cudaFreeHost(img_host));
        CHECK(cudaFree(this->buffers[this->inputIndex]));
        CHECK(cudaFree(this->buffers[this->outputIndex]));
    }

    void TRTInfer::saveEngineFile(IHostMemory *data, const std::string engine_file_path)
    {
        std::string serialize_str;
        std::ofstream serialize_output_stream;
        serialize_str.resize(data->size());
        memcpy((void *)serialize_str.data(), data->data(), data->size());
        serialize_output_stream.open(engine_file_path);
        serialize_output_stream << serialize_str;
        serialize_output_stream.close();
    }

    // std::vector<std::vector<DetectionObj>> TRTInfer::doInference(std::vector<cv::Mat> &frames, float obj_threshold, float confidence_threshold, float nms_threshold)
    // {
    //     if(!this->_is_inited)
    //     {
    //         this->gLogger.log(ILogger::Severity::kERROR,"Module not inited !");
    //         return {};
    //     }
    //     if (frames.size() == 0 || int(frames.size()) > this->input_dims.d[0])
    //     {
    //         this->gLogger.log(ILogger::Severity::kWARNING, "Invalid frames size");
    //         return {};
    //     }
    //     std::vector<std::vector<DetectionObj>> batch_res(frames.size());
    //     cudaStream_t stream = nullptr;
    //     CHECK(cudaStreamCreate(&stream));
    //     float *buffer_idx = (float *)buffers[this->inputIndex];
    //     for (size_t b = 0; b < frames.size(); ++b)
    //     {
    //         cv::Mat &img = frames[b];
    //         if (img.empty())
    //             continue;
    //         size_t size_image = img.cols * img.rows * 3;
    //         size_t size_image_dst = this->input_dims.d[3] * this->input_dims.d[2] * 3;
    //         memcpy(img_host, img.data, size_image);
    //         CHECK(cudaMemcpyAsync(img_device, img_host, size_image, cudaMemcpyHostToDevice, stream));
    //         preprocess_kernel_img(img_device, img.cols, img.rows, buffer_idx,
    //                               this->input_dims.d[3], this->input_dims.d[2], stream);
    //         buffer_idx += size_image_dst;
    //     }
    //     this->context->setOptimizationProfileAsync(0, stream);
    //     this->context->setTensorAddress(INPUT_BLOB_NAME, this->buffers[this->inputIndex]);
    //     this->context->setTensorAddress(OUTPUT_BLOB_NAME, this->buffers[this->outputIndex]);
    //     bool success = this->context->enqueueV3(stream);
    //     if (!success)
    //     {
    //         this->gLogger.log(ILogger::Severity::kERROR, "DoInference failed");
    //         CHECK(cudaStreamDestroy(stream));
    //         return {};
    //     }
    //     CHECK(cudaMemcpyAsync(this->output, buffers[this->outputIndex], frames.size() * this->output_size * sizeof(float), cudaMemcpyDeviceToHost, stream));
    //     CHECK(cudaStreamSynchronize(stream));
    //     CHECK(cudaStreamDestroy(stream));
    //
    //     this->postprocess(batch_res, frames, obj_threshold, confidence_threshold, nms_threshold);
    //
    //     return batch_res;
    // }
        std::vector<std::vector<DetectionObj>> TRTInfer::doInference(std::vector<cv::Mat> &frames, float obj_threshold, float confidence_threshold, float nms_threshold,int flag)
    {
        if(!this->_is_inited)
        {
            this->gLogger.log(ILogger::Severity::kERROR,"Module not inited !");
            return {};
        }
        if (frames.size() == 0 || int(frames.size()) > this->input_dims.d[0])
        {
            this->gLogger.log(ILogger::Severity::kWARNING, "Invalid frames size");
            return {};
        }
        std::vector<std::vector<DetectionObj>> batch_res(frames.size());
        cudaStream_t stream = nullptr;
        CHECK(cudaStreamCreate(&stream));
        float *buffer_idx = (float *)buffers[this->inputIndex];
        for (size_t b = 0; b < frames.size(); ++b)
        {
            cv::Mat &img = frames[b];
            if (img.empty())
                continue;
            size_t size_image = img.cols * img.rows * 3;
            size_t size_image_dst = this->input_dims.d[3] * this->input_dims.d[2] * 3;
            memcpy(img_host, img.data, size_image);
            CHECK(cudaMemcpyAsync(img_device, img_host, size_image, cudaMemcpyHostToDevice, stream));
            preprocess_kernel_img(img_device, img.cols, img.rows, buffer_idx,
                                  this->input_dims.d[3], this->input_dims.d[2], stream);
            buffer_idx += size_image_dst;
        }
        this->context->setOptimizationProfileAsync(0, stream);
        this->context->setTensorAddress(INPUT_BLOB_NAME, this->buffers[this->inputIndex]);
        this->context->setTensorAddress(OUTPUT_BLOB_NAME, this->buffers[this->outputIndex]);
        bool success = this->context->enqueueV3(stream);
        if (!success)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "DoInference failed");
            CHECK(cudaStreamDestroy(stream));
            return {};
        }
        CHECK(cudaMemcpyAsync(this->output, buffers[this->outputIndex], frames.size() * this->output_size * sizeof(float), cudaMemcpyDeviceToHost, stream));
        CHECK(cudaStreamSynchronize(stream));
        CHECK(cudaStreamDestroy(stream));

        this->postprocess(batch_res, frames, obj_threshold, confidence_threshold, nms_threshold, flag);

        return batch_res;
    }
    void TRTInfer::calculate_inter_frame_compensation(const int limited_fps)
    {
        std::chrono::system_clock::time_point start_t = std::chrono::system_clock::now();
        double limit_work_time = 1000000L / limited_fps;
        std::this_thread::sleep_for(std::chrono::duration<double, std::micro>(limit_work_time));
        std::chrono::system_clock::time_point end_t = std::chrono::system_clock::now();
        this->inter_frame_compensation = std::chrono::duration<double, std::micro>(end_t - start_t).count() - limit_work_time;
    }

    // std::vector<std::vector<DetectionObj>> TRTInfer::doInferenceLimitFPS(std::vector<cv::Mat> &frames, float obj_threshold, float confidence_threshold, float nms_threshold, const int limited_fps)
    // {
    //     double limit_work_time = 1000000L / limited_fps;
    //     std::chrono::system_clock::time_point start_t = std::chrono::system_clock::now();
    //     std::vector<std::vector<DetectionObj>> result = this->doInference(frames, obj_threshold, confidence_threshold, nms_threshold);
    //     std::chrono::system_clock::time_point end_t = std::chrono::system_clock::now();
    //     std::chrono::duration<double, std::micro> work_time = end_t - start_t;
    //     if (work_time.count() < limit_work_time)
    //     {
    //         std::this_thread::sleep_for(std::chrono::duration<double, std::micro>(limit_work_time - work_time.count() - this->inter_frame_compensation));
    //     }
    //     return result;
    // }

    IHostMemory *TRTInfer::createEngine(const std::string onnx_path, unsigned int maxBatchSize, int input_h, int input_w)
    {
        IBuilder *builder = createInferBuilder(this->gLogger);
        std::cout<<onnx_path<<std::endl;
        uint32_t flag = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        INetworkDefinition *network = builder->createNetworkV2(flag);

        IParser *parser = createParser(*network, gLogger);
        if (!parser->parseFromFile(onnx_path.c_str(), static_cast<int32_t>(ILogger::Severity::kWARNING)))
        {
            this->gLogger.log(ILogger::Severity::kINTERNAL_ERROR, "failed parse the onnx mode");
        }

        // 解析有错误将返回
        for (int32_t i = 0; i < parser->getNbErrors(); ++i)
        {
            std::cout << parser->getError(i)->desc() << std::endl;
        }
        this->gLogger.log(ILogger::Severity::kINTERNAL_ERROR, "successfully parse the onnx mode");
        IBuilderConfig *config = builder->createBuilderConfig();
        config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1U << 30); // 设置工作内存大小限制为1G，防止内存不足报错
        IOptimizationProfile *profile = builder->createOptimizationProfile();

        profile->setDimensions(INPUT_BLOB_NAME, OptProfileSelector::kMIN, Dims4(1, 3, input_h, input_w));
        profile->setDimensions(INPUT_BLOB_NAME, OptProfileSelector::kOPT, Dims4(int(ceil(maxBatchSize / 2.)), 3, input_h, input_w));
        profile->setDimensions(INPUT_BLOB_NAME, OptProfileSelector::kMAX, Dims4(maxBatchSize, 3, input_h, input_w));

        // Build engine
        config->addOptimizationProfile(profile);
        config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 10 << 20);
        config->setFlag(nvinfer1::BuilderFlag::kFP16); // 设置计算
        // config->setFlag(nvinfer1::BuilderFlag::kINT8);
        std::cout<<"Net::Net()"<<std::endl;
        IHostMemory *serializedModel = builder->buildSerializedNetwork(*network, *config);
        std::cout<<"Net::Net()"<<std::endl;
        this->gLogger.log(ILogger::Severity::kINTERNAL_ERROR, "successfully convert onnx to engine");

        // 销毁
        delete network;
        delete parser;
        delete config;
        delete builder;

        return serializedModel;
    }

    int TRTInfer::getInputW()
    {
        return this->input_dims.d[3];
    }

    int TRTInfer::getInputH()
    {
        return this->input_dims.d[2];
    }
}