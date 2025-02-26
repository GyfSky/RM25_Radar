//
// Created by plusseven on 24-4-22.
//
#include "../include/Inference.h"

namespace TRTInferV1
{

    void TRTInfer::nms(std::vector<Object> &input_boxes, float &nms_threshold)
    {
        std::sort(input_boxes.begin(), input_boxes.end(), [](Object a, Object b)
        { return a.confidence > b.confidence; });
        std::vector<float> vArea(input_boxes.size());
        for (int i = 0; i < int(input_boxes.size()); ++i)
        {
            vArea[i] = (input_boxes.at(i).x2 - input_boxes.at(i).x1 + 1) * (input_boxes.at(i).y2 - input_boxes.at(i).y1 + 1);
        }

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
                float xx1 = (std::max)(input_boxes[i].x1, input_boxes[j].x1);
                float yy1 = (std::max)(input_boxes[i].y1, input_boxes[j].y1);
                float xx2 = (std::min)(input_boxes[i].x2, input_boxes[j].x2);
                float yy2 = (std::min)(input_boxes[i].y2, input_boxes[j].y2);

                float w = (std::max)(float(0), xx2 - xx1 + 1);
                float h = (std::max)(float(0), yy2 - yy1 + 1);
                float inter = w * h;
                float ovr = inter / (vArea[i] + vArea[j] - inter);

                if (ovr >= nms_threshold)
                {
                    isSuppressed[j] = true;
                }
            }
        }
        // return post_nms;
        int idx_t = 0;
        input_boxes.erase(std::remove_if(input_boxes.begin(), input_boxes.end(), [&idx_t, &isSuppressed](const Object &f)
                          { return isSuppressed[idx_t++]; }),
                          input_boxes.end());
    }
    void TRTInfer::decode_output(std::vector<Object> &res, cv::Mat &frame, float *pdata, float &obj_threshold, float &confidence_threshold, float &nms_threshold)
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
        int n = 0, q = 0, i = 0, j = 0, row_ind = 0, k = 0; /// xmin,ymin,xamx,ymax,box_score, class_score
        for (n = 0; n < this->num_stride; ++n)
        {
            const float stride = pow(2, n + 3);
            int num_grid_x = (int)ceil(this->input_dims.d[3] / stride);
            int num_grid_y = (int)ceil(this->input_dims.d[2] / stride);
            for (q = 0; q < 3; ++q)
            {
                for (i = 0; i < num_grid_y; ++i)
                {
                    for (j = 0; j < num_grid_x; ++j)
                    {
                        float box_score = pdata[4];
                        if (box_score > obj_threshold)
                        {
                            int max_ind = 0;
                            float max_class_score = 0;
//                            std::vector<float> confidences;
                            for (k = 0; k < this->num_classes; ++k)
                            {
                                if (pdata[k + 5] > max_class_score)
                                {
                                    max_class_score = pdata[k + 5];
                                    max_ind = k;
                                }
//                                confidences.push_back(pdata[k + 5] * box_score);

                            }
                            max_class_score *= box_score;

                            if (max_class_score > confidence_threshold)
                            {
                                float cx = pdata[0]; /// cx
                                float cy = pdata[1]; /// cy
                                float w = pdata[2];  /// w
                                float h = pdata[3];  /// h

                                float xmin = (cx - padw - 0.5 * w) * ratiow;
                                float ymin = (cy - padh - 0.5 * h) * ratioh;
                                float xmax = (cx - padw + 0.5 * w) * ratiow;
                                float ymax = (cy - padh + 0.5 * h) * ratioh;

                                if (xmin >= 0. && ymin >= 0. && xmax <= float(frame.cols) && ymax <= float(frame.rows)){
                                    res.emplace_back(Object{max_ind, max_class_score, xmin, ymin, xmax, ymax,w,h});
//                                    res.emplace_back(Object{max_ind, max_class_score,confidences, xmin, ymin, xmax, ymax,w,h});
                                }
                            }
                        }
                        ++row_ind;
                        pdata += this->output_dims.d[2];


                    }
                }
            }
        }
        this->nms(res, nms_threshold);
    }

    void TRTInfer::postprocess(std::vector<std::vector<Object>> &batch_res, std::vector<cv::Mat> &frames, float &obj_threshold, float &confidence_threshold, float &nms_threshold)
    {
        for (int b = 0; b < int(frames.size()); ++b)
        {
            auto &res = batch_res[b];
            this->decode_output(res, frames[b], &this->output[b * this->output_size], obj_threshold, confidence_threshold, nms_threshold);
        }
    }


    std::vector<std::vector<Object>> TRTInfer::doInference_confs(std::vector<cv::Mat> &frames, float obj_threshold, float confidence_threshold, float nms_threshold)
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
        std::vector<std::vector<Object>> batch_res(frames.size());
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

        this->postprocess(batch_res, frames, obj_threshold, confidence_threshold, nms_threshold);

        return batch_res;
    }


}
