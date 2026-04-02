#ifndef RADAR2023_WITHTRT_NET_H
#define RADAR2023_WITHTRT_NET_H

#include <yaml-cpp/yaml.h>
#include <cstddef>
#include <future>

#include "Inference.h"
#include "unistd.h"
#include "stdio.h"
#include "stdlib.h"
#include <interfaces/msg/rect.hpp>
#include <interfaces/msg/cluster_rect.hpp>

class Net {
private:
    std::mutex netMutex;
    bool isArmor;
    std::string onnx_path;
    std::string trt_path;
    float conf_thres;//置信度阀值
    float nms_thres; //非极大值抑制阀值
    int batch_size;
    int num_class;
    int input_h;
    int input_w;
    int classWithoutCar;
public:
    TRTInferV1::TRTInfer myInfer;
    std::future<std::vector<std::vector<TRTInferV1::DetectionObj>>> futureObjs;
    bool is_netWorking;

    Net() = default;
    Net(std::string config_path,std::string Name = "net");
    std::vector<std::vector<TRTInferV1::DetectionObj>> NetWork_mlt(std::vector<cv::Mat> &frames);
    void Spin(std::vector<cv::Mat> &frames);
    void getCarImgs(std::vector<std::vector<TRTInferV1::DetectionObj>> Objs, cv::Mat img, std::vector<cv::Mat> &car_imgs);
    void getCarImgs(const interfaces::msg::ClusterRect rects, cv::Mat &img,std::vector<cv::Mat> &car_imgs,int cam_id);

};
#endif //RADAR2023_WITHTRT_NET_H
