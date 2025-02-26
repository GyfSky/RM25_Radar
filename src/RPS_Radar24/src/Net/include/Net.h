//
// Created by plusseven on 23-7-15.
//
# define YAML_CONFIC_PATH "/home/thesky/RM_radardemo24/src/RPS_Radar24/config/Config.yaml"
# define YAML_NETCONFIC_PATH "/home/thesky/RM_radardemo24/src/RPS_Radar24/config/net.yaml"

#ifndef RADAR2023_WITHTRT_NET_H
#define RADAR2023_WITHTRT_NET_H

// #include "../../Image/include/Image.h"
#include <yaml-cpp/yaml.h>
#include <cstddef>
#include <future>

#include "Inference.h"
#include "unistd.h"
#include "stdio.h"
#include "stdlib.h"

class Net {
private:
    YAML::Node net_config;
    std::mutex netMutex;
    bool isArmor;
    std::string onnx_path;
    std::string trt_path;
//    TRTInferV1::TRTInfer myInfer;
    float obj_thres;
    float conf_thres;//置信度阀值
    float nms_thres; //非极大值抑制阀值
    int batch_size;
    int num_class;
    int input_h;
    int input_w;
    int classWithoutCar;
public:
    TRTInferV1::TRTInfer myInfer;
    // std::promise<std::vector<std::vector<TRTInferV1::DetectionObj>>> promiseObj;  
    //将future和promise关联
    // this->futureObj = this->promiseObj.get_future();
    std::future<std::vector<std::vector<TRTInferV1::DetectionObj>>> futureObjs;
    std::future<std::vector<std::vector<TRTInferV1::Object>>> futureObjs_change;

    std::thread netMainloop;
    bool is_netWorking;
    Net() = default;
    Net(std::string Name = "net");
    // void NetWork(cv::Mat img ,std::vector<std::vector<TRTInferV1::DetectionObj>> &DetectionObjs);
    std::vector<std::vector<TRTInferV1::DetectionObj>> NetWork_mlt(std::vector<cv::Mat> &frames);
    std::vector<std::vector<TRTInferV1::Object>> NetWork_confs_mlt(std::vector<cv::Mat> &frames);
//    std::vector<std::vector<TRTInferV1::Object>> NetWork_mlt(cv::Mat img);

    // std::vector<std::vector<TRTInferV1::DetectionObj>> NetWork_mlt(std::vector<cv::Mat> frames);
    void Spin(std::vector<cv::Mat> &frames);
    void Spin_confs(std::vector<cv::Mat> &frames);

    void getCarImgs(std::vector<std::vector<TRTInferV1::DetectionObj>> Objs, cv::Mat img, std::vector<cv::Mat> &car_imgs);
    void getCarImgs(std::vector<std::vector<TRTInferV1::Object>> Objs, cv::Mat img, std::vector<cv::Mat> &car_imgs);
    // void Spin(std::vector<cv::Mat> frames);
//    void Car_Armor(
//            std::vector<TRTInferV1::DetectionObj> &DetectionObjs,
//            std::vector<Car> &cars,std::vector<Armor> &armors);
//
//    std::vector<std::vector<std::vector<int>>>  Car_Armor(
//            std::vector<TRTInferV1::Object> DetectionObjs,std::vector<std::vector<int>> list_,
//            std::vector<TRTInferV1::Object> &cars,std::vector<TRTInferV1::Object> &armors);
//
//    std::vector<std::vector<std::vector<int>>> getSTrackwithArmor(
//            std::vector<TRTInferV1::Object> DetectionObjs, int allCam,vector<std::vector<int>> list_,
//            std::vector<std::vector<STrack>> &tracked_stracks);

};
#endif //RADAR2023_WITHTRT_NET_H
