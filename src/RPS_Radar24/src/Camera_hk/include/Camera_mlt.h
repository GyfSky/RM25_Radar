//
// Created by plusseven on 23-10-0?.
//
#ifndef RM_RADARDEMO24_SRC_CAMERA_MLT_INCLUDE_CAMERA_MLT_H
#define RM_RADARDEMO24_SRC_CAMERA_MLT_INCLUDE_CAMERA_MLT_H
#pragma once
#include <future>
#include <thread>
#include "HikCamera.h"
#include "../../General/include/General.h"
#include <yaml-cpp/yaml.h>

class Camera{
public:
    bool is_MainCamSet = false;
    bool is_MainCamWork = true;
    cv::Mat imgMainWait;                                     // 主模组线程缓冲图像
    std::shared_ptr<Camera_hk::HikCamera> HikCamera_sptr_;   // 主相机地址

private:
    int CamGain_;                                        // 相机增益
    int CamExposureTime_;                                // 相机曝光时间
    int CamWhiteBalance_;                                // 相机白平衡
    int CamGamma_;                                       // 相机伽马平衡
    int CamFps_;                                         // 相机最高帧率
    int CamNum_;                                         // 相机每间隔CamNum_抽取一张图片
    std::future<void> clockFuture_;                          // 与promise关联的future对象
    std::mutex mainCamMutex_;                                // 相机读取锁
    std::thread mainCamThread_;                              // 读取相机线程
    std::promise<void> mainCamExit_;                         // 读取相机线程退出信号
    std::future<void> mainCamFuture_;
public:
    Camera();
    Camera(char g_strSerialNumber[64],std::string Name, TF Image_isSave = false_);
    void CamMainSet();
    // void CamMainWork();
    void CamMainSave();
    void CamMainClose();
};

#endif //RM_RADARDEMO24_SRC_CAMERA_MLT_INCLUDE_CAMERA_MLT_H