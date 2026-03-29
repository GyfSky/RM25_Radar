//
// Created by plusseven on 24-4-8.
//

#ifndef RPS_RADAR24_SENSORPARAM_H
#define RPS_RADAR24_SENSORPARAM_H
#include "General.h"


class SensorParam {
public:
    cv::Mat T_2world;
    cv::Mat T_2MainCam;

    //extrinsic
    cv::Mat extrinsic_Lidar2Cam;

    cv::Mat K;

    char g_strSerialNumber[64];

    Eigen::Matrix<double,4,4> Rt;//world2self
    Eigen::Matrix<double,3,3> R;
    Eigen::Matrix<double,3,1> t;
//    Eigen::Matrix<double,3,3> K_eigen;

    //2d点
    std::vector<cv::Point2d> pts_pnp_2d;
    //3d点
    std::vector<cv::Point3d> pts_pnp_3d;


    double a,b,c,d,e,f,g,h,i,tx,ty,tz;//world2self的参数
    double fx,fy,cx,cy;

    int img_w, img_h;

    bool is_setworld2self_config = false;

    //camPosition区分左右相机
    SensorParam() = default;
    SensorParam(std::string Name, CamPosition camPosition ,OurPattern ourPattern,std::string config_path);
    void setworld2self_config(cv::Mat T_main2World);
    int change2main(float &x, float &y,
           const double main_fx, const double main_fy, const double main_cx, const double main_cy);
};


#endif //RPS_RADAR24_SENSORPARAM_H
