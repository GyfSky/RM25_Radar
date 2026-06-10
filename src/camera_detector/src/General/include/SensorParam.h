#ifndef RPS_RADAR24_SENSORPARAM_H
#define RPS_RADAR24_SENSORPARAM_H
#include "General.h"


class SensorParam {
public:
    cv::Mat T_2world;
    cv::Mat T_2MainCam;
    cv::Mat extrinsic_Lidar2Cam;
    cv::Mat K;

    char g_strSerialNumber[64];

    Eigen::Matrix<double,4,4> Rt;//world2self
    Eigen::Matrix<double,3,3> R;
    Eigen::Matrix<double,3,1> t;

    std::vector<cv::Point2d> pts_pnp_2d;
    std::vector<cv::Point3d> pts_pnp_3d;

    double a,b,c,d,e,f,g,h,i,tx,ty,tz;//world2self的参数
    double fx,fy,cx,cy;
    int img_w, img_h;
    bool is_setworld2self_config = false;

    SensorParam() = default;
    SensorParam(std::string Name,OurPattern ourPattern,std::string config_path);
    void setworld2self_config(cv::Mat T_main2World);
};


#endif //RPS_RADAR24_SENSORPARAM_H
