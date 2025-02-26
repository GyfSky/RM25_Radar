//
// Created by plusseven on 24-1-20.
//

#ifndef JSONCPP_TEST_PREDICT_H
#define JSONCPP_TEST_PREDICT_H

#include"CoordinateSystem.h"
#include <ceres/ceres.h>

// 代价函数的计算模型
class CURVE_FITTING_COST {
private:
    const double _s, _t;    // x,y数据 ；在C中，const 结构体变量表示结构体中任何数据域均不允许改变，且需要另一个结构体变量进行初始化
public:
    CURVE_FITTING_COST(double s, double t) : _s(s), _t(t){}
    // 残差的计算
    template<typename T>
    bool operator()(
            const T *const vel_acc,     // 模型参数，有2维,分别是速度和加速度 // C++基础——const T、const T*、T const、const T&、const T&  https://blog.csdn.net/weixin_33460831/article/details/108852450
            T *change_shifts) const {
        change_shifts[0] = T(_s) - (vel_acc[0] * _t + 1./2 * vel_acc[1] * _t * _t ); // (s_k+1 - s_k) - (v * t + 1/2 * a * t^2)
        return true;
    }
};



class Predict {
private:
    ceres::Problem problem;    // 构建最小二乘问题
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    ceres::Solver::Summary summary;                // 优化信息
    bool isguess = true;
    double guess_speed_vel = 0.8;
    double guess_angle_vel = 0.0;
    double guess_angle_acc = 10;
    double guess_speed_acc = 0.5;
    double dt = 0.3; //TODO: 时间戳

public:
    Predict();
    void loss_track_predict(std::vector<MapVertex> &vexs ,std::vector<std::vector<MapEdge>> arcs, std::vector<STrack> &tracks);
    void loss_track_predict(const cv::Mat T_Cam2World,const double fx, const double fy,const double cx, const double cy,std::vector<STrack> &tracks);
    void loss_track_predict(std::vector<MapVertex> &vexs, std::vector<STrack> &tracks);
    void loss_track_predict(std::vector<STrack> &tracks);
    void getVelAccfromPathFitting(double *vel_acc, std::vector<double> shifts);
    std::vector<double> getAngles(std::vector<cv::Point3d> Locate3Ds);
    void getRectFromLocate3D(const cv::Mat T_Cam2World, const double fx, const double fy, const double cx, const double cy, std::vector<STrack> &tracks);
    void getRectFromLocate3D(const cv::Mat T_Cam2World, const double fx, const double fy, const double cx, const double cy, STrack &track);
    void setWindmillLocate3D();
};


#endif //JSONCPP_TEST_PREDICT_H
