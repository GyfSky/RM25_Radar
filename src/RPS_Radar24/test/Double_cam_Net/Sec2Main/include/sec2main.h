//
// Created by plusseven on 24-1-31.
//

#ifndef TRTINFERSAMPLE_SEC2MAIN_H
#define TRTINFERSAMPLE_SEC2MAIN_H
//#include "Inference.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

class Sec2main {
private:
    double a,b,c,d,e,f,g,h,i,tx,ty,tz;

    Eigen::Matrix<double,4,4> Rt;
    Eigen::Matrix<double,3,3> K_M;
    Eigen::Matrix<double,3,3> K_S;
    Eigen::Matrix<double,3,3> R;
    Eigen::Matrix<double,3,1> t;
    double main_fx,main_fy,main_cx,main_cy;

    cv::Mat change_K,change_T;
public:
    double fx,fy,cx,cy;


    Sec2main(const cv::Mat T_World2Sec, const double fx, const double fy, const double cx,const double cy);
    Sec2main(const cv::Mat T_World2Sec, const double fx, const double fy, const double cx,const double cy,
             const cv::Mat T_Main2World, const double main_fx, const double main_fy, const double main_cx, const double main_cy);
    Sec2main(const cv::Mat T_Sec2Main,const cv::Mat K_M,const cv::Mat K_S);
    Sec2main()=default;
    int change2mainCam(float &x, float &y);
    int sec2mainCam(float &x, float &y);
};


#endif //TRTINFERSAMPLE_SEC2MAIN_H
