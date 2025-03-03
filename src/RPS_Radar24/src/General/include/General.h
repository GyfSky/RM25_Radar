//
// Created by plusseven on 23-7-16.
//

#ifndef SRC_RPS_RADAR24_SRC_GENERAL_INCLUDE_GENERAK_H
#define SRC_RPS_RADAR24_SRC_GENERAL_INCLUDE_GENERAK_H

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <vector>
#include <sstream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <random>

#include <yaml-cpp/yaml.h>

#include <complex>
// #include <mathcalls.h>
#include <algorithm>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <queue>
#include <chrono>

// #include "Mouse.h"

# define CV_MAT_MATRIX_PATH "/home/thesky/RM25_Radar/src/RPS_Radar24/config/CVMAT_matrix.yml"
# define YAML_CONFIC_PATH "/home/thesky/RM25_Radar/src/RPS_Radar24/config/Config.yaml"
# define YAML_NETCONFIC_PATH "/home/thesky/RM25_Radar/src/RPS_Radar24/config/net.yaml"
# define YAML_PLACE_CONFIC_PATH  "/home/thesky/RM25_Radar/src/RPS_Radar24/config/Place.yaml"
# define YAML_COSTCONFIC_PATH "/home/thesky/RM25_Radar/src/RPS_Radar24/config/cost.yaml"
# define STITCH_CONFIC_PATH "/home/thesky/RM25_Radar/src/RPS_Radar24/config/imageStitch.yaml"


enum Application{Radar,Common};
enum TF {true_,false_};
enum PictureSource {picture_dir,single_picture,video,camera_,ros1};
enum Detection      {netDetection, jsonRect, unityRect};//？？
// enum Cameras       {DahangHikang,Dahang,Hikang,Hikang30,Hikang31,Hikang60};
enum OurPattern    {red = 0,blue = 1};
enum CamPosition   {right,left};
enum UsePort       {USB0 = 0,USB1 = 1, USB2 = 2};
enum SaveImagePath {disk02,ssdgaoyuan};


class Modes {
//private:
public:

//-------------------------------------------------------------------------------------------------------//
//                                              可改参数‘s name                                           //
//------------------------------------------------------------------------------------------------------//
    Application application;
    PictureSource pictureSource;        //图片来源
    // Cameras useCamera;  //相机的开启与否，所使用的相机
    TF isOpenMid70;
    TF isUseMid70;
    Detection detectionMode;
    OurPattern ourPattern;              //己方颜色
    TF Port_isOpen;UsePort usePort;     //串口的开启与否，所使用的串口
    TF isSave;TF Mid70_isSave;SaveImagePath saveImagePath;   //是否保存图片，保存路径
    Modes();

};

class Armor{
public:
    //param
    float conf,x1,y1,x2,y2;
    cv::Point2d Locate2D;
    cv::Rect rect;
    int cls;

    Armor(float conf,float x1,float y1,float x2,float y2,int cls){
        this->conf = conf;
        this->x1 = x1;
        this->y1 = y1;
        this->x2 = x2;
        this->y2 = y2;
        rect = cv::Rect(cv::Point2d(x1,y1),cv::Point2d(x2,y2) );
        this->cls = cls;
        Locate2D =  cv::Point2f ((x1+x2)/2,(y1+y2)/2);
    }
    Armor(cv::Rect armor, int cls,float conf = 0.8){
        this->conf = conf;
        this->rect = armor;
        this->x1 = armor.x - armor.width/2;
        this->y1 = armor.y - armor.height/2;
        this->x2 = armor.x + armor.width/2;
        this->y2 = armor.y + armor.height/2;
        this->cls = cls;
        Locate2D =  cv::Point2f ((x1+x2)/2,(y1+y2)/2);
    }
    Armor() = default;

};


class Car{
public:
    //param
    float conf=0.0,conf_armor=0.0,x1=0.0,y1=0.0,x2=0.0,y2=0.0;
    cv::Point2d Locate2D;
    cv::Point3d Locate3D;
    std::vector<Armor> ArmorsInCar; //TODO: del
    bool isGuess= false;
    Eigen::MatrixXd  ws_armorConfMatrix = Eigen::MatrixXd::Zero(1,20);
    Eigen::MatrixXd  ws_armorConfMatrix_BR = Eigen::MatrixXd::Zero(1,6);
    cv::Rect rect;
    int cls = -1;
    //make class
    Car(float conf,float x1,float y1,float x2,float y2){

        this->conf = conf;
        this->x1 = x1;
        this->y1 = y1;
        this->x2 = x2;
        this->y2 = y2;
        rect = cv::Rect(cv::Point2d(x1,y1),cv::Point2d(x2,y2) );
        Locate2D = cv::Point2d ((x1+x2)/2,y1 +(y2-y1)*0.95);
    }
    Car() = default;

    };


// template <typename T>
// T min_(T a,T b);
//double min_(double a,double b);
//double max_(double a,double b);
//double get2Ddistance(double x1,double y1, double x2, double y2);
int initornot(std::array<cv::Point2f,25> predict2d ,cv::Point xy, int pointNum);
int initornot3D(std::array<Eigen::Matrix<double, 3, 1>,25>  points_reality_3d ,cv::Point3d &xyz, int pointNum);

double getAngle180(double x, double y);
double getAngle360(double x, double y);

double line2line_distance(double p1_x, double p1_y,double p2_x,double p2_y,double len,
                          double p1_x_,double p1_y_,double p2_x_,double p2_y_,double len_ );

//void eigenMat2VecVec(Eigen::MatrixXd &eigen,std::vector<std::vector<double>> &vecVec)
//void eigenMat2VecVec(Eigen::MatrixXd &eigen,std::vector<std::vector<float>> &vecVec);

template <typename T>
void eigenMat2VecVec(Eigen::MatrixXd &eigen,std::vector<std::vector<T>> &vecVec){
    int col = eigen.cols();//列
    int raw = eigen.rows();//行
    Eigen::RowVectorXd vec_d(col);
    for(int i=0;i<raw;i++){
        vec_d = eigen.block(i,0,1,col);//行向量
        std::vector<float> vec(vec_d.data(), vec_d.data() + vec_d.size());
        vecVec.push_back(vec);
    }
}

 template <typename T>
 T min_(T a,T b){
     return (a<b?a:b);
 }

template <typename T>
T max_(T a,T b){
    return (a<b?b:a);
}


inline double get2Ddistance(double x1,double y1, double x2, double y2){
    double x = x1-x2, y = y1-y2;
    if(abs(x)<1e-6)
        x = 1e-3;
    if(abs(y)<1e-6)
        y = 1e-3;
    return sqrt(pow(x,2) + pow(y,2));
}


#endif //SRC_RPS_RADAR24_SRC_GENERAL_INCLUDE_GENERAK_H
