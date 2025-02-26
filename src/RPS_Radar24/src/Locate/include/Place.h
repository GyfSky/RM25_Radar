//
// Created by plusseven on 23-10-0?.
//
#ifndef RM_RADARDEMO24_SRC_PLACE_INCLUDE_PLACE_H
#define RM_RADARDEMO24_SRC_PLACE_INCLUDE_PLACE_H
#pragma once
//#include "MapAOV.h"
#include "../../ByteTrack/include/STrack.h"
#include <sophus/se3.hpp>

class Place {
private:
public:
    std::vector<cv::Point3d> pts_pnp_3d;
    std::vector<std::array<Eigen::Matrix<double, 3, 1>,10>> all_point_reality_3d;//TODO
//test TODO    
    std::vector<int> all_point_3d_number;
    std::vector<std::array<cv::Point2f,10>> all_point_predict_2d;
    std::vector<cv::Point2f> pts_predict_2d_point;
// 用来粗率定位的参数
    std::vector<std::array<double,3>> getH_abc_3d;
    std::vector<Eigen::Matrix<double,2,2>> matrix_change2;
    std::vector<Eigen::Matrix<double,2,2>> matrix_change3;
    std::vector<Eigen::Matrix<double,2,1>> matrix_2d;
//
    int redfly[5];
    int bluefly[5];

////from mode
    std::string selfColor;
    Place();
    void get_predict_2d(const cv::Mat T, const double fx, const double fy, const double cx, const double cy);
    void get_roughH_config();
    //TODO:
    // Place(Modes &modes) {
    //     this->selfColor = modes.selfColor;
    //     //TODO: add dist
    // }
};

#endif //RM_RADARDEMO24_SRC_PLACE_INCLUDE_PLACE_H
