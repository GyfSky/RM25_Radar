//
// Created by plusseven on 23-7-16.
//

#ifndef RADAR_GIT_NEW_MOUSE_H
#define RADAR_GIT_NEW_MOUSE_H

#include "General.h"

class Mouse {
public:
    std::string Name;
    std::vector<cv::Point2d> point2d_mouse_xy;
    cv::Mat image;
    int flag_num;
    // int flag_back;//TODO
    cv::Mat temp_image;
    // cv::VideoCapture cap;
    std::string winname;
    int pointNumber;
    //test
    cv::Rect test;
    bool is_test = false;
    int w;
    int h;
    //livox
    int paddingu;
    int paddingv;

    ~Mouse(){};
    Mouse (cv::Mat &image, std::string Name);
    Mouse (cv::Mat &image,int pointNumber = 5,std::string winname = "default");
};
    void onMouse(int event, int x, int y, int flags, void *para);
    std::vector<cv::Point2d> GetPoint2d_mouse(cv::Mat &imshowMat, std::string Name);


#endif //RADAR_GIT_NEW_MOUSE_H
