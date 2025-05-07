//
// Created by plusseven on 23-7-16.
//

#include "../include/Mouse.h"


//e.g 1
// test main
// int main(){
//     Mouse mouse;
//     mouse.flag_num = 0;
//     mouse.flag_back = 0;
//     mouse.pointNumber = 5;
//     mouse.camera_winname = "window_radar";
//     mouse.image = cv::imread("/home/plusseven/桌面/RPSradar2023/something/1439.jpg");
//     cv::namedWindow(mouse.camera_winname);
////key
//     while (true){
//         cv::setMouseCallback("window_radar", onMouse, &mouse);
//         cv::imshow("window_radar", mouse.image);
//         cv::waitKey(1);
//     }

//     std::cout << mouse.point2d_mouse_xy[0][0];
//     return 0;
// }

//e.g.  2
// std::vector<cv::Point2d> Livox::GetPoint2d_mouse_liovx(){
//     Mouse mouse(imshowDepthBackgoundMat,"livox");
//     while (mouse.point2d_mouse_xy.size() != mouse.pointNumber){
//         cv::setMouseCallback(mouse.winname, onMouse, &mouse);
//         cv::imshow(mouse.winname, mouse.image);
//         cv::waitKey(1);
//     }
//     std::cout << "over mouse_livox:" + std::to_string(mouse.point2d_mouse_xy[0].x) << std::endl;
//     return mouse.point2d_mouse_xy;
// }

Mouse::Mouse(cv::Mat &image, std::string Name){
    this->Name = Name;
    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    this->image = image;
    this->flag_num = 0;
    this->pointNumber = config[Name]["mousePointNumber"].as<int>();
    this->winname = config[Name]["winname"].as<std::string>();
    this->is_test = config["test"]["is_test"].as<bool>();
    this->w = config["test"]["rect"]["w"].as<int>();
    this->h = config["test"]["rect"]["h"].as<int>();
    if(Name == "Livox"){
        this->paddingu = config[Name]["paddingu"].as<int>();
        this->paddingv = config[Name]["paddingv"].as<int>();
    }

}

Mouse::Mouse (cv::Mat &image,int pointNumber/* = 5*/, std::string winname/* = "default"*/){
    this->image = image;
    this->flag_num = 0;
    this->pointNumber = pointNumber;
    this->winname = winname;
}

std::vector<cv::Point2d> GetPoint2d_mouse(cv::Mat &imshowMat, std::string Name){
    Mouse mouse(imshowMat,Name);
    while (mouse.point2d_mouse_xy.size() != mouse.pointNumber){
        cv::setMouseCallback(mouse.winname, onMouse, &mouse);
        cv::imshow(mouse.winname, mouse.image);
        cv::waitKey(1);
    }
    std::cout << "over mouse_livox:" + std::to_string(mouse.point2d_mouse_xy[0].x) << std::endl;
    return mouse.point2d_mouse_xy;
}

cv::Rect GetRect_mouse(cv::Mat &imshowMat, std::string Name){
    Mouse mouse(imshowMat,Name);
    while (mouse.rect_points.size()!=2){
        cv::setMouseCallback(mouse.winname, onMouseRect, &mouse);
        cv::imshow(mouse.winname, mouse.image);
        cv::waitKey(1);
    }
    double w= mouse.rect_points[1].x - mouse.rect_points[0].x;
    double h= mouse.rect_points[1].y - mouse.rect_points[0].y;
    return cv::Rect(mouse.rect_points.front().x,mouse.rect_points.front().y,w,h);
}

// cv::Rect GetPoint2d_mouse(cv::Mat &imshowMat,std::string Name){
//     YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
//     std::string win_name = config[Name]["winname"].as<std::string>();
//     Mouse mouse(imshowMat,1,win_name);
//     cv::setMouseCallback(mouse.winname, onMouse, &mouse);
    
//     cv::imshow(mouse.winname, mouse.image);
//     cv::waitKey(1);
//     std::cout << "over mouse_livox:" + std::to_string(mouse.point2d_mouse_xy[0].x) << std::endl;
//     return mouse.point2d_mouse_xy;
// }


//----------------鼠标回调函数---------------------------------
void onMouse(int event, int x, int y, int flags, void *para){
//    mouse.image = *((cv::Mat*)para);
    Mouse &mouse = *((Mouse*)para);
    if(event==cv::EVENT_LBUTTONDOWN) {   //左键按下
        if(mouse.flag_num<mouse.pointNumber){
            if(mouse.Name == "Livox"){
                mouse.point2d_mouse_xy.push_back(cv::Point2d(x,y));
            }else{
                mouse.point2d_mouse_xy.push_back(cv::Point2d(x,y));
            }
            cv::Mat img = mouse.image.clone();
            cv::circle(mouse.image, cv::Point(x, y),1, cv::Scalar(0, 255, 250), 1);
            std::cout << x <<","<< y << std::endl;
            cv::putText(mouse.image, (std::to_string(x) + "," + std::to_string(y)),cv::Point(x + 10, y + 10), cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(0, 255, 250), 1);
            cv::imshow(mouse.winname, mouse.image);
        
            mouse.flag_num = mouse.flag_num + 1;
            // mouse.flag_back = 1;
        }
        // if(mouse.is_test){
        //     mouse.test = cv::Rect(((int)mouse.point2d_mouse_xy[0].x - mouse.w/2),((int)mouse.point2d_mouse_xy[0].y - mouse.h),mouse.w,mouse.h);
        //     cv::rectangle(mouse.image,mouse.test,255,2,100);
        // }

    }
    if(event==cv::EVENT_RBUTTONDOWN) {   //右键按下
        if(mouse.flag_num>0){
            cv::Point2d temp;
            if(mouse.Name == "Livox"){
                temp=mouse.point2d_mouse_xy.back();
                mouse.point2d_mouse_xy.pop_back();
            }else{
                temp=mouse.point2d_mouse_xy.back();
                mouse.point2d_mouse_xy.pop_back();
            }
            cv::putText(mouse.image,"------",cv::Point(temp.x + 10, temp.y + 10), cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(0, 0, 255), 2);
            cv::imshow(mouse.winname, mouse.image);
            mouse.flag_num = mouse.flag_num - 1;
        }
    }
}

void onMouseRect(int event, int x, int y, int flags, void *para) {
    Mouse &mouse = *((Mouse*)para);
    if(event==cv::EVENT_MBUTTONDOWN) {
        mouse.rect_points.push_back(cv::Point2d(x,y));
        cv::Mat img = mouse.image.clone();
        cv::circle(img, cv::Point(x, y),1, cv::Scalar(0, 255, 250), 1);
        cv::putText(img, (std::to_string(x) + "," + std::to_string(y)),cv::Point(x + 10, y + 10), cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(0, 255, 250), 1);
        cv::imshow(mouse.winname, img);
    }else if(event==cv::EVENT_MOUSEMOVE&&flags & cv::EVENT_FLAG_MBUTTON) {
        cv::Mat img = mouse.image.clone();
        cv::circle(img,mouse.rect_points.front(),1, cv::Scalar(0, 255, 250), 1);
        cv::circle(img, cv::Point2d(x,y),1, cv::Scalar(0, 255, 250), 1);
        cv::rectangle(img,mouse.rect_points.front(), cv::Point2d(x,y),cv::Scalar(0, 255, 250), 1);
        cv::imshow(mouse.winname, img);
    }else if(event == cv::EVENT_MBUTTONUP) {
        mouse.rect_points.push_back(cv::Point2d(x,y));
        cv::Mat img = mouse.image.clone();
        cv::rectangle(img,mouse.rect_points.front(), cv::Point2d(x,y),cv::Scalar(0, 255, 250), 1);
        cv::imshow(mouse.winname, img);
    }
}