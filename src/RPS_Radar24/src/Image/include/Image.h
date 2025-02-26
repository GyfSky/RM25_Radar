//
// Created by plusseven on 23-10-22.
//

#ifndef RADAR2023_WITHTRT_IMAGE_H
#define RADAR2023_WITHTRT_IMAGE_H

// #include  "../General/include/General.h"
#include "../../Locate/include/CoordinateSystem.h"
#include "../../Camera_hk/include/Camera_mlt.h"
#include  "../../ByteTrack/include/BYTETracker.h"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/image_byte.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
// #include <cv_bridge/cv_bridge.h>


class  Image {
private:
//COMMON
    std::shared_ptr<Camera> Camerahk_prt = nullptr;
    // Cameras useCamera;
    PictureSource pictureSource;
    Application application;
    // cv::Mat Cam_img;
    cv::Mat Cam_cloneing;
    cv::Mat Cam_draw;
    std::string Cam_winname;
    cv::VideoCapture cap;
    bool Cam_isOpen = false;
    int input_w;
    int input_h;
    bool Image_issave;
    std::string Image_savepath;
    std::string image_dir;
    std::string image_path;
    std::string video_path;
    int start_picture;
    int serial_number;
//rosImg
    // ros::NodeHandle nh;
    // ros::Subscriber sub_img;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img;
    cv::Mat ros_img;
//net
    YAML::Node net_config;
        //RADAR
    cv::Mat map_img;
    cv::Mat map_cloneing;
    cv::Mat map_draw;
    std::string mapImage_winname;
    std::string mapImage_path;
    int map_w;
    int map_h;
    std::string dynamic_image_path;
   

public:
//else
//    std::map<int,std::string> cls_to_string;
    bool is_getPoint2d_mouse_Cam = true;
    int classWithoutCar;
    cv::Mat Cam_img;
    Image() = default;
    Image(Application application,PictureSource pictureSource,std::string Name = "Hik30",TF Image_isSave=false_,SaveImagePath saveImagePath=disk02,int serial_number = -1);
    Image(Application application,PictureSource pictureSource,char g_strSerialNumber[64],std::string Name = "Hik30",TF Image_isSave=false_,SaveImagePath saveImagePath=disk02,int serial_number = -1);
    void Init(int argc,char *argv[]);
//    cv::Mat Image_Get(int after_picture = 0);
    void GetGammaCorrection(Mat& src, Mat& dst, const float fGamma) ;
    cv::Mat Image_Get(int &after_picture,int argc, char **argv);
    void getImg(const sensor_msgs::msg::CompressedImage::ConstPtr &rosImg_ptr);
    void Image_Show();
    void draw_rusult(std::vector<Car> cars,std::vector<Armor> armors,bool isShow=true);
    void draw_rusult(std::vector<TRTInferV1::Object> objs,bool isShow=true);
    void draw_rusult(std::vector<Car> &cars,std::vector<Armor> &armors,cv::Mat img,std::string winname = "draw");
    void draw_rusult(std::vector<STrack> output_stracks, bool is_cls = false);
    void draw_rusult(cv::Rect rect, cv::Point3d xyz, cv::Mat img_draw);
//    void draw_line(Place &place);
    void draw_line(std::vector<MapVertex> &vexs);
    cv::Scalar get_color(int idx);
    void setSaveMode();
    void Close();
};
//void draw_rusult(std::vector<Car> &cars,std::vector<Armor> &armors,Image &image,std::map<int,std::string> cls_to_string);


//void SetNet(std::map<int,std::string> &cls_to_string);
//void draw_line(Place &place,cv::Mat &radar_cloneimg);


#endif //RADAR2023_WITHTRT_IMAGE_H
