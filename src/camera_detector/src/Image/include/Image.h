#ifndef RADAR2023_WITHTRT_IMAGE_H
#define RADAR2023_WITHTRT_IMAGE_H

#include "../../Locate/include/CoordinateSystem.h"
#include "../../Camera_hk/include/Camera_mlt.h"
#include  "../../ByteTrack/include/BYTETracker.h"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/detect_result.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

class  Image {
private:
    PictureSource pictureSource;
    cv::Mat Cam_cloneing;
    cv::VideoCapture cap;
    bool Cam_isOpen = false;
    int input_w;
    int input_h;
    std::string image_dir;
    std::string image_path;
    std::string video_path;
    int start_picture;
    int serial_number;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img;
    cv::Mat ros_img;
    YAML::Node config;
    cv::Mat map_cloneing;
    std::string mapImage_winname;
    std::string mapImage_path;
    int map_w;
    int map_h;
    std::string dynamic_image_path;

public:
    Application application;
    std::shared_ptr<Camera> Camerahk_prt = nullptr;
    cv::Mat map_img;
    cv::Mat map_draw;
    cv::Mat Cam_draw;
    cv::Mat img_temp;

    bool is_getPoint2d_mouse_Cam = true;
    std::string Cam_winname;
    rclcpp::Node* node;
    rclcpp::Time ros_time;
    int classWithoutCar;
    cv::Mat Cam_img;
    Image() = default;
    Image(Application application,std::string config_path,PictureSource pictureSource,char g_strSerialNumber[64],rclcpp::Node* node,std::string Name = "Hik30",int serial_number = -1);
    void Init();
    void Init_calib();
    cv::Mat Image_Get(int &after_picture);
    void getImg(const sensor_msgs::msg::CompressedImage::ConstPtr &rosImg_ptr);
    void Image_Show();
    void draw_result(std::vector<STrack> output_stracks, bool is_cls = false);
    void draw_lidar(interfaces::msg::DetectResult lidar);
    void draw_line(std::vector<MapVertex> &vexs);
    void draw_line_calib(std::vector<MapVertex> &vexs);
    cv::Scalar get_color(int idx);
    void Close();
};

#endif //RADAR2023_WITHTRT_IMAGE_H
