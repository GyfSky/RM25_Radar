//
// Created by thesky on 25-5-5.
//
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


class PrepareCalib : public rclcpp::Node {
    public:
    PrepareCalib(std::string name):Node(name){
        cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);
        image_pub_= this->create_publisher<sensor_msgs::msg::Image>("/image", 10);
        timer_image = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
            cv::Mat image=cv::imread("resource/644.jpg");
            sensor_msgs::msg::CameraInfo cam_info;
            std::vector<double> D{0.0, 0.0, 0.0, 0.0, 0.0};

            cam_info.height = 1080;
            cam_info.width = 1440;
            cam_info.distortion_model = "plumb_bob";
            cam_info.d = D;
            cam_info.k = {
                1703.57857839016,0,776.621431609538,0,1698.69114037183,558.752840063006,0,0,1
            };
            cam_info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            cam_info.binning_x = 0;
            cam_info.binning_y = 0;
            cam_info.header.frame_id = "camera";  //frame_id为camera，也就是相机名字
            cam_info.header.stamp = this->get_clock()->now();

            auto img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

            cam_info_pub_->publish(cam_info);
            image_pub_->publish(*img);
        });
        pc_sub=this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_3JEDM7A00106241", 10, std::bind(&PrepareCalib::callback, this, std::placeholders::_1));

        RCLCPP_WARN(this->get_logger(), "PrepareCalib start");
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!bag_start) {
            if (delay_time>10) {
                bag_start=true;
                std::string path="/home/thesky/RM25_Radar/livox/test";
                std::string cmd_str = "gnome-terminal -x bash -c 'cd livox && ros2 bag record -o "+path+" "+"/livox/lidar_3JEDM7A00106241 /image /camera_info"+" '" + "&";
                int ret = system(cmd_str.c_str());
                std::cout << "cmd_str: " << cmd_str << std::endl;
                if(ret != 0){
                    std::cerr << "\033[33m" << "save bag may have error !!! Please check path" << "\033[0m" <<std::endl;
                }
            }else {
                delay_time++;
            }
        }
    }

    ~PrepareCalib(){}

    private:
    rclcpp::TimerBase::SharedPtr timer_image;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
    bool bag_start =false;
    int delay_time=0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PrepareCalib>("PrepareCalib");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}