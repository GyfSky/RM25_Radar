#include "../include/Radar.h"
#include <cv_bridge/cv_bridge.h>

//modes构造函数
Modes::Modes(){
    //choose  一些选项
/////////////////////////////////////////////////////////////////////////////////////////////////////////
////------------------------------------- 这里需要修改！！！！---------------------------------------------|
    application   = Application::Radar;//？？
    // pictureSource = PictureSource::camera_ ;   //图片来源          |
    pictureSource = PictureSource::ros1 ;
    isOpenMid70   = TF::false_;
    isUseMid70    = TF::false_;
    detectionMode = Detection::netDetection;//？？
    ourPattern    = OurPattern::red;                 //己方颜色       |
    Port_isOpen   = TF::false_;                       //串口的开启与否  |
    usePort       = UsePort::USB0;                    //所使用的串口    |
    isSave        = TF::true_;                       //是否保存图片    |
//    saveImagePath = SaveImagePath::disk02;            //保存路径       |z

//compititon
//    application   = Application::Radar;
//    pictureSource = PictureSource::camera_ ;    //图片来源          |
//    isOpenMid70   = TF::true_;
//    isUseMid70    = TF::false_;
//    detectionMode = Detection::netDetection;
//    ourPattern    = OurPattern::blue;                  //己方颜色       |
//    Port_isOpen   = TF::true_  ;                       //串口的开启与否  |
//    usePort       = UsePort::USB0;                    //所使用的串口    |
//    isSave        = TF::true_;                       //是否保存图片    |


};

bool flag = false;
cv::Mat img_test;
rclcpp::Time ros_time;
// void getImgTest(const sensor_msgs::msg::CompressedImage::ConstPtr msg) {
//     cv::Mat img=cv::imdecode(msg->data, cv::IMREAD_COLOR);
//
//     img_test=img.clone();
//     flag=true;
// }

void getImgTest(const sensor_msgs::msg::CompressedImage::ConstPtr &rosImg_ptr){
    ros_time=rclcpp::Clock().now();
    img_test = cv_bridge::toCvCopy(rosImg_ptr, "bgr8")->image;
    if(!img_test.empty()){
        // cv::cvtColor(img,img_test,CV_8UC3);
        // img_test = img;
        flag=true;
    }else{
        std::cout << "error!!!!!" << std::endl;
    }
}

int main(int argc, char **argv){
    MyRadar radar;
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("img_listener");
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img;
    rclcpp::Subscription<interfaces::msg::NetDetect>::SharedPtr sub_car;
    rclcpp::Subscription<interfaces::msg::NetDetect>::SharedPtr sub_armor;
    rclcpp::Subscription<interfaces::msg::DetectResult>::SharedPtr sub_lidar;
    sub_img = nh->create_subscription<sensor_msgs::msg::CompressedImage>("/compressed_image", rclcpp::SensorDataQoS(), &getImgTest);
    // sub_img = nh->create_subscription<sensor_msgs::msg::Image>("/image", 10,
    //     [radar](const sensor_msgs::msg::Image::SharedPtr msg) {
    //         radar.MainCam_Image_ptr->Cam_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    //         // auto msg_test = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", radar.MainCam_Image_ptr->Cam_img).toImageMsg();
    //         // auto now_time = std::chrono::steady_clock::now();
    //         // auto img = cv_bridge::toCvShare(msg_test, "bgr8")->image;
    //         // auto end_time = std::chrono::steady_clock::now();
    //         // float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
    //         // std::cout<<"-----------------"<<dur_time/1000<<std::endl;
    //         flag=true;
    //     }
    // );
    sub_lidar= nh->create_subscription<interfaces::msg::DetectResult>("/lidar_detect", 10,
        [&radar](const interfaces::msg::DetectResult::SharedPtr msg) {
            radar.lidar_det=*msg;
        });
    sub_car = nh->create_subscription<interfaces::msg::NetDetect>("/car_result", rclcpp::SensorDataQoS(),
        [&radar](const interfaces::msg::NetDetect::SharedPtr msg) {
            // ros_time=ros_time=rclcpp::Clock().now();
            radar.car_det=*msg;
        });
    sub_armor = nh->create_subscription<interfaces::msg::NetDetect>("/armor_result", rclcpp::SensorDataQoS(),
        [&radar](const interfaces::msg::NetDetect::SharedPtr msg) {
            radar.armor_det=*msg;
        });

    radar.detect_pub=nh->create_publisher<interfaces::msg::DetectFrame>("/resolve_result", 10);
    // while(true){
    //     radar.Init(argc, argv);
    //     radar.Spin(argc, argv);
    //     if(cv::waitKey(1) == 'q'){
    //         radar.is_close = true;
    //     }
    //     if(radar.is_close){
    //         break;
    //     }
    // }
    while(rclcpp::ok()){
        auto now_time = std::chrono::steady_clock::now();
        rclcpp::spin_some(nh);////
        if(flag){////
            radar.MainCam_Image_ptr->Cam_img= img_test;////
            radar.time_now=ros_time;////
            radar.Init(argc, argv);
            radar.Spin(argc, argv);
            if(cv::waitKey(1) == 'q'){
                radar.is_close = true;
            }
            if(radar.is_close){
                break;
            }
        }////
        auto end_time = std::chrono::steady_clock::now();
        float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();    
        RCLCPP_WARN(nh->get_logger(), "time is %f s", dur_time/1000);
    }
    radar.Close();
    rclcpp::shutdown();
    return 0;
}