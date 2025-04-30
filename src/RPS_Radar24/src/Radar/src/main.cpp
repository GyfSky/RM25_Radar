#include "../include/Radar.h"
#include <cv_bridge/cv_bridge.h>

//modes构造函数
Modes::Modes(){
    //choose  一些选项
/////////////////////////////////////////////////////////////////////////////////////////////////////////
////------------------------------------- 这里需要修改！！！！---------------------------------------------|
    application   = Application::Radar;//？？
    pictureSource = PictureSource::camera_ ;   //图片来源          |
    // pictureSource = PictureSource::ros;
    isOpenMid70   = TF::false_;
    isUseMid70    = TF::false_;
    detectionMode = Detection::netDetection;//？？
    ourPattern    = OurPattern::red;                 //己方颜色       |
    Port_isOpen   = TF::true_;                       //串口的开启与否  |
    usePort       = UsePort::USB0;                    //所使用的串口    |
    isSave        = TF::true_;                       //是否保存图片    |
    camNumber     = 2;
//    saveImagePath = SaveImagePath::disk02;            //保存路径       |z

//compititon
//    application   = Application::Radar;
//    pictureSource = PictureSource::camera_ ;    //图片来源          |
//    isOpenMid70   = TF::false_;
//    isUseMid70    = TF::false_;
//    detectionMode = Detection::netDetection;
//    ourPattern    = OurPattern::blue;                  //己方颜色       |
//    Port_isOpen   = TF::true_  ;                       //串口的开启与否  |
//    usePort       = UsePort::USB0;                    //所使用的串口    |
//    isSave        = TF::true_;                       //是否保存图片    |
//    camNumber     = 2;


};

bool flag1 = false,flag2=false;
cv::Mat img1,img2;
rclcpp::Time ros_time;

void getImg1(const sensor_msgs::msg::CompressedImage::ConstPtr &rosImg_ptr){
    // ros_time=rclcpp::Clock().now();
    ros_time=rosImg_ptr->header.stamp;
    img1 = cv::imdecode(rosImg_ptr->data, cv::IMREAD_COLOR);
    if(!img1.empty()){
        flag1=true;
    }else{
        std::cout << "error!!!!!" << std::endl;
    }
}
void getImg2(const sensor_msgs::msg::CompressedImage::ConstPtr &rosImg_ptr){
    img2 = cv::imdecode(rosImg_ptr->data, cv::IMREAD_COLOR);
    if(!img2.empty()){
        flag2=true;
    }else{
        std::cout << "error!!!!!" << std::endl;
    }
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("camera_detector");
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_main_img;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_sec_img;
    rclcpp::Subscription<interfaces::msg::NetDetect>::SharedPtr sub_car;
    rclcpp::Subscription<interfaces::msg::NetDetect>::SharedPtr sub_armor;
    rclcpp::Subscription<interfaces::msg::DetectResult>::SharedPtr sub_lidar;
    sub_main_img = nh->create_subscription<sensor_msgs::msg::CompressedImage>("/cam/Hik60", rclcpp::SensorDataQoS(), &getImg1);
    sub_sec_img = nh->create_subscription<sensor_msgs::msg::CompressedImage>("/cam/Hik30", rclcpp::SensorDataQoS(), &getImg2);
    MyRadar radar(nh.get());
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
    radar.res_pub=nh->create_publisher<interfaces::msg::DetectRes>("/cam_result", 10);

    if (radar.getPictureSource()==ros) {
        while(rclcpp::ok()){
            auto now_time = std::chrono::steady_clock::now();
            rclcpp::spin_some(nh);
            if(flag1){
                if (!radar.is_one_cam&&!flag2) continue;
                radar.MainCam_Image_ptr->Cam_img= img1;
                if (!radar.is_one_cam)
                    radar.SecCam_Image_ptr->Cam_img= img2;
                radar.time_now=ros_time;
                radar.Init(argc, argv);
                radar.Spin(argc, argv);
                if(cv::waitKey(1) == 'q'){
                    radar.is_close = true;
                }
                if(radar.is_close){
                    break;
                }
            }
            auto end_time = std::chrono::steady_clock::now();
            float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
            std::cout<<"\033[31m"<<"time is : "<<dur_time/1000<<" s"<<"\033[0m"<<std::endl;
        }
    }else if(radar.getPictureSource()==camera_||radar.getPictureSource()==video){
        while(rclcpp::ok()){
            auto now_time = std::chrono::steady_clock::now();
            radar.Init(argc, argv);
            radar.Spin(argc, argv);
            if(cv::waitKey(1) == 'q'){
                radar.is_close = true;
            }
            if(radar.is_close){
                break;
            }
            auto end_time = std::chrono::steady_clock::now();
            float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
            std::cout<<"\033[31m"<<"time is : "<<dur_time/1000<<" s"<<"\033[0m"<<std::endl;
        }
    }

    radar.Close();
    rclcpp::shutdown();
    return 0;
}