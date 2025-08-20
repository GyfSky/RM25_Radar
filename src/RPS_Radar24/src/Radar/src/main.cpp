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
    // application   = Application::Radar;
    // pictureSource = PictureSource::camera_ ;    //图片来源          |
    // // pictureSource = PictureSource::ros;
    // isOpenMid70   = TF::false_;
    // isUseMid70    = TF::false_;
    // detectionMode = Detection::netDetection;
    // ourPattern    = OurPattern::blue;                  //己方颜色       |
    // Port_isOpen   = TF::true_;                       //串口的开启与否  |
    // usePort       = UsePort::USB0;                    //所使用的串口    |
    // isSave        = TF::true_;                       //是否保存图片    |
    // camNumber     = 2;


};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("camera_detector");

    MyRadar radar(nh);
    if (radar.getPictureSource()==ros) {
        radar.initLock.lock();
        while(!radar.is_init){
            rclcpp::spin_some(nh);
            if(radar.flag1){
                if (!radar.is_one_cam&&!radar.flag2) continue;
                radar.Init();
            }
        }
        radar.initLock.unlock();
    }else if(radar.getPictureSource()==camera_||radar.getPictureSource()==video
        ||radar.getPictureSource()==single_picture||radar.getPictureSource()==picture_dir){
        radar.initLock.lock();
        while(!radar.is_init){
            radar.Init();
        }
        radar.initLock.unlock();
    }
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_ ;
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(nh);
    executor_->spin();
    executor_->remove_node(nh);
    radar.Close();
    rclcpp::shutdown();
    return 0;
}