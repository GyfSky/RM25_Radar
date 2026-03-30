#include "../include/Radar.h"

Modes::Modes(){
    //choose  一些选项
/////////////////////////////////////////////////////////////////////////////////////////////////////////
////------------------------------------- 这里需要修改！！！！---------------------------------------------|
//    application   = Application::Radar;
//    pictureSource = PictureSource::single_picture ;    //图片来源          |
//    isOpenMid70   = TF::true_;enemys
//    isUseMid70    = TF::false_;
//    detectionMode = Detection::netDetection;
//    ourPattern    = OurPattern::red;                  //己方颜色       |
//    Port_isOpen   = TF::false_;                       //串口的开启与否  |q
//    usePort       = UsePort::USB0;                    //所使用的串口    |
//    isSave        = TF::true_;                       //是否保存图片    |

    application   = Application::Radar;
    pictureSource = PictureSource::camera_;    //图片来源          |
    // pictureSource = PictureSource::ros;
    isOpenMid70   = TF::false_;
    isUseMid70    = TF::false_;
    detectionMode = Detection::netDetection;
    ourPattern    = OurPattern::red;                  //己方颜色       |
    Port_isOpen   = TF::false_  ;                       //串口的开启与否  |
    usePort       = UsePort::USB0;                    //所使用的串口    |
    isSave        = TF::false_;                       //是否保存图片    |
    application   = Application::Radar;//？？
    camNumber     = 2;

    // application   = Application::Radar;
    // pictureSource = PictureSource::camera_;    //图片来源          |
    // // pictureSource = PictureSource::ros;
    // isOpenMid70   = TF::false_;
    // isUseMid70    = TF::false_;
    // detectionMode = Detection::netDetection;
    // ourPattern    = OurPattern::blue;                  //己方颜色       |
    // Port_isOpen   = TF::false_  ;                       //串口的开启与否  |
    // usePort       = UsePort::USB0;                    //所使用的串口    |
    // isSave        = TF::false_;                       //是否保存图片    |
    // application   = Application::Radar;//？？
    // camNumber     = 2;
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("camera_detector");

    MyRadar radar(nh,true);
    radar.calib();

    while(true){
        radar.MainCam_Image_ptr->draw_line_calib(radar.MainMapGraph_ptr->vexs);
        radar.SecCam_Image_ptr->draw_line_calib(radar.SecMapGraph_ptr->vexs);
        if(cv::waitKey(1) == 'q'){
            radar.is_close = true;
        }
        if(radar.is_close){
            break;
        }
    }
    radar.Close();
    return 0;
}