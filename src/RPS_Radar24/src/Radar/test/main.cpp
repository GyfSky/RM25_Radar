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
//    saveImagePath = SaveImagePath::disk02;            //保存路径       |z

    application   = Application::Radar;
    pictureSource = PictureSource::picture_dir ;    //图片来源          |
    isOpenMid70   = TF::false_;
    isUseMid70    = TF::false_;
    detectionMode = Detection::netDetection;
    ourPattern    = OurPattern::blue;                  //己方颜色       |
    Port_isOpen   = TF::false_  ;                       //串口的开启与否  |
    usePort       = UsePort::USB0;                    //所使用的串口    |
    isSave        = TF::false_;                       //是否保存图片    |


};


int main(int argc, char **argv){
    // MyRadar radar;
    // radar.Init(argc, argv);
    // while(true){
    //     radar.Spin(argc, argv);
    //     if(cv::waitKey(1) == 'q'){
    //         radar.is_close = true;
    //     }
    //     if(radar.is_close){
    //         break;
    //     }
    // }
    // radar.Close();
    return 0;
}