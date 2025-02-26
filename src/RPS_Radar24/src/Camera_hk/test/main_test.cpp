//
// Created by lightning on 2022/1/8.
//

#include "../include/HikCamera.h"
#include <opencv2/opencv.hpp>
#include <thread>

using namespace Camera_hk;

int main()
{
    //00F26632053
    //00F93284600
//    HikCamera camera1;
//    camera1.open("00F26632053");
//    camera1.setExposureTime(8000);
//    camera1.setGai    n(6);
int flag = -1;
    std::string save_root_dir = "/home/thesky/camera/KESU/img_DATA/test_dir";
    HikCamera camera(save_root_dir, 10, true);
    HikCamera camera2(save_root_dir, 10, true);
//    HikCamera camera;
//    camera.open("DA2024625");

    camera.setSaveMode();
    camera2.setSaveMode();
//    camera.open("00F26632053");
//    camera.open();
//    camera.open("DA3113969");
    camera2.open_thread("DA1521302");
    camera.open_thread("00F26632053");
//    camera.open("DA1521302");
//    camera.setPixelFormat2BayerRG8_8();

    camera.setExposureTime(10000);
    camera2.setExposureTime(10000);
//    camera.setPixelFormat2BayerRG8_12();
//    camera.setPixelFormat2BGR8();


    camera.setGain(8);
    camera.setGamma(1.0);
    camera.setFps(40);
    camera2.setGain(8);
    camera2.setGamma(1.0);
    camera2.setFps(40);
//    camera.setWhiteBalance();

    camera.startGrabImage();
    std::this_thread::sleep_for(std::chrono::milliseconds (2));
    camera2.startGrabImage();


    std::string writePath = "/home/thesky/图片/";
    std::string  name ;
    int i = 57;
    long time, sumTime, startTime, endTime;
    while (true)
    {
        startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        char ch = cv::waitKey(1);
        if(ch=='q') break;

        cv::Mat image = camera.getImage();
        cv::Mat image2 = camera2.getImage();

//        if(!image.empty())
        if(!image.empty() && !image2.empty())
        {
            flag = 1;
            sumTime=0;
            cv::imshow("result1",image);
            cv::imshow("result2",image2);
//            cv::imshow("result2",image_);
            if(32 == cv::waitKey(20)) {
                name = writePath + std::to_string(i)+".jpg";
                cv::imwrite(name,image.clone());
                cv::imwrite(name,image2.clone());
                std::cout << name <<std::endl;
                i++;
            }

//            std::this_thread::sleep_for(std::chrono::milliseconds (200));
            std::cout<< "camera.getFPS():  " << camera.getFPS()<<' '<< std::endl;
            std::cout<< "camera2.getFPS():  " << camera2.getFPS()<<' '<< std::endl;

        }


        endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        time = endTime-startTime;
//        std::cout << "Fps: " << 1.0/time*1000.0 << std::endl;
        if(flag ==-1){
            sumTime+=time;
//            std::cout<<"Empty   "<<camera.getFPS()<<' '<<camera.mCount<<std::endl;
        }else if(flag == 2){
            sumTime+=time;
            std::cout<<"Empty   "<<camera.getFPS()<<' '<<camera.mCount<<std::endl;
        }
//        if((flag == 1 && image.empty()) || (flag ==-1 && sumTime > 0.5) || (flag ==2 && sumTime > 5)){
//            camera.setDeviceReset();
//            std::cout << "DeviceReset ing" << std::endl;
//            sumTime = 0;
//            flag = 2;
//        }
//        camera.getResultingFrameRate();
    };
    camera.close();
    camera2.close();
}

