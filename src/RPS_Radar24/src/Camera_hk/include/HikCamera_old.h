#pragma once
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include<chrono>
namespace Camera_hk
{
    class HikCamera {
    private:
        unsigned int mIndex=0;
        void* mHandle;

    public:
        int mCount;
        float mFPS;
        std::chrono::steady_clock::time_point mLastTime;
        cv::Mat mRawImage;

    private:

    public:
        cv::Mat convertToBGR(cv::Mat image);
        void init();
        void open();
        void open(char g_strSerialNumber[64]);
        void close();
        /**
         * @details 单位us
         * @param exposure_time
         */
        void setExposureTime(float exposure_time);
        /**
         * @param channel 0 Red 1 Green 2 Blue
         * @param ratio
         */
        void setWhiteBalance(int channel,float ratio);

        /**
         *
         * @param value 0~16
         */
        void setGain(float value);
        void setGamma(float gamma);
        void setDeviceReset();
        void setFps(float fps);
        float getFPS();
//        void getResultingFrameRate();
        cv::Mat getImage();
    };

}


