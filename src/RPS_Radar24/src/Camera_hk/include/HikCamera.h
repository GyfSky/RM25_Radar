#ifndef SRC_RPS_RADAR24_SRC_CAMERA_INCLUDE_HK_CAMERA_H
#define SRC_RPS_RADAR24_SRC_CAMERA_INCLUDE_HK_CAMERA_H


#pragma once
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "MvCameraControl.h"
#include<chrono>

/***
 * @brief 像素数据的格式
 */
typedef enum {
    // general
    Mono8 = 0x01080001,
    Mono10 = 0x01100003,
    Mono12 = 0x01100005,
    RGB8Packed = 0x02180014,            // RGB 8
    BGR8Packed = 0x02180015,            // BGR 8
    YUV422_YUYV_Packed = 0x02100032,    // YUV 422 (YUYV) Packed
    YUV422Packed = 0x0210001F,

    // 016
    BayerRG8 = 0x01080009,
    BayerRG10 = 0x0110000d,
    BayerRG10Packed = 0x010C0027,
    BayerRG12 = 0x01100011,
    BayerRG12Packed = 0x010C002B,

    // CS050
    BayerGR8 = 0x01080008,
    BayerGR10 = 0x0110000c,
    BayerGR10Packed = 0x010C0026,
    BayerGR12 = 0x01100010,
    BayerGR12Packed = 0x010C002A
}PixelFormat;

/***
 * @brief ADCBit Depth: The depth of the ADC.
 */
typedef enum {
    CA_Bits_8  = 2,
    CA_Bits_12 = 0,
    CS_Bits_8  = 0,
    CS_Bits_12 = 3
}ADCBitDepth;




namespace Camera_hk
{
    class HikCamera {
    private:
        std::string deviceModel;
        unsigned int mIndex=0;
    public:
        int pic_num = -1;
        int num_frame;
        std::string save_dir;
        int mCount;
        float mFPS;
        bool is_always_save = false;
        bool is_will_always_save = false;
        PixelFormat pixelFormat = BayerRG8;  // default=BayerRG8
        std::chrono::steady_clock::time_point mLastTime;
        cv::Mat mRawImage;
        void* mHandle;

        bool g_bExit = false;
        std::mutex img_lock;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr img_pub;
        rclcpp::Node* node;
        rclcpp::Time m_time;
    private:

    public:
        HikCamera();

        /*
         * @param num_frame 每隔num_frame张图片取一张图片
         * */
        HikCamera(std::string name,std::string save_root_dir,rclcpp::Node* node,int num_frame = 1,bool is_always_save = true);
        cv::Mat convertToBGR(cv::Mat image);
        void open();
        void open(char g_strSerialNumber[64]);
        void open_thread(char g_strSerialNumber[64]);
        void startGrabImage();
        void getDeviceModel();

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
        void setFps(float fps);
        void setGamma(float gamma);
        void setPixelFormat2BGR8();
        void setPixelFormat2BayerRG8_12();
        void setPixelFormat2BayerRG8_8();
        void setDeviceReset();
        float getFPS();
        void getResultingFrameRate();
        cv::Mat getImage();
        rclcpp::Time getTime();
        static void imageCallback(unsigned char *data, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);
        static void* WorkThread(void* pUser);
        void setSaveMode();
    };

}


#endif //SRC_RPS_RADAR24_SRC_CAMERA_INCLUDE_HK_CAMERA_H
