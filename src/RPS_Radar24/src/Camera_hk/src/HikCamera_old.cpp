#include "../include/HikCamera_old.h"
#include "../include/HikCamera.h"


namespace Camera_hk
{
    void imageCallback(unsigned char *data, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser)
    {
        auto target =static_cast<HikCamera*>(pUser);
        std::chrono::steady_clock::time_point now= std::chrono::steady_clock::now();
        long duration = std::chrono::duration_cast<std::chrono::milliseconds>(now-target->mLastTime).count();
        if (duration>1000)
        {
            target->mFPS =(target->mCount);
            target->mLastTime = now;
            target->mCount = 0;
        }
        target->mCount = target->mCount + 1;
        if(pFrameInfo)
        {
            target->mRawImage = cv::Mat(pFrameInfo->nHeight,pFrameInfo->nWidth,CV_8UC1,data);
        }

    }
}
cv::Mat Camera_hk::HikCamera::convertToBGR(const cv::Mat image)
{
    cv::Mat result;
    if(!image.empty())
        cv::cvtColor(image,result,cv::COLOR_BayerBG2BGR);
    return result;
}



void Camera_hk::HikCamera::open()
{
    MV_CC_DEVICE_INFO_LIST device_list;
    memset(&device_list, 0, sizeof (MV_CC_DEVICE_INFO_LIST));

    MV_CC_EnumDevices(MV_GIGE_DEVICE|MV_USB_DEVICE,&device_list);

    MV_CC_CreateHandle(&mHandle,device_list.pDeviceInfo[mIndex]);

    MV_CC_OpenDevice(mHandle);

    MV_CC_SetEnumValue(mHandle,"TriggerMode",0);

    MV_CC_RegisterImageCallBackEx(mHandle,imageCallback, this);

    MV_CC_StartGrabbing(mHandle);
}


void Camera_hk::HikCamera::open(char  g_strSerialNumber[64])
{
    int nRet = MV_OK;

    MV_CC_DEVICE_INFO_LIST device_list;
//    memset(&device_list, 0, sizeof (MV_CC_DEVICE_INFO_LIST));


    nRet = MV_CC_EnumDevices(MV_USB_DEVICE,&device_list);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    }

    // 根据序列号选择相机
    unsigned int nIndex = -1;
    if (device_list.nDeviceNum > 0)
    {
        for (int i = 0; i < device_list.nDeviceNum; i++)
        {
            MV_CC_DEVICE_INFO* pDeviceInfo = device_list.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                continue;
            }
            else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
            {
                if (!strcmp((char*)(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber), g_strSerialNumber))
                {
                    nIndex = i;
                    break;
                }
            }
        }
    }

    if (-1 == nIndex)
    {
        printf("here are error in 序列号");
        return;

    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&mHandle, device_list.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(mHandle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
    }

    // 有的特殊语句需要在打开相机后，取图片前进行设置。
    this->init();

    nRet = MV_CC_SetEnumValue(mHandle,"TriggerMode",0);
    if(nRet != MV_OK)
    {
        printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
    }


    // 注册异常回调
    // register exception callback
    MV_CC_RegisterImageCallBackEx(mHandle,imageCallback, this);
    if (MV_OK != nRet)
    {
        printf("MV_CC_RegisterExceptionCallBack fail! nRet [%x]\n", nRet);
    }
    printf("connect succeed\n");

    // 开始取流
    // start grab image
    MV_CC_StartGrabbing(mHandle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
    }


}


void Camera_hk::HikCamera::close()
{
    MV_CC_StopGrabbing(mHandle);
    MV_CC_CloseDevice(mHandle);
    MV_CC_DestroyHandle(mHandle);
}

//fmt

void Camera_hk::HikCamera::setExposureTime(float exposure_time)
{
    MV_CC_SetFloatValue(mHandle,"ExposureTime",exposure_time);
}


void Camera_hk::HikCamera::setWhiteBalance(int channel, float ratio)
{
    switch (channel)
    {
        case 0:
            MV_CC_SetEnumValue(mHandle,"BalanceRatioSelector",0);
            MV_CC_SetFloatValue(mHandle,"BalanceRatio",ratio);
            break;
        case 1:
            MV_CC_SetEnumValue(mHandle,"BalanceRatioSelector",1);
            MV_CC_SetFloatValue(mHandle,"BalanceRatio",ratio);
            break;
        case 2:
            MV_CC_SetEnumValue(mHandle,"BalanceRatioSelector",2);
            MV_CC_SetFloatValue(mHandle,"BalanceRatio",ratio);
            break;
        default:
            break;
    }
}

/**
 * @brief 相机“固定”采集频率
 * **/
void Camera_hk::HikCamera::setFps(float fps) {
    MV_CC_SetBoolValue(mHandle,"AcquisitionFrameRateEnable", true);
    MV_CC_SetFloatValue(mHandle,"AcquisitionFrameRate", fps);
}


void Camera_hk::HikCamera::setGamma(float gamma) {
    // 使能 = open
//    MV_CC_SetBoolValue(mHandle,"SuperBayerEnable", true);
    MV_CC_SetBoolValue(mHandle,"GammaEnable", true);
    MV_CC_SetEnumValue(mHandle,"GammaSelector",1);//1:User 2:sRGB
    MV_CC_SetFloatValue(mHandle, "Gamma", gamma);
}

//void Camera_hk::HikCamera::getResultingFrameRate(){
//    MVCC_FLOATVALUE stResultingFrameRate = {0};
//    MV_CC_GetFloatValue(mHandle, "ResultingFrameRate ", &stResultingFrameRate);
//    printf("exposure time current value:%f\n", stResultingFrameRate.fCurValue);
//}


void Camera_hk::HikCamera::setGain(float value)
{
    MV_CC_SetFloatValue(mHandle, "Gain", value);
}


float Camera_hk::HikCamera::getFPS()
{
    return mFPS;
}

void Camera_hk::HikCamera::setDeviceReset() {
    MV_CC_SetCommandValue(mHandle, "DeviceReset");
}

cv::Mat Camera_hk::HikCamera::getImage()
{
    return convertToBGR(mRawImage);
}

void Camera_hk::HikCamera::init() {
    MV_CC_SetBoolValue(mHandle,"SuperBayerEnable", true);
}
