#include <sys/stat.h>
#include <thread>
#include "../include/HikCamera.h"


void HikCamera::imageCallback(unsigned char *data, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser){
    auto target =static_cast<HikCamera*>(pUser);
    std::chrono::steady_clock::time_point now= std::chrono::steady_clock::now();
    long duration = std::chrono::duration_cast<std::chrono::milliseconds>(now-target->mLastTime).count();
    if (duration>1000){
        target->mFPS =(target->mCount);
        target->mLastTime = now;
        target->mCount = 0;
    }
    target->mCount = target->mCount + 1;
    if(pFrameInfo){
        target->img_lock.lock();
        if(target->pixelFormat==BayerRG8 || target->pixelFormat==BayerGR8){
            target->m_time=rclcpp::Clock().now();
            target->mRawImage = cv::Mat(pFrameInfo->nHeight,pFrameInfo->nWidth,CV_8UC1,data);

            auto msg=cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", target->convertToBGR((target->mRawImage))).toCompressedImageMsg();
            msg->header.stamp = rclcpp::Clock().now();
            target->img_pub->publish(*msg);
        }else if(target->pixelFormat==BGR8Packed){
            target->m_time=rclcpp::Clock().now();
            target->mRawImage = cv::Mat(pFrameInfo->nHeight,pFrameInfo->nWidth,CV_8UC3,data);

            auto msg=cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", target->convertToBGR((target->mRawImage))).toCompressedImageMsg();
            msg->header.stamp = rclcpp::Clock().now();
            target->img_pub->publish(*msg);
        } else{
            std::cerr << "\033[35m" << "Please find the cv::cvtColor by yourself , you can use MVS.sh to test the mode" << "\033[0m" << std::endl;
            std::cerr << "ERROR[self]: cannot show img" << std::endl;
            exit(EXIT_FAILURE); // 终止程序执行
        }
        target->img_lock.unlock();
    }
}

void* HikCamera::WorkThread(void* pUser){
    auto target =static_cast<HikCamera*>(pUser);
    int nRet = MV_OK;
    MVCC_STRINGVALUE stStringValue = {0};
    char camSerialNumber[256] = {0};
    nRet = MV_CC_GetStringValue(target->mHandle, "DeviceSerialNumber", &stStringValue);
    if (MV_OK == nRet){
        memcpy(camSerialNumber, stStringValue.chCurValue, sizeof(stStringValue.chCurValue));
    }else{
        printf("Get DeviceUserID Failed! nRet = [%x]\n", nRet);
    }
    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(target->mHandle, "PayloadSize", &stParam);
    if (MV_OK != nRet){
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return NULL;
    }

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
    if (NULL == pData){
        return NULL;
    }
    unsigned int nDataSize = stParam.nCurValue;
    while(1){
        if(target->g_bExit){
            break;
        }

        nRet = MV_CC_GetOneFrameTimeout(target->mHandle, pData, nDataSize, &stImageInfo, 1000);
        // std::cout << "nRet: " << nRet << std::endl;

        if (nRet == MV_OK){
            printf("Cam Serial Number[%s]:GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n",
                camSerialNumber, stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
            target->img_lock.lock();

            if(target->pixelFormat==BayerRG8 || target->pixelFormat==BayerGR8){
                target->mRawImage = cv::Mat(stImageInfo.nHeight,stImageInfo.nWidth,CV_8UC1,pData);

                auto msg=cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", target->convertToBGR((target->mRawImage))).toCompressedImageMsg();
                msg->header.stamp = rclcpp::Clock().now();
                target->img_pub->publish(*msg);
            }else if(target->pixelFormat==BGR8Packed){
                target->mRawImage = cv::Mat(stImageInfo.nHeight,stImageInfo.nWidth,CV_8UC3,pData);

                auto msg=cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", target->convertToBGR((target->mRawImage))).toCompressedImageMsg();
                msg->header.stamp = rclcpp::Clock().now();
                target->img_pub->publish(*msg);
            } else{
                std::cerr << "\033[35m" << "Please find the cv::cvtColor by yourself , you can use MVS.sh to test the mode" << "\033[0m" << std::endl;
                std::cerr << "ERROR[self]: cannot show img" << std::endl;
                exit(EXIT_FAILURE); // 终止程序执行
            }
                target->img_lock.unlock();
        }else{
            printf("cam[%s]:Get One Frame failed![%x]\n", camSerialNumber, nRet);
        }
    }
    return 0;
}

HikCamera::HikCamera(std::string name,rclcpp::Node* node) {
    std::cerr << "\033[35m" << "if Fps > 120+, save img mode will make your fps down !!!!" << "\033[0m" << std::endl;
    this->node=node;
    img_pub=this->node->create_publisher<sensor_msgs::msg::CompressedImage>("/cam/"+name,rclcpp::SensorDataQoS());
}

cv::Mat HikCamera::convertToBGR(const cv::Mat image){
    cv::Mat result;
    if(!image.empty() && pixelFormat==BayerRG8){
        cv::cvtColor(image,result,cv::COLOR_BayerBG2BGR);
    }else if(!image.empty() && pixelFormat==BayerGR8){
        cv::cvtColor(image,result,cv::COLOR_BayerGB2BGR);
    }else if(!image.empty() && pixelFormat==BGR8Packed ){
        return image;
    }else if(!image.empty()){
        std::cerr << "\033[35m" << "Please find the cv::cvtColor by yourself , you can use MVS.sh to test the mode" << "\033[0m" << std::endl;
        std::cerr << "ERROR[self]: cannot show img" << std::endl;
        exit(EXIT_FAILURE); // 终止程序执行
        return result;
    }
    return result;

}

void HikCamera::open(){
    MV_CC_DEVICE_INFO_LIST device_list;
    memset(&device_list, 0, sizeof (MV_CC_DEVICE_INFO_LIST));

    MV_CC_EnumDevices(MV_GIGE_DEVICE|MV_USB_DEVICE,&device_list);

    MV_CC_CreateHandle(&mHandle,device_list.pDeviceInfo[mIndex]);

    MV_CC_OpenDevice(mHandle);

    MV_CC_SetEnumValue(mHandle,"TriggerMode",0);

    MV_CC_RegisterImageCallBackEx(mHandle,imageCallback, this);

//    MV_CC_StartGrabbing(mHandle);

    getDeviceModel();
    // 插值算法
    MV_CC_SetBayerCvtQuality(mHandle, 2);//0：快速 1：均衡 2：最优 3：最优+(linux可能不支持)
}

void HikCamera::open(char g_strSerialNumber[64]){

    int nRet = MV_OK;

    MV_CC_DEVICE_INFO_LIST device_list;
    memset(&device_list, 0, sizeof (MV_CC_DEVICE_INFO_LIST));

    nRet = MV_CC_EnumDevices(MV_USB_DEVICE,&device_list);
    if (MV_OK != nRet){
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    }

    // 根据序列号选择相机
    unsigned int nIndex = -1;
    if (device_list.nDeviceNum > 0){
        for (int i = 0; i < device_list.nDeviceNum; i++){
            MV_CC_DEVICE_INFO* pDeviceInfo = device_list.pDeviceInfo[i];
            if (NULL == pDeviceInfo){
                continue;
            }else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE){
                if (!strcmp((char*)(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber), g_strSerialNumber)){
                    nIndex = i;
                    break;
                }
            }
        }
    }

    if (-1 == nIndex){
        std::cerr << "\033[33m" << "WARRING[self]: please check 序列号, 序列号 maybe is wrong, it will open in normal" << "\033[0m" << std::endl;
        open();
        return;
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&mHandle, device_list.pDeviceInfo[nIndex]);
    if (MV_OK != nRet){
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(mHandle);
    if (MV_OK != nRet){
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
    }

    if(nRet != MV_OK){
        printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
    }

    // 注册异常回调
    // register exception callback
    MV_CC_RegisterImageCallBackEx(mHandle,imageCallback, this);
    if (MV_OK != nRet){
        printf("MV_CC_RegisterExceptionCallBack fail! nRet [%x]\n", nRet);
    }
    printf("connect succeed\n");

//    MV_CC_SetEnumValue(mHandle,"ADCBitDepth",0);

    getDeviceModel();
    // 插值算法
    MV_CC_SetBayerCvtQuality(mHandle, 2);//0：快速 1：均衡 2：最优 3：最优+(linux可能不支持)
}

void HikCamera::open_thread(char g_strSerialNumber[64]){
    int nRet = MV_OK;

    MV_CC_DEVICE_INFO_LIST device_list;
    memset(&device_list, 0, sizeof (MV_CC_DEVICE_INFO_LIST));

    nRet = MV_CC_EnumDevices(MV_USB_DEVICE,&device_list);
    if (MV_OK != nRet){
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    }

    // 根据序列号选择相机
    unsigned int nIndex = -1;
    if (device_list.nDeviceNum > 0){
        for (int i = 0; i < device_list.nDeviceNum; i++){
            MV_CC_DEVICE_INFO* pDeviceInfo = device_list.pDeviceInfo[i];
            if (NULL == pDeviceInfo){
                continue;
            }else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE){
                if (!strcmp((char*)(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber), g_strSerialNumber)){
                    nIndex = i;
                    break;
                }
            }
        }
    }

    if (-1 == nIndex){
        std::cerr << "\033[33m" << "WARRING[self]: please check 序列号, 序列号 maybe is wrong, it will open in normal" << "\033[0m" << std::endl;
        open();
        return;
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&mHandle, device_list.pDeviceInfo[nIndex]);
    if (MV_OK != nRet){
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(mHandle);
    if (MV_OK != nRet){
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
    }

    if(nRet != MV_OK){
        printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
    }

    // 设置触发模式为off
    // set trigger mode as off
    nRet = MV_CC_SetEnumValue(mHandle, "TriggerMode", MV_TRIGGER_MODE_OFF);
    if (MV_OK != nRet){
        printf("Cam[%d]: MV_CC_SetTriggerMode fail! nRet \n", nRet);
    }
    pthread_t nThreadID;
    nRet = pthread_create(&nThreadID, NULL ,WorkThread , this);
    if (nRet != 0){
        printf("Cam[%d]: thread create failed.ret\n",nRet);
    }
//    MV_CC_SetEnumValue(mHandle,"ADCBitDepth",0);
    getDeviceModel();
    // 插值算法
    MV_CC_SetBayerCvtQuality(mHandle, 2);//0：快速 1：均衡 2：最优 3：最优+(linux可能不支持)
}

void HikCamera::startGrabImage(){
    int nRet = MV_OK;
    // 开始取流
    nRet = MV_CC_StartGrabbing(mHandle);
    if (MV_OK != nRet){
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
    }
}

void HikCamera::close(){
    MV_CC_StopGrabbing(mHandle);
    MV_CC_CloseDevice(mHandle);
    MV_CC_DestroyHandle(mHandle);
}

void HikCamera::setExposureTime(float exposure_time){
    MV_CC_SetFloatValue(mHandle,"ExposureTime",exposure_time);
}

void HikCamera::setGain(float value){
    MV_CC_SetFloatValue(mHandle, "Gain", value);
}

void HikCamera::setGamma(float gamma) {
    // 使能 = open
    MV_CC_SetBoolValue(mHandle,"GammaEnable", true);
    MV_CC_SetEnumValue(mHandle,"GammaSelector",1);//1:User 2:sRGB
    MV_CC_SetFloatValue(mHandle, "Gamma", gamma);
}

void HikCamera::setPixelFormat2BGR8() {
    if(this->deviceModel == "MV-CA016-10UC"){
        MV_CC_SetEnumValue(mHandle,"ADCBitDepth",CA_Bits_12);
    }else if(this->deviceModel == "MV-CS016-10UC" || this->deviceModel == "MV-CS050-60UC"){
        MV_CC_SetEnumValue(mHandle,"ADCBitDepth",CS_Bits_12);
    }
    MV_CC_SetEnumValue(mHandle,"PixelFormat",BGR8Packed);
    pixelFormat = BGR8Packed;
}

void HikCamera::setPixelFormat2BayerRG8_12() {
    if(this->deviceModel == "MV-CA016-10UC"){
        MV_CC_SetEnumValue(mHandle,"PixelFormat",BayerRG8);
        pixelFormat = BayerRG8;
        MV_CC_SetEnumValue(mHandle,"ADCBitDepth",CA_Bits_12);
    }else if(this->deviceModel == "MV-CS016-10UC" ){
        MV_CC_SetEnumValue(mHandle,"PixelFormat",BayerRG8);
        pixelFormat = BayerRG8;
        MV_CC_SetEnumValue(mHandle,"ADCBitDepth",CS_Bits_12);
    }else if(this->deviceModel == "MV-CS050-60UC") {
        MV_CC_SetEnumValue(mHandle,"PixelFormat",BGR8Packed);
        pixelFormat = BGR8Packed;
        MV_CC_SetEnumValue(mHandle,"ADCBitDepth",CS_Bits_12);
    }else{
        std::cerr <<  "\033[35m"  << "ERROR[self]： set fail!!!" <<  "\033[0m"  << std::endl;
    }
}

void HikCamera::setPixelFormat2BayerRG8_8() {
    if(this->deviceModel == "MV-CA016-10UC"){
        MV_CC_SetEnumValue(mHandle,"PixelFormat",BayerRG8);
        pixelFormat = BayerRG8;
        MV_CC_SetEnumValue(mHandle,"ADCBitDepth",CA_Bits_8);
    }else if(this->deviceModel == "MV-CS016-10UC" ){
        MV_CC_SetEnumValue(mHandle,"PixelFormat",BayerRG8);
        pixelFormat = BayerRG8;
        MV_CC_SetEnumValue(mHandle,"ADCBitDepth",CS_Bits_8);
    }else if(this->deviceModel == "MV-CS050-60UC") {
        std::cerr << "ERROR[self]： MV-CS050-60UC don't have BayerRG modes" << std::endl;
    }else{
        std::cerr <<  "\033[33m"  << "ERROR[self]： set fail!!!" <<  "\033[0m"  << std::endl;
    }
}


float HikCamera::getFPS(){
    MVCC_FLOATVALUE fps ;
    MV_CC_GetFloatValue(mHandle,"ResultingFrameRate",&fps);
    return fps.fCurValue;
}

void HikCamera::setDeviceReset() {
    MV_CC_SetCommandValue(mHandle, "DeviceReset");
    open();
    startGrabImage();
}

cv::Mat HikCamera::getImage(){
    cv::Mat img;
    img_lock.lock();
    img = convertToBGR(mRawImage);
    img_lock.unlock();
    return img;
}

rclcpp::Time HikCamera::getTime() {
    rclcpp::Time return_time;
    img_lock.lock();
    return_time = m_time;
    img_lock.unlock();
    return return_time;
}

void HikCamera::getDeviceModel() {
    MVCC_STRINGVALUE stStringValue = {0};
    MV_CC_GetStringValue(mHandle,"DeviceModelName",&stStringValue);
    std::cout <<  "stStringValue.chCurValue " << stStringValue.chCurValue << std::endl;
    this->deviceModel = stStringValue.chCurValue;
    if(  (this->deviceModel  == "MV-CA016-10UC" )||  (this->deviceModel == "MV-CS016-10UC")){
        this->pixelFormat = BayerRG8;
    }else if( this->deviceModel  == "MV-CS050-60UC"){
        this->pixelFormat = BayerGR8;
    }else{
        std::cerr <<  "\033[35m"  << "WARRING[self]：pixelFormat is default" <<  "\033[0m"  << std::endl;
    }
}