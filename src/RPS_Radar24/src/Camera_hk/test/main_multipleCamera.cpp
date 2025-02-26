//
// Created by plusseven on 24-7-7.
//

#include "../include/MultipleCamera.h"

bool g_bExit = false;


// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
    int c;
    while ( (c = getchar()) != '\n' && c != EOF );
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
    g_bExit = true;
    sleep(1);
}
bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) // USB相机
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

void* WorkThread(void* pUser)
{
    int nRet = MV_OK;
    MVCC_STRINGVALUE stStringValue = {0};
    char camSerialNumber[256] = {0};
    nRet = MV_CC_GetStringValue(pUser, "DeviceSerialNumber", &stStringValue);
    if (MV_OK == nRet)
    {
        memcpy(camSerialNumber, stStringValue.chCurValue, sizeof(stStringValue.chCurValue));
    }
    else
    {
        printf("Get DeviceUserID Failed! nRet = [%x]\n", nRet);
    }
    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return NULL;
    }
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
    if (NULL == pData)
    {
        return NULL;
    }
    unsigned int nDataSize = stParam.nCurValue;
    while(1)
    {
        if(g_bExit)
        {
            break;
        }

        nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
        if (nRet == MV_OK)
        {
            printf("Cam Serial Number[%s]:GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n",
                   camSerialNumber, stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
            cv::Mat img = cv::Mat(stImageInfo.nHeight,stImageInfo.nWidth,CV_8UC1,pData);
            cv::cvtColor(img,img,cv::COLOR_BayerBG2BGR);
//            cv::imshow("test", img);
        }
        else
        {
            printf("cam[%s]:Get One Frame failed![%x]\n", camSerialNumber, nRet);
        }
    }
    return 0;
}

int main()
{
    int nRet = MV_OK;
    void* handle[CAMERA_NUM] = {NULL};
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        return -1;
    }
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            }
            PrintDeviceInfo(pDeviceInfo);
        }
    }
    else
    {
        printf("Find No Devices!\n");
        return -1;
    }
    if(stDeviceList.nDeviceNum < CAMERA_NUM)
    {
        printf("only have %d camera\n", stDeviceList.nDeviceNum);
        return -1;
    }

    // 提示为多相机测试
    // Tips for multicamera testing
    printf("Start %d camera Grabbing Image test\n", CAMERA_NUM);
    for(int i = 0; i < CAMERA_NUM; i++)
    {
        // 创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle[i], stDeviceList.pDeviceInfo[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            MV_CC_DestroyHandle(handle[i]);
            return -1;
        }
        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            MV_CC_DestroyHandle(handle[i]);
            return -1;
        }

    }
    for(int i = 0; i < CAMERA_NUM; i++)
    {
        // 设置触发模式为off
        // set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle[i], "TriggerMode", MV_TRIGGER_MODE_OFF);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: MV_CC_SetTriggerMode fail! nRet [%x]\n", i, nRet);
        }
        pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL ,WorkThread , handle[i]);
        if (nRet != 0)
        {
            printf("Cam[%d]: thread create failed.ret = %d\n",i, nRet);
            return -1;
        }
        cv::waitKey(50);

        // 开始取流
        // start grab image
        nRet = MV_CC_StartGrabbing(handle[i]);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: MV_CC_StartGrabbing fail! nRet [%x]\n",i, nRet);
            return -1;
        }
    }
    PressEnterToExit();
    for(int i = 0; i < CAMERA_NUM; i++)
    {
        // 停止取流
        // end grab image
        nRet = MV_CC_StopGrabbing(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            return -1;
        }
        // 关闭设备
        // close device
        nRet = MV_CC_CloseDevice(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            return -1;
        }
        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            return -1;
        }
    }
    printf("exit\n");
    return 0;
}