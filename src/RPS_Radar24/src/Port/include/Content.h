//
// Created by plusseven on 24-4-26.
//

#ifndef RADAR24_ROS_CONTENT_H
#define RADAR24_ROS_CONTENT_H

#include <cstring>
#include "CRC_Check.h"




template<typename DATA> class Content {
private:
    unsigned char seq = 0;
    int fd;
    uint16_t data_length;
    uint16_t_uchar cmd_id;
    FRAME_HEADER frameHeader;
public:
    Content(int fd, uint16_t data_length, uint16_t cmd_id);
    Content() = default;
    void OutputData(DATA mapRobotData);
    void InputData();

};


template<typename DATA>
Content<DATA>::Content(int fd, uint16_t data_length, uint16_t cmd_id) {
    this->fd = fd;
    this->data_length = data_length;
    this->cmd_id = {cmd_id};
    this->frameHeader = FRAME_HEADER {SOF,this->data_length,this->seq};
    Append_CRC8_Check_Sum(frameHeader.u_char8, 5);  //帧头 CRC8 校验

}


template<typename DATA>
void Content<DATA>::OutputData(DATA frame_cmd_data){
    int size = FRAME_HEADER_LEN + CMD_LEN + sizeof(frame_cmd_data) + CRC16_LEN;
    int ptr=0;

//    unsigned char outputData[size];
    unsigned char outputData[size];

    // 帧头frameHeader
    memcpy(outputData+ptr,frameHeader.u_char8,FRAME_HEADER_LEN);
    ptr += sizeof(frameHeader);

    //命令码cmd_id
    memcpy(outputData+ptr,cmd_id.u_char8,CMD_LEN);
    ptr += 2;

    // data(数据帧)
    memcpy(outputData+ptr,frame_cmd_data.u_char8,sizeof(frame_cmd_data.u_char8));
    Append_CRC16_Check_Sum(outputData, size);
    int write_stauts = write(fd, outputData, size);
    std::cout << "write_stauts: " << write_stauts << std::endl;

}

//template<typename DATA>
//void Content<DATA>::InputData()
//{
//    unsigned char buff[5000] = {0};
//    int inputSize = read(fd, buff, 5000);
//    int ptr=0;
//    while(ptr < inputSize)
//    {
//        if(buff[ptr] == SOF)
//        {
//            FRAME_HEADER temp_frameHeader;
//            memcpy(temp_frameHeader.u_char8,buff+ptr,FRAME_HEADER_LEN);
//            int size = FRAME_HEADER_LEN + CMD_LEN + temp_frameHeader.data.data_length + CRC16_LEN;
//            if(Verify_CRC8_Check_Sum(temp_frameHeader.u_char8, FRAME_HEADER_LEN)
//                && Verify_CRC16_Check_Sum(&buff[ptr], size))
//            {
//                ptr += FRAME_HEADER_LEN;
//                uint16_t_uchar temp_cmd_id;
//                memcpy(temp_cmd_id.u_char8, buff + ptr, CMD_LEN);
//                ptr += CMD_LEN;
//                switch (temp_cmd_id.data) {
//                    case CMD_RADAR_MARK_DATA_T:
//                    {
////                        memcpy(RADAR_MARK_DATA_T.u_char8, buff + ptr, temp_frameHeader.data.data_length);
//                        ptr += temp_frameHeader.data.data_length;
//
//                    }break;
//                    case CMD_RADAR_INFO_T:
//                    {
//
//                    }break;
//
//                }
//            }
//        }
//        ptr++;
//
//
//    }
//
//
//    std::cout << "inputSize: " << inputSize << std::endl;
//}
//

#endif //RADAR24_ROS_CONTENT_H
