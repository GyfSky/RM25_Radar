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
#endif //RADAR24_ROS_CONTENT_H
