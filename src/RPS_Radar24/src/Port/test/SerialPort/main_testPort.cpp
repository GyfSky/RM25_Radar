//
// Created by plusseven on 24-4-26.
//

#include <thread>
#include "../../include/serialport.h"

int main(){
    char *port_path1 = "/dev/ttyUSB0";
    char *port_path2 = "/dev/ttyUSB2";
//    char *port_path = "/dev/ttyUSB1";

//    uint16_t cmd_id = 0x0305;
    uint16_t data_length = 10;
    uint16_t cmd_id = 0x0305;
//    uint16_t_uchar data_length{6};

    SerialPort serialPort1;
    serialPort1.initSerialPort(port_path1);
    Content<MAP_ROBOT_DATA_T> portContent(serialPort1.fd,data_length,cmd_id);
    Content<RADAR_SENF_TO_PLANE_DATA_T> portContentq(serialPort1.fd,17,CLIENT_GRAPHIC_DRAW_ID);
    Content<RADAR_DRAW_CAHR_DATA_T> portContent2(serialPort1.fd,51,CLIENT_GRAPHIC_DRAW_ID);
    Content<RADAR_DECISION_DATA_T> port(serialPort1.fd,7,CLIENT_GRAPHIC_DRAW_ID);
    RADAR_DECISION_DATA_T radarDecisionDataT;
    radarDecisionDataT.data.data_cmd_id = INTERACTION_RADAR_DECISION;
    radarDecisionDataT.data.sender_id = 109;
    radarDecisionDataT.data.receiver_id = 0x8080;

    radarDecisionDataT.data.user_data[0] = 0;

    std::cout << " sizeof(interaction_figure_t) : "<<  sizeof(interaction_figure_t) <<std::endl;

//    SerialPort serialPort2;
//    serialPort2.initSerialPort(port_path2);
//    Content<MAP_ROBOT_DATA_T> get(serialPort2.fd,data_length,cmd_id);


    uint16_t target_id;float x,y;
    target_id = 4;x = 0.;y = 0.;

    MAP_ROBOT_DATA_T mapRobotDataT1{{target_id, x, y}};
    RADAR_SENF_TO_PLANE_DATA_T radarSenfToPlaneDataT;
    RADAR_DRAW_CAHR_DATA_T radarDrawCahrDataT;

    radarSenfToPlaneDataT.data.data_cmd_id = 0x02FF;
//    radarSenfToPlaneDataT.data.sender_id   = 109;
//    radarSenfToPlaneDataT.data.receiver_id = 106;

    radarSenfToPlaneDataT.data.sender_id   = 109;
    radarSenfToPlaneDataT.data.receiver_id = 106;

    uint8_t fly = 1;

    radarSenfToPlaneDataT.data.char_data[0] = fly;
    radarSenfToPlaneDataT.data.char_data[1] = 2;
    radarSenfToPlaneDataT.data.char_data[2] = 9;

    radarSenfToPlaneDataT.data.char_data[3] = fly;
    radarSenfToPlaneDataT.data.char_data[4] = 2;
    radarSenfToPlaneDataT.data.char_data[5] = 9;

    radarSenfToPlaneDataT.data.char_data[6] = fly;
    radarSenfToPlaneDataT.data.char_data[7] = 2;
    radarSenfToPlaneDataT.data.char_data[8] = 9;

    radarSenfToPlaneDataT.data.char_data[9] = fly;
    radarSenfToPlaneDataT.data.char_data[10] = 2;



//    radarDrawCahrDataT.data.data_cmd_id = 0x02FF;  //TODO:
//    radarDrawCahrDataT.data.data_cmd_id = 0x0110;  //TODO:
//    radarDrawCahrDataT.data.sender_id   = 9;
//    radarDrawCahrDataT.data.receiver_id = 0x0106;
//
////    InteractionFigureUnion first_char;
//    radarDrawCahrDataT.data.interactionFigure.figure_name[0] = 'F';
//    radarDrawCahrDataT.data.interactionFigure.figure_name[1] = 'L';
//    radarDrawCahrDataT.data.interactionFigure.figure_name[2] = 'Y';
//
//    radarDrawCahrDataT.data.interactionFigure.operate_tpye = 0;
//    radarDrawCahrDataT.data.interactionFigure.figure_tpye  = 7;
//    radarDrawCahrDataT.data.interactionFigure.layer        = 9;
//    radarDrawCahrDataT.data.interactionFigure.color        = 3;
//    radarDrawCahrDataT.data.interactionFigure.details_a    = 45;
//    radarDrawCahrDataT.data.interactionFigure.details_b    = 5;
//    radarDrawCahrDataT.data.interactionFigure.width        = 5;
//    radarDrawCahrDataT.data.interactionFigure.start_x      = 840;
//    radarDrawCahrDataT.data.interactionFigure.start_y      = 970;
//
//    radarDrawCahrDataT.data.char_data[0]                   = 'F';
//    radarDrawCahrDataT.data.char_data[1]                   = 'L';
//    radarDrawCahrDataT.data.char_data[2]                   = 'Y';
//    radarDrawCahrDataT.data.char_data[3]                   = '!';
//    radarDrawCahrDataT.data.char_data[4]                   = '!';


//    std::this_thread::sleep_for(std::chrono::milliseconds (10000));

    tcflush(serialPort1.fd, TCIFLUSH);
    while(true){
//        std::this_thread::sleep_for(std::chrono::milliseconds (2));
        portContent.OutputData(mapRobotDataT1);
        std::this_thread::sleep_for(std::chrono::milliseconds (100));
        portContentq.OutputData(radarSenfToPlaneDataT);
        std::this_thread::sleep_for(std::chrono::milliseconds (100));

//        portContent2.OutputData(radarDrawCahrDataT);
        std::this_thread::sleep_for(std::chrono::milliseconds (100));
        port.OutputData(radarDecisionDataT);



    }

    return 0;
}