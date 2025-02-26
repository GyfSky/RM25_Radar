//
// Created by plusseven on 24-4-26.
//

#ifndef SRC_PORT_H
#define SRC_PORT_H
#include "serialport.h"
#include "Timer.h"
#include "../../ByteTrack//include/STrack.h"
//enum OurPattern    {red,blue};
//enum UsePort       {USB0 = 0,USB1 = 1, USB2 = 2};
//enum TF {true_,false_};

class Port {
private:
    int fd;
    const char *port_path;
    std::shared_ptr<SerialPort> serialPort_ptr = nullptr ;
    std::shared_ptr<Content<MAP_ROBOT_DATA_T>> map_robot_ptr = nullptr;
    std::shared_ptr<Content<MAP_ROBOT_DATA_T_OLD>> map_old_robot_ptr = nullptr;
    std::shared_ptr<Content<RADAR_DECISION_DATA_T>> radar_decision_ptr = nullptr;
    std::shared_ptr<Content<RADAR_SENF_TO_PLANE_DATA_T>> radar_plane_ptr = nullptr;

    uint16_t sender_id;
    uint16_t plane_id;

    OurPattern ourPattern;                      //己方颜色
    int mode_num;
//    std::vector<MAP_ROBOT_DATA_T> map_datas;

    Timer timer;
    std::thread timerThread;

    bool is_time_2_55=false, is_time_1_40=false, is_time_1_00=false, is_self_guard_HP_160= true, is_dart_hit=false,
            is_rival_guard_die= true, is_rival_outpost_die= true;

    int color_index = 0;
    int cls_min_time = 14;
    double cls_up_magnification = 1.56;
    double cls_down_magnification = 0.27;
public:
//    uint8_t enemy[6];
//    uint8_t vulnerability_times[1];
    bool is_openPort;

    std::mutex STrack_lock;
    std::vector<STrack> port_out;

    std::mutex enemys_lock;
    RADAR_MARK_DATA_T enemys;
//    uint8_t enemys[6];

    std::mutex vulnerability_times_lock;
    RADAR_INFO_T vulnerability_times;

    std::mutex radarPlaneDataT_times_lock;
    RADAR_SENF_TO_PLANE_DATA_T radarPlaneDataT;

    std::mutex dartInfo_lock;
    DART_INFO_T dartInfo;
    int target = 0;

    int fly_num = 0;
    int hole_red_num = 0;
    int hole_orange_num = 0;
    int windmill_num = 0;
    int IVC_num = 2;
    int IVC_out_init = 0;

    std::mutex radarDecisionDataT_times_lock;
    RADAR_DECISION_DATA_T radarDecisionDataT; uint8_t dacision_time = 0;

    std::mutex gameStatusT_times_lock;
    GAME_STATUS_T gameStatusT;

    std::mutex gameRobotHpT_lock;
    GAME_ROBOT_HP_T gameRobotHpT;

    std::mutex eventDataT_lock;
    EVENT_DATA_T eventDataT;

//    RADAR_SENF_TO_PLANE_DATA_T radarFlyDataT;
//    RADAR_DRAW_CAHR_DATA_T radarVulnerablityDataT;

    Port(OurPattern ourPattern, int mode_num, TF is_openPort, UsePort usePort);
    void clearBuff();
    void start();
    void close();
    void autoDecisionMaking();
//    void makeSTrackData(std::vector<STrack> &out, int classWithoutCar);
    void updataSTrackData(std::vector<STrack> out);
    void updataRadarMarkData(std::vector<STrack> &out);
    void sendSTrackData();
    void sendOldSTrackData();
    void makePlaneData();
    void sendPlaneData();
    void makeDecisionData();
    void sendDecisionData();
    void sendIVCData();
//    void sendFly();
    void getData();
//    void makeDrawFlyData(uint8_t operate_tpye);
    void initVulnerabilityData();
//    void makeDrawVulnerabilityData(uint8_t operate_tpye);
    void setWarring(std::vector<bool> isWarring);
};


#endif //SRC_PORT_H
