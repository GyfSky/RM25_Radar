//
// Created by plusseven on 24-4-26.
//

#ifndef SRC_PORT_H
#define SRC_PORT_H
#include "serialport.h"
#include "Timer.h"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/robot_hp.hpp"
#include "interfaces/msg/game_state.hpp"
#include "../../ByteTrack//include/STrack.h"
enum Judgment    {self_dart,rival_dart,self_buff,self_offense,rival_offense,time_3_55,time_1_40,manual};

//enum UsePort       {USB0 = 0,USB1 = 1, USB2 = 2};
//enum TF {true_,false_};

inline std::string getDate(){
    char now[64];
    std::time_t tt;
    struct tm *ttime;
    tt = time(nullptr);
    ttime = localtime(&tt);
    strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime);
    std::string now_string(now);
    return now_string;
}

class Port {
private:
    int fd;
    const char *port_path;
    std::shared_ptr<SerialPort> serialPort_ptr = nullptr ;
    std::shared_ptr<Content<MAP_ROBOT_DATA_T>> map_robot_ptr = nullptr;
    std::shared_ptr<Content<MAP_ROBOT_DATA_T_OLD>> map_old_robot_ptr = nullptr;
    std::shared_ptr<Content<RADAR_DECISION_DATA_T>> radar_decision_ptr = nullptr;
    std::shared_ptr<Content<RADAR_SENF_TO_PLANE_DATA_T>> radar_plane_ptr = nullptr;
    std::shared_ptr<Content<RADAR_SEND_TO_SENTRY_DATA_T>> radar_sentry_ptr = nullptr;

    uint16_t sender_id;// 自己（雷达）id
    uint16_t plane_id;// 云台手id
    uint16_t sentry_id;

    OurPattern ourPattern;                      //己方颜色
    int mode_num;
//    std::vector<MAP_ROBOT_DATA_T> map_datas;

    Timer timer;
    std::thread timerThread;
    int thres=10;
    bool time_init =false;
    rclcpp::Time game_start_time;
    rclcpp::Time test_time;
    double game_during_time_=0;
    std::mutex game_time_lock_;


    int color_index = 0;
    int cls_min_time = 14;
    double cls_up_magnification = 1.56;
    double cls_down_magnification = 0.27;
public:
//    uint8_t enemy[6];
//    uint8_t vulnerability_times[1];
    bool is_openPort;
    rclcpp::Node* node;
    rclcpp::Publisher<interfaces::msg::RobotHP>::SharedPtr pub_hp;
    rclcpp::Publisher<interfaces::msg::GameState>::SharedPtr pub_game_state;

    std::mutex STrack_lock;
    std::vector<STrack> port_out;
    std::vector<STrack> sentry_out;

    std::mutex enemys_lock;
    RADAR_MARK_DATA_T enemys;
//    uint8_t enemys[6];

    std::mutex vulnerability_times_lock;
    RADAR_INFO_T vulnerability_times;

    std::mutex drone_location_lock_;
    unsigned int drone_location_x_;

    std::mutex radarPlaneDataT_times_lock;
    RADAR_SENF_TO_PLANE_DATA_T radarPlaneDataT;

    std::mutex radarSentryDataT_lock;
    RADAR_SEND_TO_SENTRY_DATA_T radarSentryDataT;

    std::mutex sentryRadarDataT_lock;
    RADAR_RECIEVE_SENTRY_DATA_T sentryRadarDataT;

    std::mutex dartInfo_lock;
    DART_INFO_T dartInfo;
    int target = 0;

    int fly_num = 0;
    int hole_red_num = 0;
    int hole_orange_num = 0;
    int dart_num = 0;
    int windmill_num = 0;
    int IVC_num = 3;
    int IVC_out_init = 0;

    std::mutex radarDecisionDataT_times_lock;
    RADAR_DECISION_DATA_T radarDecisionDataT; uint8_t dacision_time = 0;

    int dart_hit_[4]={0};
    int rival_dart_=0;
    rclcpp::Time buff_time_,self_offense_time_,rival_offense_time_,trigger_time_,t1;

    bool is_self_offense_=false,is_rival_offense_=false,is_time_3_55_=false, is_time_1_40_=false,is_time_2_55=false, is_time_1_00=false, is_self_guard_HP_160= true, is_dart_hit=false,
        is_rival_guard_die= true, is_rival_outpost_die= true;

    int judgment_condition_[8][2];
    std::map<int,int> judgment_condition_time_;
    std::map<int,string> judgment_condition_string_;

    std::mutex gameStatusT_times_lock;
    GAME_STATUS_T gameStatusT;

    std::mutex gameRobotHpT_lock;
    GAME_ROBOT_HP_T gameRobotHpT;
    bool is_outpost_die_=false;
    rclcpp::Time outpost_die_time_;

    std::mutex eventDataT_lock;
    EVENT_DATA_T eventDataT;

//    RADAR_SENF_TO_PLANE_DATA_T radarFlyDataT;
//    RADAR_DRAW_CAHR_DATA_T radarVulnerablityDataT;

    Port(OurPattern ourPattern, int mode_num, TF is_openPort, UsePort usePort,rclcpp::Node* node);
    void clearBuff();
    void start();
    void close();
    void autoDecisionMaking();
//    void makeSTrackData(std::vector<STrack> &out, int classWithoutCar);
    void updataSTrackData(std::vector<STrack> out);
    void updataSentryData(std::vector<STrack> out);
    void updataRadarMarkData(std::vector<STrack> &out);
    void updataDroneData(unsigned int x);
    void updateGameTime(double time);
    void sendSTrackData();
    void sendOldSTrackData();
    void makeSentryData();
    void sendSentryData();
    void makePlaneData();
    void sendPlaneData();
    void makeDecisionData();
    void sendDecisionData();
    void sendIVCData();
    bool checkPosition(cv::Point3d Locate3D);

    void checkSelfDart();
    void checkRivalDart();
    void checkSelfBuff();
    void checkSelfOffense();
    void checkRivalOffense();
    void checkGameTime();
    void checkManual();
    void checkTrigger();

    int getRadarMarkNum();
//    void sendFly();
    void getData();
//    void makeDrawFlyData(uint8_t operate_tpye);
    void initVulnerabilityData();
//    void makeDrawVulnerabilityData(uint8_t operate_tpye);
    void setWarring(std::vector<bool> isWarring);
};


#endif //SRC_PORT_H
