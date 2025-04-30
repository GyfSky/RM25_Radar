//
// Created by plusseven on 24-4-26.
//

#include "../include/Port.h"

Port::Port(OurPattern ourPattern, int mode_num, TF is_openPort, UsePort usePort) {
    this->ourPattern = ourPattern;
    this->mode_num = mode_num;
    if(ourPattern == red){
        this->sender_id = 9;
        this->plane_id  = 6;
        this->sentry_id = 7;
        this->color_index = 0;
    }else if(ourPattern == blue){
        this->sender_id = 109;
        this->plane_id  = 106;
        this->sentry_id = 107;
        this->color_index = mode_num;
    }
    if(is_openPort == true_){
        this->is_openPort = true;
        this->port_path  = ("/dev/ttyUSB" + std::to_string(usePort)).data();
        this->serialPort_ptr = std::shared_ptr<SerialPort>(new SerialPort());
        this->serialPort_ptr->initSerialPort(this->port_path);
        this->fd = this->serialPort_ptr->fd;
        this->map_robot_ptr =
                std::shared_ptr<Content<MAP_ROBOT_DATA_T>>(new Content<MAP_ROBOT_DATA_T>(fd,24,0x0305));
        this->map_old_robot_ptr =
                std::shared_ptr<Content<MAP_ROBOT_DATA_T_OLD >>(new Content<MAP_ROBOT_DATA_T_OLD>(fd,10,0x0305));
        this->radar_decision_ptr =
                std::shared_ptr<Content<RADAR_DECISION_DATA_T>>(new Content<RADAR_DECISION_DATA_T>(fd,7,CMD_ROBOT_INTERACTION));

        initVulnerabilityData();
//        makeDrawFlyData(1);
        this->radar_plane_ptr =
                std::shared_ptr<Content<RADAR_SENF_TO_PLANE_DATA_T >>(new Content<RADAR_SENF_TO_PLANE_DATA_T>(fd,19,CLIENT_GRAPHIC_DRAW_ID));
        this->radar_sentry_ptr =
                std::shared_ptr<Content<RADAR_SEND_TO_SENTRY_DATA_T >>(new Content<RADAR_SEND_TO_SENTRY_DATA_T>(fd,46,CMD_ROBOT_INTERACTION));
    }else{
        std::cout << "!!! PORT IS CLOSE ，IS NOT OPEN" << std::endl;
        this->is_openPort = false;
    }

}

void Port::clearBuff() {
    tcflush(fd, TCIFLUSH);
}

void Port::start() { //TODO:

    std::function<void()> getData_ = std::bind(&Port::getData, this);
    std::function<void()> sendIVCData_ = std::bind(&Port::sendIVCData, this);  // IVC 定义: 车辆间通信 - Inter-Vehicle Communications
    std::function<void()> sendSTrackData_ = std::bind(&Port::sendSTrackData, this);
//    std::function<void()> sendSTrackData_ = std::bind(&Port::sendOldSTrackData, this);
    this->timer.addTimer(getData_, 1000./10); // 每隔10hz触发一次回调函数
    this->timer.addTimer(sendSTrackData_, 1000./5); // 每隔5hz触发一次回调函数
    this->timer.addTimer(sendIVCData_, 1000./30); // 每隔30hz触发一次回调函数
    timerThread = thread(&Timer::start, &(this->timer));
}

void Port::close() {
    this->timer.stop();
    timerThread.join();
}

void Port::initVulnerabilityData() {
    this->vulnerability_times.data.radar_info = 0;
    this->vulnerability_times.data.dacideing = 0;
    this->vulnerability_times.data.else_ = 0;
}


void Port::updataSTrackData(std::vector<STrack> out) {
    STrack_lock.lock();
    this->port_out.assign(out.begin(),out.end()) ;
    STrack_lock.unlock();
}

void Port::updataSentryData(std::vector<STrack> out) {
    radarSentryDataT_lock.lock();
    this->sentry_out.assign(out.begin(),out.end());
    radarSentryDataT_lock.unlock();
}

int Port::getRadarMarkNum() {
    enemys_lock.lock();
    auto mark=enemys;
    enemys_lock.unlock();
    int num=0;
    if (mark.data.mark_engineer_progress>0)
        num++;
    if (mark.data.mark_hero_progress>0)
        num++;
    if (mark.data.mark_sentry_progress>0)
        num++;
    if (mark.data.mark_standard_3_progress>0)
        num++;
    if (mark.data.mark_standard_4_progress>0)
        num++;
    return num;
}

void Port::updataRadarMarkData(std::vector<STrack> &out){
    enemys_lock.lock();
    for(auto &track: out){
        if(((track.cls - color_index) > -1 ) && (track.cls - color_index) < mode_num ){
            //准确标记的情况且达到标记阈值
            if((track.cls_len_time > cls_min_time) && (((enemys.u_char8[0]>>(track.cls - color_index) &1) - track.judge_radar_mark_data > 0) || (enemys.u_char8[0]>>(track.cls - color_index) &1 == 1))){
                //将对应种类的置信度拉高
                track.ws_armorConfMatrix(0, track.cls) = std::min(0.98, track.ws_armorConfMatrix(0, track.cls) * cls_up_magnification);
            //模拟连续错误的情况
            }else if((enemys.u_char8[0]>>(track.cls - color_index) &1) - track.judge_radar_mark_data < 0){
                //将对应种类的置信度降低
                track.ws_armorConfMatrix(0, track.cls) = track.ws_armorConfMatrix(0, track.cls) * cls_down_magnification;
            }
            track.judge_radar_mark_data = enemys.u_char8[0]>>(track.cls - color_index) &1;
        }
    }
    enemys_lock.unlock();
}


void Port::sendSTrackData() {
    STrack_lock.lock();
    sentryRadarDataT_lock.lock();
    for (int i=0;i<5;i++) {
        if (!port_out[color_index+i].is_det&&fabs(sentryRadarDataT.data.char_data[2*i]-0)>0.01&&fabs(sentryRadarDataT.data.char_data[2*i+1]-0)>0.01) {
            port_out[color_index+i].Locate3D.x=sentryRadarDataT.data.char_data[2*i];
            port_out[color_index+i].Locate3D.y=sentryRadarDataT.data.char_data[2*i+1];
        }
    }
    sentryRadarDataT_lock.unlock();
    MAP_ROBOT_DATA_T mapRobotDataT{
            uint16_t (port_out[color_index+0].Locate3D.x*100), uint16_t (port_out[color_index+0].Locate3D.y*100),
            uint16_t (port_out[color_index+1].Locate3D.x*100), uint16_t (port_out[color_index+1].Locate3D.y*100),
            uint16_t (port_out[color_index+2].Locate3D.x*100), uint16_t (port_out[color_index+2].Locate3D.y*100),
            uint16_t (port_out[color_index+3].Locate3D.x*100), uint16_t (port_out[color_index+3].Locate3D.y*100),
            uint16_t (0), uint16_t (0),
            uint16_t (port_out[color_index+4].Locate3D.x*100), uint16_t (port_out[color_index+4].Locate3D.y*100),
    };
    map_robot_ptr->OutputData(mapRobotDataT);
    STrack_lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds (5));
}



void Port::sendOldSTrackData() {
    STrack_lock.lock();
    MAP_ROBOT_DATA_T_OLD mapRobotDataT{
            {4,0,0}};
    map_old_robot_ptr->OutputData(mapRobotDataT);
    STrack_lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds (5));
}

void Port::autoDecisionMaking(){
    vulnerability_times_lock.lock();
    gameStatusT_times_lock.lock();
    rclcpp::Time now_time=rclcpp::Clock().now();
    bool dacision_time_flag = true;
    std::cout << "dacideing: " << int(vulnerability_times.data.dacideing) << std::endl;
    std::cout << "stage_remain_time: " << int(gameStatusT.data.stage_remain_time) << std::endl;
    double game_during_time=0;

    if(int(gameStatusT.data.game_progress) == 4) {
        game_during_time=420-(now_time.seconds()-game_start_time.seconds());
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "stage_remain_time: %f",game_during_time);
    }

    // if(int(gameStatusT.data.game_progress) == 4){
    //     if(int(gameRobotHpT.data.blue_7_robot_HP > 160 && ourPattern == blue) || (int(gameRobotHpT.data.red_7_robot_HP) > 160 && ourPattern == red)){
    //         is_self_guard_HP_160 = false;
    //     }
    //     if((int(gameRobotHpT.data.red_outpost_HP) > 0 && ourPattern == blue) ||(int(gameRobotHpT.data.blue_outpost_HP) > 0 && ourPattern == red)){
    //         is_rival_outpost_die = false;
    //     }
    //     if((int(gameRobotHpT.data.red_7_robot_HP) > 0 && ourPattern == blue) ||(int(gameRobotHpT.data.blue_7_robot_HP) > 0 && ourPattern == red)){
    //         is_rival_guard_die = false;
    //     }
    // }

    int number=getRadarMarkNum();
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "number: %d", number);

    //    if(vulnerability_times.data.radar_info > 0  && vulnerability_times.data.dacideing != 1){
    if(int(gameStatusT.data.game_progress) == 4&&number>=2){
        if(!is_time_2_55 && int(game_during_time) < (2*60 + 55)){
            is_time_2_55 = true;
            std::cout << "is_time_2_55" << std::endl;
            std::ofstream fout("resource/debug.txt", std::ios::app);
            if(!fout)
                std::cout<<"file cant open!!!"<<std::endl;
            else {
                fout<<"is_time_2_55";
                fout<<std::endl;
                fout<<int(gameStatusT.data.stage_remain_time)<<std::endl;
                fout<<int(game_during_time)<<std::endl;
                fout<<"------------------"<<std::endl;
                fout.close();
            }
        }
        else if(!is_time_1_40 && int(game_during_time) < (1*60 + 40)){
            is_time_1_40 = true;
            std::cout << "is_time_1_40" << std::endl;
            std::ofstream fout("resource/debug.txt", std::ios::app);
            if(!fout)
                std::cout<<"file cant open!!!"<<std::endl;
            else {
                fout<<"is_time_1_40";
                fout<<std::endl;
                fout<<int(gameStatusT.data.stage_remain_time)<<std::endl;
                fout<<int(game_during_time)<<std::endl;
                fout<<"------------------"<<std::endl;
                fout.close();
            }
        }
//        if(!is_self_guard_HP_160 && (((int(gameRobotHpT.data.blue_7_robot_HP) <= 160) && (ourPattern == blue)) ||
//                                          ((int(gameRobotHpT.data.red_7_robot_HP) <= 160 )&& (ourPattern == red))) ){
//            is_self_guard_HP_160 = true;
//            std::cout << "is_self_guard_HP_160: " <<  int(gameRobotHpT.data.blue_7_robot_HP)  << std::endl;
//        }
//        else if(!is_rival_outpost_die && ((int(gameRobotHpT.data.red_outpost_HP) == 0 && ourPattern == blue) ||
//                                          (int(gameRobotHpT.data.blue_outpost_HP) == 0 && ourPattern == red))){
//            is_rival_outpost_die = true;
//            std::cout << "is_rival_outpost_die: " << int(gameRobotHpT.data.red_outpost_HP) << std::endl;
//        }
//        else if(!is_rival_guard_die && ((int(gameRobotHpT.data.red_7_robot_HP) == 0 && ourPattern == blue) ||
//                                        (int(gameRobotHpT.data.blue_7_robot_HP) == 0 && ourPattern == red)) ){
//            is_rival_guard_die = true;
//            std::cout << "is_rival_guard_die: " << int(gameRobotHpT.data.red_7_robot_HP) <<  std::endl;
//        }
        else if(!is_time_1_00 && int(game_during_time)< (1*60 + 00)){
            is_time_1_00 = true;
            std::cout << "is_time_1_00" << std::endl;
            std::ofstream fout("resource/debug.txt", std::ios::app);
            if(!fout)
                std::cout<<"file cant open!!!"<<std::endl;
            else {
                fout<<"is_time_1_00";
                fout<<std::endl;
                fout<<int(gameStatusT.data.stage_remain_time)<<std::endl;
                fout<<int(game_during_time)<<std::endl;
                fout<<"------------------"<<std::endl;
                fout.close();
            }
        }
//        else if(!is_dart_hit && eventDataT.data.dart_hit_time > 0){
//            is_dart_hit = true;
//            std::cout << "is_dart_hit: " << eventDataT.data.dart_hit_time  << std::endl;
//        }
        else{
            dacision_time_flag = false;//不在时间范围内
        }

    }else{
        dacision_time_flag = false;//正在触发双倍易伤
        // dacision_time = 0;//TODO是不是有问题？
    }
    // radarDecisionDataT_times_lock.lock();
    // if (int(gameStatusT.data.game_progress) == 4) {
    //     if (thres>=0) {
    //         dacision_time=0;
    //     }else {
    //         dacision_time=1;
    //     }
    //     thres--;
    // }
    // radarDecisionDataT_times_lock.unlock();
    vulnerability_times_lock.unlock();
    gameStatusT_times_lock.unlock();
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"--------------------decision time: %d",dacision_time);
    if(dacision_time_flag){
        radarDecisionDataT_times_lock.lock();
        dacision_time = uint8_t(1) + dacision_time;
        std::ofstream fout("resource/debug.txt", std::ios::app);
        if(!fout)
            std::cout<<"file cant open!!!"<<std::endl;
        else {
            fout<<"dacision_time: ";
            fout<<dacision_time<<std::endl;
            fout<<"time: ";
            fout<<int(gameStatusT.data.stage_remain_time)<<std::endl;
            fout<<"time: "<<int(game_during_time)<<std::endl;
            fout<<"------------------"<<std::endl;
            fout.close();
        }
        std::cout << "---------------------------dacision_time_flag------------------------------: " << int(dacision_time) << std::endl;
        radarDecisionDataT_times_lock.unlock();
    }

}



void Port::getData() {
    unsigned char buff[5000] = {0};
    int inputSize = read(fd, buff, 5000);
    // RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "inputSize: %d", inputSize);
    std::cout << "inputSize: " << inputSize << std::endl;
    int ptr=0;
    while(ptr < inputSize)
    {
        if(buff[ptr] == SOF)
        {
//            std::cout << "ptr: " << ptr << std::endl;
            FRAME_HEADER temp_frameHeader;
            memcpy(temp_frameHeader.u_char8,buff+ptr,FRAME_HEADER_LEN);
            int size = FRAME_HEADER_LEN + CMD_LEN + temp_frameHeader.data.data_length + CRC16_LEN;
            if(Verify_CRC8_Check_Sum(temp_frameHeader.u_char8, FRAME_HEADER_LEN))  // && Verify_CRC16_Check_Sum(&buff[ptr], size)
            {
                ptr += FRAME_HEADER_LEN;
                uint16_t_uchar temp_cmd_id;
                memcpy(temp_cmd_id.u_char8, buff + ptr, CMD_LEN);
                // std::cout << "temp_cmd_id: " << temp_cmd_id.data << std::endl;    //514
                ptr += CMD_LEN;
                switch (temp_cmd_id.data)
                {
//                    case CMD_ROBOT_INTERACTION:
//                    {
//                        CHLID_FRAME_HEADER temp_chlidFrameHeader;
//                        memcpy(temp_chlidFrameHeader.u_char8, buff + ptr, 6);
//                        ptr += 6;
////
////                        switch (temp_chlidFrameHeader.data.data_cmd_id)
////                        {
////                        }
////
////                        break;
//                    }
                    case GAME_STATE_ID:
                    {
                        gameStatusT_times_lock.lock();
                        memcpy(this->gameStatusT.u_char8, buff + ptr, temp_frameHeader.data.data_length);

                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "game_progress: %d", gameStatusT.data.game_progress);
                        std::cout << "game_progress: " << std::to_string(gameStatusT.data.game_progress) << std::endl;
                        if (int(gameStatusT.data.game_progress)==4&&!time_init) {
                            time_init=true;
                            game_start_time=rclcpp::Clock().now();
                        }
//                        std::cout << "stage_remain_time: " << std::to_string(gameStatusT.data.stage_remain_time) << std::endl;
                        gameStatusT_times_lock.unlock();
                    }break;
                    case GAME_ROBOT_HP_ID:
                    {
                        gameRobotHpT_lock.lock();
                        memcpy(this->gameRobotHpT.u_char8, buff + ptr, temp_frameHeader.data.data_length);
                        gameRobotHpT_lock.unlock();
                    }break;
                    case EVENT_DADA_ID:
                    {
                        eventDataT_lock.lock();
                        memcpy(eventDataT.u_char8, buff + ptr, temp_frameHeader.data.data_length);
                        eventDataT_lock.unlock();
//                        std::cout << "eventDataT.data.virtual_shield_remaining: " << std::to_string(eventDataT.data.virtual_shield_remaining) << std::endl;
//                        std::cout << "eventDataT.data.dart_hit_time: " << std::to_string(eventDataT.data.dart_hit_time) << std::endl;
                    }break;
                    case CMD_RADAR_MARK_DATA_T:
                    {
                        enemys_lock.lock();
                        memcpy(enemys.u_char8, buff + ptr, temp_frameHeader.data.data_length);
                        enemys_lock.unlock();
                        std::cout << "enemys: " << std::to_string(enemys.u_char8[2]) << std::endl;
                    }break;
                    case CMD_RADAR_INFO_T:
                    {
                        vulnerability_times_lock.lock();
                        memcpy(vulnerability_times.u_char8, buff + ptr, temp_frameHeader.data.data_length );
                        vulnerability_times_lock.unlock();
                        std::cout << "vulnerability_times: " << std::to_string(vulnerability_times.data.radar_info) << std::endl;
                    }break;
                    case DART_INFO_DATA_ID:
                    {
                        dartInfo_lock.lock();
                        memcpy(dartInfo.u_char8, buff + ptr, temp_frameHeader.data.data_length);
                        target = dartInfo.data.target;
                        std::cout << "------------target:  " << std::to_string(target) << std::endl;
//                        std::cout << "------------data:  " << std::to_string(dartInfo.u_char8[2]) << std::endl;
                        dartInfo_lock.unlock();
                    }break;
                    case ROBOT_INTERACTIVE_DATA_ID:
                    {
                        CHLID_FRAME_HEADER temp_chlidFrameHeader;
                        memcpy(temp_chlidFrameHeader.u_char8, buff + ptr,6);
                        if (temp_chlidFrameHeader.data.data_cmd_id==CMD_SENTRY_RADAR_T&&
                            temp_chlidFrameHeader.data.sender_id==sentry_id&&
                            temp_chlidFrameHeader.data.receiver_id==sender_id) {
                            sentryRadarDataT_lock.lock();
                            memcpy(sentryRadarDataT.u_char8, buff + ptr, temp_frameHeader.data.data_length);
                            sentryRadarDataT_lock.unlock();
                        }
                    }break;

                }
                ptr += temp_frameHeader.data.data_length ;
                ptr += CRC16_LEN;
            }
        }
        ptr++;
    }
    autoDecisionMaking();

}

void Port::makePlaneData() {
    // 用于机器人之间通信
    this->radarPlaneDataT.data.data_cmd_id = 0x02FF;
    this->radarPlaneDataT.data.sender_id   = this->sender_id;
    this->radarPlaneDataT.data.receiver_id = this->plane_id;

    uint8_t  fly = 0;
    uint8_t  hole_red = 0;
    uint8_t  hole_orange = 0;
    uint8_t  windwill = 0;
    uint8_t  dartWarning = 0;
    if(this->fly_num>0)          fly         = 1;
    if(this->hole_red_num>0)     hole_red    = 1;
    if(this->hole_orange_num>0)  hole_orange = 1;
    if(this->windmill_num>0)     windwill    = 1;
    if(this->dart_num>0)     dartWarning    = 1;

    vulnerability_times_lock.lock();
    this->radarPlaneDataT.data.char_data[0] = fly;//飞坡
    this->radarPlaneDataT.data.char_data[1] = this->vulnerability_times.data.radar_info;//可易伤次数
    this->radarPlaneDataT.data.char_data[2] = hole_red;//步兵台阶
    this->radarPlaneDataT.data.char_data[3] = hole_orange;//有车台阶
    this->radarPlaneDataT.data.char_data[4] = windwill;//打符
    this->radarPlaneDataT.data.char_data[5] = dartWarning;//飞镖
    this->radarPlaneDataT.data.char_data[6] = 9;

    this->radarPlaneDataT.data.char_data[7] = fly;
    this->radarPlaneDataT.data.char_data[8] = this->vulnerability_times.data.radar_info;
    this->radarPlaneDataT.data.char_data[9] = hole_red;
    this->radarPlaneDataT.data.char_data[10] = hole_orange;
    this->radarPlaneDataT.data.char_data[11]= windwill;
    this->radarPlaneDataT.data.char_data[12]= dartWarning;
    vulnerability_times_lock.unlock();
}

void Port::sendPlaneData() {

    radarPlaneDataT_times_lock.lock();
    makePlaneData();
    radar_plane_ptr->OutputData(this->radarPlaneDataT);
    if(fly_num > 0){
        fly_num--;
    }
    if(hole_red_num > 0){
        hole_red_num--;
    }
    if(hole_orange_num > 0){
        hole_orange_num--;
    }
    if(windmill_num > 0){
        windmill_num--;
    }
    if(dart_num > 0) {
        dart_num--;
    }
    radarPlaneDataT_times_lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds (3));

}

void Port::makeDecisionData() {
    // 雷达自主决策
    this->radarDecisionDataT.data.data_cmd_id = 0x0121;
    this->radarDecisionDataT.data.sender_id   = this->sender_id;
    // 裁判系统服务器
    this->radarDecisionDataT.data.receiver_id = 0x8080;

    this->radarDecisionDataT.data.user_data[0] = dacision_time;
}

void Port::sendDecisionData(){
    radarDecisionDataT_times_lock.lock();
    // vulnerability_times_lock.lock();
    // gameStatusT_times_lock.lock();
    // dartInfo_lock.lock();
    // if(gameStatusT.data.game_progress == 4){
    //     //int(this->vulnerability_times.data.radar_info) > 0 有触发双倍易伤的次数
    //     if(int(this->vulnerability_times.data.radar_info) > 0  && this->target > dacision_time){
    //         dacision_time = uint8_t(1) + dacision_time;
    //     }
    // } else{
    //     dacision_time = 0;
    // }
    // vulnerability_times_lock.unlock();
    // gameStatusT_times_lock.unlock();
    // dartInfo_lock.unlock();
    makeDecisionData();
    std::cout << "dacision_time: " << std::to_string(dacision_time) << std::endl;
    radar_decision_ptr->OutputData(this->radarDecisionDataT);
    radarDecisionDataT_times_lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds (3));
}

void Port::makeSentryData(){
    this->radarSentryDataT.data.data_cmd_id = 0x02FE;
    this->radarSentryDataT.data.sender_id   = this->sender_id;
    this->radarSentryDataT.data.receiver_id = this->sentry_id;
    this->radarSentryDataT.data.char_data[0] = uint16_t(this->sentry_out[color_index+0].Locate3D.x*100);
    this->radarSentryDataT.data.char_data[1] = uint16_t(this->sentry_out[color_index+0].Locate3D.y*100);
    this->radarSentryDataT.data.char_data[2] = uint16_t(this->sentry_out[color_index+1].Locate3D.x*100);
    this->radarSentryDataT.data.char_data[3] = uint16_t(this->sentry_out[color_index+1].Locate3D.y*100);
    this->radarSentryDataT.data.char_data[4] = uint16_t(this->sentry_out[color_index+2].Locate3D.x*100);
    this->radarSentryDataT.data.char_data[5] = uint16_t(this->sentry_out[color_index+2].Locate3D.y*100);
    this->radarSentryDataT.data.char_data[6] = uint16_t(this->sentry_out[color_index+3].Locate3D.x*100);
    this->radarSentryDataT.data.char_data[7] = uint16_t(this->sentry_out[color_index+3].Locate3D.y*100);
    this->radarSentryDataT.data.char_data[8] = uint16_t(this->sentry_out[color_index+4].Locate3D.x*100);
    this->radarSentryDataT.data.char_data[9] = uint16_t(this->sentry_out[color_index+4].Locate3D.y*100);
    this->radarSentryDataT.data.char_data[10] = uint16_t(this->sentry_out[color_index+0].vx_3d*100);
    this->radarSentryDataT.data.char_data[11] = uint16_t(this->sentry_out[color_index+0].vy_3d*100);
    this->radarSentryDataT.data.char_data[12] = uint16_t(this->sentry_out[color_index+1].vx_3d*100);
    this->radarSentryDataT.data.char_data[13] = uint16_t(this->sentry_out[color_index+1].vy_3d*100);
    this->radarSentryDataT.data.char_data[14] = uint16_t(this->sentry_out[color_index+2].vx_3d*100);
    this->radarSentryDataT.data.char_data[15] = uint16_t(this->sentry_out[color_index+2].vy_3d*100);
    this->radarSentryDataT.data.char_data[16] = uint16_t(this->sentry_out[color_index+3].vx_3d*100);
    this->radarSentryDataT.data.char_data[17] = uint16_t(this->sentry_out[color_index+3].vy_3d*100);
    this->radarSentryDataT.data.char_data[18] = uint16_t(this->sentry_out[color_index+4].vx_3d*100);
    this->radarSentryDataT.data.char_data[19] = uint16_t(this->sentry_out[color_index+4].vy_3d*100);
}

void Port::sendSentryData(){
    radarSentryDataT_lock.lock();
    sentryRadarDataT_lock.lock();
    for (int i=0;i<5;i++) {
        if (!sentry_out[color_index+i].is_det&&fabs(sentryRadarDataT.data.char_data[2*i]-0)>0.01&&fabs(sentryRadarDataT.data.char_data[2*i+1]-0)>0.01) {
            sentry_out[color_index+i].Locate3D.x=sentryRadarDataT.data.char_data[2*i];
            sentry_out[color_index+i].Locate3D.y=sentryRadarDataT.data.char_data[2*i+1];
        }
    }
    sentryRadarDataT_lock.unlock();
    makeSentryData();
    radar_sentry_ptr->OutputData(this->radarSentryDataT);
    radarSentryDataT_lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds (3));
}

void Port::sendIVCData(){
    if(IVC_out_init==0){
        //发送是否触发双倍易伤
        sendDecisionData();
        IVC_out_init++;
    }
    else if(IVC_out_init==1){
        //给云台手发预警信息
        sendPlaneData();
        IVC_out_init++;
    }else if(IVC_out_init==2) {
        sendSentryData();
        IVC_out_init++;
    }
    IVC_out_init = IVC_out_init % IVC_num;
}

void Port::setWarring(std::vector<bool> isWarring) {
    radarPlaneDataT_times_lock.lock();
    if(isWarring[0])  fly_num = 7;
    if(isWarring[1])  hole_red_num = 7;
    if(isWarring[2])  hole_orange_num = 7;
    if(isWarring[3])  windmill_num = 7;
    if(isWarring[4])  dart_num = 3;
    radarPlaneDataT_times_lock.unlock();
}

//
//void Port::makeSTrackData(std::vector<STrack> &out, int classWithoutCar){
//    out.resize(classWithoutCar);
//    // TODO: 需要跟据yaml的改变而改变
//    out[0 ] = *new STrack(-1,25.80,8.0); //B1
//    out[1 ] = *new STrack(-1,20.00,4.0); //B2
//    out[2 ] = *new STrack(-1,26.75,7.5); //B3
//    out[3 ] = *new STrack(-1,26.75,7.5); //B4
//    out[4 ] = *new STrack(-1,26.75,7.5); //B5
//    out[5 ] = *new STrack(-1,23.00,7.5); //B7
//
//    out[6 ] = *new STrack(-1,2.20,7.0); //R1
//    out[7 ] = *new STrack(-1,8.00,11.); //R2
//    out[8 ] = *new STrack(-1,1.25,7.5); //R3
//    out[9 ] = *new STrack(-1,1.25,7.5); //R4
//    out[10] = *new STrack(-1,1.25,7.5); //R5
//    out[11] = *new STrack(-1,5.00,7.5); //R7
//
//
//    out[0 ] = *new STrack(-1,0.1,0.1); //B1
//    out[1 ] = *new STrack(-1,0.1,0.1); //B2
//    out[2 ] = *new STrack(-1,0.1,0.1); //B3
//    out[3 ] = *new STrack(-1,0.1,0.1); //B4
//    out[4 ] = *new STrack(-1,0.1,0.1); //B5
//    out[5 ] = *new STrack(-1,23.00,7.5); //B7
//
//    out[6 ] = *new STrack(-1,0.1,0.1); //R1
//    out[7 ] = *new STrack(-1,0.1,0.1); //R2
//    out[8 ] = *new STrack(-1,0.1,0.1); //R3
//    out[9 ] = *new STrack(-1,0.1,0.1); //R4
//    out[10] = *new STrack(-1,0.1,0.1); //R5
//    out[11] = *new STrack(-1,5.00,7.5); //R7
//
//}


//
//void Port::makeDrawFlyData(uint8_t operate_tpye) {
//    this->radarFlyDataT.data.data_cmd_id = 0x0110;  //TODO:
//    this->radarFlyDataT.data.sender_id   = this->sender_id;
//    this->radarFlyDataT.data.receiver_id = this->flyPlayer_id;
//
////    InteractionFigureUnion first_char;
//    this->radarFlyDataT.data.interactionFigure.figure_name[0] = 'F';
//    this->radarFlyDataT.data.interactionFigure.figure_name[1] = 'L';
//    this->radarFlyDataT.data.interactionFigure.figure_name[2] = 'Y ';
//
//    this->radarFlyDataT.data.interactionFigure.operate_tpye = operate_tpye;
//    this->radarFlyDataT.data.interactionFigure.figure_tpye  = 7;
//    this->radarFlyDataT.data.interactionFigure.layer        = 9;
//    this->radarFlyDataT.data.interactionFigure.color        = 3;
//    this->radarFlyDataT.data.interactionFigure.details_a    = 45;
//    this->radarFlyDataT.data.interactionFigure.details_b    = 5;
//    this->radarFlyDataT.data.interactionFigure.width        = 5;
//    this->radarFlyDataT.data.interactionFigure.start_x      = 840;
//    this->radarFlyDataT.data.interactionFigure.start_y      = 970;
//
//    this->radarFlyDataT.data.char_data[0]                   = 'F';
//    this->radarFlyDataT.data.char_data[1]                   = 'L';
//    this->radarFlyDataT.data.char_data[2]                   = 'Y';
//    this->radarFlyDataT.data.char_data[3]                   = '!';
//    this->radarFlyDataT.data.char_data[4]                   = '!';
//
//}
//
//
//
//void Port::makeDrawVulnerabilityData(uint8_t operate_tpye) {
//    this->radarVulnerabilityDataT.data.data_cmd_id = 0x0110;  //TODO:
//    this->radarVulnerabilityDataT.data.sender_id   = this->sender_id;
//    this->radarVulnerabilityDataT.data.receiver_id = this->flyPlayer_id;
//
////    InteractionFigureUnion first_char;
//    this->radarVulnerabilityDataT.data.interactionFigure.figure_name[0] = 'V';
//    this->radarVulnerabilityDataT.data.interactionFigure.figure_name[1] = 'U';
//    this->radarVulnerabilityDataT.data.interactionFigure.figure_name[2] = 'L ';
//
//    this->radarVulnerabilityDataT.data.interactionFigure.operate_tpye = operate_tpye;
//    this->radarVulnerabilityDataT.data.interactionFigure.figure_tpye  = 7;
//    this->radarVulnerabilityDataT.data.interactionFigure.layer        = 9;
//    this->radarVulnerabilityDataT.data.interactionFigure.color        = 3;
//    this->radarVulnerabilityDataT.data.interactionFigure.details_a    = 30;
//    this->radarVulnerabilityDataT.data.interactionFigure.details_b    = 1;
//    this->radarVulnerabilityDataT.data.interactionFigure.width        = 3;
//    this->radarVulnerabilityDataT.data.interactionFigure.start_x      = 1800;
//    this->radarVulnerabilityDataT.data.interactionFigure.start_y      = 970;
//
//    this->radarVulnerabilityDataT.data.char_data[0]                   = this->vulnerability_times.data.radar_info;
//
//}
//
//
//
//
//void Port::sendFly() {
//    if(fly_num > 0){
//        this->makeDrawFlyData(1); //add
//        radar_fly_ptr->OutputData(this->radarFlyDataT);
//        std::this_thread::sleep_for(std::chrono::milliseconds (5));
//        radar_fly_ptr->OutputData(this->radarFlyDataT);
//        fly_num--;
//    }else{   // 3 delete
//        this->makeDrawFlyData(3); // 3 delete
//        radar_fly_ptr->OutputData(this->radarFlyDataT);
//        std::this_thread::sleep_for(std::chrono::milliseconds (5));
//        radar_fly_ptr->OutputData(this->radarFlyDataT);
//        fly_num = 0;
//    }
//}
