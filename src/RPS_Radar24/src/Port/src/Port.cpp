//
// Created by plusseven on 24-4-26.
//

#include "../include/Port.h"

Port::Port(OurPattern ourPattern, int mode_num, TF is_openPort, UsePort usePort,rclcpp::Node* node) {
    this->ourPattern = ourPattern;
    this->mode_num = mode_num;
    port_out.resize(2*this->mode_num);
    test_time=rclcpp::Clock().now();
    sentryRadarDataT_lock.lock();
    for (int i=0;i<10;i++) {
        sentryRadarDataT.data.char_data[i]=0.0;
    }
    sentryRadarDataT_lock.unlock();
    drone_location_lock_.lock();
    drone_location_x_=0;
    drone_location_lock_.unlock();

    buff_time_=rclcpp::Clock().now();
    self_offense_time_=rclcpp::Clock().now();
    rival_offense_time_=rclcpp::Clock().now();
    outpost_die_time_=rclcpp::Clock().now();
    trigger_time_=rclcpp::Clock().now();

    std::ofstream fout("resource/debug.txt", std::ios::app);
    if(!fout) {
        std::cout<<"file cant open!!!"<<std::endl;
    }else {
        fout<<"------new------    "<<getDate()<<std::endl;
        fout.close();
    }

    judgment_condition_time_.insert(std::pair<int,int>(0,200));
    judgment_condition_time_.insert(std::pair<int,int>(1,200));
    judgment_condition_time_.insert(std::pair<int,int>(2,400));
    judgment_condition_time_.insert(std::pair<int,int>(3,100));
    judgment_condition_time_.insert(std::pair<int,int>(4,100));
    judgment_condition_time_.insert(std::pair<int,int>(5,300));
    judgment_condition_time_.insert(std::pair<int,int>(6,400));
    judgment_condition_time_.insert(std::pair<int,int>(7,100));

    judgment_condition_string_.insert(std::pair<int,string>(0,"self_dart"));
    judgment_condition_string_.insert(std::pair<int,string>(1,"rival_dart"));
    judgment_condition_string_.insert(std::pair<int,string>(2,"self_buff"));
    judgment_condition_string_.insert(std::pair<int,string>(3,"self_offense"));
    judgment_condition_string_.insert(std::pair<int,string>(4,"rival_offense"));
    judgment_condition_string_.insert(std::pair<int,string>(5,"time_3_55"));
    judgment_condition_string_.insert(std::pair<int,string>(6,"time_1_40"));
    judgment_condition_string_.insert(std::pair<int,string>(7,"manual"));

    for (int i=0;i<8;i++) {
        judgment_condition_[i][0]=0;
        judgment_condition_[i][1]=judgment_condition_time_[i];
    }

    dartInfo_lock.lock();
    dartInfo.data.new_hit_target=0;
    dartInfo.data.cumulative_hit_time=0;
    dartInfo.data.target=0;
    dartInfo_lock.unlock();

    eventDataT_lock.lock();
    eventDataT.data.dart_hit_time=0;
    eventDataT.data.big_buff=0;
    eventDataT.data.small_buff=0;
    eventDataT_lock.unlock();

    enemys_lock.lock();
    enemys.data.mark_engineer_progress=0;
    enemys.data.mark_hero_progress=0;
    enemys.data.mark_sentry_progress=0;
    enemys.data.mark_standard_3_progress=0;
    enemys.data.mark_standard_4_progress=0;
    enemys_lock.unlock();

    gameStatusT_times_lock.lock();
    gameStatusT.data.game_progress=0;
    gameStatusT_times_lock.unlock();

    vulnerability_times_lock.lock();
    vulnerability_times.data.dacideing=0;
    vulnerability_times.data.radar_info=0;
    vulnerability_times_lock.unlock();

    this->node=node;
    this->pub_hp=this->node->create_publisher<interfaces::msg::RobotHP>("/robot_hp",10);
    this->pub_game_state=this->node->create_publisher<interfaces::msg::GameState>("/game_state",10);
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
    this->timer.addTimer(getData_, 1000./20); // 每隔20hz触发一次回调函数
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

void Port::updataDroneData(unsigned int x) {
    drone_location_lock_.lock();
    drone_location_x_=x;
    drone_location_lock_.unlock();
}

void Port::updateGameTime(double time) {
    game_time_lock_.lock();
    time=game_during_time_;
    game_time_lock_.unlock();
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
        if (fabs(sentryRadarDataT.data.char_data[2*i]-0)>0.01&&fabs(sentryRadarDataT.data.char_data[2*i+1]-0)>0.01)
            RCLCPP_ERROR(rclcpp::get_logger("judge"), "sentry: index: %d,x: %f,y: %f",i,sentryRadarDataT.data.char_data[2*i],sentryRadarDataT.data.char_data[2*i+1]);
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

void Port::checkSelfDart() {
    dartInfo_lock.lock();
    if (time_init&&dartInfo.data.new_hit_target>0) {
        if (dartInfo.data.cumulative_hit_time>dart_hit_[dartInfo.data.new_hit_target-1]) {
            dart_hit_[dartInfo.data.new_hit_target-1]=dartInfo.data.cumulative_hit_time;
            judgment_condition_[Judgment::self_dart][0]=1;
            judgment_condition_[Judgment::self_dart][1]=judgment_condition_time_[Judgment::self_dart];
        }
    }
    dartInfo_lock.unlock();
}

void Port::checkRivalDart() {
    eventDataT_lock.lock();
    if (time_init&&eventDataT.data.dart_hit_time>rival_dart_) {
        rival_dart_=eventDataT.data.dart_hit_time;
        judgment_condition_[Judgment::rival_dart][0]=1;
        judgment_condition_[Judgment::rival_dart][1]=judgment_condition_time_[Judgment::rival_dart];
    }
    eventDataT_lock.unlock();
}

void Port::checkSelfBuff() {
    eventDataT_lock.lock();
    if (time_init&&(eventDataT.data.big_buff==1||eventDataT.data.small_buff==1)) {
        if (rclcpp::Clock().now().seconds()-buff_time_.seconds()>47){
            buff_time_=rclcpp::Clock().now();
            judgment_condition_[Judgment::self_buff][0]=1;
        }
    }
    eventDataT_lock.unlock();
}

void Port::checkSelfOffense() {
    int car_num=0;
    STrack_lock.lock();
    for (int i=0;i<5;i++) {
        if (color_index==0&&i!=1&&port_out[mode_num+i].Locate3D.x>14.0&&port_out[mode_num+i].Locate3D.x!=0) {
            car_num++;
        }else if (color_index==mode_num&&i!=1&&port_out[i].Locate3D.x<14.0&&port_out[i].Locate3D.x!=0) {
            car_num++;
        }
    }
    STrack_lock.unlock();
    if (time_init&&car_num>=3) {
        if (!is_self_offense_) {
            is_self_offense_=true;
            self_offense_time_=rclcpp::Clock().now();
        }else if (rclcpp::Clock().now().seconds()-self_offense_time_.seconds()>5){
            judgment_condition_[Judgment::self_offense][0]=1;
            judgment_condition_[Judgment::self_offense][1]=judgment_condition_time_[Judgment::self_offense];
        }
    }else if (rclcpp::Clock().now().seconds()-self_offense_time_.seconds()<=5||judgment_condition_[Judgment::self_offense][0]==0){
        is_self_offense_=false;
    }
}

void Port::checkRivalOffense() {
    int car_num=0;
    STrack_lock.lock();
    for (int i=0;i<5;i++) {
        if (color_index==0&&i!=1&&port_out[color_index+i].Locate3D.x<14.0&&port_out[color_index+i].Locate3D.x!=0) {
            car_num++;
        }else if (color_index==mode_num&&i!=1&&port_out[color_index+i].Locate3D.x>14.0&&port_out[color_index+i].Locate3D.x!=0) {
            car_num++;
        }
    }
    STrack_lock.unlock();
    if (time_init&&car_num>=3) {
        if (!is_rival_offense_) {
            is_rival_offense_=true;
            rival_offense_time_=rclcpp::Clock().now();
        }else if (rclcpp::Clock().now().seconds()-rival_offense_time_.seconds()>5){
            judgment_condition_[Judgment::rival_offense][0]=1;
            judgment_condition_[Judgment::rival_offense][1]=judgment_condition_time_[Judgment::rival_offense];
        }
    }else if (rclcpp::Clock().now().seconds()-rival_offense_time_.seconds()<=5||judgment_condition_[Judgment::rival_offense][0]==0){
        is_rival_offense_=false;
    }
}

void Port::checkGameTime() {
    rclcpp::Time now_time=rclcpp::Clock().now();
    double game_during_time=0;
    if(time_init) {
        game_during_time=420-(now_time.seconds()-game_start_time.seconds());
        game_time_lock_.lock();
        game_during_time_=now_time.seconds()-game_start_time.seconds();
        game_time_lock_.unlock();
        RCLCPP_ERROR(rclcpp::get_logger("judge"), "stage_remain_time: %f",game_during_time);

        if(!is_time_3_55_ && int(game_during_time) < (3*60 + 55)&&int(game_during_time) > (3*60 + 40)){
            judgment_condition_[Judgment::time_3_55][0]=1;
            is_time_3_55_=true;
        }
        else if(!is_time_1_40_ && int(game_during_time) < (1*60 + 40)){
            judgment_condition_[Judgment::time_1_40][0]=1;
            is_time_1_40_=true;
        }
    }
}

void Port::checkManual() {
    dartInfo_lock.lock();
    if (time_init&&rclcpp::Clock().now().seconds()-outpost_die_time_.seconds()>10&&dartInfo.data.target==0) {
        judgment_condition_[Judgment::manual][0]=1;
    }
    dartInfo_lock.unlock();
}

void Port::checkTrigger() {
    vulnerability_times_lock.lock();
    int dacideing=vulnerability_times.data.dacideing;
    vulnerability_times_lock.unlock();
    if (time_init&&dacideing==0&&rclcpp::Clock().now().seconds()-trigger_time_.seconds()>5) {
        checkSelfDart();
        checkRivalDart();
        checkSelfBuff();
        checkSelfOffense();
        checkRivalOffense();
        checkGameTime();
        checkManual();
    }
}

void Port::autoDecisionMaking(){
    bool dacision_time_flag = false;
    checkTrigger();
    int number=getRadarMarkNum();
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "number: %d", number);

    vulnerability_times_lock.lock();
    int radar_info=vulnerability_times.data.radar_info;
    vulnerability_times_lock.unlock();

    for (int i=0;i<8;i++) {
        if (time_init&&judgment_condition_[i][0]==1) {
            if (number>=3&&radar_info>0) {
                dacision_time_flag=true;
                std::ofstream fout("resource/debug.txt", std::ios::app);
                if(!fout)
                    std::cout<<"file cant open!!!"<<std::endl;
                else {
                    fout<<"dacision_time: "<<int(dacision_time+1)<<std::endl;
                    fout<<"time: "<<int(gameStatusT.data.stage_remain_time)<<std::endl;
                    fout<<"reason: "<<string(judgment_condition_string_[i])<<std::endl;
                    fout<<"wait_time: "<<int(judgment_condition_time_[i]-judgment_condition_[i][1])<<std::endl;
                    fout<<"------------------"<<std::endl;
                    fout.close();
                }
                for (int j=0;j<8;j++) {
                    judgment_condition_[j][0]=0;
                    judgment_condition_[j][1]=judgment_condition_time_[j];
                }
                trigger_time_=rclcpp::Clock().now();
                break;
            }else if (judgment_condition_[i][1]>0){
                judgment_condition_[i][1]--;
            }else {
                std::ofstream fout("resource/debug.txt", std::ios::app);
                if(!fout)
                    std::cout<<"file cant open!!!"<<std::endl;
                else {
                    fout<<"decision false!!! "<<std::endl;
                    fout<<"time: "<<int(gameStatusT.data.stage_remain_time)<<std::endl;
                    fout<<"reason: "<<string(judgment_condition_string_[i])<<std::endl;
                    string fail_reason;
                    if (number<3) fail_reason+="car_num<3 ";
                    if (radar_info==0) fail_reason+="radar_info=0 ";
                    fout<<"fail reason: "<<string(fail_reason)<<std::endl;
                    fout<<"------------------"<<std::endl;
                    fout.close();
                }
                judgment_condition_[i][0]=0;
                judgment_condition_[i][1]=judgment_condition_time_[i];
            }
        }
    }
    interfaces::msg::JudgeDebug debug_msg;
    for (int i=0;i<8;i++) {
        debug_msg.state[i]=judgment_condition_[i][0];
        debug_msg.wait_time[i]=judgment_condition_[i][1];

    }
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
            if(Verify_CRC8_Check_Sum(temp_frameHeader.u_char8, FRAME_HEADER_LEN)&&Verify_CRC16_Check_Sum(&buff[ptr], size))  // && Verify_CRC16_Check_Sum(&buff[ptr], size)
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

                        RCLCPP_ERROR(rclcpp::get_logger("judge"), "game_progress: %d", gameStatusT.data.game_progress);
                        std::cout << "game_progress: " << std::to_string(gameStatusT.data.game_progress) << std::endl;
                        if (int(gameStatusT.data.game_progress)==4&&!time_init) {
                            time_init=true;
                            game_start_time=rclcpp::Clock().now();
                        }
                        interfaces::msg::GameState game_state;
                        game_state.game_progress=gameStatusT.data.game_progress;
                        game_state.header.stamp = rclcpp::Clock().now();
                        pub_game_state->publish(game_state);
//                        std::cout << "stage_remain_time: " << std::to_string(gameStatusT.data.stage_remain_time) << std::endl;
                        gameStatusT_times_lock.unlock();
                    }break;
                    case GAME_ROBOT_HP_ID:
                    {
                        gameRobotHpT_lock.lock();
                        memcpy(this->gameRobotHpT.u_char8, buff + ptr, temp_frameHeader.data.data_length);
                        interfaces::msg::RobotHP robotHP;
                        robotHP.blue_robot_hp[0] = gameRobotHpT.data.blue_1_robot_HP;
                        robotHP.blue_robot_hp[1] = gameRobotHpT.data.blue_2_robot_HP;
                        robotHP.blue_robot_hp[2] = gameRobotHpT.data.blue_3_robot_HP;
                        robotHP.blue_robot_hp[3] = gameRobotHpT.data.blue_4_robot_HP;
                        robotHP.blue_robot_hp[4] = gameRobotHpT.data.blue_7_robot_HP;
                        robotHP.red_robot_hp[0] = gameRobotHpT.data.red_1_robot_HP;
                        robotHP.red_robot_hp[1] = gameRobotHpT.data.red_2_robot_HP;
                        robotHP.red_robot_hp[2] = gameRobotHpT.data.red_3_robot_HP;
                        robotHP.red_robot_hp[3] = gameRobotHpT.data.red_4_robot_HP;
                        robotHP.red_robot_hp[4] = gameRobotHpT.data.red_7_robot_HP;
                        if (time_init&&!is_outpost_die_&&((color_index==0&&gameRobotHpT.data.blue_outpost_HP==0)||(color_index==mode_num&&gameRobotHpT.data.red_outpost_HP==0))) {
                            is_outpost_die_=true;
                            outpost_die_time_=rclcpp::Clock().now();
                        }
                        gameRobotHpT_lock.unlock();
                        pub_hp->publish(robotHP);
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
                        // auto tim=rclcpp::Clock().now();
                        // int gap=tim.seconds()-test_time.seconds();
                        // test_time=tim;
                        // RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "！！！！！！！！！！！！！！！！！！！！！！！！！！！,%d",gap);
                        enemys_lock.lock();
                        memcpy(enemys.u_char8, buff + ptr, temp_frameHeader.data.data_length);
                        RCLCPP_ERROR(rclcpp::get_logger("judge"),"1: %d,2: %d,3: %d,4: %d,7: %d",enemys.data.mark_hero_progress,
                            enemys.data.mark_engineer_progress,enemys.data.mark_standard_3_progress,enemys.data.mark_standard_4_progress,enemys.data.mark_sentry_progress);
                        enemys_lock.unlock();
                        // std::cout << "enemys: " << std::to_string(enemys.u_char8[2]) << std::endl;
                    }break;
                    case CMD_RADAR_INFO_T:
                    {
                        vulnerability_times_lock.lock();
                        memcpy(vulnerability_times.u_char8, buff + ptr, temp_frameHeader.data.data_length );
                        RCLCPP_ERROR(rclcpp::get_logger("judge"), "vulnerability_times: %d", vulnerability_times.data.radar_info);
                        RCLCPP_ERROR(rclcpp::get_logger("judge"), "dacideing: %d", vulnerability_times.data.dacideing);
                        vulnerability_times_lock.unlock();
                    }break;
                    case DART_INFO_DATA_ID:
                    {
                        dartInfo_lock.lock();
                        memcpy(dartInfo.u_char8, buff + ptr, temp_frameHeader.data.data_length);
                        target = dartInfo.data.target;
                        RCLCPP_ERROR(rclcpp::get_logger("judge"),"dart target: %d", target);
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
    drone_location_lock_.lock();
    uint8_t  drone_location_x=this->drone_location_x_;
    drone_location_lock_.unlock();

    if(this->fly_num>0) {
        fly         = 1;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"detect fly");
    }
    if(this->hole_red_num>0) {
        hole_red    = 1;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"detect hole_red");
    }
    if(this->hole_orange_num>0) {
        hole_orange = 1;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"detect hole_orange");
    }
    if(this->windmill_num>0) {
        windwill    = 1;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"detect windwill");
    }
    if(this->dart_num>0) {
        dartWarning    = 1;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"detect dartWarning");
    }
    vulnerability_times_lock.lock();
    this->radarPlaneDataT.data.char_data[0] = fly;//飞坡
    this->radarPlaneDataT.data.char_data[1] = this->vulnerability_times.data.radar_info;//可易伤次数
    this->radarPlaneDataT.data.char_data[2] = drone_location_x;//无人机归一化位置（0-100）
    this->radarPlaneDataT.data.char_data[3] = drone_location_x;
    this->radarPlaneDataT.data.char_data[4] = windwill;//打符
    this->radarPlaneDataT.data.char_data[5] = dartWarning;//飞镖
    this->radarPlaneDataT.data.char_data[6] = 9;

    this->radarPlaneDataT.data.char_data[7] = fly;
    this->radarPlaneDataT.data.char_data[8] = this->vulnerability_times.data.radar_info;
    this->radarPlaneDataT.data.char_data[9] = drone_location_x;
    this->radarPlaneDataT.data.char_data[10] = drone_location_x;
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

bool Port::checkPosition(cv::Point3d Locate3D) {
    return Locate3D.x<=29.0 && Locate3D.x>0.0 && Locate3D.y<=16.0 && Locate3D.y>0.0;
}

void Port::makeSentryData(){
    this->radarSentryDataT.data.data_cmd_id = 0x02FE;
    this->radarSentryDataT.data.sender_id   = this->sender_id;
    this->radarSentryDataT.data.receiver_id = this->sentry_id;
    if (checkPosition(this->sentry_out[color_index+0].Locate3D)) {
        this->radarSentryDataT.data.position[0] = uint16_t(this->sentry_out[color_index+0].Locate3D.x*100);
        this->radarSentryDataT.data.position[1] = uint16_t(this->sentry_out[color_index+0].Locate3D.y*100);
        this->radarSentryDataT.data.speed[0] = int16_t(this->sentry_out[color_index+0].vx_3d*100);
        this->radarSentryDataT.data.speed[1] = int16_t(this->sentry_out[color_index+0].vy_3d*100);
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"1 x: %d",uint16_t(this->sentry_out[color_index+0].Locate3D.x*100));
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"1 y: %d",uint16_t(this->sentry_out[color_index+0].Locate3D.y*100));
    }else {
        this->radarSentryDataT.data.position[0] =0;
        this->radarSentryDataT.data.position[1] = 0;
        this->radarSentryDataT.data.speed[0] = 0;
        this->radarSentryDataT.data.speed[1] = 0;
    }
    if (checkPosition(this->sentry_out[color_index+1].Locate3D)) {
        this->radarSentryDataT.data.position[2] = uint16_t(this->sentry_out[color_index+1].Locate3D.x*100);
        this->radarSentryDataT.data.position[3] = uint16_t(this->sentry_out[color_index+1].Locate3D.y*100);
        this->radarSentryDataT.data.speed[2] = int16_t(this->sentry_out[color_index+1].vx_3d*100);
        this->radarSentryDataT.data.speed[3] = int16_t(this->sentry_out[color_index+1].vy_3d*100);
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"2 x: %d",uint16_t(this->sentry_out[color_index+1].Locate3D.x*100));
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"2 y: %d",uint16_t(this->sentry_out[color_index+1].Locate3D.y*100));
    }else {
        this->radarSentryDataT.data.position[2] =0;
        this->radarSentryDataT.data.position[3] = 0;
        this->radarSentryDataT.data.speed[2] = 0;
        this->radarSentryDataT.data.speed[3] = 0;
    }
    if (checkPosition(this->sentry_out[color_index+2].Locate3D)) {
        this->radarSentryDataT.data.position[4] = uint16_t(this->sentry_out[color_index+2].Locate3D.x*100);
        this->radarSentryDataT.data.position[5] = uint16_t(this->sentry_out[color_index+2].Locate3D.y*100);
        this->radarSentryDataT.data.speed[4] = int16_t(this->sentry_out[color_index+2].vx_3d*100);
        this->radarSentryDataT.data.speed[5] = int16_t(this->sentry_out[color_index+2].vy_3d*100);
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"3 x: %d",uint16_t(this->sentry_out[color_index+2].Locate3D.x*100));
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"3 y: %d",uint16_t(this->sentry_out[color_index+2].Locate3D.y*100));
    }else {
        this->radarSentryDataT.data.position[4] =0;
        this->radarSentryDataT.data.position[5] = 0;
        this->radarSentryDataT.data.speed[4] = 0;
        this->radarSentryDataT.data.speed[5] = 0;
    }
    if (checkPosition(this->sentry_out[color_index+3].Locate3D)) {
        this->radarSentryDataT.data.position[6] = uint16_t(this->sentry_out[color_index+3].Locate3D.x*100);
        this->radarSentryDataT.data.position[7] = uint16_t(this->sentry_out[color_index+3].Locate3D.y*100);
        this->radarSentryDataT.data.speed[6] = int16_t(this->sentry_out[color_index+3].vx_3d*100);
        this->radarSentryDataT.data.speed[7] = int16_t(this->sentry_out[color_index+3].vy_3d*100);
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"4 x: %d",uint16_t(this->sentry_out[color_index+3].Locate3D.x*100));
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"4 y: %d",uint16_t(this->sentry_out[color_index+3].Locate3D.y*100));
    }else {
        this->radarSentryDataT.data.position[6] =0;
        this->radarSentryDataT.data.position[7] = 0;
        this->radarSentryDataT.data.speed[6] = 0;
        this->radarSentryDataT.data.speed[7] = 0;
    }
    if (checkPosition(this->sentry_out[color_index+4].Locate3D)) {
        this->radarSentryDataT.data.position[8] = uint16_t(this->sentry_out[color_index+4].Locate3D.x*100);
        this->radarSentryDataT.data.position[9] = uint16_t(this->sentry_out[color_index+4].Locate3D.y*100);
        this->radarSentryDataT.data.speed[8] = int16_t(this->sentry_out[color_index+4].vx_3d*100);
        this->radarSentryDataT.data.speed[9] = int16_t(this->sentry_out[color_index+4].vy_3d*100);
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"7 x: %d",uint16_t(this->sentry_out[color_index+4].Locate3D.x*100));
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"7 y: %d",uint16_t(this->sentry_out[color_index+4].Locate3D.y*100));
    }else {
        this->radarSentryDataT.data.position[8] =0;
        this->radarSentryDataT.data.position[9] = 0;
        this->radarSentryDataT.data.speed[8] = 0;
        this->radarSentryDataT.data.speed[9] = 0;
    }
    // this->radarSentryDataT.data.position[0] = uint16_t(this->sentry_out[color_index+0].Locate3D.x*100);
    // this->radarSentryDataT.data.position[1] = uint16_t(this->sentry_out[color_index+0].Locate3D.y*100);
    // this->radarSentryDataT.data.position[2] = uint16_t(this->sentry_out[color_index+1].Locate3D.x*100);
    // this->radarSentryDataT.data.position[3] = uint16_t(this->sentry_out[color_index+1].Locate3D.y*100);
    // this->radarSentryDataT.data.position[4] = uint16_t(this->sentry_out[color_index+2].Locate3D.x*100);
    // this->radarSentryDataT.data.position[5] = uint16_t(this->sentry_out[color_index+2].Locate3D.y*100);
    // this->radarSentryDataT.data.position[6] = uint16_t(this->sentry_out[color_index+3].Locate3D.x*100);
    // this->radarSentryDataT.data.position[7] = uint16_t(this->sentry_out[color_index+3].Locate3D.y*100);
    // this->radarSentryDataT.data.position[8] = uint16_t(this->sentry_out[color_index+4].Locate3D.x*100);
    // this->radarSentryDataT.data.position[9] = uint16_t(this->sentry_out[color_index+4].Locate3D.y*100);

    // this->radarSentryDataT.data.speed[0] = int16_t(this->sentry_out[color_index+0].vx_3d*100);
    // this->radarSentryDataT.data.speed[1] = int16_t(this->sentry_out[color_index+0].vy_3d*100);
    // this->radarSentryDataT.data.speed[2] = int16_t(this->sentry_out[color_index+1].vx_3d*100);
    // this->radarSentryDataT.data.speed[3] = int16_t(this->sentry_out[color_index+1].vy_3d*100);
    // this->radarSentryDataT.data.speed[4] = int16_t(this->sentry_out[color_index+2].vx_3d*100);
    // this->radarSentryDataT.data.speed[5] = int16_t(this->sentry_out[color_index+2].vy_3d*100);
    // this->radarSentryDataT.data.speed[6] = int16_t(this->sentry_out[color_index+3].vx_3d*100);
    // this->radarSentryDataT.data.speed[7] = int16_t(this->sentry_out[color_index+3].vy_3d*100);
    // this->radarSentryDataT.data.speed[8] = int16_t(this->sentry_out[color_index+4].vx_3d*100);
    // this->radarSentryDataT.data.speed[9] = int16_t(this->sentry_out[color_index+4].vy_3d*100);
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
    if(isWarring[0])  fly_num = 21;
    if(isWarring[1])  hole_red_num = 21;
    if(isWarring[2])  hole_orange_num = 21;
    if(isWarring[3])  windmill_num = 21;
    if(isWarring[4])  dart_num = 9;
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
