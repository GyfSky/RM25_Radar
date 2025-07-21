#include "../include/Radar.h"

PictureSource MyRadar::getPictureSource() {
    return this->Modes_ptr->pictureSource;
}

MyRadar::MyRadar(rclcpp::Node::SharedPtr node){
    this->node = node;
    after = 4000;bafter = after;int start = 0;

    for (int i=0;i<5;i++) {
        lidar_det.blue_x[i]=0.0;
        lidar_det.blue_y[i]=0.0;
        lidar_det.red_x[i]=0.0;
        lidar_det.red_y[i]=0.0;
        lidar_enhance_.red_enhance[i]=false;
        lidar_enhance_.blue_enhance[i]=false;
    }

    this->Modes_ptr = std::shared_ptr<Modes>(new Modes());

    if (this->Modes_ptr->camNumber==1) this->is_one_cam=true;
    else if (this->Modes_ptr->camNumber==2) this->is_one_cam=false;

    this->MainMapGraph_ptr = std::shared_ptr<MapGraphMtx>(new MapGraphMtx(Modes_ptr->ourPattern));

    this->KRepresent_ptr = std::shared_ptr<KRepresent>(new KRepresent());//疑似没有用到
    // this->Livox_ptr = std::shared_ptr<Livox>(new Livox());
    //网络相关
    this->MainCam_Net_ptr   = std::shared_ptr<Net>(new Net("net_60"));
    this->Armor_Net_ptr   = std::shared_ptr<Net>(new Net("net_armor"));

    std::cout << "start_SensorParam" << std::endl;

    this->MainCam_ptr = std::shared_ptr<SensorParam>(new SensorParam("Hik60",CamPosition::left,Modes_ptr->ourPattern));
    // this->MainCam_ptr = std::shared_ptr<SensorParam>(new SensorParam("TDT",CamPosition::left,Modes_ptr->ourPattern));
    //MainCam_ptr->K 将 MainCam_ptr的内参赋值给 K
    this->Lidar_ptr = std::shared_ptr<SensorParam>(new SensorParam("Livox",MainCam_ptr->K,CamPosition::right,Modes_ptr->ourPattern));

    this->costMatrix_ptr  = std::shared_ptr<CostMatrix>(new CostMatrix(Modes_ptr->ourPattern));

    //获取图像
    this->MainCam_Image_ptr = std::shared_ptr<Image>(
        new Image(Modes_ptr->application,Modes_ptr->pictureSource, "DA0926631",node.get(), "Hik60", Modes_ptr->isSave, disk02,
                      start));

    // this->CoordSolve_ptr  = std::shared_ptr<CoordSolver>(new CoordSolver(Modes_ptr->ourPattern));//英雄吊射？？

    if(!is_one_cam){
        this->SecMapGraph_ptr = std::shared_ptr<MapGraphMtx>(new MapGraphMtx(Modes_ptr->ourPattern));
        this->SecCam_Net_ptr   = std::shared_ptr<Net>(new Net("net"));
            std::this_thread::sleep_for(std::chrono::milliseconds (10));
        this->SecCam_ptr = std::shared_ptr<SensorParam>(new SensorParam("Hik30",CamPosition::right,Modes_ptr->ourPattern));
        this->SecCam_Image_ptr = std::shared_ptr<Image>(
                new Image(Common,Modes_ptr->pictureSource, "00F26632053",node.get(), "Hik30", Modes_ptr->isSave, disk02,
                          start));
        this->PretreatObjs_ptr = std::shared_ptr<PretreatObjs>(new PretreatObjs(this->MainCam_ptr, this->SecCam_ptr, false));
    }else{
        this->PretreatObjs_ptr = std::shared_ptr<PretreatObjs>(new PretreatObjs(Modes_ptr->ourPattern));
    }

    this->CooSystem_ptr = std::shared_ptr<MatrixCoordinateSystem>(new MatrixCoordinateSystem(PretreatObjs_ptr->classWithoutCar));//坐标转换
    this->classWithoutCar=PretreatObjs_ptr->classWithoutCar;
    this->BYTETracker_ptr = std::shared_ptr<BYTETracker>(new BYTETracker(Modes_ptr->ourPattern,this->CooSystem_ptr));

    //串口
    this->Port_ptr = std::shared_ptr<Port>(new Port(Modes_ptr->ourPattern, PretreatObjs_ptr->half_classWithoutCar, Modes_ptr->Port_isOpen, Modes_ptr->usePort,node.get()));

    hero_location1_= Modes_ptr->ourPattern == red? cv::Point3d(17.5392,4.1475,0.0):cv::Point3d(10.4608,10.8525,0.0);
    hero_location2_= Modes_ptr->ourPattern == red? cv::Point3d(18.132,11.349,0.0):cv::Point3d(9.868,3.651,0.0);
    hero_location3_= Modes_ptr->ourPattern == red? cv::Point3d(21.29,3.31,0.0):cv::Point3d(6.71,11.69,0.0);
    buff_location_= Modes_ptr->ourPattern == red? cv::Point3d(20.649,1.657,0.0):cv::Point3d(7.351,13.343,0.0);
    engineer_location1_= Modes_ptr->ourPattern == red? cv::Point3d(19.1,8.7,0.0):cv::Point3d(8.9,6.3,0.0);
    engineer_location2_= Modes_ptr->ourPattern == red? cv::Point3d(19.1,6.3,0.0):cv::Point3d(8.9,8.7,0.0);
    fortress_location_= Modes_ptr->ourPattern == red? cv::Point3d(21.4,7.5,0.0):cv::Point3d(6.6,7.5,0.0);
    supply_location_= Modes_ptr->ourPattern == red? cv::Point3d(25.6,13.2,0.0):cv::Point3d(2.4,1.8,0.0);
    self_central_heights_[0]= Modes_ptr->ourPattern == red? cv::Point2f(13.65,2.0):cv::Point2f(14.35,13.0);
    self_central_heights_[1]= Modes_ptr->ourPattern == red? cv::Point2f(9.5655,2.0):cv::Point2f(28-9.5655,13.0);
    self_central_heights_[2]= Modes_ptr->ourPattern == red? cv::Point2f(10.4689,3.2902):cv::Point2f(28-10.4689,15-3.2902);
    self_central_heights_[3]= Modes_ptr->ourPattern == red? cv::Point2f(10.4689,4.0118):cv::Point2f(28-10.4689,15-4.0118);
    self_central_heights_[4]= Modes_ptr->ourPattern == red? cv::Point2f(9.535,6.0641):cv::Point2f(28-9.535,8.9359);
    self_central_heights_[5]= Modes_ptr->ourPattern == red? cv::Point2f(9.535,8.9359):cv::Point2f(28-9.535,6.0641);
    self_central_heights_[6]= Modes_ptr->ourPattern == red? cv::Point2f(12.043,12.759):cv::Point2f(28-12.043,15-12.759);
    self_central_heights_[7]= Modes_ptr->ourPattern == red? cv::Point2f(13.65,13.0):cv::Point2f(14.35,2.0);


    //test
    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    std::string win_name = config["Livox"]["winname"].as<std::string>();
    std::cout << "---------------------make is ok1------------------" << std::endl;

}

MyRadar::~MyRadar(){

}
void MyRadar::STrackInit(int classWithoutCar, OurPattern ourPattern){
    out_init.resize(classWithoutCar);
    if (classWithoutCar==12) {
        // TODO: 需要跟据yaml的改变而改变？？
        out_init[0 ] = *new STrack(-1,25.80,8.0); //B1
        out_init[1 ] = *new STrack(-1,20.00,4.0); //B2
        out_init[2 ] = *new STrack(-1,26.75,7.5); //B3
        out_init[3 ] = *new STrack(-1,26.75,7.5); //B4
        out_init[4 ] = *new STrack(-1,26.75,7.5); //B5
        out_init[5 ] = *new STrack(-1,23.00,7.5); //B7

        out_init[6 ] = *new STrack(-1,2.20,7.0); //R1
        out_init[7 ] = *new STrack(-1,8.00,11.); //R2
        out_init[8 ] = *new STrack(-1,1.25,7.5); //R3
        out_init[9 ] = *new STrack(-1,1.25,7.5); //R4
        out_init[10] = *new STrack(-1,1.25,7.5); //R5
        out_init[11] = *new STrack(-1,5.00,7.5); //R7


        out_init[0 ] = *new STrack(-1,0.1,0.1); //B1
        out_init[1 ] = *new STrack(-1,0.1,0.1); //B2
        out_init[2 ] = *new STrack(-1,0.1,0.1); //B3
        out_init[3 ] = *new STrack(-1,0.1,0.1); //B4
        out_init[4 ] = *new STrack(-1,0.1,0.1); //B5
        out_init[5 ] = *new STrack(-1,23.00,7.5); //B7

        out_init[6 ] = *new STrack(-1,0.1,0.1); //R1
        out_init[7 ] = *new STrack(-1,0.1,0.1); //R2
        out_init[8 ] = *new STrack(-1,0.1,0.1); //R3
        out_init[9 ] = *new STrack(-1,0.1,0.1); //R4
        out_init[10] = *new STrack(-1,0.1,0.1); //R5
        out_init[11] = *new STrack(-1,5.00,7.5); //R7

        if(ourPattern == red){
            this->color_index = 0;
            windmill_car = {2, 3, 4};   //B3, B4, B5
        }else if(ourPattern == blue){
            this->color_index = classWithoutCar/2;
            windmill_car = {8, 9, 10};  //R3, R4, R5
        }

    }else if (classWithoutCar==10) {
        // TODO: 需要跟据yaml的改变而改变？？
        out_init[0 ] = *new STrack(-1,25.80,8.0); //B1
        out_init[1 ] = *new STrack(-1,20.00,4.0); //B2
        out_init[2 ] = *new STrack(-1,26.75,7.5); //B3
        out_init[3 ] = *new STrack(-1,26.75,7.5); //B4
        out_init[4 ] = *new STrack(-1,23.00,7.5); //B7

        out_init[5 ] = *new STrack(-1,2.20,7.0); //R1
        out_init[6 ] = *new STrack(-1,8.00,11.); //R2
        out_init[7 ] = *new STrack(-1,1.25,7.5); //R3
        out_init[8 ] = *new STrack(-1,1.25,7.5); //R4
        out_init[9] = *new STrack(-1,5.00,7.5); //R7


        out_init[0 ] = *new STrack(-1,0.1,0.1); //B1
        out_init[1 ] = *new STrack(-1,0.1,0.1); //B2
        out_init[2 ] = *new STrack(-1,0.1,0.1); //B3
        out_init[3 ] = *new STrack(-1,0.1,0.1); //B4
        out_init[4 ] = *new STrack(-1,24.00,7.5); //B7

        out_init[5 ] = *new STrack(-1,0.1,0.1); //R1
        out_init[6 ] = *new STrack(-1,0.1,0.1); //R2
        out_init[7 ] = *new STrack(-1,0.1,0.1); //R3
        out_init[8 ] = *new STrack(-1,0.1,0.1); //R4
        out_init[9] = *new STrack(-1,4.00,7.5); //R7

        if(ourPattern == red){
            this->color_index = 0;
            windmill_car = {2, 3};   //B3, B4
        }else if(ourPattern == blue){
            this->color_index = classWithoutCar/2;
            windmill_car = {7, 8};  //R3, R4
        }
    }

    this->ourPattern = ourPattern;
}
//猜敌方车辆在哪
void MyRadar::STrackGuess(int classWithoutCar){
    // if(this->out[color_index].cls == color_index)     // 英雄
    // {
    //     //标记进度为零并且连续丢失帧超过75帧
    //     if(this->out[color_index].judge_radar_mark_data == 0 && this->out[1+color_index].lost_frame_ind_num > 16){
    //         if(ourPattern == red) {
    //             std::array<cv::Point2f, 25> arr2 = {cv::Point2f(15.5988,2.714),cv::Point2f(16.4414,1.8841),cv::Point2f(18.3454,4.1387),cv::Point2f(17.4428,4.706)};
    //             if (initornot(arr2,cv::Point(this->out[color_index].Locate3D.x,this->out[color_index].Locate3D.y),4)!= -1) {
    //                 this->out[color_index].Locate3D = {17.5392,4.1475,0.0};
    //                 // this->out[color_index].hero_enhance1=true;
    //             }
    //         }else if(ourPattern == blue) {
    //             std::array<cv::Point2f, 25> arr2 = {cv::Point2f(28-15.5988,15-2.714),cv::Point2f(28-16.4414,15-1.8841),cv::Point2f(28-18.3454,15-4.1387),cv::Point2f(28-17.4428,15-4.706)};
    //             if (initornot(arr2,cv::Point(this->out[color_index].Locate3D.x,this->out[color_index].Locate3D.y),4)!= -1) {
    //                 this->out[color_index].Locate3D = {10.4608,10.8525,0.0};
    //                 // this->out[color_index].hero_enhance1=true;
    //             }
    //         }
    //     }
    // }
    int lidar_enhance[5];
    for (int i=0;i<5;i++) {
        if (ourPattern==red) {
            lidar_enhance[i]=lidar_enhance_.blue_enhance[i];
        }else if (ourPattern==blue) {
            lidar_enhance[i]=lidar_enhance_.red_enhance[i];
        }
    }
    //猜英雄
    if(this->out[color_index].lost_frame_ind_num > 16&&lidar_enhance[0]!=1&&lidar_enhance[0]!=5&&lidar_enhance[0]!=6) {
        //裁判系统标记进度为0
        if (this->out[color_index].judge_radar_mark_data == 0) {
            if (hero_guess_1_==false&&hero_guess_2_==false) {
                hero_guess_1_=true;
                this->out[color_index].Locate3D = hero_location1_;
                hero_time_1_++;
            }else if (hero_guess_1_==true) {
                if (hero_time_1_>0&&hero_time_1_<40) {
                    hero_time_1_++;
                    this->out[color_index].Locate3D = hero_location1_;
                }else if (hero_time_1_==40) {//换吊射点2猜
                    hero_time_1_=0;
                    hero_time_2_++;
                    hero_guess_2_=true;
                    hero_guess_1_=false;
                    this->out[color_index].Locate3D = hero_location2_;
                }
            }else if (hero_guess_2_==true) {
                if (hero_time_2_>0&&hero_time_2_<40) {
                    hero_time_2_++;
                    this->out[color_index].Locate3D = hero_location2_;
                }else if (hero_time_2_==40) {//换吊射点1猜
                    hero_time_2_=0;
                    hero_time_1_++;
                    hero_guess_1_=true;
                    hero_guess_2_=false;
                    this->out[color_index].Locate3D = hero_location1_;
                }
            }
        }else {//裁判系统标记进度为1
            double distance1 =get2Ddistance(this->out[color_index].Locate3D.x,this->out[color_index].Locate3D.y,hero_location1_.x,hero_location1_.y);
            double distance2 =get2Ddistance(this->out[color_index].Locate3D.x,this->out[color_index].Locate3D.y,hero_location2_.x,hero_location2_.y);
            if (distance1<distance2) {
                hero_guess_1_=true;
                hero_time_1_=30;
                hero_guess_2_=false;
                hero_time_2_=0;
                this->out[color_index].Locate3D = hero_location1_;
            }else {
                hero_guess_2_=true;
                hero_time_2_=30;
                hero_guess_1_=false;
                hero_time_1_=0;
                this->out[color_index].Locate3D = hero_location2_;
            }
        }
    }else {
        hero_guess_1_=false;
        hero_guess_2_=false;
        hero_time_1_=0;
        hero_time_2_=0;
    }

    // if(this->out[color_index].lost_frame_ind_num > 16&&!lidar_enhance[0]) {
    //     //裁判系统标记进度为0
    //     if (this->out[color_index].judge_radar_mark_data == 0) {
    //         if (game_time_<30.0) {
    //             this->out[color_index].Locate3D = ourPattern == red? cv::Point3d(17.5392,4.1475,0.0):cv::Point3d(10.4608,10.8525,0.0);
    //         }else if (hero_guess_1_==true) {
    //             this->out[color_index].Locate3D = ourPattern == red? cv::Point3d(18.132,11.349,0.0):cv::Point3d(9.868,3.651,0.0);
    //         }else if (hero_guess_2_==true) {
    //             this->out[color_index].Locate3D = ourPattern == red? cv::Point3d(18.132,11.349,0.0):cv::Point3d(9.868,3.651,0.0);
    //         }
    //     }else {//裁判系统标记进度为1
    //         cv::Point3d hero_location1=ourPattern == red? cv::Point3d(17.5392,4.1475,0.0):cv::Point3d(10.4608,10.8525,0.0);
    //         cv::Point3d hero_location2=ourPattern == red? cv::Point3d(18.132,11.349,0.0):cv::Point3d(9.868,3.651,0.0);
    //         double distance1 =get2Ddistance(this->out[color_index].Locate3D.x,this->out[color_index].Locate3D.y,hero_location1.x,hero_location1.y);
    //         double distance2 =get2Ddistance(this->out[color_index].Locate3D.x,this->out[color_index].Locate3D.y,hero_location2.x,hero_location2.y);
    //         if (distance1<distance2) {
    //             this->out[color_index].Locate3D = hero_location1;
    //         }else {
    //             this->out[color_index].Locate3D = hero_location2;
    //         }
    //     }
    // }

    //猜工程
    if(this->out[color_index+1].lost_frame_ind_num > 46&&lidar_enhance[1]!=1&&lidar_enhance[1]!=5) {
        //裁判系统标记进度为0
        if (this->out[color_index+1].judge_radar_mark_data == 0) {
            if (engineer_guess_1_==false&&engineer_guess_2_==false) {
                engineer_guess_1_=true;
                this->out[color_index+1].Locate3D = engineer_location1_;
                engineer_time_1_++;
            }else if (engineer_guess_1_==true) {
                if (engineer_time_1_>0&&engineer_time_1_<40) {
                    engineer_time_1_++;
                    this->out[color_index+1].Locate3D = engineer_location1_;
                }else if (engineer_time_1_==40) {//换点2猜
                    engineer_time_1_=0;
                    engineer_time_2_++;
                    engineer_guess_2_=true;
                    engineer_guess_1_=false;
                    this->out[color_index+1].Locate3D = engineer_location2_;
                }
            }else if (engineer_guess_2_==true) {
                if (engineer_time_2_>0&&engineer_time_2_<40) {
                    engineer_time_2_++;
                    this->out[color_index+1].Locate3D = engineer_location2_;
                }else if (engineer_time_2_==40) {//换点1猜
                    engineer_time_2_=0;
                    engineer_time_1_++;
                    engineer_guess_1_=true;
                    engineer_guess_2_=false;
                    this->out[color_index+1].Locate3D = engineer_location1_;
                }
            }
        }else {//裁判系统标记进度为1
            double distance1 =get2Ddistance(this->out[color_index+1].Locate3D.x,this->out[color_index+1].Locate3D.y,engineer_location1_.x,hero_location1_.y);
            double distance2 =get2Ddistance(this->out[color_index+1].Locate3D.x,this->out[color_index+1].Locate3D.y,engineer_location2_.x,hero_location2_.y);
            if (distance1<distance2) {
                engineer_guess_1_=true;
                engineer_time_1_=30;
                engineer_guess_2_=false;
                engineer_time_2_=0;
                this->out[color_index+1].Locate3D = engineer_location1_;
            }else {
                engineer_guess_2_=true;
                engineer_time_2_=30;
                engineer_guess_1_=false;
                engineer_time_1_=0;
                this->out[color_index+1].Locate3D = engineer_location2_;
            }
        }
    }else {
        engineer_guess_1_=false;
        engineer_guess_2_=false;
        engineer_time_1_=0;
        engineer_time_2_=0;
    }

    //猜哨兵
    int sentry_index=classWithoutCar/2-1;
    if(this->out[sentry_index+color_index].lost_frame_ind_num >46&&lidar_enhance[4]!=1){
        if (game_time_>=3&&game_time_<=15) {
            this->out[color_index+sentry_index].Locate3D = buff_location_;
        }else {
            this->out[color_index+sentry_index].Locate3D = fortress_location_;
        }
    }

    //猜3号
    if(this->out[color_index+2].lost_frame_ind_num > 46&&lidar_enhance[2]!=1) {
        //裁判系统标记进度为0
        if (game_time_>=7&&game_time_<=20) {
            this->out[color_index+2].Locate3D = buff_location_;
        }else {
            this->out[color_index+2].Locate3D = supply_location_;
        }
    }

    //猜4号
    if(this->out[color_index+3].lost_frame_ind_num > 46&&lidar_enhance[3]!=1) {
        //裁判系统标记进度为0
        if (game_time_>=7&&game_time_<=20) {
            this->out[color_index+3].Locate3D = buff_location_;
        }else {
            this->out[color_index+3].Locate3D = supply_location_;
        }
    }

    // if(this->out[2+color_index].cls == -1&&lidar_enhance[2]!=1)     // 3
    // {
    //     if(this->out[2+color_index].judge_radar_mark_data == 0 && this->out[5+color_index].lost_frame_ind_num > 46){
    //         if(ourPattern == red)
    //             this->out[2+color_index].Locate3D = {25.7157,13.5559,0.0};//补给区
    //         else if(ourPattern == blue)
    //             this->out[2+color_index].Locate3D = {2.2843,1.4441,0.0};
    //
    //     }
    // }
    // if(this->out[3+color_index].cls == -1&&lidar_enhance[3]!=1)     // 4
    // {
    //     if(this->out[3+color_index].judge_radar_mark_data == 0 && this->out[5+color_index].lost_frame_ind_num > 46){
    //         if(ourPattern == red)
    //             this->out[3+color_index].Locate3D = {25.7157,13.5559,0.0};//补给区
    //         else if(ourPattern == blue)
    //             this->out[3+color_index].Locate3D = {2.2843,1.4441,0.0};
    //
    //     }
    // }
}

void MyRadar::STrackClear(){
    for(auto &track: out){
        if(track.cls == -1){
            track.lost_frame_ind_num++;//对没跟踪的车辆进行丢失帧数加1
        }else{
            track.cls = -1;
        }
        track.Locate3D = {0.0,0.0,0.0};
        track.is_det=false;
    }
    for(auto &track: to_sentry){
        if(track.cls == -1){
            track.lost_frame_ind_num++;//对没跟踪的车辆进行丢失帧数加1
        }
        track.Locate3D = {0.0,0.0,0.0};
        track.vx_3d = 0.0;
        track.vy_3d = 0.0;
        track.is_det=false;
    }
}

void MyRadar::getDartWarning(cv::Mat img,int value) {
    cv::Mat temp=img(rect);
    cv::Mat rect_img=temp.clone();
    cv::cvtColor(rect_img,rect_img,CV_BGR2GRAY);
    cv::threshold(rect_img, rect_img, value, 255, cv::THRESH_BINARY);
    cv::Mat contourImage = cv::Mat::zeros(rect_img.size(), CV_8UC3);
    if (rclcpp::Clock().now().seconds()-dart_time.seconds()<22) {
        dart_center.clear();
        cv::putText(contourImage, "Dart", cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        RCLCPP_ERROR(node->get_logger(), "--Dart Detect!!!");
    }else {
        dart_flag=false;
        vector<vector<cv::Point>> contours;
        _OutputArray hierarchy;
        cv::findContours(rect_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        for (auto contour: contours) {
            auto minRect=cv::minAreaRect(contour);
            RCLCPP_ERROR(node->get_logger(), "Dart area: %f",minRect.size.area());
            RCLCPP_ERROR(node->get_logger(), "Dart height: %f, Dart width: %f",minRect.size.height,minRect.size.width);
            cv::Size2d rectSize = minRect.size;
            float width = rectSize.width;
            float height = rectSize.height;
            if (height/width > 1.2|| width/height >1.2||height*width < 20) continue;
            cv::Point2f vertices[4];
            minRect.points(vertices);
            auto center=minRect.center;
            dart_center.push_back(center);
            if (dart_center.size()>20) dart_center.erase(dart_center.begin());
            std::vector<cv::Point> box(vertices, vertices + 4);
            cv::polylines(contourImage, contour, true, cv::Scalar(0, 255, 255), 1);
            cv::polylines(contourImage, box, true, cv::Scalar(0, 255, 0), 1);
        }
        if (dart_center.empty())return;
        if (dart_center.front().y-dart_center.back().y>double(rect.height)/3.0) {
            dart_time=rclcpp::Clock().now();
            dart_flag=true;
            cv::circle(contourImage, dart_center.back(), 5, cv::Scalar(0, 0, 255), -1);
        }
    }
    cv::imshow("Dart", rect_img);
    cv::imshow("Dart1", contourImage);
}

void MyRadar::getRivalOffenseWarning(vector<bool> &isWarring) {
    for (int i=0;i<5;i++) {
        cv::Point2d location=cv::Point2d(this->out[i+color_index].Locate3D.x,this->out[i+color_index].Locate3D.y);
        if (location.x<=0.0||location.x>=28.0||location.y<=0.0||location.y>=15.0) continue;
        if (Modes_ptr->ourPattern==red&&location.x<13.65&&initornot(self_central_heights_, location,8)==-1) {
            rival_offense_time[i]=rival_offense_time[i]<10?rival_offense_time[i]+1:12;
        }else if (Modes_ptr->ourPattern==blue&&location.x>14.35&&initornot(self_central_heights_, location,8)==-1) {
            rival_offense_time[i]=rival_offense_time[i]<10?rival_offense_time[i]+1:12;
        }else {
            if (rival_offense_time[i]>0)
                rival_offense_time[i]=rival_offense_time[i]-1;
        }
    }
    for (int i=0;i<5;i++) {
        if (rival_offense_time[i]>=10) {
            isWarring[5]=true;
            break;
        }
    }
}

void MyRadar::Init(int argc, char **argv){
    if(is_init) return;
    MainCam_Image_ptr->Init(argc, argv);
    if(!is_one_cam){
        SecCam_Image_ptr->Init(argc, argv);
    }

    // Livox_ptr->init(argc, argv);


    // UnityRect_ptr->init(argc, argv);

// getBackgroundDepthImg
    //  while (ros::ok()){
    //      if(Livox_ptr->is_getBackgroundDepthImg){
    //          ros::spinOnce();
    //          // Livox_ptr->ShowDepthMat();
    //      }
    //      else if(Livox_ptr->is_getPoint2d_mouse_liovx){
    //          break;
    //      }
    //  }

//    std::vector<cv::Point3d> pts_pnp_3d;
//    pts_pnp_3d.emplace_back(cv::Point3d(9.5500, 0.0000, 0.40));
//    pts_pnp_3d.emplace_back(cv::Point3d(8.4100, 4.3670, 1.10));
//    pts_pnp_3d.emplace_back(cv::Point3d(7.1600, 4.3670, 1.10));
//    pts_pnp_3d.emplace_back(cv::Point3d(5.1390, 2.2020, 0.00));
//    pts_pnp_3d.emplace_back(cv::Point3d(6.5870, 6.7770, 0.6));
//    pts_pnp_3d.emplace_back(cv::Point3d(9.4100, 5.5000, 0.5));

    std::vector<cv::Point2d> pts_pnp_2d_main, pts_pnp_2d_sec;

//     pts_pnp_2d_main.emplace_back(cv::Point2d(198 ,1137));
//     pts_pnp_2d_main.emplace_back(cv::Point2d(117 ,376 ));
//     pts_pnp_2d_main.emplace_back(cv::Point2d(197 ,280 ));
//     pts_pnp_2d_main.emplace_back(cv::Point2d(1588,217 ));
//     pts_pnp_2d_main.emplace_back(cv::Point2d(1552,322 ));
//     pts_pnp_2d_main.emplace_back(cv::Point2d(2076,441 ));//

//     pts_pnp_2d_main.emplace_back(cv::Point2d(881,443 ));
//     pts_pnp_2d_main.emplace_back(cv::Point2d(771,445 ));
//     pts_pnp_2d_main.emplace_back(cv::Point2d(266,179 ));
//     pts_pnp_2d_main.emplace_back(cv::Point2d(314,126 ));
//     pts_pnp_2d_main.emplace_back(cv::Point2d(1258,133));
//     pts_pnp_2d_main.emplace_back(cv::Point2d(1096,384));//
//

//    pts_pnp_2d_main.emplace_back(cv::Point2d(335,109));
//    pts_pnp_2d_main.emplace_back(cv::Point2d(584,171));
//    pts_pnp_2d_main.emplace_back(cv::Point2d(1044,151));
//    pts_pnp_2d_main.emplace_back(cv::Point2d(1357,273));
//    pts_pnp_2d_main.emplace_back(cv::Point2d(1731,705));
//    pts_pnp_2d_main.emplace_back(cv::Point2d(158,606));//7
//
    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    bool use_saved_T = config["general"]["cam_use_saved_T"].as<bool>();
    if(MainCam_Image_ptr->is_getPoint2d_mouse_Cam ){

        while(mainCamMat.empty() ){
            mainCamMat = MainCam_Image_ptr->Image_Get(after,argc,argv);//after 为图片序号，通过加减after选择不同图片
        }
        MainCam_Image_ptr->Image_Show();

        // ROS_INFO("step1");
        std::cout<<"---step1---"<<std::endl;
        if(use_saved_T){
            std::ifstream fin("/home/thesky/RM25_Radar/resource/main2world.txt");
            if (!fin) {
                RCLCPP_ERROR(node->get_logger(),"file cant open!!!");
                RCLCPP_ERROR(node->get_logger(),"use mautually!!!");
                use_saved_T=false;
            }else{
                char buf[1024]={0};
                int num=0;
                std::vector<float> T_temp;
                while(fin>>buf){
                    T_temp.push_back(atof(buf));
                    num++;
                    if(num==16) break;
                }
                if(num==16){
                    MainCam_ptr->T_2world = (cv::Mat_<float>(4, 4)<<
                        T_temp[0],T_temp[1],T_temp[2],T_temp[3],
                        T_temp[4],T_temp[5],T_temp[6],T_temp[7],
                        T_temp[8],T_temp[9],T_temp[10],T_temp[11],
                        0,0,0,1);
                }else{
                    RCLCPP_ERROR(node->get_logger(),"file broken!!!");
                    RCLCPP_ERROR(node->get_logger(),"use mautually!!!");
                    use_saved_T=false;
                }
            }
            fin.close();
            if (!use_saved_T) {
                MainCam_ptr->pts_pnp_2d = GetPoint2d_mouse(mainCamMat,"Hik60");
                CooSystem_ptr->Get2world_matrix(MainCam_ptr->T_2world  ,MainCam_ptr->K ,MainCam_ptr->pts_pnp_2d , MainCam_ptr->pts_pnp_3d);
                std::ofstream fout("/home/thesky/RM25_Radar/resource/main2world.txt");
                if(!fout)
                    RCLCPP_ERROR(node->get_logger(),"file cant open!!!");
                else {
                    for(int i=0;i<4;i++){
                        for(int j=0;j<4;j++)
                            fout<<MainCam_ptr->T_2world.at<float>(i,j)<<" ";
                        fout<<std::endl;
                    }
                    fout<<"------------------"<<std::endl;
                    fout.close();
                }
            }
        }else{
            MainCam_ptr->pts_pnp_2d = GetPoint2d_mouse(mainCamMat,"Hik60");
            CooSystem_ptr->Get2world_matrix(MainCam_ptr->T_2world  ,MainCam_ptr->K ,MainCam_ptr->pts_pnp_2d , MainCam_ptr->pts_pnp_3d);
            std::ofstream fout("/home/thesky/RM25_Radar/resource/main2world.txt");
            if(!fout)
                RCLCPP_ERROR(node->get_logger(),"file cant open!!!");
            else {
                for(int i=0;i<4;i++){
                    for(int j=0;j<4;j++)
                        fout<<MainCam_ptr->T_2world.at<float>(i,j)<<" ";
                    fout<<std::endl;
                }
                fout<<"------------------"<<std::endl;
                fout.close();
            }
        }
//        MainCam_ptr->pts_pnp_2d = pts_pnp_2d_main;
//        SecCam_ptr->pts_pnp_2d = pts_pnp_2d_sec;
        // ROS_INFO("step2");
        std::cout<<"---step2---"<<std::endl;
        std::cout<<MainCam_ptr->K<<std::endl;

        // cv::Mat dist=(cv::Mat_<double>(1, 5)<<
        //     -0.073959250184369399,0.15749895129489391,-0.00021183318242317523,
        //     -0.00099856292005157175,-0.095662634873975388);
        // //得到xxx2world 的旋转矩阵
        // cv::Mat Ticp=(cv::Mat_<double>(4, 4)<<
        //     0.960281,0.0843165,0.26599,-1.22638,
        //     -0.0822921,0.996432,-0.0187677,9.67522,
        //     -0.266623,-0.00386661,0.963793,3.90987,
        //     0,0,0,1);
        //
        // cv::Mat Tp=(cv::Mat_<double>(4, 4)<<
        //     0.00728,-0.01686,0.99983,0.06922,
        //     -0.99978,0.01956,0.00761,0.04613,
        //     -0.01968,-0.99967,-0.01672,0.00160,
        //     0,0,0,1);
        // MainCam_ptr->T_2world=Tp.inv()*Ticp.inv();
        // MainCam_ptr->T_2world=MainCam_ptr->T_2world.inv();
        // CooSystem_ptr->Get2world_matrix(MainCam_ptr->T_2world  ,MainCam_ptr->K ,MainCam_ptr->pts_pnp_2d , MainCam_ptr->pts_pnp_3d);
        std::cout<<MainCam_ptr->T_2world<<std::endl;
        std::cout<<"得到旋转矩阵"<<std::endl;
        // ROS_INFO("step3");
        std::cout<<"---step3---"<<std::endl;

//        CooSystem_ptr->Get2world_matrix(MainCam_ptr->T_2world  ,MainCam_ptr->K ,MainCam_ptr->pts_pnp_2d , pts_pnp_3d);
////        CooSystem_ptr->Get2world_matrix(SecCam_ptr->T_2world  ,SecCam_ptr->K ,SecCam_ptr->pts_pnp_2d , pts_pnp_3d);
//        CooSystem_ptr->Get2world_matrix(SecCam_ptr->T_2world  ,SecCam_ptr->K ,SecCam_ptr->pts_pnp_2d , SecCam_ptr->pts_pnp_3d);

        MainCam_Image_ptr->is_getPoint2d_mouse_Cam = false;
        //利用转化关系将场地的3d点转成图像中的2d点
        MainMapGraph_ptr->get_predict_2d(MainCam_ptr->T_2world ,MainCam_ptr->fx,MainCam_ptr->fy,MainCam_ptr->cx,MainCam_ptr->cy);
        MainMapGraph_ptr->get_roughH_config();//？？
        MainCam_ptr->setworld2self_config(MainCam_ptr->T_2world);
    }else{//此处可以考虑引入激光雷达间接获得坐标转换关系
        std::cerr << "error" << std::endl;
    }
    rect=cv::Rect(cv::Point(0,0),cv::Point(1,1));


    if(!is_one_cam && SecCam_Image_ptr->is_getPoint2d_mouse_Cam){

        while(secCamMat.empty() ){
            secCamMat = SecCam_Image_ptr->Image_Get(after,argc,argv);
        }
        SecCam_Image_ptr->Image_Show();

        // ROS_INFO("step1");
        std::cout<<"---step1---"<<std::endl;
        if(use_saved_T){
            std::ifstream fin("/home/thesky/RM25_Radar/resource/sec2world&rect.txt");
            if (!fin) {
                RCLCPP_ERROR(node->get_logger(),"file cant open!!!");
                RCLCPP_ERROR(node->get_logger(),"use mautually!!!");
                use_saved_T=false;
            }else{
                char buf[1024]={0};
                int num=0;
                std::vector<float> T_temp;
                while(fin>>buf){
                    T_temp.push_back(atof(buf));
                    num++;
                    if(num==20) break;
                }
                if(num==20){
                    SecCam_ptr->T_2world = (cv::Mat_<float>(4, 4)<<
                        T_temp[0],T_temp[1],T_temp[2],T_temp[3],
                        T_temp[4],T_temp[5],T_temp[6],T_temp[7],
                        T_temp[8],T_temp[9],T_temp[10],T_temp[11],
                        0,0,0,1);
                    rect=cv::Rect(T_temp[16],T_temp[17],T_temp[18],T_temp[19]);
                }else{
                    RCLCPP_ERROR(node->get_logger(),"file broken!!!");
                    RCLCPP_ERROR(node->get_logger(),"use mautually!!!");
                    use_saved_T=false;
                }
            }
            fin.close();
            if (!use_saved_T) {
                rect =GetRect_mouse(secCamMat,"Hik60");
                SecCam_ptr->pts_pnp_2d = GetPoint2d_mouse(secCamMat,"Hik60");
                CooSystem_ptr->Get2world_matrix(SecCam_ptr->T_2world  ,SecCam_ptr->K ,SecCam_ptr->pts_pnp_2d , SecCam_ptr->pts_pnp_3d);
                std::ofstream fout("/home/thesky/RM25_Radar/resource/sec2world&rect.txt");
                if(!fout)
                    RCLCPP_ERROR(node->get_logger(),"file cant open!!!");
                else {
                    for(int i=0;i<4;i++){
                        for(int j=0;j<4;j++)
                            fout<<SecCam_ptr->T_2world.at<float>(i,j)<<" ";
                        fout<<std::endl;
                    }
                    fout<<rect.x<<" "<<rect.y<<" "<<rect.width<<" "<<rect.height<<std::endl;
                    fout<<"------------------"<<std::endl;
                    fout.close();
                }
            }
        }else{
            rect =GetRect_mouse(secCamMat,"Hik60");
            SecCam_ptr->pts_pnp_2d = GetPoint2d_mouse(secCamMat,"Hik60");
            CooSystem_ptr->Get2world_matrix(SecCam_ptr->T_2world  ,SecCam_ptr->K ,SecCam_ptr->pts_pnp_2d , SecCam_ptr->pts_pnp_3d);
            std::ofstream fout("/home/thesky/RM25_Radar/resource/sec2world&rect.txt");
            if(!fout)
                RCLCPP_ERROR(node->get_logger(),"file cant open!!!");
            else {
                for(int i=0;i<4;i++){
                    for(int j=0;j<4;j++)
                        fout<<SecCam_ptr->T_2world.at<float>(i,j)<<" ";
                    fout<<std::endl;
                }
                fout<<rect.x<<" "<<rect.y<<" "<<rect.width<<" "<<rect.height<<std::endl;
                fout<<"------------------"<<std::endl;
                fout.close();
            }
        }
        // rect =GetRect_mouse(secCamMat,"Hik30");
        // SecCam_ptr->pts_pnp_2d = GetPoint2d_mouse(secCamMat,"Hik30");
//        SecCam_ptr->pts_pnp_2d = pts_pnp_2d_sec;
        // ROS_INFO("step2");
        std::cout<<"---step2---"<<std::endl;
        // CooSystem_ptr->Get2world_matrix(SecCam_ptr->T_2world  ,SecCam_ptr->K ,SecCam_ptr->pts_pnp_2d , SecCam_ptr->pts_pnp_3d);
        // ROS_INFO("step3");
        std::cout<<"---step3---"<<std::endl;

//        CooSystem_ptr->Get2world_matrix(SecCam_ptr->T_2world  ,SecCam_ptr->K ,SecCam_ptr->pts_pnp_2d , pts_pnp_3d);
////        CooSystem_ptr->Get2world_matrix(SecCam_ptr->T_2world  ,SecCam_ptr->K ,SecCam_ptr->pts_pnp_2d , pts_pnp_3d);
//        CooSystem_ptr->Get2world_matrix(SecCam_ptr->T_2world  ,SecCam_ptr->K ,SecCam_ptr->pts_pnp_2d , SecCam_ptr->pts_pnp_3d);

        SecCam_Image_ptr->is_getPoint2d_mouse_Cam = false;
        SecMapGraph_ptr->get_predict_2d(SecCam_ptr->T_2world ,SecCam_ptr->fx,SecCam_ptr->fy,SecCam_ptr->cx,SecCam_ptr->cy);
        SecMapGraph_ptr->get_roughH_config();
        SecCam_ptr->setworld2self_config(SecCam_ptr->T_2world);

    }else if(!is_one_cam){
        std::cerr << "error" << std::endl;
    }

//     if(Livox_ptr->is_getPoint2d_mouse_liovx){
//         Lidar_ptr->pts_pnp_2d = GetPoint2d_mouse(Livox_ptr->outImshowDepthMat,"Livox");
//         Livox_ptr->is_getPoint2d_mouse_liovx = false;
//         Livox_ptr->is_getPoint2d_mouse_Cam = true;
//         Livox_ptr->is_getCompetitionDepthImg = true;
//         ROS_INFO("is_getPoint2d_mouse_liovx");
//         CooSystem_ptr->Get2world_matrix(Lidar_ptr->T_2world ,Lidar_ptr->K ,Lidar_ptr->pts_pnp_2d , Lidar_ptr->pts_pnp_3d);
//         Livox_ptr->SetT_matrix(MainCam_ptr->T_2world,Lidar_ptr->T_2world);
//         Livox_ptr->spin();
//     }

    // MainCam_Image_ptr->draw_line(MainMapGraph_ptr->vexs);
    // SecCam_Image_ptr->draw_line(SecMapGraph_ptr->vexs);


    //TODO:
    //test
    // Livox_ptr->spin();
    this->Save();
    if(Port_ptr->is_openPort) {
        Port_ptr->clearBuff();
        Port_ptr->start();
    }

    this->STrackInit(PretreatObjs_ptr->classWithoutCar, Modes_ptr->ourPattern);
    PretreatObjs_ptr->set_windmill_car(this->windmill_car);//？？
    this->out.assign(this->out_init.begin(), this->out_init.end());//out为输出结果
    this->to_sentry.assign(this->out_init.begin(), this->out_init.end());
    if(Port_ptr->is_openPort){
        Port_ptr->updataSTrackData(this->out_init);
        Port_ptr->updataSentryData(this->to_sentry);
    }
    std::cout << "---------------------make is ok2------------------" << std::endl;
    is_init = true;
}


void MyRadar::Save() {
    if(Modes_ptr->isSave == true_){
        YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
        std::string path = config["save"]["save_bag_path"].as<std::string>() + getDate();
        std::string topics = config["Livox"]["lidarTopicName"].as<std::string>();

        if(Modes_ptr->pictureSource==camera_){
            MainCam_Image_ptr->setSaveMode();
            this->save_main_dir = config["save"]["save_bag_path"].as<std::string>() + getDate() + "main";
            topics+=" /cam/";
            topics+=MainCam_Image_ptr->Cam_winname;
            mkdir((this->save_main_dir).c_str(), S_IRWXU);
            if(!is_one_cam){
                SecCam_Image_ptr->setSaveMode();
                this->save_sec_dir = config["save"]["save_bag_path"].as<std::string>() + getDate() + "sec";
                topics+=" /cam/";
                topics+=SecCam_Image_ptr->Cam_winname;
                mkdir((this->save_sec_dir).c_str(), S_IRWXU);
            }
        }
        topics+=" /robot_hp";
        topics+=" /game_state";
        std::string cmd_str = "gnome-terminal -x bash -c 'ros2 bag record -o " + path + " " + topics+" '" + "&";
        int ret = system(cmd_str.c_str()); // #include <stdlib.h>
        std::cout << "cmd_str: " << cmd_str << std::endl;
        std::cout << "path: " << path << std::endl;

        if(ret != 0){
            std::cerr << "\033[33m" << "save bag may have error !!! Please check path" << "\033[0m" <<std::endl;
        }
    }
}

void MyRadar::Spin(int argc, char **argv){
    std::cout<<"new frame"<<std::endl;
    if(is_one_cam){
//        std::cout << "next step0.0.0" << std::endl;
//    cars.clear();
//    lastCars.clear();
//    armors.clear();
        this->STrackClear();
        PretreatObjs_ptr->num = 0;//？？
        DetectionObjs.clear();
        frames.clear();
//        std::cout << "next step0.0" << std::endl;
        // Livox_ptr->ShowDepthMat();
//        std::cout << "next step0" << std::endl;
        // ros::spinOnce();
//        std::cout << "next step1" << std::endl;
        int a = this->after*2;
        this->mainCamMat = MainCam_Image_ptr->Image_Get(this->after,argc,argv);
        if(Modes_ptr->pictureSource==camera_)
            this->time_now = MainCam_Image_ptr->ros_time;////实机用时间戳
        // time_now=rclcpp::Clock().now();
        int after_2 = this->after+1 ;

        if(!this->mainCamMat.empty() ){
            auto startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//            std::cout << "next step3" << std::endl;
            // MainCam_Image_ptr->draw_line(MainMapGraph_ptr->vexs);
            // SecCam_Image_ptr->draw_line(SecMapGraph_ptr->vexs);
//            std::cout << "next step4" << std::endl;
//    MainCam_Image_ptr->Image_Show();
//            std::cout << "next step5" << std::endl;
            // cv::Mat mainImg_draw = mainCamMat.clone();
            // cv::Mat mainImg_save = mainCamMat.clone();
            cv::Mat test_mat;
            // cv::resize(mainCamMat,test_mat,cv::Size(0, 0),0.6,0.6,INTER_AREA);
//            std::cout << "next step6" << std::endl;
            // if(is_first){
            //     Livox_ptr->spin();
            //     is_first = false;
            // }
//        if(cv::waitKey(1) == 'q'){
//            this->is_close = true;
//        }
            cv::createTrackbar("threshold", "Dart", &value, 255);
            if(bafter !=after){//即当after改变时，才进行下一帧的识别，否则会重复识别 后面会自动加1
                std::cout << "after: " << after << std::endl;
                std::cout << "next step7" << std::endl;
                auto now_time = std::chrono::steady_clock::now();
                getDartWarning(mainCamMat,value);

                // if (car_det.index.size()==0||armor_det.index.size()==0)return;
                // std::cout<<car_det.obj.size();
                // auto time=car_det.header.stamp;
                // vector<TRTInferV1::DetectionObj> car_list;
                // for (int i=0;i<car_det.obj.size();i++) {
                //     TRTInferV1::DetectionObj temp;
                //     temp.classId = car_det.obj[i].classid;
                //     std::cout <<temp.classId<< std::endl;
                //     temp.confidence = car_det.obj[i].confidence;
                //     std::cout <<temp.confidence<< std::endl;
                //     temp.x1 = car_det.obj[i].x1;
                //     std::cout <<temp.x1<< std::endl;
                //     temp.y1 = car_det.obj[i].y1;
                //     temp.x2 = car_det.obj[i].x2;
                //     temp.y2 = car_det.obj[i].y2;
                //     car_list.push_back(temp);
                // }
                // DetectionObjs.push_back(car_list);
                // std::cout << "next step7.1" << std::endl;
                // vector<vector<TRTInferV1::DetectionObj>> Armors;
                // int num=0;
                // for (int i=0;i<armor_det.index.size();i++) {
                //     vector<TRTInferV1::DetectionObj> temp_list;
                //     for (int j=num;j<armor_det.index[i];j++) {
                //         TRTInferV1::DetectionObj temp;
                //         temp.classId = armor_det.obj[j].classid;
                //         temp.confidence = armor_det.obj[j].confidence;
                //         temp.x1 = armor_det.obj[j].x1;
                //         temp.y1 = armor_det.obj[j].y1;
                //         temp.x2 = armor_det.obj[j].x2;
                //         temp.y2 = armor_det.obj[j].y2;
                //         temp_list.push_back(temp);
                //     }
                //     num+=armor_det.index[i];
                //     Armors.push_back(temp_list);
                // }
                // std::cout << "next step7.2" << std::endl;


//----------------------------------------------------
                // DetectionObjs = MainCam_Net_ptr->futureObjs.get();
                auto netStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                std::vector<cv::Mat> main_frames({mainCamMat});
                MainCam_Net_ptr->Spin(main_frames);
                DetectionObjs.push_back( MainCam_Net_ptr->futureObjs.get()[0]);

                std::cout << "next step8" << std::endl;

                std::vector<cv::Mat> car_imgs;
                MainCam_Net_ptr->getCarImgs({DetectionObjs[0]},mainCamMat,car_imgs);

//        Armor_Net_ptr->Spin(car_imgs);
//        vector<vector<TRTInferV1::DetectionObj>> Armors(Armor_Net_ptr->futureObjs.get());

                std::vector<cv::Mat> car_imgs_clone;//疑似没用到
                //第一维为每个车辆检测框，第二维为每个车辆检测框内的装甲板检测框
                vector<vector<TRTInferV1::DetectionObj>> Armors ;


                for(auto car_img:car_imgs){
//            Armors.push_back((Armor_Net_ptr->NetWork_mlt({car_img}))[0]);
                    std::vector<Mat> car_img_s(1);
                    car_img_s[0].push_back(car_img);
                    Armors.push_back((Armor_Net_ptr->myInfer.doInference(car_img_s,0.2, 0.2, 0.45,1))[0]);
//            Armors.push_back((Armor_Net_ptr->NetWork_mlt(car_imgs))[0]);
                }
                std::map<int,int> id_map={{0,5},{1,0},{2,1},{3,2},{4,3},
                    {5,11},{6,6},{7,7},{8,8},{9,9}};
                for (auto i=Armors.begin();i!=Armors.end();i++) {
                    for (auto j=i->begin();j!=i->end();j++) {
                        j->classId=id_map[j->classId];
                    }
                }
                // MainCam_Image_ptr->draw_test(DetectionObjs[0],Armors);
//----------------------------------------------------

//        for (int i(0); i < int(car_imgs.size()); ++i)
//        {
//            for (int j(0); j < int(Armors[i].size()); ++j)
//            {
//                cv::Rect r = cv::Rect(Armors[i][j].x1, Armors[i][j].y1, Armors[i][j].x2 - Armors[i][j].x1, Armors[i][j].y2 - Armors[i][j].y1);
//                cv::rectangle(car_imgs[i], r, cv::Scalar(255, 255, 255), 1);
//                cv::putText(car_imgs[i], std::to_string(Armors[i][j].classId), cv::Point (Armors[i][j].x1, Armors[i][j].y1),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255, 100, 255),1);
//                cv::putText(car_imgs[i], std::to_string(Armors[i][j].confidence), cv::Point (Armors[i][j].x2, Armors[i][j].y1),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(100, 100, 255),1);
//                std::cout << Armors[i][j].x1 << " " << Armors[i][j].y1 << " " << Armors[i][j].x2 << " " << Armors[i][j].y2 << std::endl;
//                std::cout << Armors[i][j].classId << "|" << Armors[i][j].confidence << std::endl;
//            }
//        }
//        for(auto img: car_imgs){
//            cv::Mat img_clone = img.clone();
//            cv::imshow("test_test", img_clone);
//            cv::waitKey(300);
//        }
                std::cout << "next step9" << std::endl;

                int cam_num = 1;
                float p =0.97;
                float match_thresh = 0.85;
                std::vector<std::vector<STrack>> stracks(cam_num);

                auto end_time = std::chrono::steady_clock::now();
                float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
                std::cout<<"\033[31m"<<"---------net time: "<<dur_time<<" ms"<<"\033[0m"<<std::endl;
                auto trackStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();


                for(int i=0; i < cam_num; i++){
                    for(auto car : DetectionObjs[i]){
                        if(i==0){
                            // car.x1=car.x1/0.6;
                            // car.x2=car.x2/0.6;
                            // car.y1=car.y1/0.6;
                            // car.y2=car.y2/0.6;
                            STrack sTrack(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), car.confidence,this->classWithoutCar);
                            //图像中的二维坐标
                            sTrack.setRectInPrimaryCam(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), p);
                            stracks[i].push_back(sTrack);
                        }else{
                            std::cout << "ERROR: IN secObjs2mainObjs about SecCam_ptr" << std::endl;
                        }
                    }
                }

                auto e1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                // std::cout << "setRectInPrimaryCam: " << e1 - trackStartTime<< std::endl;
                std::vector<bool> isWarring(6, false) ;
                if(dart_flag)
                    isWarring[4]=true;
                //此处解算得到的坐标为机器人在官方小地图中的坐标
                CooSystem_ptr->solve_reality_3d(MainCam_ptr->T_2world, MainCam_ptr->fx, MainCam_ptr->fy,
                                                MainCam_ptr->cx, MainCam_ptr->cy, MainMapGraph_ptr->vexs,MainMapGraph_ptr->arcs,stracks[0], Modes_ptr->ourPattern, isWarring);
                if(Port_ptr->is_openPort) {
                    Port_ptr->updateGameTime(this->game_time_);
                    Port_ptr->updataRadarMarkData(tracked_stracks);
                    Port_ptr->updataRadarMarkData(lost_stracks);
                    Port_ptr->updataRadarMarkData(lost_predict_stracks);
                }

                auto e2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                // std::cout << "solve_reality_3d: " << e2 - e1<< std::endl;

                int car_num = 0;
                for(int cam=0; cam<cam_num ; cam++){
                    std::cout <<  "stracks.size" << stracks[cam].size() << std::endl;
                    int cam_cars_num = stracks[cam].size();
                    std::vector<int> remove_lists;
                    for(int i=0;i<cam_cars_num;i++){
                        //更新跟踪器的cls
                        //Armors[i+car_num]为第一层网络每个检测框对应的装甲板
                        bool flag=PretreatObjs_ptr->get_Armors_w_conf_Double_net(stracks[cam][i],Armors[i+car_num],car_imgs[i+car_num]);
                        if (flag) remove_lists.push_back(i);
                        std::cout << "stracks[cam][i].Locate3D: " << stracks[cam][i].cls << " " << stracks[cam][i].Locate3D.x << " " << stracks[cam][i].Locate3D.y << std::endl;
                    }
                    for (auto i:remove_lists)
                        stracks[cam].erase(stracks[cam].begin()+i);
                    car_num += cam_cars_num;
                }
                auto e3 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                // std::cout << "get_Armors_w_conf_Double_net: " << e3 - e2<< std::endl;

                std::cout << "next step9.5" << std::endl;

                std::vector<STrack> STacks,send_data;

                STacks.insert(STacks.end(),stracks[0].begin(),stracks[0].end());

                std::vector<std::vector<float>> cost_confMatrix_vec;
                int num_strack,num_cls;
                vector<vector<int> > matches_cls;
                vector<int> u_strack, u_cls;
                matches_cls.clear();u_strack.clear();u_cls.clear();
                Eigen::MatrixXd cost_confMatrix = costMatrix_ptr->getCost_confMatrix(STacks,num_strack,num_cls);
                std::cout << "cost_confMatrix _ cls:  " << std::endl << cost_confMatrix << std::endl;
                eigenMat2VecVec(cost_confMatrix, cost_confMatrix_vec);
                costMatrix_ptr->linear_assignment(cost_confMatrix_vec, num_strack, num_cls, 0.9, matches_cls, u_strack, u_cls);

                for (int i=0; i < matches_cls.size(); i++) {
                    STacks[matches_cls[i][0]].cls = matches_cls[i][1];
                    send_data.push_back(STacks[matches_cls[i][0]]);
                }
                // MainCam_Image_ptr->draw_rusult(send_data, true);
                interfaces::msg::DetectFrame temp_res;
                interfaces::msg::DetectRes final_res;
                final_res.header.stamp = time_now;
                temp_res.header.stamp = time_now;
                for (int i = 0; i < send_data.size(); i++){
                    int cls = send_data[i].cls;
                    if(cls != -1){
                        std::vector<float> tlwh = send_data[i].tlwh;
                        bool vertical = tlwh[2] / tlwh[3] > 1.6;
                        if (tlwh[2] * tlwh[3] > 20 && !vertical){
                            int half_classWithoutCar= MainCam_Image_ptr->classWithoutCar/2;
                            if( -1 < cls && cls < MainCam_Image_ptr->classWithoutCar){
                                interfaces::msg::DetectObj obj;
                                obj.x1 = tlwh[0];
                                obj.y1 = tlwh[1];
                                obj.x2 = tlwh[0]+tlwh[2];
                                obj.y2 = tlwh[1]+tlwh[3];
                                obj.confidence = send_data[i].conf_armor;
                                obj.classid = cls;
                                obj.x = send_data[i].Locate3D.x;
                                obj.y = send_data[i].Locate3D.y;
                                obj.camid=1;
                                final_res.obj.push_back(obj);
                                if(cls<half_classWithoutCar){
                                    temp_res.blue_x1[cls]=tlwh[0];
                                    temp_res.blue_y1[cls]=tlwh[1];
                                    temp_res.blue_x2[cls]=tlwh[0]+tlwh[2];
                                    temp_res.blue_y2[cls]=tlwh[1]+tlwh[3];
                                    temp_res.blue_x[cls]=send_data[i].Locate3D.x;
                                    temp_res.blue_y[cls]=send_data[i].Locate3D.y;
                                }else{
                                    temp_res.red_x1[cls-half_classWithoutCar]=tlwh[0];
                                    temp_res.red_y1[cls-half_classWithoutCar]=tlwh[1];
                                    temp_res.red_x2[cls-half_classWithoutCar]=tlwh[0]+tlwh[2];
                                    temp_res.red_y2[cls-half_classWithoutCar]=tlwh[1]+tlwh[3];
                                    temp_res.red_x[cls-half_classWithoutCar]=send_data[i].Locate3D.x;
                                    temp_res.red_y[cls-half_classWithoutCar]=send_data[i].Locate3D.y;
                                }
                            }
                        }
                    }
                }
                temp_res.self_color=Modes_ptr->ourPattern;
                final_res.self_color=Modes_ptr->ourPattern;
                detect_pub->publish(temp_res);
                res_pub->publish(final_res);
                std::cout << "-----------------step  test start--------------" << std::endl;
                rclcpp::spin_some(this->node);
                BYTETracker_ptr->update(tracked_stracks,lost_stracks, lost_predict_stracks,STacks, out,to_sentry,lidar_det,lidar_enhance_);
                std::cout << "updata is OK" << std::endl;
                auto trackEndTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                std::cout << "fps_track: " << trackEndTime - trackStartTime<< std::endl;

                this->STrackGuess(PretreatObjs_ptr->classWithoutCar);
                // for (auto stack:to_sentry) {
                //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "to_sentry: %f ,%f" , stack.vx_3d, stack.vy_3d);
                // }
                //定位英雄，进行pitch轴解算
                // if(out[6-color_index].cls != -1){
                //     double pitch = CoordSolve_ptr->dynamicCalcPitchOffset(out[6-color_index].Locate3D.x, out[6-color_index].Locate3D.y, out[6-color_index].Locate3D.z);
                //     std::cout << "pitch: " << pitch << std::endl;
                // }
//            CooSystem_ptr->solve_reality_3d(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
//                                            CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, MapGraph_ptr->vexs,MapGraph_ptr->arcs,tracked_stracks, modes.ourPattern);
//            CooSystem_ptr->solve_reality_3d(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
//                                            CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, lost_stracks);

//                std::cout << "lost_stracks " << lost_stracks.size() << std::endl;
//        Predict_ptr->loss_track_predict(lost_stracks);
//            Predict_ptr->loss_track_predict(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
//                                            CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, lost_stracks);
//            Predict_ptr->getRectFromLocate3D(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
//                                             CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, lost_stracks);
                std::cout << "lost_stracks.size():  " << lost_stracks.size() << std::endl;
                MainCam_Image_ptr->draw_line(MainMapGraph_ptr->vexs);
//        std::cout << "next step12" << std::endl;

//        std::cout << (after + start) << std::endl;
                std::cout << "next step13" << std::endl;
                bafter = after;
//                MainCam_Image_ptr->draw_rusult(STacks, true);

                //此处发布的坐标为机器人在官方小地图中的坐标
                // interfaces::msg::DetectFrame temp_res;
                // temp_res.header.stamp = time_now;
                // for (int i = 0; i < out.size(); i++){
                //     int cls = out[i].cls;
                //     if(cls != -1){
                //         std::vector<float> tlwh = out[i].tlwh;
                //         bool vertical = tlwh[2] / tlwh[3] > 1.6;
                //         if (tlwh[2] * tlwh[3] > 20 && !vertical){
                //             if( -1 < cls && cls < MainCam_Image_ptr->classWithoutCar){
                //                 if(cls<=5){
                //                     temp_res.blue_x1[cls]=tlwh[0];
                //                     temp_res.blue_y1[cls]=tlwh[1];
                //                     temp_res.blue_x2[cls]=tlwh[0]+tlwh[2];
                //                     temp_res.blue_y2[cls]=tlwh[1]+tlwh[3];
                //                     temp_res.blue_x[cls]=out[i].Locate3D.x;
                //                     temp_res.blue_y[cls]=out[i].Locate3D.y;
                //                 }else{
                //                     temp_res.red_x1[cls-6]=tlwh[0];
                //                     temp_res.red_y1[cls-6]=tlwh[1];
                //                     temp_res.red_x2[cls-6]=tlwh[0]+tlwh[2];
                //                     temp_res.red_y2[cls-6]=tlwh[1]+tlwh[3];
                //                     temp_res.red_x[cls-6]=out[i].Locate3D.x;
                //                     temp_res.red_y[cls-6]=out[i].Locate3D.y;
                //                 }
                //             }
                //         }
                //     }
                // }
                // temp_res.self_color=Modes_ptr->ourPattern;
                // // detect_frame=temp_res;
                // detect_pub->publish(temp_res);

                //激光雷达纠正
                // for (int i = 0; i < out.size(); i++){
                //     int cls = out[i].cls;
                //     if(cls != -1){
                //         std::vector<float> tlwh = out[i].tlwh;
                //         bool vertical = tlwh[2] / tlwh[3] > 1.6;
                //         if (tlwh[2] * tlwh[3] > 20 && !vertical){
                //             if( -1 < cls && cls < MainCam_Image_ptr->classWithoutCar){
                //                 if(cls<=5&&lidar_det.blue_x[cls]!=0&&lidar_det.blue_y[cls]!=0){
                //                     out[i].Locate3D.x=lidar_det.blue_x[cls];
                //                     out[i].Locate3D.y=lidar_det.blue_y[cls];
                //
                //                 }else if (cls>5&&lidar_det.red_x[cls-6]!=0&&lidar_det.red_y[cls-6]!=0){
                //                     out[i].Locate3D.x=lidar_det.red_x[cls-6];
                //                     out[i].Locate3D.y=lidar_det.red_y[cls-6];
                //                 }
                //             }
                //         }
                //     }
                // }

                MainCam_Image_ptr->draw_rusult(out, true);
//        MainCam_Image_ptr->draw_rusult(tracked_stracks, true);
//        MainCam_Image_ptr->draw_rusult(lost_stracks, true);
//                MainCam_Image_ptr->draw_rusult(lost_predict_stracks, true);
                std::cout << "lost_stracks  " << lost_stracks.size() << std::endl;
                std::cout << "lost_predict_stracks  " << lost_predict_stracks.size() << std::endl;


                getRivalOffenseWarning(isWarring);
                if(Port_ptr->is_openPort){
                    Port_ptr->setWarring(isWarring);
                    Port_ptr->updataSTrackData(this->out);
                    Port_ptr->updataSentryData(this->to_sentry);
                    Port_ptr->updataDroneData(this->drone_location_.x);
                }


//        MainCam_Image_ptr->draw_rusult(UnityRect_ptr->rect, xyz, mainImg_draw);
//        Livox_ptr->myMutex_publicDepthMat.lock();
//        cv::Mat temp = (Livox_ptr->outDepthMat.clone());
//        Livox_ptr->myMutex_publicDepthMat.unlock();
//        // UnityRect_ptr->rect
//        KRepresent_ptr->Run(temp,MainCam_ptr->T_2world,UnityRect_ptr->rect);


                //     CooSystem_ptr->pts_pnp_2d_MainCam = GetPoint2d_mouse(this->mainCamMat,"Hik30");
                //     CooSystem_ptr->Get2world_matrix(CooSystem_ptr->T_Lidar2world ,CooSystem_ptr->K_M ,CooSystem_ptr->pts_pnp_2d_lidar , MapGraph_ptr->pts_pnp_3d);
                //     CooSystem_ptr->Get2world_matrix(CooSystem_ptr->T_Main2world  ,CooSystem_ptr->K_M ,CooSystem_ptr->pts_pnp_2d_MainCam , MapGraph_ptr->pts_pnp_3d);
                //     Livox_ptr->SetT_matrix(CooSystem_ptr->T_Main2world,CooSystem_ptr->T_Lidar2world);
                //     Image_ptr->is_getPoint2d_mouse_Cam = false;
                //     Livox_ptr->spin();
                //     MapGraph_ptr->get_predict_2d(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,CooSystem_ptr->cx_M,CooSystem_ptr->cy_M);
                //     MapGraph_ptr->get_roughH_config();
                // }else if(!Image_ptr->is_getPoint2d_mouse_Cam){
                //     // Livox_ptr->spin();
                //     //mlt-thread
                //     Net_ptr->Spin(this->mainCamMat);
                //     this->DetectionObjs = Net_ptr->futureObjs.get();
                //     //common
                //     // Net_ptr->NetWork(this->mainCamMat,DetectionObjs);
                //     //classify
                //     Net_ptr->Car_Armor(DetectionObjs,cars,armors);
                //     //show result
                //     // Livox_ptr->is_workLivox = false;
                //     Image_ptr->draw_rusult(cars,armors);
                //     //put depthMat and get vector Z
                //     // Livox_ptr->lidarMainloop.join();
                //     if(!Livox_ptr->outDepthMat.empty()){
                //         if(this->Mouse_ptr == nullptr)
                //         {
                //             YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
                //             std::string win_name = config["Livox"]["winname"].as<std::string>();
                //             Livox_ptr->myMutex_publicDepthMat.lock();
                //             this->Mouse_ptr = std::shared_ptr<Mouse>(new Mouse(Livox_ptr->outImshowDepthMat,1,win_name));
                //             Livox_ptr->myMutex_publicDepthMat.unlock();
                //         }
                //         else{
                //             cv::setMouseCallback(Mouse_ptr->winname, onMouse, &(*Mouse_ptr));
                //             cv::waitKey(1);
                //             // Livox_ptr->myMutex_publicDepthMat.lock();
                //             // std::cout << (*(depthMat.ptr<cv::Vec3d>(mouse.point2d_mouse_xy[0].x, mouse.point2d_mouse_xy[0].y)))[0] << ","
                //             // Livox_ptr->myMutex_publicDepthMat.unlock();
                //         }
                //         Livox_ptr->myMutex_publicDepthMat.lock();
                //         cv::Mat temp = (Livox_ptr->outDepthMat.clone());
                //         Livox_ptr->myMutex_publicDepthMat.unlock();
                //         // UnityRect_ptr->rect
                //         KRepresent_ptr->Run(temp,CooSystem_ptr->T_Main2world,UnityRect_ptr->rect);
                //     }
                auto endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                std::cout << "fps: " << 1000. / (endTime - startTime) << std::endl;

            }
            if(Modes_ptr->isSave==true_ && Modes_ptr->pictureSource==camera_){
                cv::imwrite((this->save_main_dir + "/" +std::to_string(pic_num)+ ".jpg"),mainCamMat);
//            cv::imwrite((this->save_sec_dir + "/" +std::to_string(pic_num)+ ".jpg"),secImg_save);
                pic_num++;
            }

//        cv::waitKey(500);
        }
//        after = after  + 1;

        if(cv::waitKey(1) == 'x' || Modes_ptr->pictureSource != picture_dir){
            after++;
            cv::waitKey(1);//
        }

    }
    else{
        // std::cout << "next step0.0.0" << std::endl;
//    cars.clear();
//    lastCars.clear();
//    armors.clear();
        this->STrackClear();
        PretreatObjs_ptr->num = 0;
        DetectionObjs.clear();
        frames.clear();
        // std::cout << "next step0.0" << std::endl;
        // Livox_ptr->ShowDepthMat();
        // std::cout << "next step0" << std::endl;
        // ros::spinOnce();
        // std::cout << "next step1" << std::endl;
        int a = this->after*2;
        this->mainCamMat = MainCam_Image_ptr->Image_Get(this->after,argc,argv);
        if(Modes_ptr->pictureSource==camera_)
            this->time_now = MainCam_Image_ptr->ros_time;////实机用时间戳
        int after_2 = this->after+1 ;
        this->secCamMat = SecCam_Image_ptr->Image_Get(this->after,argc,argv);

        if(!this->mainCamMat.empty() && !this->secCamMat.empty()){
            auto startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            // std::cout << "next step3" << std::endl;
            // MainCam_Image_ptr->draw_line(MainMapGraph_ptr->vexs);
            // SecCam_Image_ptr->draw_line(SecMapGraph_ptr->vexs);
            // std::cout << "next step4" << std::endl;
//        MainCam_Image_ptr->Image_Show();
//        SecCam_Image_ptr->Image_Show();
            // std::cout << "next step5" << std::endl;
            // cv::Mat mainImg_draw = mainCamMat.clone();
            // cv::Mat mainImg_save = mainCamMat.clone();
            // cv::Mat secImg_draw = secCamMat.clone();
            // cv::Mat secImg_save = secCamMat.clone();
            // std::cout << "next step6" << std::endl;
            // if(is_first){
            //     Livox_ptr->spin();
            //     is_first = false;
            // }
//        if(cv::waitKey(1) == 'q'){
//            this->is_close = true;
//        }
            cv::createTrackbar("threshold", "Dart", &value, 255);
            if(bafter !=after){
                std::cout << "next step7" << std::endl;
                std::cout << "after: " << after << std::endl;
                getDartWarning(secCamMat,value);

                // DetectionObjs = MainCam_Net_ptr->futureObjs.get();
                std::vector<cv::Mat> main_frames({mainCamMat});
                std::vector<cv::Mat> sec_frames({secCamMat});

                MainCam_Net_ptr->Spin(main_frames);
                SecCam_Net_ptr->Spin(sec_frames);
                DetectionObjs.push_back( MainCam_Net_ptr->futureObjs.get()[0]);
                DetectionObjs.push_back( SecCam_Net_ptr->futureObjs.get()[0]);

                std::cout << "next step8" << std::endl;
                auto netStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

                std::vector<cv::Mat> car_imgs;
                MainCam_Net_ptr->getCarImgs({DetectionObjs[0]},mainCamMat,car_imgs);
                SecCam_Net_ptr->getCarImgs({DetectionObjs[1]},secCamMat, car_imgs);

//        Armor_Net_ptr->Spin(car_imgs);
//        vector<vector<TRTInferV1::DetectionObj>> Armors(Armor_Net_ptr->futureObjs.get());

                std::vector<cv::Mat> car_imgs_clone;
                vector<vector<TRTInferV1::DetectionObj>> Armors ;

//        Armors = Armor_Net_ptr->myInfer.doInference(car_imgs,0.2, 0.01, 0.45);
                for(auto car_img:car_imgs){
//            Armors.push_back((Armor_Net_ptr->NetWork_mlt({car_img}))[0]);
                    std::vector<Mat> car_img_s(1);
                    car_img_s[0].push_back(car_img);
                    Armors.push_back((Armor_Net_ptr->myInfer.doInference(car_img_s,0.2, 0.01, 0.45,1))[0]);
                }

                std::map<int,int> id_map={{0,5},{1,0},{2,1},{3,2},{4,3},
                    {5,11},{6,6},{7,7},{8,8},{9,9}};
                for (auto i=Armors.begin();i!=Armors.end();i++) {
                    for (auto j=i->begin();j!=i->end();j++) {
                        j->classId=id_map[j->classId];
                    }
                }

                auto netEndTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                std::cout << "car_imgs.size(): " << car_imgs.size() << std::endl;
                std::cout << "fps_netTime: " << (netEndTime - netStartTime) << std::endl;
                auto dur_time= netEndTime - netStartTime;
                std::cout<<"\033[31m"<<"---------net time: "<<dur_time<<" ms"<<"\033[0m"<<std::endl;
                auto trackStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

                std::string path111="/home/thesky/armors/";
                for (int i(0); i < int(car_imgs.size()); ++i)
                {
                    for (int j(0); j < int(Armors[i].size()); ++j)
                    {
                        cv::Rect r = cv::Rect(Armors[i][j].x1, Armors[i][j].y1, Armors[i][j].x2 - Armors[i][j].x1, Armors[i][j].y2 - Armors[i][j].y1);
                        // cv::rectangle(car_imgs[i], r, cv::Scalar(255, 255, 255), 1);
                        // cv::putText(car_imgs[i], std::to_string(Armors[i][j].classId), cv::Point (Armors[i][j].x1, Armors[i][j].y1),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255, 100, 255),1);
                        // cv::putText(car_imgs[i], std::to_string(Armors[i][j].confidence), cv::Point (Armors[i][j].x2, Armors[i][j].y1),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(100, 100, 255),1);
//                        std::cout << Armors[i][j].x1 << " " << Armors[i][j].y1 << " " << Armors[i][j].x2 << " " << Armors[i][j].y2 << std::endl;
//                        std::cout << Armors[i][j].classId << "|" << Armors[i][j].confidence << std::endl;
                        // std::string path=path111+std::to_string(pic_num)+"_"+std::to_string(i)+"_"+std::to_string(j)+".jpg";
                        // cv::imwrite(path,car_imgs[i](r));
                    }
                }
                // pic_num++;
                std::cout << "next step9" << std::endl;

                int cam_num = 2;
                float p =0.97;
                float match_thresh = 0.85;
                std::vector<std::vector<STrack>> stracks(cam_num);
//        for(int i=0; i < cam_num; i++){
//            for(auto car : DetectionObjs[i]){
//                if(i==0){
//                    STrack sTrack(car.x1, car.y1,car.w, car.h, car.confidence);
//                    sTrack.setRectInPrimaryCam(car.x1, car.y1,car.w, car.h, p);
//                    stracks[i].push_back(sTrack);
//                }else if(i ==1){
//                    TRTInferV1::Object sec2main_Obj = PretreatObjs_ptr->secObjs2mainObjs(car,MainCam_ptr,SecCam_ptr);
//                    STrack sTrack(sec2main_Obj.x1, sec2main_Obj.y1,sec2main_Obj.w, sec2main_Obj.h, sec2main_Obj.confidence);
//                    sTrack.setRectInPrimaryCam(car.x1, car.y1,car.w, car.h, p);
//                    stracks[i].push_back(sTrack);
//                }else{
//                    std::cout << "ERROR: IN secObjs2mainObjs about SecCam_ptr" << std::endl;
//                }
//            }
//        }

/*
 * old sec2main-------------------------------------------------------------------------------------------------------------------------
 * */
//                for(int i=0; i < cam_num; i++){
//                    for(auto car : DetectionObjs[i]){
//                        if(i==0){
//                            STrack sTrack(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), car.confidence);
//                            sTrack.setRectInPrimaryCam(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), p);
//                            stracks[i].push_back(sTrack);
//                        }else if(i ==1){
//                            TRTInferV1::DetectionObj sec2main_Obj = PretreatObjs_ptr->secObjs2mainObjs(car,MainCam_ptr,SecCam_ptr);
//                            std::cout << "car.x1: " << car.x1 << std::endl;
//                            std::cout << "car.x2: " << car.x2 << std::endl;
//                            std::cout << "sec2main_Obj.x1: " << sec2main_Obj.x1 << std::endl;
//                            std::cout << "sec2main_Obj.x2: " << sec2main_Obj.x2 << std::endl;
//                            STrack sTrack(sec2main_Obj.x1, sec2main_Obj.y1,sec2main_Obj.x2-sec2main_Obj.x1, sec2main_Obj.y2-sec2main_Obj.y1, sec2main_Obj.confidence);
//                            sTrack.setRectInPrimaryCam(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), p);
//                            stracks[i].push_back(sTrack);
//                        }else{
//                            std::cout << "ERROR: IN secObjs2mainObjs about SecCam_ptr" << std::endl;
//                        }
//                    }
//                }

/*
 * new sec2main-------------------------------------------------------------------------------------------------------------------------
 * */
                for(int i=0; i < cam_num; i++){
                    for(auto car : DetectionObjs[i]){
                        TRTInferV1::DetectionObj mainobj;
                        if(i==0){
//                            mainobj = PretreatObjs_ptr->objs2newMainObjs(car, PretreatObjs_ptr->main_translation);
//                            STrack sTrack(mainobj.x1, mainobj.y1,(mainobj.x2-mainobj.x1), (mainobj.y2-mainobj.y1), mainobj.confidence);
                            STrack sTrack(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), car.confidence,this->classWithoutCar);
                            sTrack.setRectInPrimaryCam(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), p);
                            sTrack.camid=1;
                            stracks[i].push_back(sTrack);
                        }else if(i ==1){
                            mainobj = PretreatObjs_ptr->objs2newMainObjs(car, PretreatObjs_ptr->H2);
                            STrack sTrack(mainobj.x1, mainobj.y1,mainobj.x2-mainobj.x1, mainobj.y2-mainobj.y1, mainobj.confidence,this->classWithoutCar);
                            sTrack.setRectInPrimaryCam(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), p);
                            sTrack.camid=2;
                            sTrack.sec_rect.resize(4);
                            sTrack.sec_rect = {car.x1, car.y1, car.x2, car.y2};
                            stracks[i].push_back(sTrack);

                        }else{
                            std::cout << "ERROR: IN secObjs2mainObjs about SecCam_ptr" << std::endl;
                        }
                    }
                }

                int car_num = 0;
                for(int cam=0; cam<cam_num ; cam++){
                    std::cout <<  "stracks.size" << stracks[cam].size() << std::endl;
                    int cam_cars_num = stracks[cam].size();
                    std::vector<int> remove_lists;
                    for(int i=0;i<cam_cars_num;i++){
                        bool flag=PretreatObjs_ptr->get_Armors_w_conf_Double_net(stracks[cam][i],Armors[i+car_num],car_imgs[i+car_num]);
                        if (flag) remove_lists.push_back(i);
                    }
                    for (auto i:remove_lists)
                        stracks[cam].erase(stracks[cam].begin()+i);
                    car_num += cam_cars_num;
                }

                std::vector<bool> isWarring(6, false) ;
                if(dart_flag)
                    isWarring[4]=true;
                CooSystem_ptr->solve_reality_3d(MainCam_ptr->T_2world, MainCam_ptr->fx, MainCam_ptr->fy,
                                                MainCam_ptr->cx, MainCam_ptr->cy, MainMapGraph_ptr->vexs,MainMapGraph_ptr->arcs,stracks[0], Modes_ptr->ourPattern, isWarring);
                CooSystem_ptr->solve_reality_3d(SecCam_ptr->T_2world, SecCam_ptr->fx, SecCam_ptr->fy,
                                                SecCam_ptr->cx, SecCam_ptr->cy, SecMapGraph_ptr->vexs,SecMapGraph_ptr->arcs,stracks[1], Modes_ptr->ourPattern, isWarring);
                // CooSystem_ptr->solve_reality_3d(SecCam_ptr->T_2world, SecCam_ptr->fx, SecCam_ptr->fy,
                //                                 SecCam_ptr->cx, SecCam_ptr->cy, SecMapGraph_ptr->vexs,SecMapGraph_ptr->arcs,cars, Modes_ptr->ourPattern);
                if(Port_ptr->is_openPort) {
                    Port_ptr->updateGameTime(this->game_time_);
                    Port_ptr->updataRadarMarkData(tracked_stracks);
                    Port_ptr->updataRadarMarkData(lost_stracks);
                    Port_ptr->updataRadarMarkData(lost_predict_stracks);
                }


//        Armor_Net_ptr->Spin(car_imgs);
//        vector<vector<TRTInferV1::DetectionObj>> Armors(Armor_Net_ptr->futureObjs.get());

// 图像拼接可视化
                // MainCam_Image_ptr->draw_rusult(stracks[0], true);
                // MainCam_Image_ptr->draw_rusult(stracks[1], true);

                std::cout << "next step9.5" << std::endl;

                vector<vector<float> > dists;
                int dist_size, dist_size_size;
                vector<vector<int> > matches;
                vector<int> u_main, u_sec;
                Eigen::MatrixXd cost_matrix = costMatrix_ptr->getIouAndDistancetCost(stracks[0],stracks[1],dist_size, dist_size_size);
                eigenMat2VecVec(cost_matrix,dists);
                std::cout << "dists: " << std::endl << cost_matrix << std::endl;
                costMatrix_ptr->linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_main, u_sec);

                std::vector<STrack> STacks,send_data;
                std::cout << "next step9.9" << std::endl;

                for(int i=0;i<matches.size();i++ ){
                    if(((stracks[0][matches[i][0]].tlwh[3] * stracks[0][matches[i][0]].tlwh[2]) < (stracks[1][matches[i][1]].tlwh[3] * stracks[1][matches[i][1]].tlwh[2]))
                       && (stracks[0][matches[i][0]].tlwh[2]/stracks[0][matches[i][0]].tlwh[3] < 0.2)){
                        if(stracks[1][matches[i][1]].cls > 0 || stracks[1][matches[i][1]].cls < PretreatObjs_ptr->classWithoutCar){
                            STacks.push_back(stracks[1][matches[i][1]]);
                        }
                        else{
                            stracks[1][matches[i][1]].cls = stracks[0][matches[i][0]].cls;
                            STacks.push_back(stracks[1][matches[i][1]]);
                        }
                    }else{
                        if(stracks[0][matches[i][0]].cls > 0 || stracks[0][matches[i][0]].cls < PretreatObjs_ptr->classWithoutCar){
                            STacks.push_back(stracks[0][matches[i][0]]);
                        }
                        else{
                            stracks[0][matches[i][0]].cls = stracks[1][matches[i][1]].cls;
                            STacks.push_back(stracks[0][matches[i][0]]);
                        }
                    }
                }
                for(int i=0;i<u_main.size();i++ ){
                    STacks.push_back(stracks[0][u_main[i]]);
                }
                for(int i=0;i<u_sec.size();i++ ){
                    STacks.push_back(stracks[1][u_sec[i]]);
                }

                std::vector<std::vector<float>> cost_confMatrix_vec;
                int num_strack,num_cls;
                vector<vector<int> > matches_cls;
                vector<int> u_strack, u_cls;
                matches_cls.clear();u_strack.clear();u_cls.clear();
                Eigen::MatrixXd cost_confMatrix = costMatrix_ptr->getCost_confMatrix(STacks,num_strack,num_cls);
                std::cout << "cost_confMatrix _ cls:  " << std::endl << cost_confMatrix << std::endl;
                eigenMat2VecVec(cost_confMatrix, cost_confMatrix_vec);
                costMatrix_ptr->linear_assignment(cost_confMatrix_vec, num_strack, num_cls, 0.9, matches_cls, u_strack, u_cls);

                for (int i=0; i < matches_cls.size(); i++) {
                    STacks[matches_cls[i][0]].cls = matches_cls[i][1];
                    send_data.push_back(STacks[matches_cls[i][0]]);
                }

                interfaces::msg::DetectFrame temp_res;
                interfaces::msg::DetectRes final_res;
                final_res.header.stamp = time_now;
                temp_res.header.stamp = time_now;
                for (int i = 0; i < send_data.size(); i++){
                    int cls = send_data[i].cls;
                    if(cls != -1){
                        std::vector<float> tlwh = send_data[i].tlwh;
                        bool vertical = tlwh[2] / tlwh[3] > 1.6;
                        if (tlwh[2] * tlwh[3] > 20 && !vertical){
                            int half_classWithoutCar= MainCam_Image_ptr->classWithoutCar/2;
                            if( -1 < cls && cls < MainCam_Image_ptr->classWithoutCar){
                                interfaces::msg::DetectObj obj;
                                if (send_data[i].camid==1) {
                                    obj.x1 = tlwh[0];
                                    obj.y1 = tlwh[1];
                                    obj.x2 = tlwh[0]+tlwh[2];
                                    obj.y2 = tlwh[1]+tlwh[3];
                                    obj.confidence = send_data[i].conf_armor;
                                    obj.classid = cls;
                                    obj.x = send_data[i].Locate3D.x;
                                    obj.y = send_data[i].Locate3D.y;
                                    obj.camid=1;
                                    for (int j=0;j<10;j++) {
                                        obj.conf_matrix[j] = send_data[i].ws_armorConfMatrix(0,j);
                                    }
                                    final_res.obj.push_back(obj);
                                    if(cls<half_classWithoutCar){
                                        temp_res.blue_x1[cls]=tlwh[0];
                                        temp_res.blue_y1[cls]=tlwh[1];
                                        temp_res.blue_x2[cls]=tlwh[0]+tlwh[2];
                                        temp_res.blue_y2[cls]=tlwh[1]+tlwh[3];
                                        temp_res.blue_x[cls]=send_data[i].Locate3D.x;
                                        temp_res.blue_y[cls]=send_data[i].Locate3D.y;
                                    }else{
                                        temp_res.red_x1[cls-half_classWithoutCar]=tlwh[0];
                                        temp_res.red_y1[cls-half_classWithoutCar]=tlwh[1];
                                        temp_res.red_x2[cls-half_classWithoutCar]=tlwh[0]+tlwh[2];
                                        temp_res.red_y2[cls-half_classWithoutCar]=tlwh[1]+tlwh[3];
                                        temp_res.red_x[cls-half_classWithoutCar]=send_data[i].Locate3D.x;
                                        temp_res.red_y[cls-half_classWithoutCar]=send_data[i].Locate3D.y;
                                    }
                                }else if (send_data[i].camid==2) {
                                    obj.x1 = send_data[i].sec_rect[0];
                                    obj.y1 = send_data[i].sec_rect[1];
                                    obj.x2 = send_data[i].sec_rect[2];
                                    obj.y2 = send_data[i].sec_rect[3];
                                    obj.confidence = send_data[i].conf_armor;
                                    obj.classid = cls;
                                    obj.x = send_data[i].Locate3D.x;
                                    obj.y = send_data[i].Locate3D.y;
                                    obj.camid=2;
                                    for (int j=0;j<10;j++) {
                                        obj.conf_matrix[j] = send_data[i].ws_armorConfMatrix(0,j);
                                    }
                                    final_res.obj.push_back(obj);
                                    if(cls<half_classWithoutCar){
                                        temp_res.blue_x1[cls]=tlwh[0];
                                        temp_res.blue_y1[cls]=tlwh[1];
                                        temp_res.blue_x2[cls]=tlwh[0]+tlwh[2];
                                        temp_res.blue_y2[cls]=tlwh[1]+tlwh[3];
                                        temp_res.blue_x[cls]=send_data[i].Locate3D.x;
                                        temp_res.blue_y[cls]=send_data[i].Locate3D.y;
                                    }else{
                                        temp_res.red_x1[cls-half_classWithoutCar]=tlwh[0];
                                        temp_res.red_y1[cls-half_classWithoutCar]=tlwh[1];
                                        temp_res.red_x2[cls-half_classWithoutCar]=tlwh[0]+tlwh[2];
                                        temp_res.red_y2[cls-half_classWithoutCar]=tlwh[1]+tlwh[3];
                                        temp_res.red_x[cls-half_classWithoutCar]=send_data[i].Locate3D.x;
                                        temp_res.red_y[cls-half_classWithoutCar]=send_data[i].Locate3D.y;
                                    }
                                }
                            }
                        }
                    }
                }
                temp_res.self_color=Modes_ptr->ourPattern;
                final_res.self_color=Modes_ptr->ourPattern;
                // detect_frame=temp_res;
                detect_pub->publish(temp_res);
                res_pub->publish(final_res);


                std::cout << "-----------------step  test start--------------" << std::endl;


//        for(int i=0;i<DetectionObjs[0].size();i++){
//            float x = (DetectionObjs[0][i].x2+DetectionObjs[0][i].x1)/2.,
//                    y = DetectionObjs[0][i].y2,
//                    w = (DetectionObjs[0][i].x2-DetectionObjs[0][i].x1),
//                    h = (DetectionObjs[0][i].y2-DetectionObjs[0][i].y1);
//            MainCam_ptr->change2main(x,y,MainCam_ptr->fx,MainCam_ptr->fy,MainCam_ptr->cx,MainCam_ptr->cy);
//            DetectionObjs[0][i].x1 = x - w/2;
//            DetectionObjs[0][i].x2 = x + w/2;
//            DetectionObjs[0][i].y1 = y - h;
//            DetectionObjs[0][i].y2 = y ;
//        }


//        std::cout << "-----------------------------" << std::endl;
//        std::vector<std::vector<int>> list_;
//        std::vector<TRTInferV1::Object> all2main_Objs
//                = PretreatObjs_ptr->secObjs2mainObjs(DetectionObjs[0],DetectionObjs[1],
//                                                     MainCam_ptr,SecCam_ptr,0.90,list_);
//        std::cout << "-----------------step  test  end--------------" << std::endl;

                //    std::cout << "next step9" << std::endl;
//                MainCam_Image_ptr->draw_rusult(STacks, true);

                // BYTETracker_ptr->update(tracked_stracks,lost_stracks, lost_predict_stracks,STacks, out);
                rclcpp::spin_some(this->node);
                BYTETracker_ptr->update(tracked_stracks,lost_stracks, lost_predict_stracks,STacks, out,to_sentry,lidar_det,lidar_enhance_);
                std::cout << "updata is OK" << std::endl;
                auto trackEndTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                std::cout << "fps_track: " << trackEndTime - trackStartTime << std::endl;

                this->STrackGuess(PretreatObjs_ptr->classWithoutCar);
                // if(out[6-color_index].cls != -1){
                //     double pitch = CoordSolve_ptr->dynamicCalcPitchOffset(out[6-color_index].Locate3D.x, out[6-color_index].Locate3D.y, out[6-color_index].Locate3D.z);
                //     std::cout << "pitch: " << pitch << std::endl;
                // }

//            CooSystem_ptr->solve_reality_3d(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
//                                            CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, MapGraph_ptr->vexs,MapGraph_ptr->arcs,tracked_stracks, modes.ourPattern);
//            CooSystem_ptr->solve_reality_3d(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
//                                            CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, lost_stracks);

                std::cout << "lost_stracks " << lost_stracks.size() << std::endl;
//        Predict_ptr->loss_track_predict(lost_stracks);
//            Predict_ptr->loss_track_predict(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
//                                            CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, lost_stracks);
//            Predict_ptr->getRectFromLocate3D(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
//                                             CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, lost_stracks);
                std::cout << "lost_stracks.size():  " << lost_stracks.size() << std::endl;
                MainCam_Image_ptr->draw_line(MainMapGraph_ptr->vexs);
                SecCam_Image_ptr->draw_line(SecMapGraph_ptr->vexs);
//        std::cout << "next step12" << std::endl;

//        MainCam_Image_ptr->draw_rusult(cars ,armors, false);
//        std::cout << (after + start) << std::endl;
                std::cout << "next step13" << std::endl;
                bafter = after;
                MainCam_Image_ptr->draw_lidar(this->lidar_det1);
                MainCam_Image_ptr->draw_rusult(out, true);
//                MainCam_Image_ptr->draw_rusult(tracked_stracks, true);
//                MainCam_Image_ptr->draw_rusult(lost_stracks, true);
//                MainCam_Image_ptr->draw_rusult(lost_predict_stracks, true);
                std::cout << "lost_stracks  " << lost_stracks.size() << std::endl;
                std::cout << "lost_predict_stracks  " << lost_predict_stracks.size() << std::endl;

                getRivalOffenseWarning(isWarring);
                if(Port_ptr->is_openPort){
                    Port_ptr->setWarring(isWarring);
                    Port_ptr->updataSTrackData(this->out);
                    Port_ptr->updataSentryData(this->to_sentry);
                    Port_ptr->updataDroneData(this->drone_location_.x);
                }


//        MainCam_Image_ptr->draw_rusult(UnityRect_ptr->rect, xyz, mainImg_draw);
//        Livox_ptr->myMutex_publicDepthMat.lock();
//        cv::Mat temp = (Livox_ptr->outDepthMat.clone());
//        Livox_ptr->myMutex_publicDepthMat.unlock();
//        // UnityRect_ptr->rect
//        KRepresent_ptr->Run(temp,MainCam_ptr->T_2world,UnityRect_ptr->rect);


                //     CooSystem_ptr->pts_pnp_2d_MainCam = GetPoint2d_mouse(this->mainCamMat,"Hik30");
                //     CooSystem_ptr->Get2world_matrix(CooSystem_ptr->T_Lidar2world ,CooSystem_ptr->K_M ,CooSystem_ptr->pts_pnp_2d_lidar , MapGraph_ptr->pts_pnp_3d);
                //     CooSystem_ptr->Get2world_matrix(CooSystem_ptr->T_Main2world  ,CooSystem_ptr->K_M ,CooSystem_ptr->pts_pnp_2d_MainCam , MapGraph_ptr->pts_pnp_3d);
                //     Livox_ptr->SetT_matrix(CooSystem_ptr->T_Main2world,CooSystem_ptr->T_Lidar2world);
                //     Image_ptr->is_getPoint2d_mouse_Cam = false;
                //     Livox_ptr->spin();
                //     MapGraph_ptr->get_predict_2d(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,CooSystem_ptr->cx_M,CooSystem_ptr->cy_M);
                //     MapGraph_ptr->get_roughH_config();
                // }else if(!Image_ptr->is_getPoint2d_mouse_Cam){
                //     // Livox_ptr->spin();
                //     //mlt-thread
                //     Net_ptr->Spin(this->mainCamMat);
                //     this->DetectionObjs = Net_ptr->futureObjs.get();
                //     //common
                //     // Net_ptr->NetWork(this->mainCamMat,DetectionObjs);
                //     //classify
                //     Net_ptr->Car_Armor(DetectionObjs,cars,armors);
                //     //show result
                //     // Livox_ptr->is_workLivox = false;
                //     Image_ptr->draw_rusult(cars,armors);
                //     //put depthMat and get vector Z
                //     // Livox_ptr->lidarMainloop.join();
                //     if(!Livox_ptr->outDepthMat.empty()){
                //         if(this->Mouse_ptr == nullptr)
                //         {
                //             YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
                //             std::string win_name = config["Livox"]["winname"].as<std::string>();
                //             Livox_ptr->myMutex_publicDepthMat.lock();
                //             this->Mouse_ptr = std::shared_ptr<Mouse>(new Mouse(Livox_ptr->outImshowDepthMat,1,win_name));
                //             Livox_ptr->myMutex_publicDepthMat.unlock();
                //         }
                //         else{
                //             cv::setMouseCallback(Mouse_ptr->winname, onMouse, &(*Mouse_ptr));
                //             cv::waitKey(1);
                //             // Livox_ptr->myMutex_publicDepthMat.lock();
                //             // std::cout << (*(depthMat.ptr<cv::Vec3d>(mouse.point2d_mouse_xy[0].x, mouse.point2d_mouse_xy[0].y)))[0] << ","
                //             // Livox_ptr->myMutex_publicDepthMat.unlock();
                //         }
                //         Livox_ptr->myMutex_publicDepthMat.lock();
                //         cv::Mat temp = (Livox_ptr->outDepthMat.clone());
                //         Livox_ptr->myMutex_publicDepthMat.unlock();
                //         // UnityRect_ptr->rect
                //         KRepresent_ptr->Run(temp,CooSystem_ptr->T_Main2world,UnityRect_ptr->rect);
                //     }
                auto endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                std::cout << "fps: " << 1000. / (endTime - startTime) << std::endl;

            }
            if(Modes_ptr->isSave==true_ && Modes_ptr->pictureSource==camera_){
                cv::imwrite((this->save_main_dir + "/" +std::to_string(pic_num)+ ".jpg"),mainCamMat);
                cv::imwrite((this->save_sec_dir + "/" +std::to_string(pic_num)+ ".jpg"),secCamMat);
                pic_num++;
            }

//            after++;

//        cv::waitKey(500);


        }
        if(cv::waitKey(1) == 'x' || (Modes_ptr->pictureSource != picture_dir)){
            after++;
        }
        cv::waitKey(1);//

    }

//    if(cv::waitKey(1) == 'q'){
//        this->is_close = true;
//    }

}


void MyRadar::Close(){
    std::cout << "close" << std::endl;

    if(Modes_ptr->isSave == TF::true_){
        std::cout << "try to solve problem" << std::endl;
        // ros::V_string v_nodes;
        // ros::master::getNodes(v_nodes);

        // std::string node_name_ = "/" + this->node_name;

        // auto it = std::find(v_nodes.begin(), v_nodes.end(), node_name_.c_str());
        // if (it != v_nodes.end()){
        //     std::string cmd_str_0 = "rosnode kill " + node_name_;
        //     int ret = system(cmd_str_0.c_str());
        //     std::cout << "## stop rosbag record cmd: " << cmd_str_0 << std::endl;
        // }
    }
    // Livox_ptr->close();
    if(Port_ptr->is_openPort) {
        Port_ptr->close();
    }


    std::cout << "close" << std::endl;

//    ros::shutdown();
//    if(Modes_ptr->isOpenMid70){
//        std::string cmd_str_1 = "rosnode kill -a";
//        int ret = system(cmd_str_1.c_str());
//    }

    MainCam_Image_ptr->Close();
    if(!is_one_cam){
        SecCam_Image_ptr->Close();
    }
}