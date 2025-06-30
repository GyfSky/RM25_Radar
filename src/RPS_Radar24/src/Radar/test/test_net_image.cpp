#include "../include/Radar.h"


MyRadar::MyRadar(rclcpp::Node* node){
    after = 1500;bafter = after;int start = 0;

    this->Modes_ptr = std::shared_ptr<Modes>(new Modes());

    this->MainCam_Net_ptr   = std::shared_ptr<Net>(new Net("net_60"));
    this->SecCam_Net_ptr   = std::shared_ptr<Net>(new Net("net"));
//    this->Armor_Net_ptr   = std::shared_ptr<Net>(new Net("net_armor"));
    myInfer.getDevice(0);
    myInfer.initModule("/home/thesky/RM25_Radar/src/RPS_Radar24/model/best_armor.trt", 16, 12);


    std::cout << "start_SensorParam" << std::endl;

    this->MainCam_ptr = std::shared_ptr<SensorParam>(new SensorParam("Hik60",CamPosition::right,Modes_ptr->ourPattern));
    this->SecCam_ptr = std::shared_ptr<SensorParam>(new SensorParam("Hik30",CamPosition::left,Modes_ptr->ourPattern));
//    this->Lidar_ptr = std::shared_ptr<SensorParam>(new SensorParam("Livox",MainCam_ptr->K,CamPosition::right,Modes_ptr->ourPattern));

//    this->PretreatObjs_ptr = std::shared_ptr<PretreatObjs>(new PretreatObjs());
//    this->BYTETracker_ptr = std::shared_ptr<BYTETracker>(new BYTETracker(Modes_ptr->ourPattern));

    std::cout << "have look" << std::endl;

    // this->MainCam_Image_ptr = std::shared_ptr<Image>(
    //     new Image(Modes_ptr->application,Modes_ptr->pictureSource, "DA0926631", "Hik60", Modes_ptr->isSave, disk02,
    //                   start));
    // this->SecCam_Image_ptr = std::shared_ptr<Image>(
    //     new Image(Common,Modes_ptr->pictureSource, "00F26632053", "Hik30", Modes_ptr->isSave, disk02,
    //                   start));

    this->Port_ptr = std::shared_ptr<Port>(new Port(Modes_ptr->ourPattern, 12, Modes_ptr->Port_isOpen, Modes_ptr->usePort,node));


    //test
    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    std::string win_name = config["Livox"]["winname"].as<std::string>();
    std::cout << "---------------------make is ok1------------------" << std::endl;

}

MyRadar::~MyRadar(){

}

void MyRadar::Init(int argc, char **argv){

    MainCam_Image_ptr->Init(argc, argv);
    SecCam_Image_ptr->Init(argc, argv);


    if(MainCam_Image_ptr->is_getPoint2d_mouse_Cam && SecCam_Image_ptr->is_getPoint2d_mouse_Cam){

        while(mainCamMat.empty() && secCamMat.empty()){
            mainCamMat = MainCam_Image_ptr->Image_Get(after,argc,argv);
            secCamMat = SecCam_Image_ptr->Image_Get(after,argc,argv);

        }
        MainCam_Image_ptr->Image_Show();
        SecCam_Image_ptr->Image_Show();

    }else{
        std::cerr << "error" << std::endl;
    }


    //TODO:
    //test

    //    this->Save();
    if(Port_ptr->is_openPort) {
        Port_ptr->clearBuff();
        Port_ptr->start();
    }
    std::cout << "---------------------make is ok2------------------" << std::endl;
}


void MyRadar::Save() {
    if(Modes_ptr->isSave == true_){
        YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);

        this->node_name = config["save"]["node_name"].as<std::string>();
        char now[64];
        std::time_t tt;
        struct tm *ttime;
        tt = time(nullptr);
        ttime = localtime(&tt);
        strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime);
        std::string now_string(now);
        std::string path = config["save"]["save_bag_path"].as<std::string>() + now_string +".bag";
//        std::string path = "/media/plusseven/KESU/img_DATA/test_dir/2024-05-16_15_51_31.bag";
        std::string topics = config["Livox"]["lidarTopicName"].as<std::string>();

        //----------------------------------------------------------------------------------------------------------------

        std::string all_node_name = "__name:=" +  this->node_name;
//        std::string cmd_str = "rosbag record -O " + path + " " + topics + " " + all_node_name + " &";
        std::string cmd_str = "gnome-terminal -x bash -c 'rosbag record -O " + path + " " + topics + " " + all_node_name + " '";
//        std::string cmd_str = "rosbag record -O /media/plusseven/KESU/img_DATA/test_dir/bag_name.bag /livox/lidar __name:=mid70 &" ;
        int ret = system(cmd_str.c_str()); // #include <stdlib.h>
        std::cout << "cmd_str: " << cmd_str << std::endl;
        std::cout << "path: " << path << std::endl;
//-------------------------------------------------------------------------------------------------------------------------
        if(ret != 0){
            std::cerr << "\033[33m" << "save bag may have error !!! Please check path" << "\033[0m" <<std::endl;
        }

        if(Modes_ptr->pictureSource==camera_){
            MainCam_Image_ptr->setSaveMode();
            SecCam_Image_ptr->setSaveMode();
        }
    }
}


void MyRadar::Spin(int argc, char **argv){
    bool is_only_port = false;
    if (!is_only_port){
        std::cout << "next step0.0.0" << std::endl;
        std::cout << "after: " << after << std::endl;
//    cars.clear();
//    lastCars.clear();
//    armors.clear();
        out.clear();
        DetectionObjs.clear();
        frames.clear();
        std::cout << "next step0.0" << std::endl;
        std::cout << "next step0" << std::endl;
        // ros::spinOnce();
        std::cout << "next step1" << std::endl;


        int a = this->after * 2;


        this->mainCamMat = MainCam_Image_ptr->Image_Get(this->after,argc,argv);
        int after_2 = this->after+1 ;
        this->secCamMat = SecCam_Image_ptr->Image_Get(this->after,argc,argv);



        if(!this->mainCamMat.empty() && !this->secCamMat.empty()){
            auto startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            cv::Mat mainImg_draw = mainCamMat.clone();
            cv::Mat secImg_draw = secCamMat.clone();

            int gamma = 0.7;
            cv::Mat main_gamma, sec_gamma;
            MainCam_Image_ptr->GetGammaCorrection(mainImg_draw,main_gamma, gamma);
            SecCam_Image_ptr->GetGammaCorrection(secImg_draw, sec_gamma, gamma);

            // DetectionObjs = MainCam_Net_ptr->futureObjs.get();
            auto netStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::vector<cv::Mat> main_frames({mainCamMat});
            std::vector<cv::Mat> sec_frames({secCamMat});
            MainCam_Net_ptr->Spin(main_frames);
            SecCam_Net_ptr->Spin(sec_frames);
            DetectionObjs.push_back( MainCam_Net_ptr->futureObjs.get()[0]);
            DetectionObjs.push_back( SecCam_Net_ptr->futureObjs.get()[0]);

            std::vector<cv::Mat> car_imgs;
            MainCam_Net_ptr->getCarImgs({DetectionObjs[0]},mainCamMat,car_imgs);
            SecCam_Net_ptr->getCarImgs({DetectionObjs[1]},secCamMat, car_imgs);

            std::vector<cv::Mat> car_imgs_clone;
            vector<vector<TRTInferV1::DetectionObj>> Armors ;
//        for(int i=0;i<10;i++){
//            car_imgs_clone = car_imgs;
//            Armors = Armor_Net_ptr->NetWork_mlt(car_imgs_clone);
//        }
//        Armors = Armor_Net_ptr->NetWork_mlt(car_imgs);
            Armors = myInfer.doInference(car_imgs,0.2, 0.5, 0.45,0);
            for (int i(0); i < int(car_imgs.size()); ++i)
            {
                for (int j(0); j < int(Armors[i].size()); ++j)
                {
                    cv::Rect r = cv::Rect(Armors[i][j].x1, Armors[i][j].y1, Armors[i][j].x2 - Armors[i][j].x1, Armors[i][j].y2 - Armors[i][j].y1);
                    cv::rectangle(car_imgs[i], r, cv::Scalar(255, 255, 255), 1);
                    cv::putText(car_imgs[i], std::to_string(Armors[i][j].classId), cv::Point (Armors[i][j].x1, Armors[i][j].y1),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255, 100, 255),1);
                    cv::putText(car_imgs[i], std::to_string(Armors[i][j].confidence), cv::Point (Armors[i][j].x2, Armors[i][j].y1),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(100, 100, 255),1);
                    std::cout << Armors[i][j].x1 << " " << Armors[i][j].y1 << " " << Armors[i][j].x2 << " " << Armors[i][j].y2 << std::endl;
                    std::cout << Armors[i][j].classId << "|" << Armors[i][j].confidence << std::endl;
                }
            }
//            for(auto img: car_imgs){
//                cv::Mat img_clone = img.clone();
//                cv::imshow("test_test", img_clone);
//                cv::waitKey(300);
//            }
//        MainCam_Net_ptr->Spin_confs({main_gamma});
//        SecCam_Net_ptr->Spin_confs({sec_gamma});

            auto netEndTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::cout << "fps_net: " << 1000. / (netEndTime - netStartTime) << std::endl;

            auto endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::cout << "fps: " << 1000. / (endTime - startTime) << std::endl;

        }



        int every = 1;
        after++;
        if(cv::waitKey(1) == 'x'){
            this->after = this->after + every;
        }
        if(cv::waitKey(1) == 'z'){
            this->after = this->after - every;
        }
//        cv::waitKey(10);
    }

//    if(cv::waitKey(1) == 'q'){
//        this->is_close = true;
//    }

    if(Port_ptr->is_openPort){
        std::vector<STrack> test; STrack test1;
        test1.cls = 4; test1.Locate3D.x = 0.;test1.Locate3D.y = 0.;
        test.push_back(test1);
        Port_ptr->updataSTrackData(test);
    }


}


void MyRadar::Close(){
    std::cout << "close" << std::endl;

    if(Modes_ptr->isSave == TF::true_){
        std::cout << "try to solve problem" << std::endl;
        // ros::V_string v_nodes;
        // ros::master::getNodes(v_nodes);//

        // std::string node_name_ = "/" + this->node_name;

        // auto it = std::find(v_nodes.begin(), v_nodes.end(), node_name_.c_str());
        // if (it != v_nodes.end()){
        //     std::string cmd_str_0 = "rosnode kill " + node_name_;
        //     int ret = system(cmd_str_0.c_str());
        //     std::cout << "## stop rosbag record cmd: " << cmd_str_0 << std::endl;
        // }
    }
    // Livox_ptr->close();

    std::cout << "close" << std::endl;

//    ros::shutdown();
//
//    if(Modes_ptr->isOpenMid70){
//        std::string cmd_str_1 = "rosnode kill -a";
//        int ret = system(cmd_str_1.c_str());
//    }


    MainCam_Image_ptr->Close();
    SecCam_Image_ptr->Close();
}

