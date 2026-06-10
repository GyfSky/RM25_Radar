#include "../include/Radar.h"

double getTimeByRosTime(rclcpp::Time ros_time){
    double ros_time_value =ros_time.nanoseconds()/1e9;
    return ros_time_value;
}

PictureSource MyRadar::getPictureSource() {
    return this->Modes_ptr->pictureSource;
}

MyRadar::MyRadar(rclcpp::Node::SharedPtr node){
    this->node = node;
    after = 1490;bafter = after;int start = 0;

    std::string pkg_share = ament_index_cpp::get_package_share_directory("radar_bringup");
    config_path = std::filesystem::path(pkg_share) / "config/Config.yaml";

    node->declare_parameter<bool>("debug",true);
    node->declare_parameter<int>("dartValue",200);
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

    //网络相关
    this->MainCam_Net_ptr   = std::shared_ptr<Net>(new Net(config_path,"net_60"));
    this->Armor_Net_ptr   = std::shared_ptr<Net>(new Net(config_path,"net_armor"));

    this->MainCam_ptr = std::shared_ptr<SensorParam>(new SensorParam("Hik60",Modes_ptr->ourPattern,config_path));
    // this->MainCam_ptr = std::shared_ptr<SensorParam>(new SensorParam("TDT",CamPosition::left,Modes_ptr->ourPattern));

    this->costMatrix_ptr  = std::shared_ptr<CostMatrix>(new CostMatrix(Modes_ptr->ourPattern,this->config_path));
    //获取图像
    this->MainCam_Image_ptr = std::shared_ptr<Image>(
        new Image(Modes_ptr->application,this->config_path,Modes_ptr->pictureSource, "DA0926631",node.get(), "Hik60",start));

    if(!is_one_cam){
        this->SecMapGraph_ptr = std::shared_ptr<MapGraphMtx>(new MapGraphMtx(Modes_ptr->ourPattern));
        this->SecCam_Net_ptr   = std::shared_ptr<Net>(new Net(config_path,"net"));
            std::this_thread::sleep_for(std::chrono::milliseconds (10));
        this->SecCam_ptr = std::shared_ptr<SensorParam>(new SensorParam("Hik30",Modes_ptr->ourPattern,config_path));
        this->SecCam_Image_ptr = std::shared_ptr<Image>(
                new Image(Common,this->config_path,Modes_ptr->pictureSource, "00F26632053",node.get(), "Hik30",start));
        this->PretreatObjs_ptr = std::shared_ptr<PretreatObjs>(new PretreatObjs(this->MainCam_ptr, this->SecCam_ptr, false,this->config_path));
    }else{
        this->PretreatObjs_ptr = std::shared_ptr<PretreatObjs>(new PretreatObjs(Modes_ptr->ourPattern,this->config_path));
    }

    this->CooSystem_ptr = std::shared_ptr<MatrixCoordinateSystem>(new MatrixCoordinateSystem(PretreatObjs_ptr->classWithoutCar));//坐标转换
    this->classWithoutCar=PretreatObjs_ptr->classWithoutCar;
    this->BYTETracker_ptr = std::shared_ptr<BYTETracker>(new BYTETracker(Modes_ptr->ourPattern,this->CooSystem_ptr,this->config_path));

    //串口
    this->Port_ptr = std::shared_ptr<Port>(new Port(Modes_ptr->ourPattern, PretreatObjs_ptr->half_classWithoutCar, Modes_ptr->Port_isOpen, Modes_ptr->usePort, config_path,node.get()));
    this->STrackInit(this->classWithoutCar, Modes_ptr->ourPattern);

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
    tunnel_slanted_[0]=cv::Point2f(16.059,2.217);
    tunnel_slanted_[1]=cv::Point2f(18.101,5.147);
    tunnel_slanted_[2]=cv::Point2f(19.377,4.725);
    tunnel_slanted_[3]=cv::Point2f(17.713,2.217);

    callBackGroup_=node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);//重入（Reentrant：每时刻允许多个线程） 互斥（MutuallyExclusive：每时刻只允许1个线程）
    timerGroup_=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group=callBackGroup_;

    this->cluster_rect_sub_=this->node->create_subscription<interfaces::msg::ClusterRect>("/lidar_rects",1,std::bind(&MyRadar::rectsCallBack,this,std::placeholders::_1),options);
    this->sub_main_img = this->node->create_subscription<sensor_msgs::msg::CompressedImage>("/cam/Hik60", rclcpp::SensorDataQoS(),std::bind(&MyRadar::cam1CallBack,this,std::placeholders::_1),options);
    this->sub_sec_img = this->node->create_subscription<sensor_msgs::msg::CompressedImage>("/cam/Hik30", rclcpp::SensorDataQoS(),std::bind(&MyRadar::cam2CallBack,this,std::placeholders::_1),options);
    this->sub_lidar_det= this->node->create_subscription<interfaces::msg::DetectResult>("/lidar_detect", 1,std::bind(&MyRadar::lidarDetCallBack,this,std::placeholders::_1),options);
    this->sub_lidar_enh = this->node->create_subscription<interfaces::msg::LidarEnhance>("/lidar_enhance", 1,std::bind(&MyRadar::lidarEnhanceCallBack,this,std::placeholders::_1),options);
    this->sub_drone_location=this->node->create_subscription<interfaces::msg::DroneLocation>("/drone_location", 1,std::bind(&MyRadar::droneCallBack,this,std::placeholders::_1),options);

    this->detect_pub=this->node->create_publisher<interfaces::msg::DetectFrame>("/resolve_result", 10);
    this->res_pub=this->node->create_publisher<interfaces::msg::DetectRes>("/cam_result", 3);
    this->cluster_target_pub_=this->node->create_publisher<interfaces::msg::ClusterTarget>("/cluster_target", 1);

    //debug
    this->map_pub_=this->node->create_publisher<sensor_msgs::msg::Image>("/game_map", rclcpp::SensorDataQoS());
    this->main_pub_=this->node->create_publisher<sensor_msgs::msg::Image>("/cam_main", rclcpp::SensorDataQoS());;
    this->sec_pub_=this->node->create_publisher<sensor_msgs::msg::Image>("/cam_sec", rclcpp::SensorDataQoS());;
    this->img1_pub_=this->node->create_publisher<sensor_msgs::msg::Image>("/cam_img1", rclcpp::SensorDataQoS());;
    this->img2_pub_=this->node->create_publisher<sensor_msgs::msg::Image>("/cam_img2", rclcpp::SensorDataQoS());;
    this->dart_pub_=this->node->create_publisher<sensor_msgs::msg::Image>("/dart_img", rclcpp::SensorDataQoS());;

    timer_=node->create_wall_timer(std::chrono::milliseconds(100), [this](){
        if (!this->is_init) return;
        auto now_time = std::chrono::steady_clock::now();
        this->Spin();
        auto end_time = std::chrono::steady_clock::now();
        float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
        std::cout<<"\033[31m"<<"time is : "<<dur_time/1000<<" s"<<"\033[0m"<<std::endl;
    },
    timerGroup_);
    std::cout << "---------------------make is ok1------------------" << std::endl;

}

MyRadar::MyRadar(rclcpp::Node::SharedPtr node,bool flag){
    this->node = node;
    std::string pkg_share = ament_index_cpp::get_package_share_directory("radar_bringup");
    config_path = std::filesystem::path(pkg_share) / "config/Config.yaml";

    after = 1490;bafter = after;int start = 0;

    this->Modes_ptr = std::shared_ptr<Modes>(new Modes());

    if (this->Modes_ptr->camNumber==1) this->is_one_cam=true;
    else if (this->Modes_ptr->camNumber==2) this->is_one_cam=false;

    this->MainMapGraph_ptr = std::shared_ptr<MapGraphMtx>(new MapGraphMtx(Modes_ptr->ourPattern));
    this->MainCam_ptr = std::shared_ptr<SensorParam>(new SensorParam("Hik60",Modes_ptr->ourPattern,config_path));
    this->MainCam_Image_ptr = std::shared_ptr<Image>(
        new Image(Modes_ptr->application,this->config_path,Modes_ptr->pictureSource, "DA0926631",node.get(), "Hik60",start));
    if(!is_one_cam){
        this->SecMapGraph_ptr = std::shared_ptr<MapGraphMtx>(new MapGraphMtx(Modes_ptr->ourPattern));
        this->SecCam_ptr = std::shared_ptr<SensorParam>(new SensorParam("Hik30",Modes_ptr->ourPattern,config_path));
        this->SecCam_Image_ptr = std::shared_ptr<Image>(
                new Image(Common,this->config_path,Modes_ptr->pictureSource, "00F26632053",node.get(), "Hik30",start));
    }

    this->CooSystem_ptr = std::shared_ptr<MatrixCoordinateSystem>(new MatrixCoordinateSystem(10));//坐标转换
    this->classWithoutCar=10;
    this->sub_main_img = this->node->create_subscription<sensor_msgs::msg::CompressedImage>("/cam/Hik60", rclcpp::SensorDataQoS(),std::bind(&MyRadar::cam1CallBack,this,std::placeholders::_1));
    this->sub_sec_img = this->node->create_subscription<sensor_msgs::msg::CompressedImage>("/cam/Hik30", rclcpp::SensorDataQoS(),std::bind(&MyRadar::cam2CallBack,this,std::placeholders::_1));
    std::cout << "---------------------make is ok1------------------" << std::endl;
}

MyRadar::~MyRadar(){

}

void MyRadar::lidarEnhanceCallBack(const interfaces::msg::LidarEnhance::SharedPtr msg) {
    lidarLock.lock();
    lidar_enhance_=*msg;
    lidarLock.unlock();
}

void MyRadar::lidarDetCallBack(const interfaces::msg::DetectResult::SharedPtr msg) {
    lidarLock.lock();
    lidar_det=*msg;
    lidarLock.unlock();
}

void MyRadar::droneCallBack(const interfaces::msg::DroneLocation::SharedPtr msg) {
    droneLock.lock();
    drone_location_=*msg;
    droneLock.unlock();
}

void MyRadar::cam1CallBack(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    cv::Mat img1 = cv::imdecode(msg->data, cv::IMREAD_COLOR);
    if(!img1.empty()){
        this->flag1=true;
        cam1Lock.lock();
        this->MainCam_Image_ptr->img_temp= img1.clone();
        cam1Lock.unlock();

        timeSyncLock.lock();
        cam1.push_back(img1.clone());
        if (this->Modes_ptr->pictureSource==ros) {
            this->time_now=msg->header.stamp;
        }
        time1.push_back(getTimeByRosTime(msg->header.stamp));
        if (cam1.size()> 60) {
            cam1.erase(cam1.begin());
            time1.erase(time1.begin());
        }
        timeSyncLock.unlock();
    }else{
        std::cout << "error!!!!!" << std::endl;
    }
}

void MyRadar::cam2CallBack(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    cv::Mat img2 = cv::imdecode(msg->data, cv::IMREAD_COLOR);
    if(!img2.empty()){
        this->flag2=true;
        cam2Lock.lock();
        this->SecCam_Image_ptr->img_temp= img2.clone();
        cam2Lock.unlock();

        timeSyncLock.lock();
        cam2.push_back(img2.clone());
        if (this->Modes_ptr->pictureSource==ros) {
            this->time_now=msg->header.stamp;
        }
        time2.push_back(getTimeByRosTime(msg->header.stamp));
        if (cam2.size()>100) {
            cam2.erase(cam2.begin());
            time2.erase(time2.begin());
        }
        timeSyncLock.unlock();
    }else{
        std::cout << "error!!!!!" << std::endl;
    }
}

void MyRadar::rectsCallBack(const interfaces::msg::ClusterRect::SharedPtr msg) {
    auto now_time = std::chrono::steady_clock::now();
    auto startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if(first_sub_lidar_) {
        first_sub_lidar_=false;
        timeSyncLock.lock();
        cam1.clear();
        cam2.clear();
        time1.clear();
        time2.clear();
        timeSyncLock.unlock();
        return;
    }
    double time=getTimeByRosTime(msg->header.stamp),min_gap1=10.0,min_gap2=10.0;
    int index1=-1,index2=-1;

    std::vector<cv::Mat> temp_cam1,temp_cam2;
    std::vector<double> temp_time1,temp_time2;
    timeSyncLock.lock();
    if (cam1.empty()||cam2.empty()) {
        timeSyncLock.unlock();
        return;
    }
    temp_cam1.resize(cam1.size());
    temp_cam2.resize(cam2.size());
    temp_time1.resize(time1.size());
    temp_time2.resize(time2.size());

    temp_cam1.assign(cam1.begin(), cam1.end());
    temp_cam2.assign(cam2.begin(), cam2.end());
    temp_time1.assign(time1.begin(), time1.end());
    temp_time2.assign(time2.begin(), time2.end());
    timeSyncLock.unlock();

        // timeSyncLock.lock();
        // temp_cam1=(cam1);temp_cam2=(cam2);
        // temp_time1=(time1);temp_time2=(time2);

    for (int i=0;i<temp_time1.size();i++) {
        if (fabs(time-temp_time1[i])<min_gap1) {
            min_gap1=fabs(time-temp_time1[i]);
            index1=i;
        }
    }
    for (int i=0;i<temp_time2.size();i++) {
        if (fabs(time-temp_time2[i])<min_gap2) {
            min_gap2=fabs(time-temp_time2[i]);
            index2=i;
        }
    }
    // RCLCPP_ERROR(rclcpp::get_logger("df"),"%f,%f",min_gap1,min_gap2);
    std::vector<cv::Mat> car_imgs;
    cv::Mat img1,img2;
    if (index1!=-1&&index2!=-1) {
        img1=temp_cam1[index1];
        img2=temp_cam2[index2];
    }
    // timeSyncLock.unlock();

    if (index1!=-1&&index2!=-1&&min_gap1<0.2&&min_gap2<0.2) {
        interfaces::msg::ClusterRect lidar_rects;
        auto net_config = YAML::LoadFile(this->config_path);
        for (int j=0;j<msg->rects1.size();j++) {
            double side_length1=1700.0/sqrt(pow(msg->rects1[j].x,2)+pow(msg->rects1[j].y,2)+pow(msg->rects1[j].z,2));
            cv::Point p1,p2;
            p1=cv::Point(msg->rects1[j].cx-side_length1/2.0,msg->rects1[j].cy-side_length1/2.0);
            p2=cv::Point(msg->rects1[j].cx+side_length1/2.0,msg->rects1[j].cy+side_length1/2.0);
            if (p2.x<=0||p2.y<=0||p1.x>=img1.cols||p1.y>=img1.rows) continue;
            if (p1.x<0&&p2.x>0) p1.x=0;
            if (p1.y<0&&p2.y>0) p1.y=0;
            if (p1.x<img1.cols&&p2.x>img1.cols) p2.x=img1.cols;
            if (p1.y<img1.rows&&p2.y>img1.rows) p2.y=img1.rows;

            lidar_rects.rects1.push_back(msg->rects1[j]);
            lidar_rects.rects1.back().x1=p1.x;
            lidar_rects.rects1.back().y1=p1.y;
            lidar_rects.rects1.back().x2=p2.x;
            lidar_rects.rects1.back().y2=p2.y;
        }

        for (int j=0;j<msg->rects2.size();j++) {
            cv::Point p1,p2;

            double side_length2=1100.0/sqrt(pow(msg->rects2[j].x,2)+pow(msg->rects2[j].y,2)+pow(msg->rects2[j].z,2));
            p1=cv::Point(msg->rects2[j].cx-side_length2/2.0,msg->rects2[j].cy-side_length2/2.0);
            p2=cv::Point(msg->rects2[j].cx+side_length2/2.0,msg->rects2[j].cy+side_length2/2.0);
            if (p2.x<=0||p2.y<=0||p1.x>=img2.cols||p1.y>=img2.rows) continue;
            if (p1.x<0&&p2.x>0) p1.x=0;
            if (p1.y<0&&p2.y>0) p1.y=0;
            if (p1.x<img2.cols&&p2.x>img2.cols) p2.x=img2.cols;
            if (p1.y<img2.rows&&p2.y>img2.rows) p2.y=img2.rows;

            lidar_rects.rects2.push_back(msg->rects2[j]);
            lidar_rects.rects2.back().x1=p1.x;
            lidar_rects.rects2.back().y1=p1.y;
            lidar_rects.rects2.back().x2=p2.x;
            lidar_rects.rects2.back().y2=p2.y;
        }
        std::vector<bool> isSuppressed1(lidar_rects.rects1.size(), false);
        std::vector<bool> isSuppressed2(lidar_rects.rects2.size(), false);

        std::vector<bool> isFlags1(lidar_rects.rects1.size(), false);
        std::vector<bool> isFlags2(lidar_rects.rects2.size(), false);

        //点云的类非极大值抑制，效果不好，没用
        // for (int i=0;i<lidar_rects.rects1.size();i++) {
        //     if (isSuppressed1[i]) continue;
        //     cv::Rect rect1=cv::Rect(cv::Point(lidar_rects.rects1[i].x1,lidar_rects.rects1[i].y1),
        //         cv::Point(lidar_rects.rects1[i].x2,lidar_rects.rects1[i].y2));
        //     double dis1=sqrt(pow(lidar_rects.rects1[i].x,2)+pow(lidar_rects.rects1[i].y,2));
        //     for (int j=i+1;j<lidar_rects.rects1.size();j++) {
        //         if (isSuppressed1[j]) continue;
        //         cv::Rect rect2=cv::Rect(cv::Point(lidar_rects.rects1[j].x1,lidar_rects.rects1[j].y1),
        //         cv::Point(lidar_rects.rects1[j].x2,lidar_rects.rects1[j].y2));
        //         cv::Rect Intersection = rect1 & rect2;
        //         cv::Rect Union = rect1|rect2;
        //         double iou=double(Intersection.area())/double(Union.area());
        //         double dis2=sqrt(pow(lidar_rects.rects1[j].x,2)+pow(lidar_rects.rects1[j].y,2));
        //         if (iou>0.2&&dis1>dis2) {
        //             isSuppressed1[i]=true;
        //         }else if (iou>0.2) {
        //             isSuppressed1[j]=true;
        //         }
        //         // if (iou>0.25) {
        //         //     isSuppressed1[i]=true;
        //         //     isSuppressed1[j]=true;
        //         // }
        //     }
        // }
        // for (int i=0;i<lidar_rects.rects2.size();i++) {
        //     if (isSuppressed2[i]) continue;
        //     cv::Rect rect1=cv::Rect(cv::Point(lidar_rects.rects2[i].x1,lidar_rects.rects2[i].y1),
        //         cv::Point(lidar_rects.rects2[i].x2,lidar_rects.rects2[i].y2));
        //     double dis1=sqrt(pow(lidar_rects.rects2[i].x,2)+pow(lidar_rects.rects2[i].y,2));
        //     for (int j=i+1;j<lidar_rects.rects2.size();j++) {
        //         if (isSuppressed2[j]) continue;
        //         cv::Rect rect2=cv::Rect(cv::Point(lidar_rects.rects2[j].x1,lidar_rects.rects2[j].y1),
        //         cv::Point(lidar_rects.rects2[j].x2,lidar_rects.rects2[j].y2));
        //         cv::Rect Intersection = rect1 & rect2;
        //         cv::Rect Union = rect1|rect2;
        //         double iou=double(Intersection.area())/double(Union.area());
        //         double dis2=sqrt(pow(lidar_rects.rects2[j].x,2)+pow(lidar_rects.rects2[j].y,2));
        //         if (iou>0.2&&dis1>dis2) {
        //             isSuppressed2[i]=true;
        //         }else if (iou>0.2) {
        //             isSuppressed2[j]=true;
        //         }
        //         // if (iou>0.25) {
        //         //     isSuppressed2[i]=true;
        //         //     isSuppressed2[j]=true;
        //         // }
        //     }
        // }

        for (int i=lidar_rects.rects1.size()-1;i>=0;i--) {
            if (isSuppressed1[i])
                lidar_rects.rects1.erase(lidar_rects.rects1.begin()+i);
            else {
                cv::rectangle(img1,cv::Point(lidar_rects.rects1[i].x1,lidar_rects.rects1[i].y1),
                   cv::Point(lidar_rects.rects1[i].x2,lidar_rects.rects1[i].y2),cv::Scalar(0,255,0),2);
                if (initornot(tunnel_slanted_,cv::Point2d(lidar_rects.rects1[i].x,lidar_rects.rects1[i].y),4)==1)
                    isFlags1[i]=true;
            }
        }

        for (int i=lidar_rects.rects2.size()-1;i>=0;i--){
            if (isSuppressed2[i])
                lidar_rects.rects2.erase(lidar_rects.rects2.begin()+i);
            else {
                cv::rectangle(img2,cv::Point(lidar_rects.rects2[i].x1,lidar_rects.rects2[i].y1),
                    cv::Point(lidar_rects.rects2[i].x2,lidar_rects.rects2[i].y2),cv::Scalar(0,255,0),2);
                if (initornot(tunnel_slanted_,cv::Point2d(lidar_rects.rects2[i].x,lidar_rects.rects2[i].y),4)==1)
                    isFlags2[i]=true;
            }
        }

        MainCam_Net_ptr->getCarImgs(lidar_rects,img1,car_imgs,1);
        MainCam_Net_ptr->getCarImgs(lidar_rects,img2,car_imgs,2);

        //保存数据集
        // std::string path111="/home/thesky/cars/";
        // save_count++;
        // if (save_count==2) {
        //     save_count=0;
        //     for (int i(0); i < int(car_imgs.size()); ++i)
        //     {
        //         std::string path=path111+getDate()+std::to_string(pic_num)+"_"+std::to_string(i)+".jpg";
        //         cv::imwrite(path,car_imgs[i]);
        //     }
        //     pic_num++;
        // }

        vector<vector<TRTInferV1::DetectionObj>> Armors ;
        netLock.lock();
        for (int i=0;i<car_imgs.size();i++) {
            std::vector<Mat> car_img_s(1);
            car_img_s[0].push_back(car_imgs[i]);
            vector<TRTInferV1::DetectionObj> armor=(Armor_Net_ptr->myInfer.doInference(car_img_s,0.65, 0.45))[0];
            if (armor.empty()&&i<lidar_rects.rects1.size()) {
                double side_length1=1.25*1700.0/sqrt(pow(msg->rects1[i].x,2)+pow(msg->rects1[i].y,2)+pow(msg->rects1[i].z,2));
                cv::Point p1,p2;
                p1=cv::Point(msg->rects1[i].cx-side_length1/2.0,msg->rects1[i].cy-side_length1/2.0);
                p2=cv::Point(msg->rects1[i].cx+side_length1/2.0,msg->rects1[i].cy+side_length1/2.0);

                if (!(p1.x>=0 && p1.y>=0 && p2.x<=img1.cols && p2.y<=img1.rows)) {
                    if (p2.x<=0||p2.y<=0||p1.x>=img1.cols||p1.y>=img1.rows) {
                        Armors.push_back(armor);
                        continue;
                    }
                    if (p1.x<0&&p2.x>0) p1.x=0;
                    if (p1.y<0&&p2.y>0) p1.y=0;
                    if (p1.x<img1.cols&&p2.x>img1.cols) p2.x=img1.cols;
                    if (p1.y<img1.rows&&p2.y>img1.rows) p2.y=img1.rows;
                }
                // cv::rectangle(img1,p1,p2,cv::Scalar(0,255,0),2);
                lidar_rects.rects1[i].x1=p1.x;
                lidar_rects.rects1[i].y1=p1.y;
                lidar_rects.rects1[i].x2=p2.x;
                lidar_rects.rects1[i].y2=p2.y;
                car_img_s[0]=img1(cv::Rect(p1,p2));
                car_imgs[i]=img1(cv::Rect(p1,p2));
                armor=(Armor_Net_ptr->myInfer.doInference(car_img_s,0.65, 0.45))[0];
            }else if (armor.empty()) {
                double side_length2=1.3*1100.0/sqrt(pow(msg->rects2[i-lidar_rects.rects1.size()].x,2)+pow(msg->rects2[i-lidar_rects.rects1.size()].y,2)+pow(msg->rects2[i-lidar_rects.rects1.size()].z,2));
                cv::Point p1,p2;
                p1=cv::Point(msg->rects2[i-lidar_rects.rects1.size()].cx-side_length2/2.0,msg->rects2[i-lidar_rects.rects1.size()].cy-side_length2/2.0);
                p2=cv::Point(msg->rects2[i-lidar_rects.rects1.size()].cx+side_length2/2.0,msg->rects2[i-lidar_rects.rects1.size()].cy+side_length2/2.0);

                if (!(p1.x>=0 && p1.y>=0 && p2.x<=img2.cols && p2.y<=img2.rows)) {
                    if (p2.x<=0||p2.y<=0||p1.x>=img2.cols||p1.y>=img2.rows) {
                        Armors.push_back(armor);
                        continue;
                    }
                    if (p1.x<0&&p2.x>0) p1.x=0;
                    if (p1.y<0&&p2.y>0) p1.y=0;
                    if (p1.x<img2.cols&&p2.x>img2.cols) p2.x=img2.cols;
                    if (p1.y<img2.rows&&p2.y>img2.rows) p2.y=img2.rows;
                }
                // cv::rectangle(img2,p1,p2,cv::Scalar(0,255,0),2);
                lidar_rects.rects2[i-lidar_rects.rects1.size()].x1=p1.x;
                lidar_rects.rects2[i-lidar_rects.rects1.size()].y1=p1.y;
                lidar_rects.rects2[i-lidar_rects.rects1.size()].x2=p2.x;
                lidar_rects.rects2[i-lidar_rects.rects1.size()].y2=p2.y;
                car_img_s[0]=img2(cv::Rect(p1,p2));
                car_imgs[i]=img2(cv::Rect(p1,p2));
                armor=(Armor_Net_ptr->myInfer.doInference(car_img_s,0.65, 0.45))[0];
            }
            Armors.push_back(armor);
        }
        netLock.unlock();

        //保存数据集
        // std::string path111="/home/thesky/armors/";
        // save_count++;
        // if (save_count==30) {
        //     save_count=0;
        //     for (int i(0); i < int(car_imgs.size()); ++i)
        //     {
        //         for (int j(0); j < int(Armors[i].size()); ++j)
        //         {
        //             cv::Rect r = cv::Rect(Armors[i][j].x1, Armors[i][j].y1, Armors[i][j].x2 - Armors[i][j].x1, Armors[i][j].y2 - Armors[i][j].y1);
        //             std::string path=path111+getDate()+std::to_string(pic_num)+"_"+std::to_string(i)+"_"+std::to_string(j)+".jpg";
        //             cv::imwrite(path,car_imgs[i](r));
        //         }
        //     }
        //     pic_num++;
        // }

        std::map<int,int> id_map={{0,5},{1,0},{2,1},{3,2},{4,3},
            {5,11},{6,6},{7,7},{8,8},{9,9}};
        for (auto i=Armors.begin();i!=Armors.end();i++) {
            for (auto j=i->begin();j!=i->end();j++) {
                j->classId=id_map[j->classId];
            }
        }
        auto process_now_time = std::chrono::steady_clock::now();

        //不经过get_Armors_w_conf_Double_net函数处理，直接输出网络原始结果
        // interfaces::msg::ClusterTarget cluster_target;
        // cluster_target.header.stamp = msg->header.stamp;
        // for (int i=0;i<msg->rects1.size();i++) {
        //     interfaces::msg::CostMatrix target;
        //     for (int i=0;i<10;i++) target.cost_matrix[i]=0;
        //     cluster_target.clusters.push_back(target);
        // }
        // for (int i=0;i<lidar_rects.rects1.size();i++) {
        //     int cluster_id=lidar_rects.rects1[i].cluster_id;
        //     for (auto armor:Armors[i]) {
        //         cv::Rect r = cv::Rect(armor.x1, armor.y1, armor.x2 - armor.x1, armor.y2 - armor.y1);
        //         int class_id=armor.classId;
        //         if (!PretreatObjs_ptr->check_color(car_imgs[i](r),class_id)) continue;
        //         if (class_id>=5&&class_id<=9) {
        //             class_id-=1;
        //         }else if (class_id==11) {
        //             class_id-=2;
        //         }else if (class_id==4||class_id==10) {
        //             continue;
        //         }
        //         for (int j=0;j<10;j++)
        //             cluster_target.clusters[cluster_id].cost_matrix[class_id]+=armor.confidence;
        //     }
        // }
        // for (int i=0;i<lidar_rects.rects2.size();i++) {
        //     int cluster_id=lidar_rects.rects2[i].cluster_id;
        //     for (auto armor:Armors[i+lidar_rects.rects1.size()]) {
        //         cv::Rect r = cv::Rect(armor.x1, armor.y1, armor.x2 - armor.x1, armor.y2 - armor.y1);
        //         int class_id=armor.classId;
        //         if (!PretreatObjs_ptr->check_color(car_imgs[i+lidar_rects.rects1.size()](r),class_id)) continue;
        //         if (class_id>=5&&class_id<=9) {
        //             class_id-=1;
        //         }else if (class_id==11) {
        //             class_id-=2;
        //         }else if (class_id==4||class_id==10) {
        //             continue;
        //         }
        //         for (int j=0;j<10;j++)
        //             cluster_target.clusters[cluster_id].cost_matrix[class_id]+=armor.confidence;
        //     }
        // }
        // cluster_target.self_color=Modes_ptr->ourPattern;
        // cluster_target_pub_->publish(cluster_target);

        int cam_num = 2;
        float p =0.97;
        float match_thresh = 0.85;
        std::vector<std::vector<STrack>> stracks(cam_num);

        for(auto car : lidar_rects.rects1){
            STrack sTrack(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), 0.9,this->classWithoutCar,car.cluster_id);
            sTrack.setRectInPrimaryCam(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), p);
            sTrack.camid=1;
            stracks[0].push_back(sTrack);
        }

        for(auto car : lidar_rects.rects2){
            TRTInferV1::DetectionObj mainobj;
            mainobj = PretreatObjs_ptr->objs2newMainObjs(car, PretreatObjs_ptr->H2);
            STrack sTrack(mainobj.x1, mainobj.y1,mainobj.x2-mainobj.x1, mainobj.y2-mainobj.y1, mainobj.confidence,this->classWithoutCar,car.cluster_id);
            sTrack.setRectInPrimaryCam(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), p);
            sTrack.camid=2;
            sTrack.sec_rect.resize(4);
            sTrack.sec_rect = {car.x1, car.y1, car.x2, car.y2};
            stracks[1].push_back(sTrack);
        }

        int car_num = 0;
        for(int cam=0; cam<cam_num ; cam++){
            int cam_cars_num = stracks[cam].size();
            std::vector<int> remove_lists;
            for(int i=0;i<cam_cars_num;i++){
                bool flag=PretreatObjs_ptr->get_Armors_w_conf_Double_net(stracks[cam][i],Armors[i+car_num],car_imgs[i+car_num]);
                if (flag) remove_lists.push_back(i);
            }
            sort(remove_lists.begin(),remove_lists.end(),std::greater<>());
            for (auto i:remove_lists) {
                stracks[cam].erase(stracks[cam].begin()+i);
                if (cam==0)
                    isFlags1.erase(isFlags1.begin()+i);
                else if (cam==1)
                    isFlags2.erase(isFlags2.begin()+i);
                // stracks[cam][i].cls=-1;
            }
            car_num += cam_cars_num;
        }

        //测试发现只能这么将变量整体赋值加锁
        //若对整个处理过程加锁会跑死(红方)
        accTimeLock.lock();
        auto acc_time=acc_time_;
        accTimeLock.unlock();
        int status[5]{};
        if (this->ourPattern==red) {
            int i=0;
            for (auto strack=stracks[0].begin();strack!=stracks[0].end();strack++) {
                if (isFlags1[i]&&stracks[0][i].cls>=5) {
                    acc_time[stracks[0][i].cls-5]++;
                    if (acc_time[stracks[0][i].cls-5]>=3)
                        status[stracks[0][i].cls-5]=2;
                    else {
                        status[stracks[0][i].cls-5]=1;
                        strack->cls=10;
                        strack->ws_armorConfMatrix=Eigen::MatrixXd::Zero(1,10);
                    }
                }
                i++;
            }
            i=0;
            for (auto strack=stracks[1].begin();strack!=stracks[1].end();strack++) {
                if (isFlags2[i]&&stracks[1][i].cls>=5) {
                    acc_time[stracks[1][i].cls-5]++;
                    if (acc_time[stracks[1][i].cls-5]>=3)
                        status[stracks[1][i].cls-5]=2;
                    else {
                        status[stracks[1][i].cls-5]=1;
                        strack->cls=10;
                        strack->ws_armorConfMatrix=Eigen::MatrixXd::Zero(1,10);
                    }
                }
                i++;
            }
        }else if (this->ourPattern==blue) {
            int i=0;
            for (auto strack=stracks[0].begin();strack!=stracks[0].end();strack++) {
                if (isFlags1[i]&&stracks[0][i].cls<5) {
                    acc_time[stracks[0][i].cls]++;
                    if (acc_time[stracks[0][i].cls]>=3)
                        status[stracks[0][i].cls]=2;
                    else {
                        status[stracks[0][i].cls]=1;
                        strack->cls=10;
                        strack->ws_armorConfMatrix=Eigen::MatrixXd::Zero(1,10);
                    }
                }
                i++;
            }
            i=0;
            for (auto strack=stracks[1].begin();strack!=stracks[1].end();strack++) {
                if (isFlags2[i]&&stracks[1][i].cls<5) {
                    acc_time[stracks[1][i].cls]++;
                    if (acc_time[stracks[1][i].cls]>=3)
                        status[stracks[1][i].cls]=2;
                    else {
                        status[stracks[1][i].cls]=1;
                        strack->cls=10;
                        strack->ws_armorConfMatrix=Eigen::MatrixXd::Zero(1,10);
                    }
                }
                i++;
            }
        }
        for (int i=0;i<5;i++) {
            if (status[i]==0) {
                acc_time[i]=0;
            }
            else if (status[i]==2) {
                acc_time[i]=2;
            }
        }

        accTimeLock.lock();
        acc_time_=acc_time;
        accTimeLock.unlock();

        interfaces::msg::ClusterTarget cluster_target;
        cluster_target.header.stamp = msg->header.stamp;
        for (int i=0;i<msg->rects1.size();i++) {
            interfaces::msg::CostMatrix target;
            for (int i=0;i<10;i++) target.cost_matrix[i]=0;
            cluster_target.clusters.push_back(target);
        }

        for (int i=0;i<stracks[0].size();i++) {
            int cluster_id=stracks[0][i].cluster_id;
            for (int j=0;j<10;j++)
                cluster_target.clusters[cluster_id].cost_matrix[j]+=stracks[0][i].ws_armorConfMatrix(0,j);
            int id=stracks[0][i].cls;
            if (id>=0&&id<=9)
                putText(img1, net_config["class_mapping"][id].as<std::string>(), Point(stracks[0][i].tlbr[0], stracks[0][i].tlbr[1] - 5),0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
            // else if (id==10)
            //     putText(img1, "car", Point(stracks[0][i].tlbr[0], stracks[0][i].tlbr[1] - 5),0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);

        }
        for (int i=0;i<stracks[1].size();i++) {
            int cluster_id=stracks[1][i].cluster_id;
            for (int j=0;j<10;j++)
                cluster_target.clusters[cluster_id].cost_matrix[j]+=stracks[1][i].ws_armorConfMatrix(0,j);
            int id=stracks[1][i].cls;
            if (id>=0&&id<=9)
                putText(img2, net_config["class_mapping"][id].as<std::string>(), Point(stracks[1][i].sec_rect[0], stracks[1][i].sec_rect[1] - 5),0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
            // else if (id==-1)
            //     putText(img2, "car", Point(stracks[1][i].sec_rect[0], stracks[1][i].sec_rect[1] - 5),0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
        }
        cluster_target.self_color=Modes_ptr->ourPattern;
        cluster_target_pub_->publish(cluster_target);

        for (int i=0;i<lidar_rects.rects1.size(); i++) {
            for (auto armor:Armors[i]) {
                int id=armor.classId;
                if (id>=5&&id<=9) {
                    id-=1;
                }else if (id==11) {
                    id-=2;
                }else if (id==4||id==10) {
                    continue;
                }
                putText(img1, net_config["class_mapping"][id].as<std::string>(), Point(armor.x1+lidar_rects.rects1[i].x1, armor.y1+lidar_rects.rects1[i].y1 - 5),0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
            }
        }
        for (int i=0;i<lidar_rects.rects2.size(); i++) {
            for (auto armor:Armors[i+lidar_rects.rects1.size()]) {
                int id=armor.classId;
                if (id>=5&&id<=9) {
                    id-=1;
                }else if (id==11) {
                    id-=2;
                }else if (id==4||id==10) {
                    continue;
                }
                putText(img2, net_config["class_mapping"][id].as<std::string>(), Point(armor.x1+lidar_rects.rects2[i].x1, armor.y1+lidar_rects.rects2[i].y1 - 5),0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
            }
        }

        bool is_debug=this->node->get_parameter("debug").as_bool();
        if (is_debug) {
            auto msg=cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img1).toImageMsg();
            msg->header.stamp = rclcpp::Clock().now();
            this->img1_pub_->publish(*msg);

            msg=cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img2).toImageMsg();
            this->img2_pub_->publish(*msg);
        }
    }else {
        interfaces::msg::ClusterTarget cluster_target;
        cluster_target.header.stamp = msg->header.stamp;
        for (int i=0;i<msg->rects1.size();i++) {
            interfaces::msg::CostMatrix target;
            for (int i=0;i<10;i++) target.cost_matrix[i]=0;
            cluster_target.clusters.push_back(target);
        }
        cluster_target.self_color=Modes_ptr->ourPattern;
        cluster_target_pub_->publish(cluster_target);
    }
    auto end_time = std::chrono::steady_clock::now();
    float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
    RCLCPP_ERROR(rclcpp::get_logger("cam"), "Callback time is %f ms", dur_time);
}

void MyRadar::STrackInit(int classWithoutCar, OurPattern ourPattern){
    out_init.resize(classWithoutCar);
    if (classWithoutCar==12) {
        //需要跟据yaml的改变而改变
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
        //需要跟据yaml的改变而改变
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

void MyRadar::getDartWarning(cv::Mat img) {
    int dartValue=this->node->get_parameter("dartValue").as_int();
    cv::Mat temp=img(rect);
    cv::Mat rect_img=temp.clone();
    cv::cvtColor(rect_img,rect_img,CV_BGR2GRAY);
    cv::threshold(rect_img, rect_img, dartValue, 255, cv::THRESH_BINARY);
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
            // RCLCPP_ERROR(node->get_logger(), "Dart area: %f",minRect.size.area());
            // RCLCPP_ERROR(node->get_logger(), "Dart height: %f, Dart width: %f",minRect.size.height,minRect.size.width);
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
    // cv::imshow("Dart", rect_img);
    bool is_debug=this->node->get_parameter("debug").as_bool();
    if (is_debug) {
        auto msg=cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", contourImage).toImageMsg();
        msg->header.stamp = rclcpp::Clock().now();
        this->dart_pub_->publish(*msg);
    }
}

void MyRadar::getRivalOffenseWarning(vector<bool> &isWarring) {
    for (int i=0;i<5;i++) {
        if (this->out[i+color_index].lost_frame_ind_num>9) continue;
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

void MyRadar::calib() {
    if(is_init) return;
    MainCam_Image_ptr->Init_calib();
    if(!is_one_cam){
        SecCam_Image_ptr->Init_calib();
    }
    std::vector<cv::Point2d> pts_pnp_2d_main, pts_pnp_2d_sec;
    if(MainCam_Image_ptr->is_getPoint2d_mouse_Cam ){
        while(mainCamMat.empty() ){
            spin_some(node);
            if (this->getPictureSource()==ros) {
                cam1Lock.lock();
                MainCam_Image_ptr->Cam_img=MainCam_Image_ptr->img_temp;
                cam1Lock.unlock();
            }
            mainCamMat = MainCam_Image_ptr->Image_Get(after);//after 为图片序号，通过加减after选择不同图片
        }
        MainCam_Image_ptr->Image_Show();

        MainCam_ptr->pts_pnp_2d = GetPoint2d_mouse(mainCamMat,"Hik60",this->config_path);
        CooSystem_ptr->Get2world_matrix(MainCam_ptr->T_2world  ,MainCam_ptr->K ,MainCam_ptr->pts_pnp_2d , MainCam_ptr->pts_pnp_3d);
        std::ofstream fout("resource/main2world.txt");
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
            spin_some(node);
            if (this->getPictureSource()==ros) {
                cam2Lock.lock();
                SecCam_Image_ptr->Cam_img=SecCam_Image_ptr->img_temp;
                cam2Lock.unlock();
            }
            secCamMat = SecCam_Image_ptr->Image_Get(after);
        }
        SecCam_Image_ptr->Image_Show();

        rect =GetRect_mouse(secCamMat,"Hik60",config_path);
        SecCam_ptr->pts_pnp_2d = GetPoint2d_mouse(secCamMat,"Hik60",config_path);
        CooSystem_ptr->Get2world_matrix(SecCam_ptr->T_2world  ,SecCam_ptr->K ,SecCam_ptr->pts_pnp_2d , SecCam_ptr->pts_pnp_3d);
        std::ofstream fout("resource/sec2world&rect.txt");
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

        SecCam_Image_ptr->is_getPoint2d_mouse_Cam = false;
        SecMapGraph_ptr->get_predict_2d(SecCam_ptr->T_2world ,SecCam_ptr->fx,SecCam_ptr->fy,SecCam_ptr->cx,SecCam_ptr->cy);
        SecMapGraph_ptr->get_roughH_config();
        SecCam_ptr->setworld2self_config(SecCam_ptr->T_2world);

    }else if(!is_one_cam){
        std::cerr << "error" << std::endl;
    }

    is_init = true;
}

void MyRadar::Init(){
    if(is_init) return;
    MainCam_Image_ptr->Init();
    if(!is_one_cam){
        SecCam_Image_ptr->Init();
    }

    std::vector<cv::Point2d> pts_pnp_2d_main, pts_pnp_2d_sec;
    YAML::Node config = YAML::LoadFile(config_path);
    bool use_saved_T = config["general"]["cam_use_saved_T"].as<bool>();
    if(MainCam_Image_ptr->is_getPoint2d_mouse_Cam ){

        while(mainCamMat.empty() ){
            if (this->getPictureSource()==ros) {
                cam1Lock.lock();
                MainCam_Image_ptr->Cam_img=MainCam_Image_ptr->img_temp;
                cam1Lock.unlock();
            }
            mainCamMat = MainCam_Image_ptr->Image_Get(after);//after 为图片序号，通过加减after选择不同图片
        }
        if(use_saved_T){
            std::ifstream fin("resource/main2world.txt");
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
                MainCam_ptr->pts_pnp_2d = GetPoint2d_mouse(mainCamMat,"Hik60",this->config_path);
                CooSystem_ptr->Get2world_matrix(MainCam_ptr->T_2world  ,MainCam_ptr->K ,MainCam_ptr->pts_pnp_2d , MainCam_ptr->pts_pnp_3d);
                std::ofstream fout("resource/main2world.txt");
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
            MainCam_ptr->pts_pnp_2d = GetPoint2d_mouse(mainCamMat,"Hik60",this->config_path);
            CooSystem_ptr->Get2world_matrix(MainCam_ptr->T_2world  ,MainCam_ptr->K ,MainCam_ptr->pts_pnp_2d , MainCam_ptr->pts_pnp_3d);
            std::ofstream fout("resource/main2world.txt");
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
            if (this->getPictureSource()==ros) {
                cam2Lock.lock();
                SecCam_Image_ptr->Cam_img=SecCam_Image_ptr->img_temp;
                cam2Lock.unlock();
            }
            secCamMat = SecCam_Image_ptr->Image_Get(after);
        }
        if(use_saved_T){
            std::ifstream fin("resource/sec2world&rect.txt");
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
                rect =GetRect_mouse(secCamMat,"Hik60",this->config_path);
                SecCam_ptr->pts_pnp_2d = GetPoint2d_mouse(secCamMat,"Hik60",this->config_path);
                CooSystem_ptr->Get2world_matrix(SecCam_ptr->T_2world  ,SecCam_ptr->K ,SecCam_ptr->pts_pnp_2d , SecCam_ptr->pts_pnp_3d);
                std::ofstream fout("resource/sec2world&rect.txt");
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
            rect =GetRect_mouse(secCamMat,"Hik60",this->config_path);
            SecCam_ptr->pts_pnp_2d = GetPoint2d_mouse(secCamMat,"Hik60",this->config_path);
            CooSystem_ptr->Get2world_matrix(SecCam_ptr->T_2world  ,SecCam_ptr->K ,SecCam_ptr->pts_pnp_2d , SecCam_ptr->pts_pnp_3d);
            std::ofstream fout("resource/sec2world&rect.txt");
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

        SecCam_Image_ptr->is_getPoint2d_mouse_Cam = false;
        SecMapGraph_ptr->get_predict_2d(SecCam_ptr->T_2world ,SecCam_ptr->fx,SecCam_ptr->fy,SecCam_ptr->cx,SecCam_ptr->cy);
        SecMapGraph_ptr->get_roughH_config();
        SecCam_ptr->setworld2self_config(SecCam_ptr->T_2world);

    }else if(!is_one_cam){
        std::cerr << "error" << std::endl;
    }
    this->Save();
    if(Port_ptr->is_openPort) {
        Port_ptr->clearBuff();
        Port_ptr->start();
    }

    PretreatObjs_ptr->set_windmill_car(this->windmill_car);//？？
    this->out.assign(this->out_init.begin(), this->out_init.end());//out为输出结果
    this->to_sentry.assign(this->out_init.begin(), this->out_init.end());
    if(Port_ptr->is_openPort){
        Port_ptr->updataSTrackData(this->out_init);
        Port_ptr->updataSentryData(this->to_sentry);
    }
    is_init = true;
}


void MyRadar::Save() {
    if(Modes_ptr->isSave == true_){
        YAML::Node config = YAML::LoadFile(config_path);
        std::string path = config["save"]["save_bag_path"].as<std::string>() + getDate();
        std::string topics = config["save"]["lidarTopicName"].as<std::string>();

        if(Modes_ptr->pictureSource==camera_){
            this->save_main_dir = config["save"]["save_bag_path"].as<std::string>() + getDate() + "main";
            topics+=" /cam/";
            topics+=MainCam_Image_ptr->Cam_winname;
            mkdir((this->save_main_dir).c_str(), S_IRWXU);
            if(!is_one_cam){
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

void MyRadar::Spin(){
    if(is_one_cam){
        this->STrackClear();
        PretreatObjs_ptr->num = 0;//？？
        DetectionObjs.clear();
        frames.clear();
        int a = this->after*2;
        this->mainCamMat = MainCam_Image_ptr->Image_Get(this->after);
        if(Modes_ptr->pictureSource==camera_)
            this->time_now = MainCam_Image_ptr->ros_time;////实机用时间戳
        // time_now=rclcpp::Clock().now();

        if(!this->mainCamMat.empty() ){
            auto startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            if(bafter !=after){//即当after改变时，才进行下一帧的识别，否则会重复识别 后面会自动加1
                auto now_time = std::chrono::steady_clock::now();
                getDartWarning(mainCamMat);
                std::vector<cv::Mat> main_frames({mainCamMat});
                MainCam_Net_ptr->Spin(main_frames);
                DetectionObjs.push_back( MainCam_Net_ptr->futureObjs.get()[0]);

                std::vector<cv::Mat> car_imgs;
                MainCam_Net_ptr->getCarImgs({DetectionObjs[0]},mainCamMat,car_imgs);

                std::vector<cv::Mat> car_imgs_clone;//疑似没用到
                //第一维为每个车辆检测框，第二维为每个车辆检测框内的装甲板检测框
                vector<vector<TRTInferV1::DetectionObj>> Armors ;

                for(auto car_img:car_imgs){
                    std::vector<Mat> car_img_s(1);
                    car_img_s[0].push_back(car_img);
                    Armors.push_back((Armor_Net_ptr->myInfer.doInference(car_img_s,0.65, 0.45))[0]);
                }
                std::map<int,int> id_map={{0,5},{1,0},{2,1},{3,2},{4,3},
                    {5,11},{6,6},{7,7},{8,8},{9,9}};
                for (auto i=Armors.begin();i!=Armors.end();i++) {
                    for (auto j=i->begin();j!=i->end();j++) {
                        j->classId=id_map[j->classId];
                    }
                }

                int cam_num = 1;
                float p =0.97;
                float match_thresh = 0.85;
                std::vector<std::vector<STrack>> stracks(cam_num);

                auto end_time = std::chrono::steady_clock::now();
                float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
                // std::cout<<"\033[31m"<<"---------net time: "<<dur_time<<" ms"<<"\033[0m"<<std::endl;
                auto trackStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

                for(int i=0; i < cam_num; i++){
                    for(auto car : DetectionObjs[i]){
                        if(i==0){
                            STrack sTrack(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), car.confidence,this->classWithoutCar);
                            //图像中的二维坐标
                            sTrack.setRectInPrimaryCam(car.x1, car.y1,(car.x2-car.x1), (car.y2-car.y1), p);
                            stracks[i].push_back(sTrack);
                        }else{
                            std::cout << "ERROR: IN secObjs2mainObjs about SecCam_ptr" << std::endl;
                        }
                    }
                }

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

                int car_num = 0;
                for(int cam=0; cam<cam_num ; cam++){
                    int cam_cars_num = stracks[cam].size();
                    std::vector<int> remove_lists;
                    for(int i=0;i<cam_cars_num;i++){
                        bool flag=PretreatObjs_ptr->get_Armors_w_conf_Double_net(stracks[cam][i],Armors[i+car_num],car_imgs[i+car_num]);
                        if (flag) remove_lists.push_back(i);
                    }
                    sort(remove_lists.begin(),remove_lists.end(),std::greater<>());
                    for (auto i:remove_lists)
                        stracks[cam].erase(stracks[cam].begin()+i);
                    car_num += cam_cars_num;
                }
                std::vector<STrack> STacks,send_data;

                STacks.insert(STacks.end(),stracks[0].begin(),stracks[0].end());

                std::vector<std::vector<float>> cost_confMatrix_vec;
                int num_strack,num_cls;
                vector<vector<int> > matches_cls;
                vector<int> u_strack, u_cls;
                matches_cls.clear();u_strack.clear();u_cls.clear();
                Eigen::MatrixXd cost_confMatrix = costMatrix_ptr->getCost_confMatrix(STacks,num_strack,num_cls);
                // std::cout << "cost_confMatrix _ cls:  " << std::endl << cost_confMatrix << std::endl;
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
                this->lidarLock.lock();
                interfaces::msg::LidarEnhance lidar_enhance=lidar_enhance_;
                interfaces::msg::DetectResult lidar_det1=lidar_det;
                this->lidarLock.unlock();
                BYTETracker_ptr->update(tracked_stracks,lost_stracks, lost_predict_stracks,STacks, out,to_sentry,lidar_det1,lidar_enhance);
                auto trackEndTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                // std::cout << "fps_track: " << trackEndTime - trackStartTime<< std::endl;

                this->STrackGuess(this->classWithoutCar);
                MainCam_Image_ptr->draw_line(MainMapGraph_ptr->vexs);
                bafter = after;
                MainCam_Image_ptr->draw_result(out, true);

                this->droneLock.lock();
                int drone_location=this->drone_location_.x;
                this->droneLock.unlock();
                getRivalOffenseWarning(isWarring);
                if(Port_ptr->is_openPort){
                    Port_ptr->setWarring(isWarring);
                    Port_ptr->updataSTrackData(this->out);
                    Port_ptr->updataSentryData(this->to_sentry);
                    Port_ptr->updataDroneData(drone_location);
                }
                auto endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                std::cout << "fps: " << 1000. / (endTime - startTime) << std::endl;
            }
        }
        if(cv::waitKey(1) == 'x' || Modes_ptr->pictureSource != picture_dir){
            after++;
        }
    }
    else{
        this->STrackClear();
        PretreatObjs_ptr->num = 0;
        DetectionObjs.clear();
        frames.clear();
        int a = this->after*2;
        if (this->getPictureSource()==ros) {
            cam1Lock.lock();
            MainCam_Image_ptr->Cam_img=MainCam_Image_ptr->img_temp.clone();
            cam1Lock.unlock();
            cam2Lock.lock();
            SecCam_Image_ptr->Cam_img=SecCam_Image_ptr->img_temp.clone();
            cam2Lock.unlock();
        }
        this->mainCamMat = MainCam_Image_ptr->Image_Get(this->after);
        if(Modes_ptr->pictureSource==camera_)
            this->time_now = MainCam_Image_ptr->ros_time;////实机用时间戳
        this->secCamMat = SecCam_Image_ptr->Image_Get(this->after);

        if(!this->mainCamMat.empty() && !this->secCamMat.empty()){
            auto startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            if(bafter !=after){
                getDartWarning(secCamMat);

                std::vector<cv::Mat> main_frames({mainCamMat});
                std::vector<cv::Mat> sec_frames({secCamMat});
                netLock.lock();
                MainCam_Net_ptr->Spin(main_frames);
                SecCam_Net_ptr->Spin(sec_frames);
                netLock.unlock();

                auto result=MainCam_Net_ptr->futureObjs.get();
                if (result.empty()) return;
                DetectionObjs.push_back( result[0]);

                result=SecCam_Net_ptr->futureObjs.get();
                if (result.empty()) return;
                DetectionObjs.push_back( result[0]);

                auto netStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                std::vector<cv::Mat> car_imgs;
                MainCam_Net_ptr->getCarImgs({DetectionObjs[0]},mainCamMat,car_imgs);
                SecCam_Net_ptr->getCarImgs({DetectionObjs[1]},secCamMat, car_imgs);

                // std::string path111="/home/thesky/cars/";
                // save_count++;
                // if (save_count==15) {
                //     save_count=0;
                //     for (int i(0); i < int(car_imgs.size()); ++i)
                //     {
                //         std::string path=path111+getDate()+std::to_string(pic_num)+"_"+std::to_string(i)+".jpg";
                //         cv::imwrite(path,car_imgs[i]);
                //     }
                //     pic_num++;
                // }

                // std::string path111="/home/thesky/cars/";
                // save_count++;
                // if (save_count==30) {
                //     save_count=0;
                //     for (int i(0); i < int(car_imgs.size()); ++i)
                //     {
                //         std::string path=path111+getDate()+std::to_string(pic_num)+"_"+std::to_string(i)+".jpg";
                //         cv::imwrite(path,car_imgs[i]);
                //     }
                //     pic_num++;
                // }

                std::vector<cv::Mat> car_imgs_clone;
                vector<vector<TRTInferV1::DetectionObj>> Armors ;
                netLock.lock();
                for(auto car_img:car_imgs){
                    std::vector<Mat> car_img_s(1);
                    car_img_s[0].push_back(car_img);
                    Armors.push_back((Armor_Net_ptr->myInfer.doInference(car_img_s,0.65, 0.45))[0]);
                }
                netLock.unlock();

                std::map<int,int> id_map={{0,5},{1,0},{2,1},{3,2},{4,3},
                    {5,11},{6,6},{7,7},{8,8},{9,9}};
                for (auto i=Armors.begin();i!=Armors.end();i++) {
                    for (auto j=i->begin();j!=i->end();j++) {
                        j->classId=id_map[j->classId];
                    }
                }
                auto netEndTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                auto dur_time= netEndTime - netStartTime;
                // std::cout<<"\033[31m"<<"---------net time: "<<dur_time<<" ms"<<"\033[0m"<<std::endl;
                auto trackStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

                // std::string path111="/home/thesky/armors/";
                // save_count++;
                // if (save_count==20) {
                //     save_count=0;
                //     for (int i(0); i < int(car_imgs.size()); ++i)
                //     {
                //         for (int j(0); j < int(Armors[i].size()); ++j)
                //         {
                //             cv::Rect r = cv::Rect(Armors[i][j].x1, Armors[i][j].y1, Armors[i][j].x2 - Armors[i][j].x1, Armors[i][j].y2 - Armors[i][j].y1);
                //             std::string path=path111+getDate()+std::to_string(pic_num)+"_"+std::to_string(i)+"_"+std::to_string(j)+".jpg";
                //             cv::imwrite(path,car_imgs[i](r));
                //         }
                //     }
                //     pic_num++;
                // }

                int cam_num = 2;
                float p =0.97;
                float match_thresh = 0.85;
                std::vector<std::vector<STrack>> stracks(cam_num);
                for(int i=0; i < cam_num; i++){
                    for(auto car : DetectionObjs[i]){
                        TRTInferV1::DetectionObj mainobj;
                        if(i==0){
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
                if(Port_ptr->is_openPort) {
                    Port_ptr->updateGameTime(this->game_time_);
                    Port_ptr->updataRadarMarkData(tracked_stracks);
                    Port_ptr->updataRadarMarkData(lost_stracks);
                    Port_ptr->updataRadarMarkData(lost_predict_stracks);
                }

                vector<vector<float> > dists;
                int dist_size, dist_size_size;
                vector<vector<int> > matches;
                vector<int> u_main, u_sec;
                Eigen::MatrixXd cost_matrix = costMatrix_ptr->getIouAndDistancetCost(stracks[0],stracks[1],dist_size, dist_size_size);
                eigenMat2VecVec(cost_matrix,dists);
                // std::cout << "dists: " << std::endl << cost_matrix << std::endl;
                costMatrix_ptr->linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_main, u_sec);

                std::vector<STrack> STacks,send_data;
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
                // std::cout << "cost_confMatrix _ cls:  " << std::endl << cost_confMatrix << std::endl;
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

                this->lidarLock.lock();
                interfaces::msg::LidarEnhance lidar_enhance=lidar_enhance_;
                interfaces::msg::DetectResult lidar_det1=lidar_det;
                this->lidarLock.unlock();
                BYTETracker_ptr->update(tracked_stracks,lost_stracks, lost_predict_stracks,STacks, out,to_sentry,lidar_det1,lidar_enhance);
                auto trackEndTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                // std::cout << "fps_track: " << trackEndTime - trackStartTime << std::endl;

                this->STrackGuess(this->classWithoutCar);
                MainCam_Image_ptr->draw_line(MainMapGraph_ptr->vexs);
                SecCam_Image_ptr->draw_line(SecMapGraph_ptr->vexs);

                bafter = after;
                MainCam_Image_ptr->draw_result(out, true);
                bool is_debug=this->node->get_parameter("debug").as_bool();
                if (is_debug) {
                    auto msg=cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", MainCam_Image_ptr->Cam_draw).toImageMsg();
                    msg->header.stamp = rclcpp::Clock().now();
                    this->main_pub_->publish(*msg);
                    msg=cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", SecCam_Image_ptr->Cam_draw).toImageMsg();
                    this->sec_pub_->publish(*msg);

                    if (MainCam_Image_ptr->application==Radar) {
                        msg=cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", MainCam_Image_ptr->map_draw).toImageMsg();
                        this->map_pub_->publish(*msg);
                    }
                }

                this->droneLock.lock();
                int drone_location=this->drone_location_.x;
                this->droneLock.unlock();
                getRivalOffenseWarning(isWarring);
                if(Port_ptr->is_openPort){
                    Port_ptr->setWarring(isWarring);
                    Port_ptr->updataSTrackData(this->out);
                    Port_ptr->updataSentryData(this->to_sentry);
                    Port_ptr->updataDroneData(drone_location);
                }
                auto endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                std::cout << "fps: " << 1000. / (endTime - startTime) << std::endl;
            }
        }
        if(cv::waitKey(1) == 'x' || (Modes_ptr->pictureSource != picture_dir)){
            after++;
        }
    }
}

void MyRadar::Close(){
    if(Port_ptr->is_openPort) {
        Port_ptr->close();
    }

    MainCam_Image_ptr->Close();
    if(!is_one_cam){
        SecCam_Image_ptr->Close();
    }
}