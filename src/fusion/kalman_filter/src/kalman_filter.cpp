#include "kalman_filter.h"

#include <opencv2/imgproc/types_c.h>

namespace upc_radar{
    KalmanFilter::KalmanFilter(const rclcpp::NodeOptions& node_options):rclcpp::Node("kalman_filter_node",node_options),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_){

        prepareParameter();

        if (is_one_lidar) {
            sub_pc = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_dynamic", 5, std::bind(&KalmanFilter::callback, this, std::placeholders::_1));
            RCLCPP_WARN(this->get_logger(), "is_one_lidar is true, only one lidar will be used.");
        }else {
            mid70_sub.subscribe(this, "/livox/mid70");
            avia_sub.subscribe(this, "/livox/avia");
            MySyncPolicy sync_policy(5);
            sync_policy.setMaxIntervalDuration(rclcpp::Duration(0,100000000));
            sync=std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(std::ref(sync_policy),avia_sub,mid70_sub);
            sync->registerCallback(&KalmanFilter::PCTimeSynC, this);
        }
        drone1_sub_.subscribe(this, "/livox/avia/drone");
        drone2_sub_.subscribe(this, "/livox/mid70/drone");
        MySyncPolicy drone_sync_policy(5);
        drone_sync_policy.setMaxIntervalDuration(rclcpp::Duration(0,100000000));
        drone_sync_=std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(std::ref(drone_sync_policy),drone1_sub_,drone2_sub_);
        drone_sync_->registerCallback(&KalmanFilter::droneTimeSynC, this);

        // sub_detect= this->create_subscription<interfaces::msg::DetectFrame>("/resolve_result", 10, std::bind(&KalmanFilter::detectCallback, this, std::placeholders::_1));

        lidar_enh_pub_=this->create_publisher<interfaces::msg::LidarEnhance>("/lidar_enhance", 1);
        pub_kalman = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_kalman", 10);
        pub_cluster = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", 10);
        pub_kmeans = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_kmeans", 10);
        pub_vis = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vis_point", 10);
        lidar_detect_pub = this->create_publisher<interfaces::msg::DetectResult>("/lidar_detect", 1);
        pub_drone_=this->create_publisher<interfaces::msg::DroneLocation>("/drone_location", 1);

        sub_cam=this->create_subscription<interfaces::msg::DetectRes>("/cam_result", 3, std::bind(&KalmanFilter::camCallback, this, std::placeholders::_1));
        // sub_robot_hp=this->create_subscription<interfaces::msg::RobotHP>("/robot_hp", 10, std::bind(&KalmanFilter::robotHPCallback, this, std::placeholders::_1));

        net_pub=this->create_publisher<sensor_msgs::msg::PointCloud2>("/net_3D", 10);
        pc_pub= this->create_publisher<sensor_msgs::msg::PointCloud2>("/pc_3D", 10);

        show_img1=cv::imread("/home/thesky/RM25_Radar/resource/img/cam1.jpg");
        cv::namedWindow("cam1",0);
        cv::resizeWindow("cam1",640,480);
        cv::imshow("cam1",show_img1);

        show_img2=cv::imread("/home/thesky/RM25_Radar/resource/img/cam2.jpg");
        cv::namedWindow("cam2",0);
        cv::resizeWindow("cam2",640,480);
        cv::imshow("cam2",show_img2);

        RCLCPP_WARN(this->get_logger(), "Kalman_filter_Node has been started.");
    }

    void KalmanFilter::prepareParameter() {
        declare_parameter<double>("kalman.detect_r",3.0);
        declare_parameter<double>("kalman.car_max_speed",2.0);

        declare_parameter<double>("kalman.dt_",0.1);
        declare_parameter<double>("kalman.sigma_q_x",40.0);
        declare_parameter<double>("kalman.sigma_q_y",40.0);
        declare_parameter<double>("kalman.sigma_r_x",5.0);
        declare_parameter<double>("kalman.sigma_r_y",5.0);

        declare_parameter<double>("cost.distance_weight",1.0);
        declare_parameter<double>("cost.distance_thres",6.0);
        declare_parameter<double>("cost.color_weight",0.6);
        declare_parameter<double>("fusion.match_thresh",0.45);
        declare_parameter<double>("clu.match_thres",0.85);
        declare_parameter<double>("clu.dis_thres",1.8);
        declare_parameter<double>("clu.min_dis_thres",0.8);

        declare_parameter<double>("cam.match_thres",0.9);
        declare_parameter<double>("cam.dis_thres",1.8);
        declare_parameter<double>("cam.dis_weight",0.95);
        declare_parameter<double>("cam.his_weight",0.05);
        declare_parameter<double>("cam.time_offset",0.0);
        declare_parameter<int>("classWithoutCar",12);
        declare_parameter<int>("cluster.min_pts",6);
        declare_parameter<double>("cluster.eps",0.4);

        declare_parameter<double>("rect.iou_weight",0.75);
        declare_parameter<double>("rect.dis_weight",0.25);
        declare_parameter<double>("rect.iou_thres",0.05);
        declare_parameter<double>("rect.dis_thres",2.3);
        declare_parameter<double>("rect.match_thres",0.0);
        declare_parameter<bool>("rect.use_rect2d",true);

        is_one_lidar=declare_parameter<bool>("is_one_lidar",false);

        camera_matrix1=cv::Matx33d(2291.14722575626,0,1103.67246,0,2291.55696902824,1086.45687,0,0,1);
        lidar2cam1=cv::Matx44d(0.48001 ,  -0.03327 , 0.87663 ,  0.00706   ,
-0.87188 , -0.12860 , 0.47253 ,  0.26716   ,
0.09701 ,  -0.99114 , -0.09073 , -0.21789 ,
0.00000 ,  0.00000  , 0.00000  , 1.00000 );
        lidar2cam1=lidar2cam1.inv();

        camera_matrix2=cv::Matx33d(1703.57857839016,0,776.621431609538,0,1698.69114037183,558.752840063006,0,0,1);
        lidar2cam2=cv::Matx44d(-0.26869 , -0.07683 , 0.96016 ,  -0.17722  ,
-0.95942 , 0.10986  , -0.25969 , -0.10152  ,
-0.08553 , -0.99097 , -0.10323 , 0.11610   ,
0.00000 ,  0.00000  , 0.00000 ,  1.00000);
        lidar2cam2=lidar2cam2.inv();

        for (int i=0;i<5;i++) {
            robot_hp.blue_robot_hp[i]=100;
            robot_hp.red_robot_hp[i]=100;
        }

        prepareLocation();

    }

    void KalmanFilter::prepareLocation() {
        // tunnel_slanted_[0]=cv::Point2f(15.4988,2.614);
        // tunnel_slanted_[1]=cv::Point2f(16.4014,1.7841);
        // tunnel_slanted_[2]=cv::Point2f(18.8454,4.6387);
        // tunnel_slanted_[3]=cv::Point2f(17.8028,5.206);
        tunnel_slanted_[0]=cv::Point2f(16.0,1.293);
        tunnel_slanted_[1]=cv::Point2f(15.6,2.07);
        tunnel_slanted_[2]=cv::Point2f(18.0,5.50);
        tunnel_slanted_[3]=cv::Point2f(20.658,4.548);
        tunnel_slanted_[4]=cv::Point2f(16.3,1.003);

        hero_location_[0]=cv::Point2f(16.9,2.193);
        hero_location_[1]=cv::Point2f(16.5,2.97);
        hero_location_[2]=cv::Point2f(18.0,5.50);
        hero_location_[3]=cv::Point2f(20.658,4.548);

        tunnel_horizontal_[0]=cv::Point2f(16.0,1.293);
        tunnel_horizontal_[1]=cv::Point2f(15.6,2.07);
        tunnel_horizontal_[2]=cv::Point2f(13.901,1.844);
        tunnel_horizontal_[3]=cv::Point2f(14.42,1.293);

        supply_[0]=cv::Point2f(24.5,10.95);
        supply_[1]=cv::Point2f(24.5,15.0);
        supply_[2]=cv::Point2f(28.0,15.0);
        supply_[3]=cv::Point2f(28.0,10.95);

        outpost_[0]=cv::Point2f(17.5,12.4);
        outpost_[1]=cv::Point2f(17.5,10.2);
        outpost_[2]=cv::Point2f(18.8,10.2);
        outpost_[3]=cv::Point2f(18.8,12.4);

        little_engine_l_[0]=cv::Point2f(18.165,9.65);
        little_engine_l_[1]=cv::Point2f(18.165,7.8889);
        little_engine_l_[2]=cv::Point2f(19.3,7.8889);
        little_engine_l_[3]=cv::Point2f(19.3,9.65);

        little_engine_r_[0]=cv::Point2f(18.165,7.1111);
        little_engine_r_[1]=cv::Point2f(18.165,5.35);
        little_engine_r_[2]=cv::Point2f(19.3,5.35);
        little_engine_r_[3]=cv::Point2f(19.3,7.1111);

        high_way_[0]=cv::Point2f(20.932,15.0);
        high_way_[1]=cv::Point2f(20.932,11.65);
        high_way_[2]=cv::Point2f(24.5,11.65);
        high_way_[3]=cv::Point2f(24.5,15.0);
    }

    void KalmanFilter::checkLocation(std::vector<Kalman_filter_plus> &KFs) {
        std::vector<int> remove_index;
        for (int i=0;i<KFs.size();i++) {
            auto point=KFs[i].predict_point;
            if (initornot(high_way_,point,4)==1) {
                if (KFs[i].now_place!=highway) {
                    KFs[i].last_place=KFs[i].now_place;
                    KFs[i].now_place=highway;
                }
            }else if (initornot(supply_,point,4)==1) {
                if (KFs[i].now_place!=supply) {
                    KFs[i].last_place=KFs[i].now_place;
                    KFs[i].now_place=supply;
                    if (KFs[i].last_place==highway) {
                        remove_index.push_back(i);
                    }
                }
            }else {
                if (KFs[i].now_place!=ordinary) {
                    KFs[i].last_place=KFs[i].now_place;
                    KFs[i].now_place=ordinary;
                }
            }
        }
        for (auto index:remove_index)
            KFs.erase(KFs.begin()+index);
    }

    void KalmanFilter::robotHPCallback(const interfaces::msg::RobotHP::SharedPtr msg) {
        robot_hp=*msg;
        std::vector<Kalman_filter_plus> KFs_;
        mtx.lock();
        KFs_= KFs;
        mtx.unlock();

        for (auto kf:KFs_) {
            int color=kf.get_color();
            int num=kf.get_number();
            if (color==1&&robot_hp.blue_robot_hp[num]==0) {
                kf.death=true;
                kf.death_cls.first=color;
                kf.death_cls.second=num;
            }else if (color==0&&robot_hp.red_robot_hp[num]==0) {
                kf.death=true;
                kf.death_cls.first=color;
                kf.death_cls.second=num;
            }else {
                kf.death=false;
            }
        }

        mtx.lock();
        KFs= KFs_;
        mtx.unlock();
    }

    void KalmanFilter::getImg1(const sensor_msgs::msg::CompressedImage::ConstPtr &rosImg_ptr){
        show_img1 = cv::imdecode(rosImg_ptr->data, cv::IMREAD_COLOR).clone();
        if(!show_img1.empty()){
        }else{
            std::cout << "error!!!!!" << std::endl;
        }
    }

    void KalmanFilter::camCallback(const interfaces::msg::DetectRes::SharedPtr msg) {
        rclcpp::Time time = msg->header.stamp;
        std::vector<Kalman_filter_plus> KFs_;
        cam_msg=*msg;
        mtx.lock();
        KFs_= KFs;
        mtx.unlock();

        double offset=get_parameter("cam.time_offset").as_double();
        use_rect2d=get_parameter("rect.use_rect2d").as_bool();
        self_color=msg->self_color;

        std::map<double,int> time_map;
        std::vector<std::array<double, 5>> det;//x,y,颜色，编号
        pcl::PointCloud<pcl::PointXYZ> ori_net;
        pcl::PointCloud<pcl::PointXYZ> ori_pc;
        std::cout<<msg->obj.size();
        std::vector<bool> overlap_cam(msg->obj.size(),false);
        std::vector<bool> is_det(KFs_.size(),false);
        for (int i=0;i<msg->obj.size();i++) {
            if (msg->obj[i].x==0||msg->obj[i].y==0) continue;
            for (int j=i+1;j<msg->obj.size();j++) {
                if (!(msg->obj[i].x2 < msg->obj[j].x1 || msg->obj[j].x2 < msg->obj[i].x1) &&
                    !(msg->obj[i].y1 > msg->obj[j].y2 || msg->obj[j].y1 > msg->obj[i].y2)&&
                    msg->obj[j].x!=0&&msg->obj[j].y!=0) {
                    overlap_cam[i]=true;
                    overlap_cam[j]=true;
                }
            }
        }

        for (int i=0;i<msg->obj.size();i++) {
            interfaces::msg::DetectObj obj=msg->obj[i];
            if(obj.x!=0&&obj.y!=0){
                for(auto &kf : KFs_){
                    time_map[kf.camera_find_match(time,offset)]++;
                }
                if(self_color==1){//己方为蓝色时将坐标反转，因为此时相机传回的是准确的，雷达聚类的是反转的
                    ori_net.points.push_back(pcl::PointXYZ(28-obj.x,15-obj.y,2));
                }else {
                    ori_net.points.push_back(pcl::PointXYZ(obj.x,obj.y,2));
                }
                if (!use_rect2d||overlap_cam[i]) {
                    if(self_color==1){//己方为蓝色时将坐标反转，因为此时相机传回的是准确的，雷达聚类的是反转的
                        obj.x=28-obj.x;
                        obj.y=15-obj.y;
                    }
                    if (obj.classid<5) {
                        det.push_back({obj.x,obj.y,1,obj.classid,i});
                    }else {
                        det.push_back({obj.x,obj.y,0,obj.classid-5,i});
                    }
                }
            }
        }

        double time_id =
                std::ranges::max_element(time_map, [](auto&& pair_a, auto&& pair_b) {
                return pair_a.second < pair_b.second;})->first;

        if (fabs(time_id-0)<=0.00001) return;

        std::vector<bool> overlap_lidar(KFs_.size(),false);
        for (int i=0;i<KFs_.size();i++) {
            double min_time=10;
            cv::Rect rect_pc1,rect_pc2;
            for (int k=0;k<KFs_[i].history.size();k++) {
                if (fabs(KFs_[i].history[k].first-time_id)<=min_time) {
                    min_time=fabs(KFs_[i].history[k].first-time_id);
                    rect_pc1=KFs_[i].rect_2d1[k];
                    rect_pc2=KFs_[i].rect_2d2[k];
                }
            }
            if (min_time>0.5) continue;
            for (int j=i+1;j<KFs_.size();j++) {
                double min_time=10;
                cv::Rect rect_t1,rect_t2;
                for (int k=0;k<KFs_[j].history.size();k++) {
                    if (fabs(KFs_[j].history[k].first-time_id)<=min_time) {
                        min_time=fabs(KFs_[j].history[k].first-time_id);
                        rect_t1=KFs_[j].rect_2d1[k];
                        rect_t2=KFs_[j].rect_2d2[k];
                    }
                }
                if (min_time>0.5) continue;
                cv::Rect Intersection1 = rect_pc1&rect_t1 , Intersection2 = rect_pc2&rect_t2;
                cv::Rect Union1 = rect_pc1|rect_t1 , Union2 = rect_pc2|rect_t2;
                if (double(Intersection1.area())/double(Union1.area())>0||double(Intersection2.area())/double(Union2.area())>0) {
                    overlap_lidar[i]=true;
                    overlap_lidar[j]=true;
                }
            }
        }

        cv::Mat img1=show_img1.clone();
        cv::Mat img2=show_img2.clone();
        for (int i=0;i<msg->obj.size();i++) {
            if (!overlap_cam[i]) {
                interfaces::msg::DetectObj obj=msg->obj[i];
                if (obj.camid==1) {
                    cv::rectangle(img1,cv::Point(obj.x1,obj.y1),cv::Point(obj.x2,obj.y2),cv::Scalar(0,255,0),2);
                    cv::putText(img1,std::to_string(obj.camid),cv::Point(obj.x1+10,obj.y1+10),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0),2);
                }
                else if (obj.camid==2) {
                    cv::rectangle(img2,cv::Point(obj.x1,obj.y1),cv::Point(obj.x2,obj.y2),cv::Scalar(0,255,0),2);
                    cv::putText(img2,std::to_string(obj.camid),cv::Point(obj.x1+10,obj.y1+10),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0),2);

                }
            }
        }

        double rect_iou_weight=get_parameter("rect.iou_weight").as_double();
        double rect_dis_weight=get_parameter("rect.dis_weight").as_double();
        double rect_iou_thres=get_parameter("rect.iou_thres").as_double();
        double rect_dis_thres=get_parameter("rect.dis_thres").as_double();
        double rect_match_thres=get_parameter("rect.match_thres").as_double();

        for(int i=0;i<msg->obj.size();i++) {
            if (!overlap_cam[i]&&use_rect2d) {
                int index=-1;
                double iou=0.0;
                double match_value=0.0;
                cv::Rect rect_cam(cv::Point(msg->obj[i].x1, msg->obj[i].y1), cv::Point(msg->obj[i].x2 ,msg->obj[i].y2));
                for (int j=0;j<KFs_.size();j++) {
                    double min_time=10;
                    cv::Rect rect_pc;
                    pcl::PointXY point;
                    for (int k=0;k<KFs_[j].history.size();k++) {
                        if (fabs(KFs_[j].history[k].first-time_id)<=min_time) {
                            min_time=fabs(KFs_[j].history[k].first-time_id);
                            if (msg->obj[i].camid==1)
                                rect_pc=KFs_[j].rect_2d1[k];
                            else if (msg->obj[i].camid==2)
                                rect_pc=KFs_[j].rect_2d2[k];
                            point.x=KFs_[j].history[k].second.x;
                            point.y=KFs_[j].history[k].second.y;
                        }
                    }
                    if (min_time>0.5||overlap_lidar[j]) continue;
                    cv::Rect Intersection = rect_cam & rect_pc;
                    cv::Rect Union = rect_cam|rect_pc;
                    if (double(Intersection.area())/double(Union.area())>rect_iou_thres) {
                        double distance=sqrt(pow(msg->obj[i].x-point.x,2)+pow(msg->obj[i].y-point.y,2));
                        if (self_color==1) distance=sqrt(pow(28.0-msg->obj[i].x-point.x,2)+pow(15.0-msg->obj[i].y-point.y,2));
                        distance=distance<rect_dis_thres?1-distance/rect_dis_thres:0;
                        double temp_value=rect_iou_weight*(double(Intersection.area())/double(Union.area()))+rect_dis_weight*distance;
                        if (temp_value>match_value) {
                            index=j;
                            match_value=temp_value;
                        }
                    }
                }
                // RCLCPP_ERROR(this->get_logger(), "---------------------iou %f",match_value);

                if (match_value>rect_match_thres&&index!=-1) {
                    interfaces::msg::DetectObj obj=msg->obj[i];
                    if (obj.classid<5) {
                        KFs_[index].detect_history.push_back(std::make_pair(1,obj.classid));
                    }else {
                        KFs_[index].detect_history.push_back(std::make_pair(0,obj.classid-5));
                    }
                    Eigen::MatrixXd new_ConfMatrix= Eigen::MatrixXd::Zero(1,10);
                    for (int i=0;i<5;i++) new_ConfMatrix(0,i)=obj.conf_matrix[i+5];
                    for (int i=5;i<10;i++) new_ConfMatrix(0,i)=obj.conf_matrix[i-5];
                    KFs_[index].ws_armorConfMatrix = new_ConfMatrix * obj.confidence * 1.0/3.0 + (1.0 - obj.confidence * 1.0/3.0 ) * KFs_[index].ws_armorConfMatrix;
                    KFs_[index].detect_time.push_back(KFs_[index].GetTimeByRosTime(time));
                    if(KFs_[index].detect_history.size() > KFs_[index].max_detect_history){
                        KFs_[index].detect_history.erase(KFs_[index].detect_history.begin());
                        KFs_[index].detect_time.erase(KFs_[index].detect_time.begin());
                    }
                    is_det[index]=true;
                    // RCLCPP_ERROR(this->get_logger(),"---------------------sucess detect %d",index);
                }else {
                    interfaces::msg::DetectObj obj=msg->obj[i];
                    if(self_color==1){//己方为蓝色时将坐标反转，因为此时相机传回的是准确的，雷达聚类的是反转的
                        obj.x=28-obj.x;
                        obj.y=15-obj.y;
                    }
                    if (obj.classid<5) {
                        det.push_back({obj.x,obj.y,1,obj.classid,i});
                    }else {
                        det.push_back({obj.x,obj.y,0,obj.classid-5,i});
                    }
                }
            }
        }

        for (int j=0;j<KFs_.size();j++) {
            if (!is_det[j])continue;
            double min_time=10;
            cv::Rect rect_pc1,rect_pc2;
            pcl::PointXYZ pc;
            for (int k=0;k<KFs_[j].history.size();k++) {
                if (fabs(KFs_[j].history[k].first-time_id)<=min_time) {
                    min_time=fabs(KFs_[j].history[k].first-time_id);
                    rect_pc1=KFs_[j].rect_2d1[k];
                    if (!is_one_lidar)
                        rect_pc2=KFs_[j].rect_2d2[k];
                    pc=pcl::PointXYZ(KFs_[j].history[k].second.x,KFs_[j].history[k].second.y,KFs_[j].history[k].second.z);
                }
            }
            if (min_time>0.5) continue;
            ori_pc.push_back(pc);
            cv::rectangle(img1,cv::Point(rect_pc1.x,rect_pc1.y),cv::Point(rect_pc1.x+rect_pc1.height,rect_pc1.y+rect_pc1.width),cv::Scalar(0,0,255),2);
            if (!is_one_lidar)
                cv::rectangle(img2,cv::Point(rect_pc2.x,rect_pc2.y),cv::Point(rect_pc2.x+rect_pc2.height,rect_pc2.y+rect_pc2.width),cv::Scalar(0,0,255),2);
        }
        cv::imshow("cam1",img1);
        cv::imshow("cam2",img2);
        cv::waitKey(1);

        std::vector<std::vector<float> > dists;
        int dist_size=0,dist_size_size=0;
        double dis_thres=get_parameter("cam.dis_thres").as_double();
        double match_thres=get_parameter("cam.match_thres").as_double();
        double dis_weight=get_parameter("cam.dis_weight").as_double();
        double his_weight=get_parameter("cam.his_weight").as_double();

        // Eigen::MatrixXd cost_matrix = getFusionCost(KFs_,det,dist_size, dist_size_size,2.6,time_id);
        Eigen::MatrixXd cost_matrix = getFusionCost(KFs_,det,dist_size, dist_size_size,dis_thres,time_id,is_det,dis_weight,his_weight);
        eigenMat2VecVec(cost_matrix,dists);

        std::vector<std::vector<int> > matches;
        std::vector<int> u_track,u_cluster;
        linear_assignment(dists, dist_size, dist_size_size,match_thres,matches,u_cluster,u_track);

        for(auto match : matches){
            KFs_[match[0]].detect_history.push_back(std::make_pair(det[match[1]][2], det[match[1]][3]));
            Eigen::MatrixXd new_ConfMatrix= Eigen::MatrixXd::Zero(1,10);
            for (int i=0;i<5;i++) new_ConfMatrix(0,i)=msg->obj[det[match[1]][4]].conf_matrix[i+5];
            for (int i=5;i<10;i++) new_ConfMatrix(0,i)=msg->obj[det[match[1]][4]].conf_matrix[i-5];
            KFs_[match[0]].ws_armorConfMatrix = new_ConfMatrix * msg->obj[det[match[1]][4]].confidence * 1.0/3.0 + (1.0 - msg->obj[det[match[1]][4]].confidence * 1.0/3.0 ) * KFs_[match[0]].ws_armorConfMatrix;
            KFs_[match[0]].detect_time.push_back(KFs_[match[0]].GetTimeByRosTime(time));
            if(KFs_[match[0]].detect_history.size() > KFs_[match[0]].max_detect_history){
                KFs_[match[0]].detect_history.erase(KFs_[match[0]].detect_history.begin());
                KFs_[match[0]].detect_time.erase(KFs_[match[0]].detect_time.begin());
            }
        }
        sensor_msgs::msg::PointCloud2 det_3D;
        pcl::toROSMsg(ori_net,det_3D);
        det_3D.header.frame_id="rm_frame";
        det_3D.header.stamp=time;
        net_pub->publish(det_3D);

        pcl::toROSMsg(ori_pc,det_3D);
        det_3D.header.frame_id="rm_frame";
        det_3D.header.stamp=time;
        pc_pub->publish(det_3D);

        mtx.lock();
        KFs= KFs_;
        mtx.unlock();
    }

    //记入噪音点
    std::vector<int> KalmanFilter::normalDBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,double eps,size_t min_points) {

        std::vector<pcl::Indices> nbs(cloud->points.size());
        pcl::KdTreeFLANN<pcl::PointXYZ> kd_Tree;
        kd_Tree.setInputCloud(cloud);
        tbb::parallel_for(tbb::blocked_range<int>(0, int(cloud->points.size())),
            [&](const tbb::blocked_range<int> &r) {
                for (int idx = r.begin(); idx < r.end(); ++idx) {
                    std::vector<float> dists2;
                    kd_Tree.radiusSearch(cloud->points[idx],eps,nbs[idx],dists2);
                }
            }
        );

        std::vector<int> labels(cloud->points.size(), -2);

        int cluster_label = 0;
        for (size_t idx = 0; idx < cloud->points.size(); ++idx) {
            //代表当前点已经处理过了
            if (labels[idx] != -2) {
                continue;
            }

            // Check density.
            if (nbs[idx].size() < min_points) {
                labels[idx] = -1;
                continue;
            }

            BucketSet nbs_next(cloud->points.size());
            BucketSet nbs_visited(cloud->points.size());
            for (int nb : nbs[idx])
                nbs_next.insert(nb);
            nbs_visited.insert(int(idx));

            labels[idx] = cluster_label;

            while (nbs_next.size > 0) {
                int nb = nbs_next.now_first;
                nbs_next.erase(nb);
                nbs_visited.insert(nb);

                // Noise label.
                if (labels[nb] == -1) {
                    labels[nb] = cluster_label;
                }
                // Not undefined label.
                if (labels[nb] != -2) {
                    continue;
                }
                labels[nb] = cluster_label;

                if (nbs[nb].size() >= min_points)
                    for (int qnb : nbs[nb])
                        //即如果nbs_visited中没有qnb，则将qnb加入到nbs_next中
                            if (!nbs_visited.bucket[qnb])
                                nbs_next.insert(qnb);
            }

            cluster_label++;
        }

        return labels;
    }

    std::vector<int> KalmanFilter::normalDBSCAN(const open3d::geometry::PointCloud cloud,
        double eps,size_t min_points,std::vector<std::vector<int>> &nbs){

        open3d::geometry::KDTreeFlann kdtree(cloud);

        tbb::parallel_for(tbb::blocked_range<int>(0, int(cloud.points_.size())),
            [&](const tbb::blocked_range<int> &r) {
                for (int idx = r.begin(); idx < r.end(); ++idx) {
                    std::vector<double> dists2;
                    kdtree.SearchRadius(cloud.points_[idx], eps, nbs[idx], dists2);
                }
            });

        std::vector<int> labels(cloud.points_.size(), -2);
        int cluster_label = 0;
        for (size_t idx = 0; idx < cloud.points_.size(); ++idx) {
            // Label is not undefined.
            if (labels[idx] != -2) {
                continue;
            }

            // Check density.
            if (nbs[idx].size() < min_points) {
                labels[idx] = -1;
                continue;
            }
            BucketSet nbs_next(cloud.points_.size());
            BucketSet nbs_visited(cloud.points_.size());
            for (int nb : nbs[idx])
                nbs_next.insert(nb);
            nbs_visited.insert(int(idx));

            labels[idx] = cluster_label;

            while (nbs_next.size > 0) {
                int nb = nbs_next.now_first;
                nbs_next.erase(nb);
                nbs_visited.insert(nb);

                // Noise label.
                if (labels[nb] == -1) {
                    labels[nb] = cluster_label;
                }
                // Not undefined label.
                if (labels[nb] != -2) {
                    continue;
                }
                labels[nb] = cluster_label;

                if (nbs[nb].size() >= min_points) {
                    labels[nb] = cluster_label;
                    for (int qnb : nbs[nb])
                        //即如果nbs_visited中没有qnb，则将qnb加入到nbs_next中
                        if (!nbs_visited.bucket[qnb])
                            nbs_next.insert(qnb);
                }
            }

            cluster_label++;
        }
        return labels;
    }

    std::vector<int> KalmanFilter::normalDBSCAN(const open3d::geometry::PointCloud cloud,
        double eps,size_t min_points,std::vector<std::vector<int>> &nbs,std::vector<std::vector<int>> &clusters){

        open3d::geometry::KDTreeFlann kdtree(cloud);

        tbb::parallel_for(tbb::blocked_range<int>(0, int(cloud.points_.size())),
            [&](const tbb::blocked_range<int> &r) {
                for (int idx = r.begin(); idx < r.end(); ++idx) {
                    std::vector<double> dists2;
                    kdtree.SearchRadius(cloud.points_[idx], eps, nbs[idx], dists2);
                }
            });

        // std::vector<std::thread> threads;
        // int cloud_size=cloud.points_.size();
        // int step=cloud_size/24;
        // for(int i=0;i<24;i++){
        //     threads.push_back(std::thread([i,step,&nbs,&cloud,this,eps,&kdtree](){
        //         for(int j=i*step;j<(i+1)*step;j++){
        //             std::vector<double> dists2;
        //             kdtree.SearchRadius(cloud.points_[j], eps, nbs[j], dists2);
        //         }
        //     }));
        // }
        // for(auto &t:threads){
        //     t.join();
        // }
        auto now_time1 = std::chrono::steady_clock::now();

        std::vector<int> labels(cloud.points_.size(), -2);
        int cluster_label = 0;
        for (size_t idx = 0; idx < cloud.points_.size(); ++idx) {
            // Label is not undefined.
            if (labels[idx] != -2) {
                continue;
            }

            // Check density.
            if (nbs[idx].size() < min_points) {
                labels[idx] = -1;
                continue;
            }
            std::vector<int> cluster;
            BucketSet nbs_next(cloud.points_.size());
            BucketSet nbs_visited(cloud.points_.size());
            for (int nb : nbs[idx])
                nbs_next.insert(nb);
            nbs_visited.insert(int(idx));

            labels[idx] = cluster_label;
            cluster.push_back(int(idx));

            while (nbs_next.size > 0) {
                int nb = nbs_next.now_first;
                nbs_next.erase(nb);
                nbs_visited.insert(nb);

                // Noise label.
                if (labels[nb] == -1) {
                    // labels[nb] = cluster_label;
                    cluster.push_back(nb);
                }
                // Not undefined label.
                if (labels[nb] != -2) {
                    continue;
                }
                cluster.push_back(nb);

                if (nbs[nb].size() >= min_points) {
                    labels[nb] = cluster_label;
                    for (int qnb : nbs[nb])
                        //即如果nbs_visited中没有qnb，则将qnb加入到nbs_next中
                        if (!nbs_visited.bucket[qnb])
                            nbs_next.insert(qnb);
                }else {
                    labels[nb] = -1;
                }
            }

            cluster_label++;
            clusters.push_back(cluster);
        }
        // auto end_time1 = std::chrono::steady_clock::now();
        // float dur_time1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time1 - now_time1).count();
        // RCLCPP_WARN(this->get_logger(), "cluster Callback time is %f ms", dur_time1);
        return labels;
    }

    void KalmanFilter::getCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector<open3d::geometry::PointCloud> &out){
        if (cloud->empty()) {return;}
        eps=get_parameter("cluster.eps").as_double();
        min_points=get_parameter("cluster.min_pts").as_int();

        open3d::geometry::PointCloud in_cloud;
        for(size_t i=0;i<cloud->points.size();i++)
            in_cloud.points_.push_back(Eigen::Vector3d(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z));


        std::vector<int> labels;
        std::vector<std::vector<int>> nbs(in_cloud.points_.size());
        std::vector<std::vector<int>> clusters;
        labels=normalDBSCAN(in_cloud,eps, min_points,nbs);
        // labels=normalDBSCAN(cloud,eps, min_points);
        // labels=in_cloud.ClusterDBSCAN(eps, min_points);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> points;
        std::vector<open3d::geometry::PointCloud> pcs;
        open3d::geometry::PointCloud pc_noise;
        std::vector<Eigen::Vector3d> grav;
        int max_l = *std::max_element(labels.begin(), labels.end());
        // int max_l = clusters.size();

        for (int i = 0; i <= max_l; i++) {
            pcs.emplace_back(open3d::geometry::PointCloud());
            points.emplace_back(new pcl::PointCloud<pcl::PointXYZ>);
        }

        for (size_t i = 0; i < in_cloud.points_.size(); i++) {
            if(labels[i] >= 0) {
                pcs[labels[i]].points_.push_back(in_cloud.points_[i]);
                points[labels[i]]->push_back(pcl::PointXYZ(in_cloud.points_[i](0),in_cloud.points_[i](1),in_cloud.points_[i](2)));
            }else
                pc_noise.points_.push_back(in_cloud.points_[i]);
        }
        // for (int i = 0; i < max_l; i++) {
        //     for (auto index:clusters[i]) {
        //         pcs[i].points_.push_back(in_cloud.points_[index]);
        //         points[i]->push_back(pcl::PointXYZ(in_cloud.points_[index](0),in_cloud.points_[index](1),in_cloud.points_[index](2)));
        //     }
        // }

        for (int i = 0; i <=max_l; i++) {
            //open3d obb
            // if (!hasAtLeastThreeUniquePoints(pcs[i].points_)) {
            //     out.push_back(pcs[i]);
            //     continue;
            // }
            // auto obb=pcs[i].GetOrientedBoundingBox(true);
            // auto pointss=obb.GetBoxPoints();
            // double width1=sqrt(pow(pointss[0][0]-pointss[1][0],2)+pow(pointss[0][1]-pointss[1][1],2)+pow(pointss[0][2]-pointss[1][2],2));
            // double height1=sqrt(pow(pointss[0][0]-pointss[2][0],2)+pow(pointss[0][1]-pointss[2][1],2)+pow(pointss[0][2]-pointss[2][2],2));
            // double depth1=sqrt(pow(pointss[0][0]-pointss[3][0],2)+pow(pointss[0][1]-pointss[3][1],2)+pow(pointss[0][2]-pointss[3][2],2));
            // if (height1>0.025&&width1>0.025&&depth1>0.025&&height1<1.5&&width1<1.5&&depth1<1.5) {
            //     out.push_back(pcs[i]);
            // }

            //法向量
            // auto center=pcs[i].GetCenter();
            // pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> nest;
            // nest.setKSearch(points[i]->points.size()); // 设置拟合时采用的点数
            // nest.setInputCloud(points[i]);
            // pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
            // nest.compute(*normals);
            // normals->points[0].normal;
            // double nx=0,ny=0,nz=0;
            // for (auto i:normals->points) {
            //     nx+=i.normal_x;
            //     ny+=i.normal_y;
            //     nz+=i.normal_z;
            // }
            // Eigen::Vector3d nor;
            // nor<<nx/normals->points.size(),ny/normals->points.size(),nz/normals->points.size();
            // Eigen::Vector3d pos;
            // pos<<-1.22638,9.67522,3.90987;
            // double num=nor.dot(center-pos);
            // if (fabs(num)>=2.0) {
            //     out.push_back(pcs[i]);
            // }

            //pcl obb
            // pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
            // feature_extractor.setInputCloud (points[i]);
            // feature_extractor.compute ();
            // std::vector <float> moment_of_inertia;
            // std::vector <float> eccentricity;
            // pcl::PointXYZ min_point_OBB;
            // pcl::PointXYZ max_point_OBB;
            // pcl::PointXYZ position_OBB;
            // Eigen::Matrix3f rotational_matrix_OBB;
            //
            // // 获取惯性矩
            // feature_extractor.getMomentOfInertia (moment_of_inertia);
            // // 获取离心率
            // feature_extractor.getEccentricity (eccentricity);
            // // 获取OBB盒子
            // feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
            // Eigen::Matrix4f Tran;
            // Tran<<rotational_matrix_OBB(0,0),rotational_matrix_OBB(0,1),rotational_matrix_OBB(0,2),position_OBB.x,
            //       rotational_matrix_OBB(1,0),rotational_matrix_OBB(1,1),rotational_matrix_OBB(1,2),position_OBB.y,
            //       rotational_matrix_OBB(2,0),rotational_matrix_OBB(2,1),rotational_matrix_OBB(2,2),position_OBB.z,
            //         0,0,0,1;
            //
            // double width=max_point_OBB.x-min_point_OBB.x;
            // double height=max_point_OBB.y-min_point_OBB.y;
            // double depth=max_point_OBB.z-min_point_OBB.z;
            //
            // std::cout<<"width:"<<width<<"height:"<<height<<"depth:"<<depth<<std::endl;
            // // std::cout<<"width1:"<<width1<<"height1:"<<height1<<"depth1:"<<depth1<<std::endl;
            // if (depth>0.025) {
                out.push_back(pcs[i]);
            // }
        }
    }

    void KalmanFilter::guessWithoutClass(std::vector<Kalman_filter_plus> &KFs_,std::vector<int> u_strack,visualization_msgs::msg::MarkerArray &vis_array,std::vector<std::vector<int>> history_cost) {
        for (auto index:u_strack) {
            // if (KFs_[index].last_time>1.5) {
            //     continue;
            // }
            auto point=KFs_[index].predict_point;
            // 打符点下位置 英雄
            if (initornot(hero_location_,point,4)==1) {
                //self_color==1为自己为蓝方
                if (self_color==1) {
                    if (lidar_enhance_[0][0]==2||lidar_enhance_[0][0]==3||lidar_enhance_[0][0]==4||
                        (detect_res.red_x[0]==0&&detect_res.red_y[0]==0&&(lidar_enhance_[0][0]==0||
                            lidar_enhance_[0][0]==5||lidar_enhance_[0][0]==6))) {
                        detect_res.red_x[0]= point.x;
                        detect_res.red_y[0]= point.y;
                        detect_res.v_x[0] = KFs_[index].KF.statePost.at<float>(1);
                        detect_res.v_y[0] = KFs_[index].KF.statePost.at<float>(3);
                        vis_kal_maker(1,vis_array,0,0,point.x,point.y,KFs_[index].detect_history.size(),history_cost[1][0]);
                        lidar_enhance_[0][0]=6;
                    }
                }else if (self_color==0) {
                    if (lidar_enhance_[1][0]==2||lidar_enhance_[1][0]==3||lidar_enhance_[1][0]==4||
                        (detect_res.blue_x[0]==0&&detect_res.blue_y[0]==0&&(lidar_enhance_[1][0]==0||
                            lidar_enhance_[1][0]==5||lidar_enhance_[1][0]==6))) {
                        detect_res.blue_x[0]= point.x;
                        detect_res.blue_y[0]= point.y;
                        detect_res.v_x[0] = KFs_[index].KF.statePost.at<float>(1);
                        detect_res.v_y[0] = KFs_[index].KF.statePost.at<float>(3);
                        vis_kal_maker(1,vis_array,1,0,point.x,point.y,KFs_[index].detect_history.size(),history_cost[0][0]);
                        lidar_enhance_[1][0]=6;
                    }
                }
                continue;
            }
            // 前哨站位置 英雄
            if (initornot(outpost_,point,4)==1) {
                //self_color==1为自己为蓝方
                if (self_color==1) {
                    if (lidar_enhance_[0][0]==2||lidar_enhance_[0][0]==3||lidar_enhance_[0][0]==4||
                        (detect_res.red_x[0]==0&&detect_res.red_y[0]==0&&(lidar_enhance_[0][0]==0||
                            lidar_enhance_[0][0]==5||lidar_enhance_[0][0]==6))) {
                        detect_res.red_x[0]= point.x;
                        detect_res.red_y[0]= point.y;
                        detect_res.v_x[0] = KFs_[index].KF.statePost.at<float>(1);
                        detect_res.v_y[0] = KFs_[index].KF.statePost.at<float>(3);
                        vis_kal_maker(1,vis_array,0,0,point.x,point.y,KFs_[index].detect_history.size(),history_cost[1][0]);
                        lidar_enhance_[0][0]=5;
                    }
                }else if (self_color==0) {
                    if (lidar_enhance_[1][0]==2||lidar_enhance_[1][0]==3||lidar_enhance_[1][0]==4||
                        (detect_res.blue_x[0]==0&&detect_res.blue_y[0]==0&&(lidar_enhance_[1][0]==0||
                            lidar_enhance_[1][0]==5||lidar_enhance_[1][0]==6))) {
                        detect_res.blue_x[0]= point.x;
                        detect_res.blue_y[0]= point.y;
                        detect_res.v_x[0] = KFs_[index].KF.statePost.at<float>(1);
                        detect_res.v_y[0] = KFs_[index].KF.statePost.at<float>(3);
                        vis_kal_maker(1,vis_array,1,0,point.x,point.y,KFs_[index].detect_history.size(),history_cost[0][0]);
                        lidar_enhance_[1][0]=5;
                    }
                }
                continue;
            }
            // 左侧小资源岛 工程
            if (initornot(little_engine_l_,point,4)==1) {
                //self_color==1为自己为蓝方
                if (self_color==1) {
                    if (lidar_enhance_[0][1]==2||lidar_enhance_[0][1]==3||lidar_enhance_[0][1]==4||
                        (detect_res.red_x[1]==0&&detect_res.red_y[1]==0&&(lidar_enhance_[0][1]==0||
                            lidar_enhance_[0][1]==5))) {
                        detect_res.red_x[1]= point.x;
                        detect_res.red_y[1]= point.y;
                        detect_res.v_x[1] = KFs_[index].KF.statePost.at<float>(1);
                        detect_res.v_y[1] = KFs_[index].KF.statePost.at<float>(3);
                        vis_kal_maker(1,vis_array,0,1,point.x,point.y,KFs_[index].detect_history.size(),history_cost[1][1]);
                        lidar_enhance_[0][1]=5;
                    }
                }else if (self_color==0) {
                    if (lidar_enhance_[1][1]==2||lidar_enhance_[1][1]==3||lidar_enhance_[1][1]==4||
                        (detect_res.blue_x[1]==0&&detect_res.blue_y[1]==0&&(lidar_enhance_[1][1]==0||
                            lidar_enhance_[1][1]==5))) {
                        detect_res.blue_x[1]= point.x;
                        detect_res.blue_y[1]= point.y;
                        detect_res.v_x[1] = KFs_[index].KF.statePost.at<float>(1);
                        detect_res.v_y[1] = KFs_[index].KF.statePost.at<float>(3);
                        vis_kal_maker(1,vis_array,1,1,point.x,point.y,KFs_[index].detect_history.size(),history_cost[0][1]);
                        lidar_enhance_[1][1]=5;
                    }
                }
                continue;
            }
            // 右侧小资源岛 工程
            if (initornot(little_engine_r_,point,4)==1) {
                //self_color==1为自己为蓝方
                if (self_color==1) {
                    if (lidar_enhance_[0][1]==2||lidar_enhance_[0][1]==3||lidar_enhance_[0][1]==4||
                        (detect_res.red_x[1]==0&&detect_res.red_y[1]==0&&(lidar_enhance_[0][1]==0||
                            lidar_enhance_[0][1]==5))) {
                        detect_res.red_x[1]= point.x;
                        detect_res.red_y[1]= point.y;
                        detect_res.v_x[1] = KFs_[index].KF.statePost.at<float>(1);
                        detect_res.v_y[1] = KFs_[index].KF.statePost.at<float>(3);
                        vis_kal_maker(1,vis_array,0,1,point.x,point.y,KFs_[index].detect_history.size(),history_cost[1][1]);
                        lidar_enhance_[0][1]=5;
                    }
                }else if (self_color==0) {
                    if (lidar_enhance_[1][1]==2||lidar_enhance_[1][1]==3||lidar_enhance_[1][1]==4||
                        (detect_res.blue_x[1]==0&&detect_res.blue_y[1]==0&&(lidar_enhance_[1][1]==0||
                            lidar_enhance_[1][1]==5))) {
                        detect_res.blue_x[1]= point.x;
                        detect_res.blue_y[1]= point.y;
                        detect_res.v_x[1] = KFs_[index].KF.statePost.at<float>(1);
                        detect_res.v_y[1] = KFs_[index].KF.statePost.at<float>(3);
                        vis_kal_maker(1,vis_array,1,1,point.x,point.y,KFs_[index].detect_history.size(),history_cost[0][1]);
                        lidar_enhance_[1][1]=5;
                    }
                }
            }
        }
    }

    void KalmanFilter::guessWithClass(std::vector<Kalman_filter_plus> &KFs_,std::vector<std::vector<int>> matches_cls,std::vector<int> &remove_KFs_) {
        for (auto match:matches_cls) {
            //补给区
            if(KFs_[match[0]].output_point.x>=24.5&&KFs_[match[0]].output_point.y>=10.6&&KFs_[match[0]].last_time>0.3&&
               !(KFs_[match[0]].history.back().second.y>11.65&&KFs_[match[0]].history.back().second.x<24.3)){
                //自己是红方
                if (self_color==0&&match[1]>=5) {
                    if (match[1]==6) {
                        KFs_[match[0]].output_point.x=26.2;
                        KFs_[match[0]].output_point.y=14.1;
                        RCLCPP_ERROR(this->get_logger(),"enter mill!!!"); //工程兑矿区
                    }else {
                        KFs_[match[0]].output_point.x=25.6;
                        KFs_[match[0]].output_point.y=13.2;
                        RCLCPP_ERROR(this->get_logger(),"enter supply!!!"); //其他车补给点
                    }
                    lidar_enhance_[1][match[1]-5]=1;
                    detect_res.blue_x[match[1]-5]=KFs_[match[0]].output_point.x;
                    detect_res.blue_y[match[1]-5]=KFs_[match[0]].output_point.y;
                    detect_res.v_x[match[1]-5] = 0;
                    detect_res.v_y[match[1]-5] = 0;
                    remove_KFs_.push_back(match[0]);
                }else if (self_color==1&&match[1]<5) {
                    if (match[1]==1) {
                        KFs_[match[0]].output_point.x=1.8;
                        KFs_[match[0]].output_point.y=0.9;
                        RCLCPP_ERROR(this->get_logger(),"enter mill!!!"); //工程兑矿区
                    }else {
                        KFs_[match[0]].output_point.x=2.4;
                        KFs_[match[0]].output_point.y=1.8;
                        RCLCPP_ERROR(this->get_logger(),"enter supply!!!"); //其他车补给点
                    }
                    lidar_enhance_[0][match[1]]=1;
                    detect_res.red_x[match[1]]=KFs_[match[0]].output_point.x;
                    detect_res.red_y[match[1]]=KFs_[match[0]].output_point.y;
                    detect_res.v_x[match[1]] = 0;
                    detect_res.v_y[match[1]] = 0;
                    remove_KFs_.push_back(match[0]);
                }
                continue;
            }
            //倾斜隧道
            if(initornot(tunnel_slanted_,KFs_[match[0]].output_point,5)==1&&KFs_[match[0]].last_time>1.3) {
                //自己是红方
                if (self_color==0&&match[1]==5) {
                    KFs_[match[0]].output_point.x=17.5392;
                    KFs_[match[0]].output_point.y=4.1475;
                    lidar_enhance_[1][match[1]-5]=4;
                    detect_res.blue_x[match[1]-5]=KFs_[match[0]].output_point.x;
                    detect_res.blue_y[match[1]-5]=KFs_[match[0]].output_point.y;
                    detect_res.v_x[match[1]-5] = 0;
                    detect_res.v_y[match[1]-5] = 0;
                    remove_KFs_.push_back(match[0]);
                }else if (self_color==1&&match[1]==0) {
                    KFs_[match[0]].output_point.x=10.4608;
                    KFs_[match[0]].output_point.y=10.8525;
                    lidar_enhance_[0][match[1]]=4;
                    detect_res.red_x[match[1]]=KFs_[match[0]].output_point.x;
                    detect_res.red_y[match[1]]=KFs_[match[0]].output_point.y;
                    detect_res.v_x[match[1]] = 0;
                    detect_res.v_y[match[1]] = 0;
                    remove_KFs_.push_back(match[0]);
                }else if (match[1]<5){
                    fake_kfs[0][match[1]].location=KFs_[match[0]].output_point;
                    fake_kfs[0][match[1]].is_first=true;
                    double speed=sqrt(pow(KFs_[match[0]].KF.statePost.at<float>(1),2)+pow(KFs_[match[0]].KF.statePost.at<float>(3),2));
                    if (KFs_[match[0]].KF.statePost.at<float>(1)>0) {
                        fake_kfs[0][match[1]].v_x=speed*sin(35*M_PI/180);
                        fake_kfs[0][match[1]].v_y=speed*cos(35*M_PI/180);
                    }else {
                        fake_kfs[0][match[1]].v_x=-speed*sin(35*M_PI/180);
                        fake_kfs[0][match[1]].v_y=-speed*cos(35*M_PI/180);
                    }
                    lidar_enhance_[0][match[1]]=2;
                    if (self_color==1) {
                        fake_kfs[0][match[1]].location.x=28-fake_kfs[0][match[1]].location.x;
                        fake_kfs[0][match[1]].location.y=15-fake_kfs[0][match[1]].location.y;
                        fake_kfs[0][match[1]].v_x=-fake_kfs[0][match[1]].v_x;
                        fake_kfs[0][match[1]].v_y=-fake_kfs[0][match[1]].v_y;
                        detect_res.v_x[match[1]]=fake_kfs[0][match[1]].v_x;
                        detect_res.v_y[match[1]]=fake_kfs[0][match[1]].v_y;
                    }
                    remove_KFs_.push_back(match[0]);
                }else if (match[1]>=5){
                    fake_kfs[1][match[1]-5].location=KFs_[match[0]].output_point;
                    fake_kfs[1][match[1]-5].is_first=true;
                    double speed=sqrt(pow(KFs_[match[0]].KF.statePost.at<float>(1),2)+pow(KFs_[match[0]].KF.statePost.at<float>(3),2));
                    if (KFs_[match[0]].KF.statePost.at<float>(1)>0) {
                        fake_kfs[1][match[1]-5].v_x=speed*sin(35*M_PI/180);
                        fake_kfs[1][match[1]-5].v_y=speed*cos(35*M_PI/180);
                    }else {
                        fake_kfs[1][match[1]-5].v_x=-speed*sin(35*M_PI/180);
                        fake_kfs[1][match[1]-5].v_y=-speed*cos(35*M_PI/180);
                    }
                    lidar_enhance_[1][match[1]-5]=2;
                    if (self_color==0) {
                        detect_res.v_x[match[1]-5]=fake_kfs[1][match[1]-5].v_x;
                        detect_res.v_y[match[1]-5]=fake_kfs[1][match[1]-5].v_y;
                    }else {
                        fake_kfs[1][match[1]-5].location.x=28-fake_kfs[1][match[1]-5].location.x;
                        fake_kfs[1][match[1]-5].location.y=15-fake_kfs[1][match[1]-5].location.y;
                        fake_kfs[1][match[1]-5].v_x=-fake_kfs[1][match[1]-5].v_x;
                        fake_kfs[1][match[1]-5].v_y=-fake_kfs[1][match[1]-5].v_y;
                    }
                    remove_KFs_.push_back(match[0]);
                }
                continue;
            }
            //水平隧道
            if(initornot(tunnel_horizontal_,KFs_[match[0]].output_point,4)==1&&KFs_[match[0]].last_time>0.2) {
                if (match[1]<5){
                    fake_kfs[0][match[1]].location=KFs_[match[0]].output_point;
                    fake_kfs[0][match[1]].is_first=true;
                    double speed=sqrt(pow(KFs_[match[0]].KF.statePost.at<float>(1),2)+pow(KFs_[match[0]].KF.statePost.at<float>(3),2));
                    if (KFs_[match[0]].KF.statePost.at<float>(1)>0) {
                        fake_kfs[0][match[1]].v_x=speed;
                        fake_kfs[0][match[1]].v_y=0;
                    }else {
                        fake_kfs[0][match[1]].v_x=-speed;
                        fake_kfs[0][match[1]].v_y=0;
                    }
                    lidar_enhance_[0][match[1]]=3;
                    if (self_color==1) {
                        fake_kfs[0][match[1]].location.x=28-fake_kfs[0][match[1]].location.x;
                        fake_kfs[0][match[1]].location.y=15-fake_kfs[0][match[1]].location.y;
                        fake_kfs[0][match[1]].v_x=-fake_kfs[0][match[1]].v_x;
                        fake_kfs[0][match[1]].v_y=-fake_kfs[0][match[1]].v_y;
                        detect_res.v_x[match[1]]=fake_kfs[0][match[1]].v_x;
                        detect_res.v_y[match[1]]=fake_kfs[0][match[1]].v_y;
                    }
                    remove_KFs_.push_back(match[0]);
                }
                else if (match[1]>=5){
                    fake_kfs[1][match[1]-5].location=KFs_[match[0]].output_point;
                    fake_kfs[1][match[1]-5].is_first=true;
                    double speed=sqrt(pow(KFs_[match[0]].KF.statePost.at<float>(1),2)+pow(KFs_[match[0]].KF.statePost.at<float>(3),2));
                    if (KFs_[match[0]].KF.statePost.at<float>(1)>0) {
                        fake_kfs[1][match[1]-5].v_x=speed;
                        fake_kfs[1][match[1]-5].v_y=0;
                    }else {
                        fake_kfs[1][match[1]-5].v_x=-speed;
                        fake_kfs[1][match[1]-5].v_y=0;
                    }
                    lidar_enhance_[1][match[1]-5]=3;
                    if (self_color==0) {
                        detect_res.v_x[match[1]-5]=fake_kfs[1][match[1]-5].v_x;
                        detect_res.v_y[match[1]-5]=fake_kfs[1][match[1]-5].v_y;
                    }else {
                        fake_kfs[1][match[1]-5].location.x=28-fake_kfs[1][match[1]-5].location.x;
                        fake_kfs[1][match[1]-5].location.y=15-fake_kfs[1][match[1]-5].location.y;
                        fake_kfs[1][match[1]-5].v_x=-fake_kfs[1][match[1]-5].v_x;
                        fake_kfs[1][match[1]-5].v_y=-fake_kfs[1][match[1]-5].v_y;
                    }
                    remove_KFs_.push_back(match[0]);
                }
            }
        }
    }

    void KalmanFilter::speedSimulation() {
        for (int i=0;i<5;i++) {
            if (lidar_enhance_[0][i]==2) {
                pcl::PointXY location=self_color==0?fake_kfs[0][i].location:pcl::PointXY(28-fake_kfs[0][i].location.x,15-fake_kfs[0][i].location.y);
                if (initornot(tunnel_slanted_,location,5)!=1){
                    if (fake_kfs[0][i].v_x>0) {
                        double speed=sqrt(pow(fake_kfs[0][i].v_x,2)+pow(fake_kfs[0][i].v_y,2));
                        fake_kfs[0][i].v_x=speed;
                        fake_kfs[0][i].v_y=0;
                        lidar_enhance_[0][i]=3;
                    }else {
                        lidar_enhance_[0][i]=4;
                    }
                    detect_res.red_x[i]=fake_kfs[0][i].location.x;
                    detect_res.red_y[i]=fake_kfs[0][i].location.y;
                }else {
                    if (fake_kfs[0][i].is_first==true) {
                        detect_res.red_x[i]=fake_kfs[0][i].location.x;
                        detect_res.red_y[i]=fake_kfs[0][i].location.y;
                        fake_kfs[0][i].is_first=false;
                    }else {
                        fake_kfs[0][i].location.x=fake_kfs[0][i].location.x+fake_kfs[0][i].v_x*0.2;
                        fake_kfs[0][i].location.y=fake_kfs[0][i].location.y+fake_kfs[0][i].v_y*0.2;
                        detect_res.red_x[i]=fake_kfs[0][i].location.x;
                        detect_res.red_y[i]=fake_kfs[0][i].location.y;
                    }
                }
            }else if (lidar_enhance_[0][i]==3) {
                pcl::PointXY location=self_color==0?fake_kfs[0][i].location:pcl::PointXY(28-fake_kfs[0][i].location.x,15-fake_kfs[0][i].location.y);
                if (initornot(tunnel_horizontal_,location,4)!=1){
                    if (fake_kfs[0][i].v_x<0) {
                        double speed=sqrt(pow(fake_kfs[0][i].v_x,2)+pow(fake_kfs[0][i].v_y,2));
                        fake_kfs[0][i].v_x=-speed*sin(35*M_PI/180)*2.0;
                        fake_kfs[0][i].v_y=-speed*cos(35*M_PI/180)*2.0;
                        lidar_enhance_[0][i]=2;
                    }else {
                        lidar_enhance_[0][i]=4;
                    }
                    detect_res.red_x[i]=fake_kfs[0][i].location.x;
                    detect_res.red_y[i]=fake_kfs[0][i].location.y;
                }else {
                    if (fake_kfs[0][i].is_first==true) {
                        detect_res.red_x[i]=fake_kfs[0][i].location.x;
                        detect_res.red_y[i]=fake_kfs[0][i].location.y;
                        fake_kfs[0][i].is_first=false;
                    }else {
                        fake_kfs[0][i].location.x=fake_kfs[0][i].location.x+fake_kfs[0][i].v_x*0.2;
                        fake_kfs[0][i].location.y=fake_kfs[0][i].location.y+fake_kfs[0][i].v_y*0.2;
                        detect_res.red_x[i]=fake_kfs[0][i].location.x;
                        detect_res.red_y[i]=fake_kfs[0][i].location.y;
                    }
                }
            }
            if (lidar_enhance_[1][i]==2) {
                pcl::PointXY location=self_color==0?fake_kfs[1][i].location:pcl::PointXY(28-fake_kfs[1][i].location.x,15-fake_kfs[1][i].location.y);
                if (initornot(tunnel_slanted_,location,5)!=1){
                    if (fake_kfs[1][i].v_x>0) {
                        double speed=sqrt(pow(fake_kfs[1][i].v_x,2)+pow(fake_kfs[1][i].v_y,2));
                        fake_kfs[1][i].v_x=speed;
                        fake_kfs[1][i].v_y=0;
                        lidar_enhance_[1][i]=3;
                    }else {
                        lidar_enhance_[1][i]=4;
                    }
                    detect_res.blue_x[i]=fake_kfs[1][i].location.x;
                    detect_res.blue_y[i]=fake_kfs[1][i].location.y;
                }else {
                    if (fake_kfs[1][i].is_first==true) {
                        detect_res.blue_x[i]=fake_kfs[1][i].location.x;
                        detect_res.blue_y[i]=fake_kfs[1][i].location.y;
                        fake_kfs[1][i].is_first=false;
                    }else {
                        fake_kfs[1][i].location.x=fake_kfs[1][i].location.x+fake_kfs[1][i].v_x*0.2;
                        fake_kfs[1][i].location.y=fake_kfs[1][i].location.y+fake_kfs[1][i].v_y*0.2;
                        detect_res.blue_x[i]=fake_kfs[1][i].location.x;
                        detect_res.blue_y[i]=fake_kfs[1][i].location.y;
                    }
                }
            }else if (lidar_enhance_[1][i]==3) {
                pcl::PointXY location=self_color==0?fake_kfs[1][i].location:pcl::PointXY(28-fake_kfs[1][i].location.x,15-fake_kfs[1][i].location.y);
                if (initornot(tunnel_horizontal_,location,4)!=1){
                    if (fake_kfs[1][i].v_x<0) {
                        double speed=sqrt(pow(fake_kfs[1][i].v_x,2)+pow(fake_kfs[1][i].v_y,2));
                        fake_kfs[1][i].v_x=-speed*sin(35*M_PI/180)*2.0;
                        fake_kfs[1][i].v_y=-speed*cos(35*M_PI/180)*2.0;
                        lidar_enhance_[1][i]=2;
                    }else {
                        lidar_enhance_[1][i]=4;
                    }
                    detect_res.blue_x[i]=fake_kfs[1][i].location.x;
                    detect_res.blue_y[i]=fake_kfs[1][i].location.y;
                }else {
                    if (fake_kfs[1][i].is_first==true) {
                        detect_res.blue_x[i]=fake_kfs[1][i].location.x;
                        detect_res.blue_y[i]=fake_kfs[1][i].location.y;
                        fake_kfs[1][i].is_first=false;
                    }else {
                        fake_kfs[1][i].location.x=fake_kfs[1][i].location.x+fake_kfs[1][i].v_x*0.2;
                        fake_kfs[1][i].location.y=fake_kfs[1][i].location.y+fake_kfs[1][i].v_y*0.2;
                        detect_res.blue_x[i]=fake_kfs[1][i].location.x;
                        detect_res.blue_y[i]=fake_kfs[1][i].location.y;
                    }
                }
            }
        }
    }

    cv::Point3d KalmanFilter::lidarToCamera(pcl::PointXYZ lidar_point,cv::Matx33d cam_matrix,cv::Matx44d lidar2cam){
        cv::Matx41d lidar_coor{lidar_point.x, lidar_point.y, lidar_point.z, 1.0f};
        cv::Matx31d camera_coor =cam_matrix*(lidar2cam*lidar_coor).get_minor<3, 1>(0, 0);
        double u = camera_coor(0) / camera_coor(2);
        double v = camera_coor(1) / camera_coor(2);
        double d = camera_coor(2);
        //(v, u) 是像素的坐标，其中 v 是行索引（通常对应于图像的高度），u 是列索引（通常对应于图像的宽度）
        return cv::Point3d(u,v,d);
    }

    void KalmanFilter::detectCallback(const interfaces::msg::DetectFrame::SharedPtr msg){
        rclcpp::Time time = msg->header.stamp;
        std::vector<Kalman_filter_plus> KFs_;
        mtx.lock();
        KFs_= KFs;
        mtx.unlock();

        double offset=get_parameter("cam.time_offset").as_double();
        self_color=msg->self_color;
        detect_msg=*msg;
        std::map<double,int> time_map;
        std::vector<std::array<double, 5>> det;//x,y,颜色，编号

        pcl::PointCloud<pcl::PointXYZ> ori_net;///
        for(int i=0;i<6;i++){
            pcl::PointXY red_point;
            red_point.x = msg->red_x[i];
            red_point.y = msg->red_y[i];
            if(red_point.x!=0&&red_point.y!=0){
                if(self_color==1){//己方为蓝色时将坐标反转，因为此时相机传回的是准确的，雷达聚类的是反转的
                    red_point.x=28-red_point.x;
                    red_point.y=15-red_point.y;
                }
                ori_net.points.push_back(pcl::PointXYZ(red_point.x,red_point.y,0.5));///
                for(auto &kf : KFs_){
                    time_map[kf.camera_find_match(time,offset)]++;
                }
                det.push_back({red_point.x,red_point.y,0,i,i});
            }
            pcl::PointXY blue_point;
            blue_point.x = msg->blue_x[i];
            blue_point.y = msg->blue_y[i];
            if(blue_point.x!=0&&blue_point.y!=0){
                if(self_color==1){
                    blue_point.x=28-blue_point.x;
                    blue_point.y=15-blue_point.y;
                }
                ori_net.points.push_back(pcl::PointXYZ(blue_point.x,blue_point.y,0.5));///
                for(auto &kf : KFs_){
                    time_map[kf.camera_find_match(time,offset)]++;
                }
                det.push_back({blue_point.x,blue_point.y,1,i,i});
            }
        }
        double time_id =
                std::ranges::max_element(time_map, [](auto&& pair_a, auto&& pair_b) {
                return pair_a.second < pair_b.second;})->first;

        if (fabs(time_id-0)<=0.00001) return;

        std::vector<std::vector<float> > dists;
        int dist_size=0,dist_size_size=0;
        double dis_thres=get_parameter("cam.dis_thres").as_double();
        double match_thres=get_parameter("cam.match_thres").as_double();
        double dis_weight=get_parameter("cam.dis_weight").as_double();
        double his_weight=get_parameter("cam.his_weight").as_double();

        // Eigen::MatrixXd cost_matrix = getFusionCost(KFs_,det,dist_size, dist_size_size,2.6,time_id);
        std::vector<bool> is_det(KFs_.size(),false);
        Eigen::MatrixXd cost_matrix = getFusionCost(KFs_,det,dist_size, dist_size_size,dis_thres,time_id,is_det,dis_weight,his_weight);
        eigenMat2VecVec(cost_matrix,dists);

        std::vector<std::vector<int> > matches;
        std::vector<int> u_track,u_cluster;
        linear_assignment(dists, dist_size, dist_size_size,match_thres,matches,u_cluster,u_track);

        cv::Mat img=show_img1.clone();
        auto transform = Eigen::Affine3f::Identity();
        if(!get_lidar2world){
            try{
                transform_stamped = tf_buffer_.lookupTransform("rm_frame", "livox_frame", tf2::TimePointZero);
                get_lidar2world=true;
            }catch (tf2::TransformException &ex){
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Transform error: %s", ex.what());
                return;
            }
        }
        transform.translation() <<
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z;
        Eigen::Quaternionf rotation(
            transform_stamped.transform.rotation.w,
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z);
        transform.rotate(rotation);
        pcl::PointCloud<pcl::PointXYZ> temp,net_det;
        for(auto match : matches){
            KFs_[match[0]].detect_history.push_back(std::make_pair(det[match[1]][2], det[match[1]][3]));
            KFs_[match[0]].detect_time.push_back(KFs_[match[0]].GetTimeByRosTime(time));
            net_det.push_back(pcl::PointXYZ(det[match[1]][0], det[match[1]][1],0.5));
            for (auto pair:KFs_[match[0]].history) {
                if (fabs(pair.first-time_id)<0.001) {
                    temp.push_back(pcl::PointXYZ(pair.second.x,pair.second.y,pair.second.z));
                    break;
                }
            }
            if (det[match[1]][2]==1) {
                cv::rectangle(img,cv::Point(msg->blue_x1[det[match[1]][3]],detect_msg.blue_y1[det[match[1]][3]]),cv::Point(detect_msg.blue_x2[det[match[1]][3]],detect_msg.blue_y2[det[match[1]][3]]),cv::Scalar(255, 0, 0),2);
            }
            if (det[match[1]][2]==0) {
                cv::rectangle(img,cv::Point(msg->red_x1[det[match[1]][3]],detect_msg.red_y1[det[match[1]][3]]),cv::Point(detect_msg.red_x2[det[match[1]][3]],detect_msg.red_y2[det[match[1]][3]]),cv::Scalar(0, 0, 255),2);
            }
            if(KFs_[match[0]].detect_history.size() > KFs_[match[0]].max_detect_history){
                KFs_[match[0]].detect_history.erase(KFs_[match[0]].detect_history.begin());
                KFs_[match[0]].detect_time.erase(KFs_[match[0]].detect_time.begin());
            }
        }
        ori_net.header.frame_id = "rm_frame";
        net_det.header.frame_id = "rm_frame";
        temp.header.frame_id = "rm_frame";
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(net_det, output);
        output.header.frame_id = "rm_frame";
        output.header.stamp = msg->header.stamp;
        net_pub->publish(output);
        pcl::toROSMsg(ori_net, output);
        output.header.stamp = msg->header.stamp;
        net_pub->publish(output);

        pcl::transformPointCloud(temp,temp, transform.inverse());
        for(size_t i=0;i<temp.points.size();i++){
            cv::Point3d pts_2d=lidarToCamera(temp.points[i],camera_matrix1,lidar2cam1);
            if(pts_2d.y>=0&&pts_2d.y<4096&&pts_2d.x>=0&&pts_2d.x<3000){
                cv::circle(img,cv::Point(pts_2d.y,pts_2d.x),20,cv::Scalar(0, 255, 255),-1);
            }
        }
        pcl::toROSMsg(temp, output);
        output.header.stamp = msg->header.stamp;
        pc_pub->publish(output);
        // cv::imshow("depth",img);
        // cv::waitKey(1);

        mtx.lock();
        KFs= KFs_;
        mtx.unlock();
    }

    void KalmanFilter::check(std::vector<Kalman_filter_plus> &KFs_){
        for (int i=0;i<KFs_.size();i++) {
            int i_color=KFs_[i].get_color(),i_number=KFs_[i].get_number(),max_index=i;
            int max_freq=KFs_[i].get_freq(i_color,i_number).first;
            std::vector<int> same_lists;
            same_lists.push_back(i);
            for(int j=i+1;j<KFs_.size();j++) {
                int j_color=KFs_[j].get_color(),j_number=KFs_[j].get_number();
                int j_freq=KFs_[j].get_freq(j_color,j_number).first;
                if (i_color==j_color&&i_number==j_number) {
                    if (j_freq>max_freq) {
                        max_freq=j_freq;
                        max_index=j;
                    }
                    same_lists.push_back(j);
                }
            }
            for (auto index:same_lists) {
                std::set<int,std::greater<>> remove_detect;
                if (index!=max_index) {
                    for (int i=0;i<KFs_[index].detect_history.size();i++) {
                        if (KFs_[index].detect_history[i].first==i_color&&KFs_[index].detect_history[i].second==i_number)
                            remove_detect.insert(i);
                    }
                }
                for (auto idx:remove_detect) {
                    KFs_[index].detect_history.erase(KFs_[index].detect_history.begin()+idx);
                }
            }
        }
    }

    //检查是否有超出地图边界的及近似重合的检测器，并将其删除predict_point
    void KalmanFilter::checkKFs(std::vector<Kalman_filter_plus> &KFs_){
        std::set<int,std::greater<>> remove_KFs_;
        for(int i=0;i<KFs_.size();i++){
            // for(int j=i+1;j<KFs_.size();j++){
            //     if(KFs_[i].Distance(KFs_[i].predict_point,KFs_[j].predict_point)<=0.35)
            //         remove_KFs_.insert(j);
            // }

            if(KFs_[i].predict_point.x>=28.0||KFs_[i].predict_point.x<=0.0||
                KFs_[i].predict_point.y>=15.0||KFs_[i].predict_point.y<=0.0)
                remove_KFs_.insert(i);
            int history_size=KFs_[i].history.size();
            if (history_size==1)continue;
            double distance=Distance(KFs_[i].history[history_size-1].second,KFs_[i].history[history_size-2].second);
            double time=KFs_[i].history[history_size-1].first-KFs_[i].history[history_size-2].first;
            double speed_x=KFs_[i].KF.statePost.at<float>(1);
            double speed_y=KFs_[i].KF.statePost.at<float>(3);
            double speed=sqrt(speed_x*speed_x+speed_y*speed_y);
            // std::cout<<"distance:"<<distance<<"   time:"<<time<<"  speed:"<<distance/time<<std::endl;
            if(speed>4.7)
                remove_KFs_.insert(i);
        }
        for (auto index:remove_KFs_) {
            KFs_.erase(KFs_.begin()+index);
        }
    }

    void KalmanFilter::checkClass(std::vector<Kalman_filter_plus> &KFs_) {
        int num=0;
        std::vector<std::vector<int>> history={{0,0,0,0,0,0},{0,0,0,0,0,0}};

        for(int i=0;span==0&&i<KFs_.size();i++){
            if(KFs_[i].detect_history.size()==0)continue;
            //此处还需要优化，避免出现多个卡尔曼对象对应到同一个红蓝点
            int i_color=KFs_[i].get_color(),i_number=KFs_[i].get_number(),max_index=i;
            // std::cout<<"---"<<i_color<<" "<<i_number<<"---"<<std::endl;
            auto freq_time=KFs_[i].get_freq(i_color,i_number);
            //time_index更新时间最近的卡尔曼索引，max_index最多匹配的卡尔曼索引
            int max_freq=freq_time.first,time_index=i;
            double max_time=freq_time.second;

            std::vector<int> same_lists;
            same_lists.push_back(i);
            if (history[i_color][i_number]>0)continue;

            for(int j=0;j<KFs_.size();j++) {
                int j_color=KFs_[j].get_color(),j_number=KFs_[j].get_number();
                auto j_freq_time=KFs_[j].get_freq(j_color,j_number);
                int j_freq=j_freq_time.first;
                double j_time=j_freq_time.second;
                if (i_color==j_color&&i_number==j_number) {
                    if (j_freq>max_freq) {
                        max_freq=j_freq;
                        max_index=j;
                    }
                    if (j_time>max_time) {
                        max_time=j_time;
                        time_index=j;
                    }
                    same_lists.push_back(j);
                }
            }
            //对不是最近更新的卡尔曼对象，删除时间最久的一个检测记录
            if (same_lists.size()>1) {
                for (auto index:same_lists) {
                    std::set<int,std::greater<>> remove_detect;
                    double min_time=max_time;
                    int min_index=-1;
                    if (index!=time_index) {
                        for (int i=0;i<KFs_[index].detect_history.size();i++) {
                            if (KFs_[index].detect_history[i].first==i_color&&KFs_[index].detect_history[i].second==i_number) {
                                if (KFs_[index].detect_time[i]<min_time) {
                                    min_time=KFs_[index].detect_time[i];
                                    min_index=i;
                                }
                            }
                        }
                        if (min_index!=-1)
                            remove_detect.insert(min_index);
                    }
                    for (auto idx:remove_detect) {
                        KFs_[index].detect_history.erase(KFs_[index].detect_history.begin()+idx);
                        KFs_[index].detect_time.erase(KFs_[index].detect_time.begin()+idx);
                    }
                    // KFs_[index].ws_armorConfMatrix(0,i_color*5+i_number)=std::max(0.1,KFs_[index].ws_armorConfMatrix(0,i_color*5+i_number)-0.1);
                }
            }
        //     //最终检测结果采用最多匹配的卡尔曼对象
        //     if(i_color == 1){//蓝色
        //         history[1][i_number]++;
        //         vis_kal_maker(1,vis_array,1,i_number,KFs_[max_index].predict_point.x,KFs_[max_index].predict_point.y,KFs_[max_index].detect_history.size(),history[1][i_number]);
        //         if (KFs_[max_index].last_time>1.5) {
        //             continue;
        //         }
        //         detect_res.blue_x[i_number] = KFs_[max_index].predict_point.x;
        //         detect_res.blue_y[i_number] = KFs_[max_index].predict_point.y;
        //         if(self_color==0) {
        //             detect_res.v_x[i_number] = KFs_[max_index].KF.statePost.at<float>(1);
        //             detect_res.v_y[i_number] = KFs_[max_index].KF.statePost.at<float>(3);
        //         }
        //     }else if(i_color == 0){//红色
        //         history[0][i_number]++;
        //         vis_kal_maker(1,vis_array,0,i_number,KFs_[max_index].predict_point.x,KFs_[max_index].predict_point.y,KFs_[max_index].detect_history.size(),history[0][i_number]);
        //         if (KFs_[max_index].last_time>1.5) {
        //             continue;
        //         }
        //         detect_res.red_x[i_number] = KFs_[max_index].predict_point.x;
        //         detect_res.red_y[i_number] = KFs_[max_index].predict_point.y;
        //         if(self_color==1) {
        //             detect_res.v_x[i_number] = KFs_[max_index].KF.statePost.at<float>(1);
        //             detect_res.v_y[i_number] = KFs_[max_index].KF.statePost.at<float>(3);
        //         }
        //     }
        // num++;
        }
    }

    void KalmanFilter::clearOutPut() {
        for (int i=0;i<5;i++) {
            if (lidar_enhance_[0][i]==0||lidar_enhance_[0][i]==5||lidar_enhance_[0][i]==6) {
                if (self_color==1) {
                    detect_res.v_x[i]=0;
                    detect_res.v_y[i]=0;
                }
                if (lidar_enhance_[0][i]==5||lidar_enhance_[0][i]==6)
                    lidar_enhance_[0][i]=0;
                detect_res.red_x[i]=0;
                detect_res.red_y[i]=0;
            }
            if (lidar_enhance_[1][i]==0||lidar_enhance_[1][i]==5||lidar_enhance_[1][i]==6) {
                if (self_color==0) {
                    detect_res.v_x[i]=0;
                    detect_res.v_y[i]=0;
                }
                if (lidar_enhance_[1][i]==5||lidar_enhance_[1][i]==6)
                    lidar_enhance_[1][i]=0;
                detect_res.blue_x[i]=0;
                detect_res.blue_y[i]=0;
            }
        }
    }

    void KalmanFilter::getRect2d(std::vector<open3d::geometry::PointCloud> pcs,Eigen::Transform<float, 3, 2> transform,std::vector<cv::Rect> &rects,int camid) {
        // std::vector<cv::Point3d> pts;
        for(auto pc:pcs){
            // cv::Mat img=show_img.clone();
            pc.Transform(transform.matrix().cast<double>().inverse());
            cv::Point2f max(-100000,-100000),min(100000,100000);
            for (auto point:pc.points_) {
                cv::Point3d pts_2d;
                if (camid==1)
                    pts_2d=lidarToCamera(pcl::PointXYZ(point[0],point[1],point[2]),camera_matrix1,lidar2cam1);
                else if (camid==2)
                    pts_2d=lidarToCamera(pcl::PointXYZ(point[0],point[1],point[2]),camera_matrix2,lidar2cam2);

                if (pts_2d.y>max.y) {
                    max.y=pts_2d.y;
                }
                if (pts_2d.x>max.x) {
                    max.x=pts_2d.x;
                }
                if (pts_2d.x<min.x) {
                    min.x=pts_2d.x;
                }
                if (pts_2d.y<min.y) {
                    min.y=pts_2d.y;
                }
            }
            rects.push_back(cv::Rect(min,max));
            // if (camid==1) {
            //     auto ttt=lidarToCamera(pcl::PointXYZ(pc.GetCenter()[0],pc.GetCenter()[1],pc.GetCenter()[2]),camera_matrix1,lidar2cam1);
            //     pts.push_back(ttt);
            // }else if (camid==2) {
            //     auto ttt=lidarToCamera(pcl::PointXYZ(pc.GetCenter()[0],pc.GetCenter()[1],pc.GetCenter()[2]),camera_matrix2,lidar2cam2);
            //     pts.push_back(ttt);
            // }
        }
        // if (camid==1) {
        //     cv::Mat img1=show_img1.clone();
        //     for (int i=0;i<pcs.size();i++) {
        //         cv::rectangle(img1,cv::Point(rects[i].x,rects[i].y),cv::Point(rects[i].x+rects[i].height,rects[i].y+rects[i].width),cv::Scalar(0,0,255),2);
        //         cv::circle(img1,cv::Point(pts[i].x,pts[i].y),5,cv::Scalar(0,0,255),-1);
        //     }
        //     cv::imshow("cam1",img1);
        //     cv::waitKey(1);
        // }else if (camid==2) {
        //     cv::Mat img2=show_img2.clone();
        //     for (int i=0;i<pcs.size();i++) {
        //         cv::rectangle(img2,cv::Point(rects[i].x,rects[i].y),cv::Point(rects[i].x+rects[i].height,rects[i].y+rects[i].width),cv::Scalar(0,0,255),2);
        //         cv::circle(img2,cv::Point(pts[i].x,pts[i].y),5,cv::Scalar(0,0,255),-1);
        //     }
        //     cv::imshow("cam2",img2);
        //     cv::waitKey(1);
        // }
    }

    void KalmanFilter::getClusterID(std::vector<ClusPC>&clus_pcs,std::vector<open3d::geometry::PointCloud> pcs,std::vector<cv::Rect>rects) {
        clus_pcs.resize(pcs.size());
        std::vector<std::array<double, 8>> det;
        std::vector<bool> overlap(cam_msg.obj.size(),false);
        std::vector<bool> is_det(pcs.size(),false);
        pcl::PointCloud<pcl::PointXYZ> ori_net;
        for (int i=0;i<cam_msg.obj.size();i++) {
            if (cam_msg.obj[i].x==0||cam_msg.obj[i].y==0) continue;
            for (int j=i+1;j<cam_msg.obj.size();j++) {
                if (!(cam_msg.obj[i].x2 < cam_msg.obj[j].x1 || cam_msg.obj[j].x2 < cam_msg.obj[i].x1) &&
                    !(cam_msg.obj[i].y1 > cam_msg.obj[j].y2 || cam_msg.obj[j].y1 > cam_msg.obj[i].y2)&&
                    cam_msg.obj[j].x!=0&&cam_msg.obj[j].y!=0) {
                    overlap[i]=true;
                    overlap[j]=true;
                }
            }
        }
        for (int i=0;i<cam_msg.obj.size();i++) {
            interfaces::msg::DetectObj obj=cam_msg.obj[i];
            if(obj.x!=0&&obj.y!=0){
                ori_net.points.push_back(pcl::PointXYZ(obj.x,obj.y,2));
                if (overlap[i]) {
                    if(self_color==1){//己方为蓝色时将坐标反转，因为此时相机传回的是准确的，雷达聚类的是反转的
                        obj.x=28-obj.x;
                        obj.y=15-obj.y;
                    }
                    if (obj.classid<5) {
                        det.push_back({obj.x,obj.y,1,obj.classid,obj.x1,obj.y1,obj.x2,obj.y2});
                    }else {
                        det.push_back({obj.x,obj.y,0,obj.classid-5,obj.x1,obj.y1,obj.x2,obj.y2});
                    }
                }
            }
        }

        cv::Mat img=show_img1.clone();
        for (int i=0;i<cam_msg.obj.size();i++) {
            if (!overlap[i]) {
                interfaces::msg::DetectObj obj=cam_msg.obj[i];
                cv::rectangle(img,cv::Point(obj.x1,obj.y1),cv::Point(obj.x2,obj.y2),cv::Scalar(0,255,0),2);
            }
        }

        for(int i=0;i<cam_msg.obj.size();i++) {
            if (!overlap[i]) {
                int index;
                double iou=0;
                cv::Rect rect_cam(cv::Point(cam_msg.obj[i].x1, cam_msg.obj[i].y1), cv::Point(cam_msg.obj[i].x2 ,cam_msg.obj[i].y2));
                Eigen::Vector3d center;
                for (int j=0;j<pcs.size();j++) {
                    cv::Rect rect_pc;
                    rect_pc=rects[j];
                    cv::Rect Intersection = rect_cam & rect_pc;
                    cv::Rect Union = rect_cam|rect_pc;
                    if (double(Intersection.area())/double(Union.area())>iou) {
                        iou=double(Intersection.area())/double(Union.area());
                        index=j;
                        center =pcs[j].GetCenter();
                    }
                }
                // RCLCPP_ERROR(this->get_logger(), "---------------------iou %f",iou);
                interfaces::msg::DetectObj obj=cam_msg.obj[i];
                if (iou>0.2) {
                    ClusPC clus_pc;
                    if (obj.classid<5) {
                        clus_pc.color=1;
                        clus_pc.num=obj.classid;
                    }else {
                        clus_pc.color=0;
                        clus_pc.num=obj.classid-5;
                    }
                    clus_pc.time=GetTimeByRosTime(cam_msg.header.stamp);
                    clus_pc.center=center;
                    is_det[index]=true;
                    clus_pcs[index]=clus_pc;
                    // RCLCPP_ERROR(this->get_logger(),"---------------------sucess detect %d",index);
                }else {
                    if(self_color==1){//己方为蓝色时将坐标反转，因为此时相机传回的是准确的，雷达聚类的是反转的
                        obj.x=28-obj.x;
                        obj.y=15-obj.y;
                    }
                    if (obj.classid<5) {
                        det.push_back({obj.x,obj.y,1,obj.classid,obj.x1,obj.y1,obj.x2,obj.y2});
                    }else {
                        det.push_back({obj.x,obj.y,0,obj.classid-5,obj.x1,obj.y1,obj.x2,obj.y2});
                    }
                }
            }
        }

        std::vector<std::vector<float> > dists;
        int dist_size=0,dist_size_size=0;
        double dis_thres=get_parameter("cam.dis_thres").as_double();
        double match_thres=get_parameter("cam.match_thres").as_double();
        double dis_weight=get_parameter("cam.dis_weight").as_double();
        double his_weight=get_parameter("cam.his_weight").as_double();

        Eigen::MatrixXd cost_matrix = getFusionCost(pcs,det,rects,dist_size, dist_size_size,dis_thres,is_det,dis_weight,his_weight);
        eigenMat2VecVec(cost_matrix,dists);

        std::vector<std::vector<int> > matches;
        std::vector<int> u_track,u_cluster;
        linear_assignment(dists, dist_size, dist_size_size,match_thres,matches,u_cluster,u_track);

        for(auto match : matches){
            ClusPC clus_pc;
            clus_pc.center=pcs[match[0]].GetCenter();
            clus_pc.color=det[match[1]][2];
            clus_pc.num=det[match[1]][3];
            clus_pc.time=GetTimeByRosTime(cam_msg.header.stamp);
            clus_pcs[match[0]]=clus_pc;
        }
        for (auto index:u_cluster) {
            if (is_det[index]) continue;
            ClusPC clus_pc;
            clus_pc.center=pcs[index].GetCenter();
            clus_pcs[index]=clus_pc;
        }

        for (int j=0;j<pcs.size();j++) {
            if (!is_det[j])continue;
            cv::Rect rect_pc=rects[j];
            cv::rectangle(img,cv::Point(rect_pc.x,rect_pc.y),cv::Point(rect_pc.x+rect_pc.height,rect_pc.y+rect_pc.width),cv::Scalar(0,0,255),2);
        }
        cv::imshow("depth",img);
        cv::waitKey(1);
    }

    void KalmanFilter::updateKFs(std::vector<Kalman_filter_plus> &KFs_,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xy,const std::vector<cv::Rect> &rects1,
        const std::vector<cv::Rect> &rects2,const std::vector<open3d::geometry::PointCloud> &pcs,pcl::PointCloud<pcl::PointXYZ> &kmeans_out,rclcpp::Time time) {

        std::vector<cv::Point3d> centers1;
        for(auto &kf : KFs_){//一开始KFs_中没有元素，因此此处循环不会进行 若有元素，则进行预测更新
            kf.update_predict_point();
            kf.has_updated = false;
            centers1.push_back(lidarToCamera(pcl::PointXYZ(kf.predict_point.x,kf.predict_point.y,kf.history.back().second.z),camera_matrix1,lidar2cam1));
        }
        // checkKFs(KFs_);

        double clu_match_thres=get_parameter("clu.match_thres").as_double();
        double clu_dis_thres=get_parameter("clu.dis_thres").as_double();
        double clu_min_dis_thres=get_parameter("clu.min_dis_thres").as_double();

        std::vector<std::vector<float> > dists;
        int dist_size=0,dist_size_size=0;
        Eigen::MatrixXd cost_matrix = getCloudCost(*cloud_xy,KFs_,rects1,centers1,dist_size, dist_size_size,clu_dis_thres);
        eigenMat2VecVec(cost_matrix,dists);
        std::vector<std::vector<int> > matches;
        std::vector<int> u_track,u_cluster;
        //进行匹配
        linear_assignment(dists, dist_size, dist_size_size,clu_match_thres ,matches,u_cluster,u_track);
        std::vector<KUData> again_clus(pcs.size());
        std::vector<bool> is_kmeans(KFs_.size(),false);

        for (auto index:u_track) {
            Kalman_filter_plus kf=KFs_[index];
            if (kf.last_time>2.5||kf.detect_history.size()==0||KFs_[index].kmeans_time_>15) continue;
            for(int i=0;i<pcs.size();i++) {
                std::array<cv::Point2f,5> AABB;
                auto min_pc=pcs[i].GetMinBound();
                auto max_pc=pcs[i].GetMaxBound();
                AABB[0]=cv::Point2f(min_pc[0]-0.1,min_pc[1]-0.1);
                AABB[1]=cv::Point2f(min_pc[0]-0.1,max_pc[1]+0.1);
                AABB[2]=cv::Point2f(max_pc[0]+0.1,max_pc[1]+0.1);
                AABB[3]=cv::Point2f(max_pc[0]+0.1,min_pc[1]-0.1);
                if (initornot(AABB,kf.predict_point,4)==1) {
                    again_clus[i].mutli_in+=1;
                    again_clus[i].utrack_idx.push_back(index);
                    is_kmeans[index]=true;
                    break;
                }
            }
        }

        for(auto match : matches){
            if (again_clus[match[0]].mutli_in>0) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr new_pc(new pcl::PointCloud<pcl::PointXYZ>);
                for(auto point:pcs[match[0]].points_)
                    new_pc->points.push_back(pcl::PointXYZ(point[0],point[1],point[2]));

                int num_dimension = 3;
                int num_points = new_pc->size();
                std::vector<std::vector<float>> features(num_points, std::vector<float>(num_dimension));
                for (int i = 0; i < new_pc->points.size(); ++i){
                    features[i][0] = new_pc->points[i].x;
                    features[i][1] = new_pc->points[i].y;
                    features[i][2] = new_pc->points[i].z;
                }

                pcl::Kmeans kmean(num_points, num_dimension);
                kmean.setInputData(features);
                kmean.setClusterSize(again_clus[match[0]].mutli_in+1);
                kmean.kMeans();

                auto center = kmean.get_centroids();

                for (auto point:center) {
                    kmeans_out.points.push_back(pcl::PointXYZ(point[0],point[1],point[2]));
                    again_clus[match[0]].kmeans_pc.points.push_back(pcl::PointXYZI(point[0],point[1],point[2],0));
                    // std::cout<<"x: "<<point[0]<<"y: "<<point[1]<<"z: "<<point[2]<<std::endl;
                }
                std::vector<Kalman_filter_plus> new_KFs;
                new_KFs.push_back(KFs_[match[1]]);
                for(auto index:again_clus[match[0]].utrack_idx) {
                    new_KFs.push_back(KFs_[index]);
                }

                std::vector<std::vector<float> > new_dists;
                int new_dist_size=0,new_dist_size_size=0;
                Eigen::MatrixXd new_cost_matrix = getCloudCost(again_clus[match[0]].kmeans_pc,new_KFs,new_dist_size, new_dist_size_size,clu_dis_thres);
                eigenMat2VecVec(new_cost_matrix,new_dists);
                std::vector<std::vector<int> > new_matches;
                std::vector<int> new_u_track,new_u_cluster;
                //进行匹配
                linear_assignment(new_dists, new_dist_size, new_dist_size_size,clu_match_thres ,new_matches,new_u_cluster,new_u_track);
                for(auto new_match : new_matches) {
                    if (new_match[1]==0) {
                        KFs_[match[1]].update(again_clus[match[0]].kmeans_pc.points[new_match[0]],time,rects1[match[0]]);
                        KFs_[match[1]].kmeans_time_++;
                        KFs_[match[1]].rect_2d2.push_back(rects2[match[0]]);
                        if (KFs_[match[1]].rect_2d2.size()>KFs_[match[1]].max_history)
                            KFs_[match[1]].rect_2d2.erase(KFs_[match[1]].rect_2d2.begin());
                    }else {
                        KFs_[again_clus[match[0]].utrack_idx[new_match[1]-1]].update(again_clus[match[0]].kmeans_pc.points[new_match[0]],time,rects1[match[0]]);
                        KFs_[again_clus[match[0]].utrack_idx[new_match[1]-1]].kmeans_time_++;
                        KFs_[again_clus[match[0]].utrack_idx[new_match[1]-1]].rect_2d2.push_back(rects2[match[0]]);
                        if (KFs_[again_clus[match[0]].utrack_idx[new_match[1]-1]].rect_2d2.size()>KFs_[again_clus[match[0]].utrack_idx[new_match[1]-1]].max_history)
                            KFs_[again_clus[match[0]].utrack_idx[new_match[1]-1]].rect_2d2.erase(KFs_[again_clus[match[0]].utrack_idx[new_match[1]-1]].rect_2d2.begin());
                    }
                }
            }else {
                KFs_[match[1]].update(cloud_xy->points[match[0]],time,rects1[match[0]]);
                KFs_[match[1]].kmeans_time_=0;
                KFs_[match[1]].rect_2d2.push_back(rects2[match[0]]);
                if (KFs_[match[1]].rect_2d2.size()>KFs_[match[1]].max_history)
                    KFs_[match[1]].rect_2d2.erase(KFs_[match[1]].rect_2d2.begin());
            }
        }

        //TODO：没有更新的kalman 用距离找符合阈值的最近的聚类去更新

        // for (auto index:u_track) {
        //     if (is_kmeans[index]) continue;
        //     double min_distance = 10000.0;
        //     int min_index = -1;
        //     for (int i=0;i<cloud_xy->size();i++) {
        //         double distance = KFs_[index].Distance(KFs_[index].predict_point,pcl::PointXY(cloud_xy->points[i].x,cloud_xy->points[i].y));
        //         if (distance<clu_dis_thres*clu_match_thres&&distance<min_distance) {
        //             min_index=i;
        //             min_distance=distance;
        //         }
        //     }
        //     if (min_index != -1) {
        //         KFs_[index].update(cloud_xy->points[min_index],time,rects1[min_index]);
        //         KFs_[index].rect_2d2.push_back(rects2[min_index]);
        //         if (KFs_[index].rect_2d2.size()>KFs_[index].max_history)
        //             KFs_[index].rect_2d2.erase(KFs_[index].rect_2d2.begin());
        //     }
        // }

        //新增kalman
        for (auto cluster:u_cluster) {
            double min_dis=10000.0;
            for (auto match : matches) {
                double distance=Distance(cloud_xy->points[cluster],cloud_xy->points[match[0]]);
                if (distance<min_dis) {
                    min_dis=distance;
                }
            }
            if (min_dis>clu_min_dis_thres) {
                Kalman_filter_plus kf(cloud_xy->points[cluster], time,reinterpret_cast<rclcpp::Node*>(this),rects1[cluster]);//time为最后更新时间
                kf.rect_2d2.push_back(rects2[cluster]);
                KFs_.push_back(kf);
            }
        }
        checkKFs(KFs_);
    }

    void KalmanFilter::droneTimeSynC(const sensor_msgs::msg::PointCloud2::SharedPtr msg1, const sensor_msgs::msg::PointCloud2::SharedPtr msg2) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr receive_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg1, *cloud1);
        pcl::fromROSMsg(*msg2, *cloud2);
        *receive_cloud+=*cloud1;
        *receive_cloud+=*cloud2;
        if (receive_cloud->empty()) {return;}

        open3d::geometry::PointCloud in_cloud;
        for(size_t i=0;i<receive_cloud->points.size();i++)
            in_cloud.points_.push_back(Eigen::Vector3d(receive_cloud->points[i].x,receive_cloud->points[i].y,receive_cloud->points[i].z));
        std::vector<int> labels;
        std::vector<std::vector<int>> nbs(in_cloud.points_.size());
        std::vector<std::vector<int>> clusters;
        labels=normalDBSCAN(in_cloud,1.4, 10,nbs);

        std::vector<open3d::geometry::PointCloud> pcs;
        open3d::geometry::PointCloud pc_noise;
        int max_l = *std::max_element(labels.begin(), labels.end());
        for (int i = 0; i <= max_l; i++) {
            pcs.emplace_back(open3d::geometry::PointCloud());
        }
        if (pcs.empty()) {return;}
        for (size_t i = 0; i < in_cloud.points_.size(); i++) {
            if(labels[i] >= 0) {
                pcs[labels[i]].points_.push_back(in_cloud.points_[i]);
            }else
                pc_noise.points_.push_back(in_cloud.points_[i]);
        }

        int max_index=0;
        if (max_l>=1) max_index=std::distance(pcs.begin(), std::max_element(pcs.begin(), pcs.end(), compareBySize));
        interfaces::msg::DroneLocation drone_location;
        drone_location.header.stamp = msg1->header.stamp;
        drone_location.x = int(((26.2-pcs[max_index].GetCenter()[0])/15.2)*100.0);
        if (drone_location.x>100)drone_location.x=100;
        if (drone_location.x<0)drone_location.x=0;
        pub_drone_->publish(drone_location);
    }

    void KalmanFilter::PCTimeSynC(const sensor_msgs::msg::PointCloud2::SharedPtr msg1, const sensor_msgs::msg::PointCloud2::SharedPtr msg2) {
        rclcpp::Time time = msg1->header.stamp;
        std::vector<Kalman_filter_plus> KFs_;
        mtx.lock();
        KFs_= KFs;
        mtx.unlock();

        clearOutPut();
        auto transform = Eigen::Affine3f::Identity();
        if(!get_lidar2world){
            try{
                transform_stamped = tf_buffer_.lookupTransform("rm_frame", "lidar_avia_frame", tf2::TimePointZero);
                get_lidar2world=true;
            }catch (tf2::TransformException &ex){
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Transform error: %s", ex.what());
                return;
            }
        }
        transform.translation() <<
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z;
        Eigen::Quaternionf rotation(
            transform_stamped.transform.rotation.w,
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z);
        transform.rotate(rotation);

        auto now_time = std::chrono::steady_clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr receive_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg1, *cloud1);
        pcl::fromROSMsg(*msg2, *cloud2);
        if (cloud1->empty()) {return;}
        if (cloud2->empty()) {return;}

        *receive_cloud+=*cloud1;
        *receive_cloud+=*cloud2;

        auto now_time1 = std::chrono::steady_clock::now();
        std::vector<open3d::geometry::PointCloud> pcs;
        getCluster(receive_cloud,pcs);
        std::vector<cv::Rect> rects1,rects2;
        getRect2d(pcs,transform,rects1,1);
        getRect2d(pcs,transform,rects2,2);

        std::vector<ClusPC> clus_pcs;
        // getClusterID(clus_pcs,pcs,rects);
        auto end_time1 = std::chrono::steady_clock::now();
        float dur_time1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time1 - now_time1).count();
        // RCLCPP_WARN(this->get_logger(), "cluster Callback time is %f ms", dur_time1);

        for(int i=0;i<pcs.size();i++){
            pcl::PointXYZI point_xy;
            point_xy.x = pcs[i].GetCenter()[0];
            point_xy.y = pcs[i].GetCenter()[1];
            point_xy.z = pcs[i].GetCenter()[2];
            point_xy.intensity = 0;
            cloud_xy->points.push_back(point_xy);
        }
        if(cloud_xy->points.size() == 0)return;

        sensor_msgs::msg::PointCloud2 cluster_out;
        pcl::toROSMsg(*cloud_xy, cluster_out);
        cluster_out.header.frame_id = "rm_frame";
        cluster_out.header.stamp = msg1->header.stamp;
        pub_cluster->publish(cluster_out);

        pcl::PointCloud<pcl::PointXYZ> kmeans_out;
        updateKFs(KFs_,cloud_xy,rects1,rects2,pcs,kmeans_out,time);

        sensor_msgs::msg::PointCloud2 output_kms;
        pcl::toROSMsg(kmeans_out, output_kms);
        output_kms.header.frame_id = "rm_frame";
        output_kms.header.stamp = msg1->header.stamp;
        pub_kmeans->publish(output_kms);

        std::vector<int> remove_KFs_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int i = KFs_.size() - 1; i >= 0; i--){
            KFs_[i].forword_predict();
            if(KFs_[i].last_time>3.5)
                remove_KFs_.push_back(i);
        }
        sort(remove_KFs_.begin(),remove_KFs_.end(),std::greater<>());
        for (auto index:remove_KFs_)
            KFs_.erase(KFs_.begin() + index);

        // check();
        visualization_msgs::msg::MarkerArray vis_array;

        for(int i = KFs_.size() - 1; i >= 0; i--){
            pcl::PointXYZRGB point;
            point.x = KFs_[i].predict_point.x;
            point.y = KFs_[i].predict_point.y;
            point.z = 1.5;
            int color = KFs_[i].get_color();
            switch (color){
            case 1://blue
                point.b = 255;
                break;

            case 0://red
                point.r = 255;
                break;

            default:
                point.r = 255;
                point.g = 255;
                point.b = 255;
                break;
            }
            cloud_filtered->points.push_back(point);
        }

        for(int i=0;i<6;i++){
            if(detect_msg.red_x[i]!=0&&detect_msg.red_y[i]!=0){
                // cloud_filtered->points.push_back(pcl::PointXYZRGB(detect_msg.red_x[i],detect_msg.red_y[i],1.5,255,241,67));
                vis_maker(2,vis_array,0,i,detect_msg.red_x[i],detect_msg.red_y[i]);
            }

            if(detect_msg.blue_x[i]!=0&&detect_msg.blue_y[i]!=0){
                // cloud_filtered->points.push_back(pcl::PointXYZRGB(detect_msg.blue_x[i],detect_msg.blue_y[i],1.5,0,176,240));
                vis_maker(2,vis_array,1,i,detect_msg.blue_x[i],detect_msg.blue_y[i]);
            }
        }

        cloud_filtered->header.frame_id = "rm_frame";
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header.frame_id = "rm_frame";
        output.header.stamp = msg1->header.stamp;
        pub_kalman->publish(output);

        std::vector<std::vector<int>> history_cost={{0,0,0,0,0,0},{0,0,0,0,0,0}};
        checkClass(KFs_);
        checkLocation(KFs_);

        //利用匈牙利匹配输出最终定位结果
        std::vector<std::vector<float>> cost_confMatrix_vec;
        int num_strack,num_cls;
        std::vector<std::vector<int> > matches_cls;
        std::vector<int> u_strack, u_cls;
        matches_cls.clear();u_strack.clear();u_cls.clear();
        Eigen::MatrixXd cost_confMatrix = getCost_confMatrix(KFs_,num_strack,num_cls);
        // std::cout << "cost_confMatrix_cls:  " << std::endl << cost_confMatrix << std::endl;
        eigenMat2VecVec(cost_confMatrix, cost_confMatrix_vec);
        linear_assignment(cost_confMatrix_vec, num_strack, num_cls,0.95, matches_cls, u_strack, u_cls);

        for (auto match : matches_cls) {
            if(match[1]>=5){//蓝色
                history_cost[1][match[1]-5]++;
                vis_kal_maker(1,vis_array,1,match[1]-5,KFs_[match[0]].predict_point.x,KFs_[match[0]].predict_point.y,KFs_[match[0]].detect_history.size(),history_cost[1][match[1]-5]);
                // if (KFs_[match[0]].last_time>1.5) {
                //     continue;
                // }
                detect_res.blue_x[match[1]-5] = KFs_[match[0]].output_point.x;
                detect_res.blue_y[match[1]-5] = KFs_[match[0]].output_point.y;
                if(self_color==0) {
                    detect_res.v_x[match[1]-5] = KFs_[match[0]].KF.statePost.at<float>(1);
                    detect_res.v_y[match[1]-5] = KFs_[match[0]].KF.statePost.at<float>(3);
                }
                if (KFs_[match[0]].last_time<0.5)
                    lidar_enhance_[1][match[1]-5]=0;
            }else{//红色
                history_cost[0][match[1]]++;
                vis_kal_maker(1,vis_array,0,match[1],KFs_[match[0]].predict_point.x,KFs_[match[0]].predict_point.y,KFs_[match[0]].detect_history.size(),history_cost[0][match[1]]);
                // if (KFs_[match[0]].last_time>1.5) {
                //     continue;
                // }
                detect_res.red_x[match[1]] = KFs_[match[0]].output_point.x;
                detect_res.red_y[match[1]] = KFs_[match[0]].output_point.y;
                if(self_color==1) {
                    detect_res.v_x[match[1]] = KFs_[match[0]].KF.statePost.at<float>(1);
                    detect_res.v_y[match[1]] = KFs_[match[0]].KF.statePost.at<float>(3);
                }
                if (KFs_[match[0]].last_time<0.5)
                    lidar_enhance_[0][match[1]]=0;
            }
        }

        //未知点（无种类）进行特殊位置猜测
        guessWithoutClass(KFs_,u_strack,vis_array,history_cost);

        remove_KFs_.clear();
        //进行特殊位置跟踪器处理及猜点
        guessWithClass(KFs_,matches_cls,remove_KFs_);

        //进行速度模拟
        speedSimulation();

        sort(remove_KFs_.begin(),remove_KFs_.end(),std::greater<>());
        for (auto index:remove_KFs_)
            KFs_.erase(KFs_.begin() + index);

        //self_color==1为自己为蓝方
        if(self_color==1){
            //地图默认以红方的角点为原点，因此己方为蓝方时需要转换坐标系
            for(int i=0;i<6;i++){
                if(detect_res.blue_x[i]!=0&&detect_res.blue_y[i]!=0&&
                (lidar_enhance_[1][i]==0||lidar_enhance_[1][i]==5||lidar_enhance_[1][i]==6)){
                    detect_res.blue_x[i]=28-detect_res.blue_x[i];
                    detect_res.blue_y[i]=15-detect_res.blue_y[i];
                }
                if(detect_res.red_x[i]!=0&&detect_res.red_y[i]!=0&&
                (lidar_enhance_[0][i]==0||lidar_enhance_[0][i]==5||lidar_enhance_[0][i]==6)){
                    detect_res.red_x[i]=28-detect_res.red_x[i];
                    detect_res.red_y[i]=15-detect_res.red_y[i];
                    detect_res.v_x[i]=-detect_res.v_x[i];
                    detect_res.v_y[i]=-detect_res.v_y[i];
                }
            }
        }
        for(int i=0;i<6;i++){
            if(detect_res.blue_x[i]!=0&&detect_res.blue_y[i]!=0){
                vis_maker(3,vis_array,1,i,detect_res.blue_x[i],detect_res.blue_y[i]);
            }
            if(detect_res.red_x[i]!=0&&detect_res.red_y[i]!=0){
                vis_maker(3,vis_array,0,i,detect_res.red_x[i],detect_res.red_y[i]);
            }
        }
        interfaces::msg::LidarEnhance lidar_enhance;
        for (int i=0;i<5;i++) {
            lidar_enhance.blue_enhance[i]=lidar_enhance_[1][i];
            lidar_enhance.red_enhance[i]=lidar_enhance_[0][i];
        }
        detect_res.header.stamp = rclcpp::Clock().now();
        lidar_enhance.header.stamp = rclcpp::Clock().now();
        lidar_detect_pub->publish(detect_res);
        pub_vis->publish(vis_array);
        lidar_enh_pub_->publish(lidar_enhance);

        mtx.lock();
        KFs= KFs_;
        mtx.unlock();

        auto end_time = std::chrono::steady_clock::now();
        float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
        RCLCPP_WARN(this->get_logger(), "Kalman Callback time is %f ms", dur_time);
        // span++;
        // span=span%2;
    }
    /*
    void KalmanFilter::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        rclcpp::Time time = msg->header.stamp;
        std::vector<Kalman_filter_plus> KFs_;
        mtx.lock();
        KFs_= KFs;
        mtx.unlock();

        auto transform = Eigen::Affine3f::Identity();
        if(!get_lidar2world){
            try{
                transform_stamped = tf_buffer_.lookupTransform("rm_frame", "livox_frame", tf2::TimePointZero);
                get_lidar2world=true;
            }catch (tf2::TransformException &ex){
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Transform error: %s", ex.what());
                return;
            }
        }
        transform.translation() <<
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z;
        Eigen::Quaternionf rotation(
            transform_stamped.transform.rotation.w,
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z);
        transform.rotate(rotation);

        auto now_time = std::chrono::steady_clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        acc_clouds.push_back(*cloud);
        if (acc_clouds.size()>7) {
            acc_clouds.erase(acc_clouds.begin());
            *cloud=acc_clouds.front();
        }else {
            return;
        }
        auto now_time1 = std::chrono::steady_clock::now();
        std::vector<open3d::geometry::PointCloud> pcs;
        getCluster(cloud,pcs);
        std::vector<cv::Rect> rects;
        getRect2d(pcs,transform,rects,1);
        std::vector<Clus_pc> clus_pcs;
        getClusterID(clus_pcs,pcs,rects);
        auto end_time1 = std::chrono::steady_clock::now();
        float dur_time1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time1 - now_time1).count();
        RCLCPP_WARN(this->get_logger(), "cluster Callback time is %f ms", dur_time1);
        for(int i=0;i<pcs.size();i++){
            pcl::PointXYZI point_xy;
            point_xy.x = pcs[i].GetCenter()[0];
            point_xy.y = pcs[i].GetCenter()[1];
            point_xy.z = pcs[i].GetCenter()[2];
            point_xy.intensity = 0;
            cloud_xy->points.push_back(point_xy);
        }
        if(cloud_xy->points.size() == 0)return;
        sensor_msgs::msg::PointCloud2 cluster_out;
        pcl::toROSMsg(*cloud_xy, cluster_out);
        cluster_out.header.frame_id = "rm_frame";
        cluster_out.header.stamp = msg->header.stamp;
        pub_cluster->publish(cluster_out);

        for(auto &kf : KFs_){//一开始KFs_中没有元素，因此此处循环不会进行
                       //若有元素，则进行预测更新
            kf.update_predict_point();
            kf.has_updated = false;
        }

        checkKFs(KFs_);

        double clu_match_thres=get_parameter("clu.match_thres").as_double();
        double clu_dis_thres=get_parameter("clu.dis_thres").as_double();
        double clu_min_dis_thres=get_parameter("clu.min_dis_thres").as_double();

        std::vector<std::vector<float> > dists;
        int dist_size=0,dist_size_size=0;
        Eigen::MatrixXd cost_matrix = getCloudCost(clus_pcs,KFs_,dist_size, dist_size_size,clu_dis_thres);
        // std::cout<<cost_matrix<<std::endl;
        eigenMat2VecVec(cost_matrix,dists);
        std::vector<std::vector<int> > matches;
        std::vector<int> u_track,u_cluster;
        //进行匹配
        linear_assignment(dists, dist_size, dist_size_size,clu_match_thres ,matches,u_cluster,u_track);
        std::vector<KUData> again_clus(pcs.size());
        for (auto index:u_track) {
            Kalman_filter_plus kf=KFs[index];
            if (kf.last_time>1||kf.detect_history.size()==0) continue;
            for(int i=0;i<pcs.size();i++) {
                std::array<cv::Point2f,5> AABB;
                auto min_pc=pcs[i].GetMinBound();
                auto max_pc=pcs[i].GetMaxBound();
                AABB[0]=cv::Point2f(min_pc[0],min_pc[1]);
                AABB[1]=cv::Point2f(min_pc[0],max_pc[1]);
                AABB[2]=cv::Point2f(max_pc[0],max_pc[1]);
                AABB[3]=cv::Point2f(max_pc[0],min_pc[1]);
                if (initornot(AABB,kf.predict_point,4)==1) {
                    again_clus[i].mutli_in+=1;
                    again_clus[i].I_utrack.push_back(index);
                    break;
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZ> kmeans_out;
        for(auto match : matches){
            if (again_clus[match[0]].mutli_in>0) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr new_pc(new pcl::PointCloud<pcl::PointXYZ>);
                for(auto point:pcs[match[0]].points_)
                    new_pc->points.push_back(pcl::PointXYZ(point[0],point[1],point[2]));

                int num_dimension = 3;
                int num_points = new_pc->size();
                std::vector<std::vector<float>> features(num_points, std::vector<float>(num_dimension));
                for (int i = 0; i < new_pc->points.size(); ++i){
                    features[i][0] = new_pc->points[i].x;
                    features[i][1] = new_pc->points[i].y;
                    features[i][2] = new_pc->points[i].z;
                }

                pcl::Kmeans kmean(num_points, num_dimension);
                kmean.setInputData(features);
                //两类
                kmean.setClusterSize(again_clus[match[0]].mutli_in+1);
                kmean.kMeans();

                auto center = kmean.get_centroids();

                for (auto point:center) {
                    kmeans_out.points.push_back(pcl::PointXYZ(point[0],point[1],point[2]));
                    again_clus[match[0]].kmeans_pc.points.push_back(pcl::PointXYZI(point[0],point[1],point[2],0));
                    std::cout<<"x: "<<point[0]<<"y: "<<point[1]<<"z: "<<point[2]<<std::endl;
                }
                std::vector<Kalman_filter_plus> new_KFs;
                new_KFs.push_back(KFs_[match[1]]);
                for(auto index:again_clus[match[0]].I_utrack) {
                    new_KFs.push_back(KFs_[index]);
                }

                std::vector<std::vector<float> > new_dists;
                int new_dist_size=0,new_dist_size_size=0;
                Eigen::MatrixXd new_cost_matrix = getCloudCost(again_clus[match[0]].kmeans_pc,new_KFs,new_dist_size, new_dist_size_size,clu_dis_thres);
                eigenMat2VecVec(new_cost_matrix,new_dists);
                std::vector<std::vector<int> > new_matches;
                std::vector<int> new_u_track,new_u_cluster;
                //进行匹配
                linear_assignment(new_dists, new_dist_size, new_dist_size_size,clu_match_thres ,new_matches,new_u_cluster,new_u_track);
                for(auto new_match : new_matches) {
                    if (new_match[1]==0) {
                        KFs_[match[1]].update(again_clus[match[0]].kmeans_pc.points[new_match[0]],time,rects[match[0]]);
                        if (clus_pcs[match[0]].color!=2) {
                            KFs_[match[1]].detect_history.push_back(std::pair(clus_pcs[match[0]].color,clus_pcs[match[0]].num));
                            KFs_[match[1]].detect_time.push_back(clus_pcs[match[0]].time);
                            if (KFs_[match[1]].detect_history.size()>KFs_[match[1]].max_detect_history) {
                                KFs_[match[1]].detect_history.erase(KFs_[match[1]].detect_history.begin());
                                KFs_[match[1]].detect_time.erase(KFs_[match[1]].detect_time.begin());
                            }
                        }
                    }else {
                        KFs_[again_clus[match[0]].I_utrack[new_match[1]-1]].update(again_clus[match[0]].kmeans_pc.points[new_match[0]],time,rects[match[0]]);
                        if (clus_pcs[match[0]].color!=2) {
                            KFs_[again_clus[match[0]].I_utrack[new_match[1]-1]].detect_history.push_back(std::pair(clus_pcs[match[0]].color,clus_pcs[match[0]].num));
                            KFs_[again_clus[match[0]].I_utrack[new_match[1]-1]].detect_time.push_back(clus_pcs[match[0]].time);
                            if (KFs_[again_clus[match[0]].I_utrack[new_match[1]-1]].detect_history.size()>KFs_[again_clus[match[0]].I_utrack[new_match[1]-1]].max_detect_history) {
                                KFs_[again_clus[match[0]].I_utrack[new_match[1]-1]].detect_history.erase(KFs_[again_clus[match[0]].I_utrack[new_match[1]-1]].detect_history.begin());
                                KFs_[again_clus[match[0]].I_utrack[new_match[1]-1]].detect_time.erase(KFs_[again_clus[match[0]].I_utrack[new_match[1]-1]].detect_time.begin());
                            }
                        }
                    }
                }
            }else {
                KFs_[match[1]].update(pcl::PointXYZI(clus_pcs[match[0]].center[0],clus_pcs[match[0]].center[1],clus_pcs[match[0]].center[2]),time,rects[match[0]]);
                if (clus_pcs[match[0]].color!=2) {
                    KFs_[match[1]].detect_history.push_back(std::pair(clus_pcs[match[0]].color,clus_pcs[match[0]].num));
                    KFs_[match[1]].detect_time.push_back(clus_pcs[match[0]].time);
                    if (KFs_[match[1]].detect_history.size()>KFs_[match[1]].max_detect_history) {
                        KFs_[match[1]].detect_history.erase(KFs_[match[1]].detect_history.begin());
                        KFs_[match[1]].detect_time.erase(KFs_[match[1]].detect_time.begin());
                    }
                }
            }
        }
        if (matches.size()==0) {
            for (int index=0;index<clus_pcs.size();index++) {
                Kalman_filter_plus kf(pcl::PointXYZI(clus_pcs[index].center[0],clus_pcs[index].center[1],clus_pcs[index].center[2]), time,reinterpret_cast<rclcpp::Node*>(this),rects[index]);//time为最后更新时间
                if (clus_pcs[index].color!=2) {
                    kf.detect_history.push_back(std::pair(clus_pcs[index].color,clus_pcs[index].num));
                    kf.detect_time.push_back(clus_pcs[index].time);
                    if (kf.detect_history.size()>kf.max_detect_history) {
                        kf.detect_history.erase(kf.detect_history.begin());
                        kf.detect_time.erase(kf.detect_time.begin());
                    }
                }
                KFs_.push_back(kf);
            }
        }
        for (auto cluster:u_cluster) {
            double min_dis=10000.0;
            for (auto match : matches) {
                double distance=Distance(clus_pcs[cluster].center,clus_pcs[match[0]].center);
                if (distance<min_dis) {
                    min_dis=distance;
                }
            }
            if (min_dis>clu_min_dis_thres) {
                Kalman_filter_plus kf(pcl::PointXYZI(clus_pcs[cluster].center[0],clus_pcs[cluster].center[1],clus_pcs[cluster].center[2]), time,reinterpret_cast<rclcpp::Node*>(this),rects[cluster]);//time为最后更新时间
                if (clus_pcs[cluster].color!=2) {
                    kf.detect_history.push_back(std::pair(clus_pcs[cluster].color,clus_pcs[cluster].num));
                    kf.detect_time.push_back(clus_pcs[cluster].time);
                    if (kf.detect_history.size()>kf.max_detect_history) {
                        kf.detect_history.erase(kf.detect_history.begin());
                        kf.detect_time.erase(kf.detect_time.begin());
                    }
                }
                KFs_.push_back(kf);
            }
        }

        sensor_msgs::msg::PointCloud2 output_kms;
        pcl::toROSMsg(kmeans_out, output_kms);
        output_kms.header.frame_id = "rm_frame";
        output_kms.header.stamp = msg->header.stamp;
        pub_kmeans->publish(output_kms);
        checkKFs(KFs_);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        //过滤出红蓝方有位置信息的检测点


        std::vector<Kalman_filter_plus> blue_KFs_,red_KFs_;
        for(int i = KFs_.size() - 1; i >= 0; i--){
            if(KFs_[i].last_time>2.5){
                KFs_.erase(KFs_.begin() + i);
            }
        }
        // check();
        interfaces::msg::DetectResult detect_res;
        visualization_msgs::msg::MarkerArray vis_array;

        for(int i = KFs_.size() - 1; i >= 0; i--){
            pcl::PointXYZRGB point;
            point.x = KFs_[i].predict_point.x;
            point.y = KFs_[i].predict_point.y;
            point.z = 1.5;
            int color = KFs_[i].get_color();
            switch (color){
            case 1://blue
                point.b = 255;
                // cloud_filtered->points.push_back(point);
                break;

            case 0://red
                point.r = 255;
                // cloud_filtered->points.push_back(point);
                break;

            default:
                point.r = 255;
                point.g = 255;
                point.b = 255;
                break;
            }
            cloud_filtered->points.push_back(point);
        }

        for(int i=0;i<6;i++){
            if(detect_msg.red_x[i]!=0&&detect_msg.red_y[i]!=0){
                // cloud_filtered->points.push_back(pcl::PointXYZRGB(detect_msg.red_x[i],detect_msg.red_y[i],1.5,255,241,67));
                vis_maker(2,vis_array,0,i,detect_msg.red_x[i],detect_msg.red_y[i]);
            }

            if(detect_msg.blue_x[i]!=0&&detect_msg.blue_y[i]!=0){
                // cloud_filtered->points.push_back(pcl::PointXYZRGB(detect_msg.blue_x[i],detect_msg.blue_y[i],1.5,0,176,240));
                vis_maker(2,vis_array,1,i,detect_msg.blue_x[i],detect_msg.blue_y[i]);
            }
        }
        for(int i=0;i<6;i++){
            if(dep_msg.red_x[i]!=0&&dep_msg.red_y[i]!=0){
                // cloud_filtered->points.push_back(pcl::PointXYZRGB(dep_msg.red_x[i],dep_msg.red_y[i],1.5,255,241,67));
                vis_maker(4,vis_array,0,i,dep_msg.red_x[i],dep_msg.red_y[i]);
            }

            if(dep_msg.blue_x[i]!=0&&dep_msg.blue_y[i]!=0){
                // cloud_filtered->points.push_back(pcl::PointXYZRGB(dep_msg.blue_x[i],dep_msg.blue_y[i],1.5,0,176,240));
                vis_maker(4,vis_array,1,i,dep_msg.blue_x[i],dep_msg.blue_y[i]);
            }
        }

        cloud_filtered->header.frame_id = "rm_frame";
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header.frame_id = "rm_frame";
        output.header.stamp = msg->header.stamp;
        pub_->publish(output);


        int num=0;
        std::vector<std::vector<int>> history_cost={{0,0,0,0,0,0},{0,0,0,0,0,0}};
        std::vector<std::vector<int>> history={{0,0,0,0,0,0},{0,0,0,0,0,0}};
        // std::vector<std::vector<bool>> history={{0,0,0,0,0,0},{0,0,0,0,0,0}};

        std::vector<std::vector<float>> cost_confMatrix_vec;
        int num_strack,num_cls;
        std::vector<std::vector<int> > matches_cls;
        std::vector<int> u_strack, u_cls;
        matches_cls.clear();u_strack.clear();u_cls.clear();
        Eigen::MatrixXd cost_confMatrix = getCost_confMatrix(KFs_,num_strack,num_cls);
        std::cout << "cost_confMatrix_cls:  " << std::endl << cost_confMatrix << std::endl;
        eigenMat2VecVec(cost_confMatrix, cost_confMatrix_vec);
        linear_assignment(cost_confMatrix_vec, num_strack, num_cls,0.95, matches_cls, u_strack, u_cls);

        for (auto match : matches_cls) {
            if(match[1]>=5){//蓝色
                history_cost[1][match[1]-5]++;
                vis_kal_maker(1,vis_array,1,match[1]-5,KFs_[match[0]].predict_point.x,KFs_[match[0]].predict_point.y,KFs_[match[0]].detect_history.size(),history_cost[1][match[1]-5]);
                if (KFs_[match[0]].last_time>1.5) {
                    continue;
                }
                detect_res.blue_x[match[1]-5] = KFs_[match[0]].predict_point.x;
                detect_res.blue_y[match[1]-5] = KFs_[match[0]].predict_point.y;
                if(self_color==0) {
                    detect_res.v_x[match[1]-5] = KFs_[match[0]].KF.statePost.at<float>(1);
                    detect_res.v_y[match[1]-5] = KFs_[match[0]].KF.statePost.at<float>(3);
                }
            }else{//红色
                history_cost[0][match[1]]++;
                vis_kal_maker(1,vis_array,0,match[1],KFs_[match[0]].predict_point.x,KFs_[match[0]].predict_point.y,KFs_[match[0]].detect_history.size(),history_cost[0][match[1]]);
                if (KFs_[match[0]].last_time>1.5) {
                    continue;
                }
                detect_res.red_x[match[1]] = KFs_[match[0]].predict_point.x;
                detect_res.red_y[match[1]] = KFs_[match[0]].predict_point.y;
                if(self_color==1) {
                    detect_res.v_x[match[1]] = KFs_[match[0]].KF.statePost.at<float>(1);
                    detect_res.v_y[match[1]] = KFs_[match[0]].KF.statePost.at<float>(3);
                }
            }
        }

        // //TODO:change
        //self_color==1为自己为蓝方
        if(self_color==1){
            //地图默认以红方的角点为原点，因此己方为蓝方时需要转换坐标系
            for(int i=0;i<6;i++){
                if(detect_res.blue_x[i]!=0&&detect_res.blue_y[i]!=0){
                    detect_res.blue_x[i]=28-detect_res.blue_x[i];
                    detect_res.blue_y[i]=15-detect_res.blue_y[i];
                }
                if(detect_res.red_x[i]!=0&&detect_res.red_y[i]!=0){
                    detect_res.red_x[i]=28-detect_res.red_x[i];
                    detect_res.red_y[i]=15-detect_res.red_y[i];
                    detect_res.v_x[i]=-detect_res.v_x[i];
                    detect_res.v_y[i]=-detect_res.v_y[i];
                }
            }
        }
        for(int i=0;i<6;i++){
            if(detect_res.blue_x[i]!=0&&detect_res.blue_y[i]!=0){
                vis_maker(3,vis_array,1,i,detect_res.blue_x[i],detect_res.blue_y[i]);
            }
            if(detect_res.red_x[i]!=0&&detect_res.red_y[i]!=0){
                vis_maker(3,vis_array,0,i,detect_res.red_x[i],detect_res.red_y[i]);
            }
        }
        detect_res.header.stamp = rclcpp::Clock().now();
        lidar_detect_pub_->publish(detect_res);
        pub_point_->publish(vis_array);

        mtx.lock();
        KFs= KFs_;
        mtx.unlock();

        auto end_time = std::chrono::steady_clock::now();
        float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
        RCLCPP_WARN(this->get_logger(), "Kalman Callback time is %f ms", dur_time);
    }
    */
    /**/
    void KalmanFilter::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        rclcpp::Time time = msg->header.stamp;
        std::vector<Kalman_filter_plus> KFs_;
        mtx.lock();
        KFs_= KFs;
        mtx.unlock();

        auto transform = Eigen::Affine3f::Identity();
        if(!get_lidar2world){
            try{
                transform_stamped = tf_buffer_.lookupTransform("rm_frame", "livox_frame", tf2::TimePointZero);
                get_lidar2world=true;
            }catch (tf2::TransformException &ex){
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Transform error: %s", ex.what());
                return;
            }
        }
        transform.translation() <<
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z;
        Eigen::Quaternionf rotation(
            transform_stamped.transform.rotation.w,
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z);
        transform.rotate(rotation);

        auto now_time = std::chrono::steady_clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        auto now_time1 = std::chrono::steady_clock::now();
        std::vector<open3d::geometry::PointCloud> pcs;
        getCluster(cloud,pcs);
        std::vector<cv::Rect> rects;
        getRect2d(pcs,transform,rects,1);
        auto end_time1 = std::chrono::steady_clock::now();
        float dur_time1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time1 - now_time1).count();
        RCLCPP_WARN(this->get_logger(), "cluster Callback time is %f ms", dur_time1);
        for(int i=0;i<pcs.size();i++){
            pcl::PointXYZI point_xy;
            point_xy.x = pcs[i].GetCenter()[0];
            point_xy.y = pcs[i].GetCenter()[1];
            point_xy.z = pcs[i].GetCenter()[2];
            point_xy.intensity = 0;
            cloud_xy->points.push_back(point_xy);
        }
        if(cloud_xy->points.size() == 0)return;
        sensor_msgs::msg::PointCloud2 cluster_out;
        pcl::toROSMsg(*cloud_xy, cluster_out);
        cluster_out.header.frame_id = "rm_frame";
        cluster_out.header.stamp = msg->header.stamp;
        pub_cluster->publish(cluster_out);

        // std::vector<bool> is_updated(cloud_xy->points.size(), false);
        for(auto &kf : KFs_){//一开始KFs_中没有元素，因此此处循环不会进行
                       //若有元素，则进行预测更新
            kf.update_predict_point();
            kf.has_updated = false;
            ////采用遍历所有卡尔曼找聚类的策略，效果不好，可能没有细调
            // std::vector<int> match_cls_indexs;
            // for(int i = 0;i < cloud_xy->points.size(); i++){
            //     if(kf.match(cloud_xy->points[i])){
            //         match_cls_indexs.push_back(i);
            //     }
            // }
            // if(match_cls_indexs.size() == 1){
            //     kf.update(cloud_xy->points[match_cls_indexs[0]],time);//time为加入history的时间
            //     is_updated[match_cls_indexs[0]] = true;
            //     // std::cout<<"update kf"<<std::endl;
            // }else if(match_cls_indexs.size() > 1){
            //     float min_distance = 1000000;
            //     int min_index = 0;
            //     for(auto index : match_cls_indexs){
            //         float distance = kf.Distance(kf.predict_point, cloud_xy->points[index]);
            //         if(distance < min_distance){
            //             min_distance = distance;
            //             min_index = index;
            //         }
            //     }
            //     kf.update(cloud_xy->points[min_index], time);//time为加入history的时间
            //     is_updated[min_index] = true;
            //     // std::cout<<"find kf"<<std::endl;
            // }
        }
        // for(int i=0;i<cloud_xy->points.size();i++){
        //     if(is_updated[i])continue;
        //     std::vector<int> match_kf_indexs;
        //     for(int j = 0; j < this->KFs_.size(); j++){
        //         if(KFs_[j].match(cloud_xy->points[i])){
        //             match_kf_indexs.push_back(j);
        //         }
        //     }
        //     if(match_kf_indexs.size() == 0){
        //         Kalman_filter_plus kf(cloud_xy->points[i], time,reinterpret_cast<rclcpp::Node*>(this));//time为最后更新时间
        //         KFs_.push_back(kf);
        //         // std::cout<<"new kf"<<std::endl;
        //     }
        // }
        checkKFs(KFs_);
        // //添加对多个聚类匹配到同一个卡尔曼对象时的处理，但会导致飞的卡尔曼增多
        // std::unordered_map<int,std::vector<int>> target_list;
        // std::vector<bool> is_updated(KFs_.size(),false);
        // for(int i=0;i<cloud_xy->points.size();i++){
        //     std::vector<int> match_kf_indexs;
        //     for(int j = 0; j < this->KFs_.size(); j++){
        //         if(KFs_[j].match(cloud_xy->points[i])){
        //             match_kf_indexs.push_back(j);
        //         }
        //     }
        //     if(match_kf_indexs.size() == 0){
        //         Kalman_filter_plus kf(cloud_xy->points[i], time,reinterpret_cast<rclcpp::Node*>(this));//time为最后更新时间
        //         KFs_.push_back(kf);
        //     }
        //     else if(match_kf_indexs.size() == 1){
        //         target_list[match_kf_indexs[0]].push_back(i);
        //         is_updated[match_kf_indexs[0]] = true;
        //     }else{
        //         float min_distance = 1000000;
        //         int min_index = 0;
        //         for(auto index : match_kf_indexs){
        //             float distance = KFs_[index].Distance(KFs_[index].predict_point, cloud_xy->points[i]);
        //             if(distance < min_distance){
        //                 min_distance = distance;
        //                 min_index = index;
        //             }
        //         }
        //         target_list[min_index].push_back(i);
        //         is_updated[min_index] = true;
        //     }
        // }
        // for (auto it = target_list.begin(); it != target_list.end(); ++it) {
        //     if(it->second.size()==1){
        //         KFs_[it->first].update(cloud_xy->points[it->second[0]],time);
        //     }else{
        //         int min_index=-1;
        //         int min_dis=10000;
        //         for(auto index : target_list[it->first]){
        //             double dis=KFs_[it->first].Distance(KFs_[it->first].predict_point,cloud_xy->points[index]);
        //             if(dis<min_dis){
        //                 min_dis=dis;
        //                 min_index=index;
        //             }
        //         }
        //         if(KFs_[it->first].history.size()<7){
        //             KFs_[it->first].history.clear();
        //         }else{
        //             for(auto index:target_list[it->first]){
        //                 if(index==min_index) continue;
        //
        //                 std::vector<int> match_kf_indexs;
        //                 for(int j=0;j<is_updated.size();j++){
        //                     if(is_updated[j]) continue;
        //                     if(KFs_[j].loose_match(cloud_xy->points[index])){
        //                         match_kf_indexs.push_back(j);
        //                     }
        //                 }
        //                 if(match_kf_indexs.size() == 0){
        //                     Kalman_filter_plus kf(cloud_xy->points[index],time,reinterpret_cast<rclcpp::Node*>(this));//time为最后更新时间
        //                     KFs_.push_back(kf);
        //                 }
        //                 else if(match_kf_indexs.size() == 1){
        //                     KFs_[match_kf_indexs[0]].update(cloud_xy->points[index], time);//time为加入history的时间
        //                 }else{
        //                     float min_distance = 1000000;
        //                     int min_index = 0;
        //                     for(auto i:match_kf_indexs){
        //                         float distance = KFs_[i].Distance(KFs_[i].predict_point,cloud_xy->points[index]);
        //                         if(distance < min_distance){
        //                             min_distance = distance;
        //                             min_index = i;
        //                         }
        //                     }
        //                     KFs_[min_index].update(cloud_xy->points[index], time);//time为加入history的时间
        //                 }
        //             }
        //         }
        //         KFs_[it->first].update(cloud_xy->points[min_index],time);
        //     }
        // }
        double clu_match_thres=get_parameter("clu.match_thres").as_double();
        double clu_dis_thres=get_parameter("clu.dis_thres").as_double();
        double clu_min_dis_thres=get_parameter("clu.min_dis_thres").as_double();

        std::vector<std::vector<float> > dists;
        int dist_size=0,dist_size_size=0;
        Eigen::MatrixXd cost_matrix = getCloudCost(*cloud_xy,KFs_,dist_size, dist_size_size,clu_dis_thres);
        // std::cout<<cost_matrix<<std::endl;
        eigenMat2VecVec(cost_matrix,dists);
        std::vector<std::vector<int> > matches;
        std::vector<int> u_track,u_cluster;
        //进行匹配
        linear_assignment(dists, dist_size, dist_size_size,clu_match_thres ,matches,u_cluster,u_track);
        std::vector<KUData> again_clus(pcs.size());
        for (auto index:u_track) {
            Kalman_filter_plus kf=KFs[index];
            if (kf.last_time>1) continue;
            for(int i=0;i<pcs.size();i++) {
                std::array<cv::Point2f,5> AABB;
                auto min_pc=pcs[i].GetMinBound();
                auto max_pc=pcs[i].GetMaxBound();
                AABB[0]=cv::Point2f(min_pc[0]-0.1,min_pc[1]-0.1);
                AABB[1]=cv::Point2f(min_pc[0]-0.1,max_pc[1]+0.1);
                AABB[2]=cv::Point2f(max_pc[0]+0.1,max_pc[1]+0.1);
                AABB[3]=cv::Point2f(max_pc[0]+0.1,min_pc[1]-0.1);
                if (initornot(AABB,kf.predict_point,4)==1) {
                    again_clus[i].mutli_in+=1;
                    again_clus[i].utrack_idx.push_back(index);
                    break;
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZ> kmeans_out;
        for(auto match : matches){
            if (again_clus[match[0]].mutli_in>0) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr new_pc(new pcl::PointCloud<pcl::PointXYZ>);
                for(auto point:pcs[match[0]].points_)
                    new_pc->points.push_back(pcl::PointXYZ(point[0],point[1],point[2]));

                int num_dimension = 3;
                int num_points = new_pc->size();
                std::vector<std::vector<float>> features(num_points, std::vector<float>(num_dimension));
                for (int i = 0; i < new_pc->points.size(); ++i){
                    features[i][0] = new_pc->points[i].x;
                    features[i][1] = new_pc->points[i].y;
                    features[i][2] = new_pc->points[i].z;
                }

                pcl::Kmeans kmean(num_points, num_dimension);
                kmean.setInputData(features);
                //两类
                kmean.setClusterSize(again_clus[match[0]].mutli_in+1);
                kmean.kMeans();

                auto center = kmean.get_centroids();

                for (auto point:center) {
                    kmeans_out.points.push_back(pcl::PointXYZ(point[0],point[1],point[2]));
                    again_clus[match[0]].kmeans_pc.points.push_back(pcl::PointXYZI(point[0],point[1],point[2],0));
                    std::cout<<"x: "<<point[0]<<"y: "<<point[1]<<"z: "<<point[2]<<std::endl;
                }
                std::vector<Kalman_filter_plus> new_KFs;
                new_KFs.push_back(KFs_[match[1]]);
                for(auto index:again_clus[match[0]].utrack_idx) {
                    new_KFs.push_back(KFs_[index]);
                }

                std::vector<std::vector<float> > new_dists;
                int new_dist_size=0,new_dist_size_size=0;
                Eigen::MatrixXd new_cost_matrix = getCloudCost(again_clus[match[0]].kmeans_pc,new_KFs,new_dist_size, new_dist_size_size,clu_dis_thres);
                eigenMat2VecVec(new_cost_matrix,new_dists);
                std::vector<std::vector<int> > new_matches;
                std::vector<int> new_u_track,new_u_cluster;
                //进行匹配
                linear_assignment(new_dists, new_dist_size, new_dist_size_size,clu_match_thres ,new_matches,new_u_cluster,new_u_track);
                for(auto new_match : new_matches) {
                    if (new_match[1]==0) {
                        KFs_[match[1]].update(again_clus[match[0]].kmeans_pc.points[new_match[0]],time,rects[match[0]]);
                    }else {
                        KFs_[again_clus[match[0]].utrack_idx[new_match[1]-1]].update(again_clus[match[0]].kmeans_pc.points[new_match[0]],time,rects[match[0]]);
                    }
                }
            }else {
                KFs_[match[1]].update(cloud_xy->points[match[0]],time,rects[match[0]]);
            }
        }
        if (matches.size()==0) {
            for (int index=0;index<cloud_xy->size();index++) {
                Kalman_filter_plus kf(cloud_xy->points[index], time,reinterpret_cast<rclcpp::Node*>(this),rects[index]);//time为最后更新时间
                KFs_.push_back(kf);
            }
        }
        for (auto cluster:u_cluster) {
            double min_dis=10000.0;
            for (auto match : matches) {
                double distance=Distance(cloud_xy->points[cluster],cloud_xy->points[match[0]]);
                if (distance<min_dis) {
                    min_dis=distance;
                }
            }
            if (min_dis>clu_min_dis_thres) {
                Kalman_filter_plus kf(cloud_xy->points[cluster], time,reinterpret_cast<rclcpp::Node*>(this),rects[cluster]);//time为最后更新时间
                KFs_.push_back(kf);
            }
        }

        sensor_msgs::msg::PointCloud2 output_kms;
        pcl::toROSMsg(kmeans_out, output_kms);
        output_kms.header.frame_id = "rm_frame";
        output_kms.header.stamp = msg->header.stamp;
        pub_kmeans->publish(output_kms);
        //默认卡尔曼匹配策略，可能出现一辆车两个聚类导致卡尔曼飘飞，两辆车距离过近导致只有一个卡尔曼，但卡尔曼飘飞的情况减少
        // for(auto point : cloud_xy->points){//对于每个点
        // //如果遍历所有卡尔曼都没找到能够匹配的，新建一个卡尔曼
        // //若找到了1个，则更新这个卡尔曼
        // //若找到了多个，则更新距离最近的那个
        //     std::vector<int> match_kf_indexs;
        //     for(int i = 0; i < this->KFs_.size(); i++){
        //         if(KFs_[i].match(point)){
        //             match_kf_indexs.push_back(i);
        //         }
        //     }
        //     if(match_kf_indexs.size() == 0){
        //         Kalman_filter_plus kf(point, time,reinterpret_cast<rclcpp::Node*>(this));//time为最后更新时间
        //         KFs_.push_back(kf);
        //         // std::cout<<"new kf"<<std::endl;
        //     }
        //     else if(match_kf_indexs.size() == 1){
        //         KFs_[match_kf_indexs[0]].update(point, time);//time为加入history的时间
        //         // std::cout<<"update kf"<<std::endl;
        //     }else{
        //         float min_distance = 1000000;
        //         int min_index = 0;
        //         for(auto index : match_kf_indexs){
        //             float distance = KFs_[index].Distance(KFs_[index].predict_point, point);
        //             if(distance < min_distance){
        //                 min_distance = distance;
        //                 min_index = index;
        //             }
        //         }
        //         KFs_[min_index].update(point, time);//time为加入history的时间
        //         // std::cout<<"find kf"<<std::endl;
        //     }
        // }
        checkKFs(KFs_);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        //过滤出红蓝方有位置信息的检测点
        //TODO：需要对己方为蓝方的时候作处理

        // std::vector<std::array<int, 4>> blue_det,red_det;//x,y,颜色，编号
        // for(int i=0;i<6;i++){
        //     if(detect_msg.red_x[i]!=0&&detect_msg.red_y[i]!=0) {
        //         if (self_color==1)
        //             red_det.push_back({28-detect_msg.red_x[i],15-detect_msg.red_y[i],0,i});
        //         else
        //             red_det.push_back({detect_msg.red_x[i],detect_msg.red_y[i],0,i});
        //     }
        //     if(detect_msg.blue_x[i]!=0&&detect_msg.blue_y[i]!=0) {
        //         if (self_color==1)
        //             blue_det.push_back({28-detect_msg.blue_x[i],15-detect_msg.blue_y[i],1,i});
        //         else
        //             blue_det.push_back({detect_msg.blue_x[i],detect_msg.blue_y[i],1,i});
        //     }
        // }
        std::vector<Kalman_filter_plus> blue_KFs_,red_KFs_;
        // for(int i = KFs_.size() - 1; i >= 0; i--){
        //     if (KFs_[i].last_time>0.7&&KFs_[i].last_time<=2.5) {
        //         //进入特殊位置
        //         if(KFs_[i].predict_point.x>=25.1&&KFs_[i].predict_point.y>=11.1){
        //             int car_num=KFs_[i].get_number();
        //             if(car_num==1){
        //                 KFs_[i].predict_point.x=26.9;
        //                 KFs_[i].predict_point.y=11.6;
        //                 RCLCPP_ERROR(this->get_logger(),"enter mill!!!"); //工程兑矿区
        //             }else if (car_num!=-1){
        //                 KFs_[i].predict_point.x=26.31;
        //                 KFs_[i].predict_point.y=14.05;
        //                 RCLCPP_ERROR(this->get_logger(),"enter supply!!!"); //其他车补给点
        //             }
        //         }else if(KFs_[i].predict_point.x<=20.18&&KFs_[i].predict_point.x>=18.96&&KFs_[i].predict_point.y>=12.46&&KFs_[i].predict_point.y<=13.71){
        //             int car_num=KFs_[i].get_number();
        //             if(car_num==2||car_num==3||car_num==4){
        //                 KFs_[i].predict_point.x=19.58;
        //                 KFs_[i].predict_point.y=13.15;
        //                 RCLCPP_ERROR(this->get_logger(),"enter energy!!!"); //打符点
        //             }
        //         }
        //     }else if(KFs_[i].last_time>2.5&&(KFs_[i].last_time)<10){
        //         int car_num=KFs_[i].get_number();
        //         if (car_num==-1||KFs_[i].predict_point.x<=25.1||KFs_[i].predict_point.y<=11.1)
        //             KFs_.erase(KFs_.begin() + i);
        //
        //     }else if(KFs_[i].last_time>=10){
        //         if(KFs_[i].get_number()!=1){//不是工程的车辆卡尔曼删除
        //             KFs_.erase(KFs_.begin() + i);
        //         }
        //     }else if(KFs_[i].last_time<=1){
        //         int color=KFs_[i].get_color();
        //         if(color==0){
        //             red_KFs_.push_back(KFs_[i]);
        //         }else if(color==1){
        //             blue_KFs_.push_back(KFs_[i]);
        //         }
        //     }
        // }
        std::vector<int> remove_KFs_;
        for(int i = KFs_.size() - 1; i >= 0; i--){
            KFs_[i].forword_predict();
            KFs_[i].forword_predict();
            KFs_[i].forword_predict();
            if(KFs_[i].last_time>2.5){
                remove_KFs_.push_back(i);
            }
        }
        sort(remove_KFs_.begin(),remove_KFs_.end(),std::greater<>());
        for (auto index:remove_KFs_)
            KFs_.erase(KFs_.begin() + index);
        // check();
        interfaces::msg::DetectResult detect_res;
        //
        // std::vector<std::vector<float> > b_dists,r_dists;
        // int b_dist_size=0,b_dist_size_size=0,r_dist_size=0,r_dist_size_size=0;
        //
        // double distance_weight=get_parameter("cost.distance_weight").as_double();
        // double color_weight=get_parameter("cost.color_weight").as_double();
        // double distance_thres=get_parameter("cost.distance_thres").as_double();
        // match_thresh=get_parameter("fusion.match_thresh").as_double();
        //
        // //得到代价矩阵
        // Eigen::MatrixXd b_cost_matrix = getFusionCost(blue_KFs_,blue_det,b_dist_size, b_dist_size_size,
        //     distance_weight,distance_thres,color_weight);
        // Eigen::MatrixXd r_cost_matrix = getFusionCost(red_KFs_,red_det,r_dist_size, r_dist_size_size,
        //     distance_weight,distance_thres,color_weight);
        // // std::cout<<cost_matrix<<std::endl;
        // //转换代价矩阵的形式
        // eigenMat2VecVec(b_cost_matrix,b_dists);
        // eigenMat2VecVec(r_cost_matrix,r_dists);
        //
        // std::vector<std::vector<int> > b_matches,r_matches;
        // std::vector<int> b_u_track,b_u_detection,r_u_track,r_u_detection;
        // //进行匹配
        // linear_assignment(b_dists, b_dist_size, b_dist_size_size,match_thresh,b_matches,b_u_track,b_u_detection);
        // linear_assignment(r_dists, r_dist_size, r_dist_size_size,match_thresh,r_matches,r_u_track,r_u_detection);


        // std::cout<<matches.size()<<std::endl;
        // std::cout<<u_track.size()<<std::endl;
        visualization_msgs::msg::MarkerArray vis_array;
        // for (int i = 0; i < b_matches.size(); i++){
        //     Kalman_filter_plus kf = blue_KFs_[b_matches[i][0]];
        //     std::array<int, 4> det=blue_det[b_matches[i][1]];
        //     // detect_res.blue_x[det[3]] = kf.predict_point.x;
        //     // detect_res.blue_y[det[3]] = kf.predict_point.y;
        //
        //     vis_maker(0,vis_array,det[2],det[3],kf.predict_point.x,kf.predict_point.y);
        // }
        //
        // for (int i = 0; i < r_matches.size(); i++){
        //     Kalman_filter_plus kf = red_KFs_[r_matches[i][0]];
        //     std::array<int, 4> det=red_det[r_matches[i][1]];
        //     // detect_res.red_x[det[3]] = kf.predict_point.x;
        //     // detect_res.red_y[det[3]] = kf.predict_point.y;
        //
        //     vis_maker(0,vis_array,det[2],det[3],kf.predict_point.x,kf.predict_point.y);
        // }


        for(int i = KFs_.size() - 1; i >= 0; i--){
            pcl::PointXYZRGB point;
            point.x = KFs_[i].predict_point.x;
            point.y = KFs_[i].predict_point.y;
            point.z = 1.5;
            int color = KFs_[i].get_color();
            switch (color){
            case 1://blue
                point.b = 255;
                // cloud_filtered->points.push_back(point);
                break;

            case 0://red
                point.r = 255;
                // cloud_filtered->points.push_back(point);
                break;
            
            default:
                point.r = 255;
                point.g = 255;
                point.b = 255;
                break;
            }
            cloud_filtered->points.push_back(point);
        }

        for(int i=0;i<6;i++){
            if(detect_msg.red_x[i]!=0&&detect_msg.red_y[i]!=0){
                // cloud_filtered->points.push_back(pcl::PointXYZRGB(detect_msg.red_x[i],detect_msg.red_y[i],1.5,255,241,67));
                vis_maker(2,vis_array,0,i,detect_msg.red_x[i],detect_msg.red_y[i]);
            }

            if(detect_msg.blue_x[i]!=0&&detect_msg.blue_y[i]!=0){
                // cloud_filtered->points.push_back(pcl::PointXYZRGB(detect_msg.blue_x[i],detect_msg.blue_y[i],1.5,0,176,240));
                vis_maker(2,vis_array,1,i,detect_msg.blue_x[i],detect_msg.blue_y[i]);
            }
        }

        cloud_filtered->header.frame_id = "rm_frame";
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header.frame_id = "rm_frame";
        output.header.stamp = msg->header.stamp;
        pub_kalman->publish(output);


        int num=0;
        std::vector<std::vector<int>> history_cost={{0,0,0,0,0,0},{0,0,0,0,0,0}};
        std::vector<std::vector<int>> history={{0,0,0,0,0,0},{0,0,0,0,0,0}};
        for(int i=0;i<KFs_.size();i++){
            if(KFs_[i].detect_history.size()==0)continue;
            //此处还需要优化，避免出现多个卡尔曼对象对应到同一个红蓝点
            int i_color=KFs_[i].get_color(),i_number=KFs_[i].get_number(),max_index=i;
            // std::cout<<"---"<<i_color<<" "<<i_number<<"---"<<std::endl;
            auto freq_time=KFs_[i].get_freq(i_color,i_number);
            int max_freq=freq_time.first,time_index=i;
            double max_time=freq_time.second;

            std::vector<int> same_lists;
            same_lists.push_back(i);
            if (history[i_color][i_number]>0)continue;

            for(int j=0;j<KFs_.size();j++) {
                int j_color=KFs_[j].get_color(),j_number=KFs_[j].get_number();
                auto j_freq_time=KFs_[j].get_freq(j_color,j_number);
                int j_freq=j_freq_time.first;
                double j_time=j_freq_time.second;
                if (i_color==j_color&&i_number==j_number) {
                    if (j_freq>max_freq) {
                        max_freq=j_freq;
                        max_index=j;
                    }
                    if (j_time>max_time) {
                        max_time=j_time;
                        time_index=j;
                    }
                    same_lists.push_back(j);
                }
            }

            if (same_lists.size()>1) {
                for (auto index:same_lists) {
                    std::set<int,std::greater<>> remove_detect;
                    double min_time=max_time;
                    int min_index=-1;
                    if (index!=time_index) {
                        for (int i=0;i<KFs_[index].detect_history.size();i++) {
                            if (KFs_[index].detect_history[i].first==i_color&&KFs_[index].detect_history[i].second==i_number) {
                                if (KFs_[index].detect_time[i]<min_time) {
                                    min_time=KFs_[index].detect_time[i];
                                    min_index=i;
                                }
                            }
                        }
                        if (min_index!=-1)
                            remove_detect.insert(min_index);
                    }
                    for (auto idx:remove_detect) {
                        KFs_[index].detect_history.erase(KFs_[index].detect_history.begin()+idx);
                        KFs_[index].detect_time.erase(KFs_[index].detect_time.begin()+idx);
                    }
                }
            }
        //
        //     if(i_color == 1){//蓝色
        //         history[1][i_number]++;
        //         vis_kal_maker(1,vis_array,1,i_number,KFs_[max_index].predict_point.x,KFs_[max_index].predict_point.y,KFs_[max_index].detect_history.size(),history[1][i_number]);
        //         if (KFs_[max_index].last_time>1.5) {
        //             if (detect_msg.blue_x[i_number]!=0&&detect_msg.blue_y[i_number]!=0) {
        //                 // detect_res.blue_x[i_number] = detect_msg.blue_x[i_number];
        //                 // detect_res.blue_y[i_number] = detect_msg.blue_y[i_number];
        //
        //             }
        //             continue;
        //         }
        //         detect_res.blue_x[i_number] = KFs_[max_index].predict_point.x;
        //         detect_res.blue_y[i_number] = KFs_[max_index].predict_point.y;
        //         if(self_color==0) {
        //             detect_res.v_x[i_number] = KFs_[max_index].KF.statePost.at<float>(1);
        //             detect_res.v_y[i_number] = KFs_[max_index].KF.statePost.at<float>(3);
        //         }
        //         // std::cout<<"kalman index: "<<num<<" detection : "<<1<<" "<<number<<std::endl;
        //     }else if(i_color == 0){//红色
        //         history[0][i_number]++;
        //         vis_kal_maker(1,vis_array,0,i_number,KFs_[max_index].predict_point.x,KFs_[max_index].predict_point.y,KFs_[max_index].detect_history.size(),history[0][i_number]);
        //         if (KFs_[max_index].last_time>1.5) {
        //             if (detect_msg.red_x[i_number]!=0&&detect_msg.red_y[i_number]!=0) {
        //                 // detect_res.red_x[i_number] = detect_msg.red_x[i_number];
        //                 // detect_res.red_y[i_number] = detect_msg.red_y[i_number];
        //
        //             }
        //             continue;
        //         }
        //
        //         detect_res.red_x[i_number] = KFs_[max_index].predict_point.x;
        //         detect_res.red_y[i_number] = KFs_[max_index].predict_point.y;
        //         if(self_color==1) {
        //             detect_res.v_x[i_number] = KFs_[max_index].KF.statePost.at<float>(1);
        //             detect_res.v_y[i_number] = KFs_[max_index].KF.statePost.at<float>(3);
        //         }
        //         // std::cout<<"kalman index: "<<num<<" detection : "<<0<<" "<<number<<std::endl;
        //     }
        //     num++;
        }

        std::vector<std::vector<float>> cost_confMatrix_vec;
        int num_strack,num_cls;
        std::vector<std::vector<int> > matches_cls;
        std::vector<int> u_strack, u_cls;
        matches_cls.clear();u_strack.clear();u_cls.clear();
        Eigen::MatrixXd cost_confMatrix = getCost_confMatrix(KFs_,num_strack,num_cls);
        // std::cout << "cost_confMatrix_cls:  " << std::endl << cost_confMatrix << std::endl;
        eigenMat2VecVec(cost_confMatrix, cost_confMatrix_vec);
        linear_assignment(cost_confMatrix_vec, num_strack, num_cls,0.95, matches_cls, u_strack, u_cls);

        for (auto match : matches_cls) {
            if(match[1]>=5){//蓝色
                history_cost[1][match[1]-5]++;
                vis_kal_maker(1,vis_array,1,match[1]-5,KFs_[match[0]].predict_point.x,KFs_[match[0]].predict_point.y,KFs_[match[0]].detect_history.size(),history_cost[1][match[1]-5]);
                if (KFs_[match[0]].last_time>1.5) {
                    continue;
                }
                detect_res.blue_x[match[1]-5] = KFs_[match[0]].predict_point.x;
                detect_res.blue_y[match[1]-5] = KFs_[match[0]].predict_point.y;
                if(self_color==0) {
                    detect_res.v_x[match[1]-5] = KFs_[match[0]].KF.statePost.at<float>(1);
                    detect_res.v_y[match[1]-5] = KFs_[match[0]].KF.statePost.at<float>(3);
                }
            }else{//红色
                history_cost[0][match[1]]++;
                vis_kal_maker(1,vis_array,0,match[1],KFs_[match[0]].predict_point.x,KFs_[match[0]].predict_point.y,KFs_[match[0]].detect_history.size(),history_cost[0][match[1]]);
                if (KFs_[match[0]].last_time>1.5) {
                    continue;
                }
                detect_res.red_x[match[1]] = KFs_[match[0]].predict_point.x;
                detect_res.red_y[match[1]] = KFs_[match[0]].predict_point.y;
                if(self_color==1) {
                    detect_res.v_x[match[1]] = KFs_[match[0]].KF.statePost.at<float>(1);
                    detect_res.v_y[match[1]] = KFs_[match[0]].KF.statePost.at<float>(3);
                }
            }
        }

        // //TODO:change
        // for(int i=0;i<KFs_.size();i++){
        //     if(KFs_[i].detect_history.size()==0)continue;
        //     //此处还需要优化，避免出现多个卡尔曼对象对应到同一个红蓝点
        //     int i_color=KFs_[i].get_color(),i_number=KFs_[i].get_number(),max_index=i;
        //     // std::cout<<"---"<<i_color<<" "<<i_number<<"---"<<std::endl;
        //     auto freq_time=KFs_[i].get_freq(i_color,i_number);
        //     int max_freq=freq_time.first,time_index=i;
        //     double max_time=freq_time.second;
        //
        //     std::vector<int> same_lists;
        //     same_lists.push_back(i);
        //     if (history[i_color][i_number]>0)continue;
        //
        //     for(int j=0;j<KFs_.size();j++) {
        //         int j_color=KFs_[j].get_color(),j_number=KFs_[j].get_number();
        //         auto j_freq_time=KFs_[j].get_freq(j_color,j_number);
        //         int j_freq=j_freq_time.first;
        //         double j_time=j_freq_time.second;
        //         if (i_color==j_color&&i_number==j_number) {
        //             if (j_freq>max_freq) {
        //                 max_freq=j_freq;
        //                 max_index=j;
        //             }
        //             if (j_time>max_time) {
        //                 max_time=j_time;
        //                 time_index=j;
        //             }
        //             same_lists.push_back(j);
        //         }
        //     }
        //
        //     if (same_lists.size()>1) {
        //         for (auto index:same_lists) {
        //             std::set<int> remove_detect;
        //             double min_time=max_time;
        //             int min_index=-1;
        //             if (index!=time_index) {
        //                 for (int i=0;i<KFs_[index].detect_history.size();i++) {
        //                     if (KFs_[index].detect_history[i].first==i_color&&KFs_[index].detect_history[i].second==i_number) {
        //                         if (KFs_[index].detect_time[i]<min_time) {
        //                             min_time=KFs_[index].detect_time[i];
        //                             min_index=i;
        //                         }
        //                     }
        //                 }
        //                 if (min_index!=-1)
        //                     remove_detect.insert(min_index);
        //             }
        //             for (auto idx:remove_detect) {
        //                 KFs_[index].detect_history.erase(KFs_[index].detect_history.begin()+idx);
        //                 KFs_[index].detect_time.erase(KFs_[index].detect_time.begin()+idx);
        //             }
        //         }
        //     }
        //
        //     if(i_color == 1){//蓝色
        //         history[1][i_number]++;
        //         vis_kal_maker(1,vis_array,1,i_number,KFs_[max_index].predict_point.x,KFs_[max_index].predict_point.y,KFs_[max_index].detect_history.size(),history[1][i_number]);
        //         if (KFs_[max_index].last_time>1.5) {
        //             if (detect_msg.blue_x[i_number]!=0&&detect_msg.blue_y[i_number]!=0) {
        //                 // detect_res.blue_x[i_number] = detect_msg.blue_x[i_number];
        //                 // detect_res.blue_y[i_number] = detect_msg.blue_y[i_number];
        //
        //             }
        //             continue;
        //         }
        //         detect_res.blue_x[i_number] = KFs_[max_index].predict_point.x;
        //         detect_res.blue_y[i_number] = KFs_[max_index].predict_point.y;
        //         if(self_color==0) {
        //             detect_res.v_x[i_number] = KFs_[max_index].KF.statePost.at<float>(1);
        //             detect_res.v_y[i_number] = KFs_[max_index].KF.statePost.at<float>(3);
        //         }
        //         // std::cout<<"kalman index: "<<num<<" detection : "<<1<<" "<<number<<std::endl;
        //     }else if(i_color == 0){//红色
        //         history[0][i_number]++;
        //         vis_kal_maker(1,vis_array,0,i_number,KFs_[max_index].predict_point.x,KFs_[max_index].predict_point.y,KFs_[max_index].detect_history.size(),history[0][i_number]);
        //         if (KFs_[max_index].last_time>1.5) {
        //             if (detect_msg.red_x[i_number]!=0&&detect_msg.red_y[i_number]!=0) {
        //                 // detect_res.red_x[i_number] = detect_msg.red_x[i_number];
        //                 // detect_res.red_y[i_number] = detect_msg.red_y[i_number];
        //
        //             }
        //             continue;
        //         }
        //
        //         detect_res.red_x[i_number] = KFs_[max_index].predict_point.x;
        //         detect_res.red_y[i_number] = KFs_[max_index].predict_point.y;
        //         if(self_color==1) {
        //             detect_res.v_x[i_number] = KFs_[max_index].KF.statePost.at<float>(1);
        //             detect_res.v_y[i_number] = KFs_[max_index].KF.statePost.at<float>(3);
        //         }
        //         // std::cout<<"kalman index: "<<num<<" detection : "<<0<<" "<<number<<std::endl;
        //     }
        //     num++;
        // }

        //self_color==1为自己为蓝方
        if(self_color==1){
            //地图默认以红方的角点为原点，因此己方为蓝方时需要转换坐标系
            for(int i=0;i<6;i++){
                if(detect_res.blue_x[i]!=0&&detect_res.blue_y[i]!=0){
                    detect_res.blue_x[i]=28-detect_res.blue_x[i];
                    detect_res.blue_y[i]=15-detect_res.blue_y[i];
                }else{
                    // detect_res.blue_x[i]=detect_msg.blue_x[i];
                    // detect_res.blue_y[i]=detect_msg.blue_y[i];
                }
                if(detect_res.red_x[i]!=0&&detect_res.red_y[i]!=0){
                    detect_res.red_x[i]=28-detect_res.red_x[i];
                    detect_res.red_y[i]=15-detect_res.red_y[i];
                    detect_res.v_x[i]=-detect_res.v_x[i];
                    detect_res.v_y[i]=-detect_res.v_y[i];
                }else{
                    // detect_res.red_x[i]=detect_msg.red_x[i];
                    // detect_res.red_y[i]=detect_msg.red_y[i];
                }
            }
        }else{
            for(int i=0;i<6;i++){
                if(detect_res.blue_x[i]==0||detect_res.blue_y[i]==0){
                    // detect_res.blue_x[i]=detect_msg.blue_x[i];
                    // detect_res.blue_y[i]=detect_msg.blue_y[i];
                }
                if(detect_res.red_x[i]==0||detect_res.red_y[i]==0){
                    // detect_res.red_x[i]=detect_msg.red_x[i];
                    // detect_res.red_y[i]=detect_msg.red_y[i];
                }
            }
        }
        for(int i=0;i<6;i++){
            if(detect_res.blue_x[i]!=0&&detect_res.blue_y[i]!=0){
                vis_maker(3,vis_array,1,i,detect_res.blue_x[i],detect_res.blue_y[i]);
            }
            if(detect_res.red_x[i]!=0&&detect_res.red_y[i]!=0){
                vis_maker(3,vis_array,0,i,detect_res.red_x[i],detect_res.red_y[i]);
            }
        }
        detect_res.header.stamp = rclcpp::Clock().now();
        lidar_detect_pub->publish(detect_res);
        pub_vis->publish(vis_array);

        mtx.lock();
        KFs= KFs_;
        mtx.unlock();

        auto end_time = std::chrono::steady_clock::now();
        float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
        RCLCPP_WARN(this->get_logger(), "Kalman Callback time is %f ms", dur_time);
    }
    /**/
}
RCLCPP_COMPONENTS_REGISTER_NODE(upc_radar::KalmanFilter)