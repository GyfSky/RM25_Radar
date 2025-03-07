#include "kalman_filter.h"

namespace upc_radar{
    KalmanFilter::KalmanFilter(const rclcpp::NodeOptions& node_options):rclcpp::Node("kalman_filter_node",node_options),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_){
        declare_parameter<double>("kalman.detect_r",1.7);
        declare_parameter<double>("kalman.car_max_speed",2.0);

        declare_parameter<double>("kalman.dt_",0.1);
        declare_parameter<double>("kalman.sigma_q_x",50.0);
        declare_parameter<double>("kalman.sigma_q_y",50.0);
        declare_parameter<double>("kalman.sigma_r_x",0.1);
        declare_parameter<double>("kalman.sigma_r_y",0.1);

        declare_parameter<double>("cost.distance_weight",0.4);
        declare_parameter<double>("cost.distance_thres",0.6);
        declare_parameter<double>("cost.color_weight",0.6);
        declare_parameter<double>("fusion.match_thresh",0.45);
        declare_parameter<double>("clu.match_thres",0.85);
        declare_parameter<double>("clu.dis_thres",2.2);
        declare_parameter<double>("clu.min_dis_thres",0.4);

        declare_parameter<double>("cam.match_thres",0.8);
        declare_parameter<double>("cam.dis_thres",1.7);
        declare_parameter<double>("cam.dis_wight",0.95);
        declare_parameter<double>("cam.his_wight",0.05);
        declare_parameter<double>("cam.time_offset",1.2);
        declare_parameter<int>("classWithoutCar",12);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", 10, std::bind(&KalmanFilter::callback, this, std::placeholders::_1));
        sub_dep_ = this->create_subscription<interfaces::msg::DetectResult>("fusion_result", 100, std::bind(&KalmanFilter::dep_callback, this, std::placeholders::_1));
        sub_detect_= this->create_subscription<interfaces::msg::DetectFrame>("/resolve_result", rclcpp::SensorDataQoS(), std::bind(&KalmanFilter::detect_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_kalman", 10);
        pub_point_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vis_point", 10);
        lidar_detect_pub_ = this->create_publisher<interfaces::msg::DetectResult>("/lidar_detect", 10);


        RCLCPP_WARN(this->get_logger(), "Kalman_filter_Node has been started.");

        //-------------------------------------------//
        net_pub_=this->create_publisher<sensor_msgs::msg::PointCloud2>("/net_3D", 10);
        ori_pub_=this->create_publisher<sensor_msgs::msg::PointCloud2>("/ori_3D", 10);
        test_pub_= this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_3D", 10);
        cv::Mat image=cv::imread("/home/thesky/RM25_Radar/resource/2.png");
        show_img.create(image.rows, image.cols, CV_8UC3);
        show_img=image.clone();
        // cv::namedWindow("depth",0);
        // cv::resizeWindow("depth",640,480);
        camera_matrix1=cv::Matx33d(3574.7899975395749,0,2035.5162216399472,0,3570.1522168951046,1512.3941063924631,0,0,1);
        lidar2cam1=cv::Matx44d(0.00728,-0.01686,0.99983,0.06922,-0.99978,0.01956,0.00761,0.04613,-0.01968,-0.99967,-0.01672,0.00160,0,0,0,1);
        lidar2cam1=lidar2cam1.inv();
    }

    void KalmanFilter::dep_callback(const interfaces::msg::DetectResult::SharedPtr msg) {
        dep_msg=*msg;
    }

    cv::Point3d KalmanFilter::lidarToCamera(pcl::PointXYZ lidar_point,cv::Matx33d cam_matrix,cv::Matx44d lidar2cam){
        cv::Matx41d lidar_coor{lidar_point.x, lidar_point.y, lidar_point.z, 1.0f};
        cv::Matx31d camera_coor =cam_matrix*(lidar2cam*lidar_coor).get_minor<3, 1>(0, 0);
        double u = camera_coor(0) / camera_coor(2);
        double v = camera_coor(1) / camera_coor(2);
        double d = camera_coor(2);
        //(v, u) 是像素的坐标，其中 v 是行索引（通常对应于图像的高度），u 是列索引（通常对应于图像的宽度）
        return cv::Point3d(v,u,d);
    }

    void KalmanFilter::detect_callback(const interfaces::msg::DetectFrame::SharedPtr msg){
        rclcpp::Time time = msg->header.stamp;
        double offset=get_parameter("cam.time_offset").as_double();
        self_color=msg->self_color;
        detect_msg=*msg;
        std::map<double,int> time_map;
        std::vector<std::array<double, 4>> det;//x,y,颜色，编号

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
                for(auto &kf : KFs){
                    time_map[kf.camera_find_match(time,offset)]++;
                }
                det.push_back({red_point.x,red_point.y,0,i});
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
                for(auto &kf : KFs){
                    time_map[kf.camera_find_match(time,offset)]++;
                }
                det.push_back({blue_point.x,blue_point.y,1,i});
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
        double dis_wight=get_parameter("cam.dis_wight").as_double();
        double his_wight=get_parameter("cam.his_wight").as_double();

        // Eigen::MatrixXd cost_matrix = getFusionCost(KFs,det,dist_size, dist_size_size,2.6,time_id);
        Eigen::MatrixXd cost_matrix = getFusionCost(KFs,det,dist_size, dist_size_size,dis_thres,time_id,dis_wight,his_wight);
        eigenMat2VecVec(cost_matrix,dists);

        std::vector<std::vector<int> > matches;
        std::vector<int> u_track,u_cluster;
        linear_assignment(dists, dist_size, dist_size_size,match_thres,matches,u_cluster,u_track);

        cv::Mat img=show_img.clone();
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
            KFs[match[0]].detect_history.push_back(std::make_pair(det[match[1]][2], det[match[1]][3]));
            net_det.push_back(pcl::PointXYZ(det[match[1]][0], det[match[1]][1],0.5));
            for (auto pair:KFs[match[0]].history) {
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
            if(KFs[match[0]].detect_history.size() > KFs[match[0]].max_detect_history){
                KFs[match[0]].detect_history.erase(KFs[match[0]].detect_history.begin());
            }
        }
        ori_net.header.frame_id = "rm_frame";
        net_det.header.frame_id = "rm_frame";
        temp.header.frame_id = "rm_frame";
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(net_det, output);
        output.header.frame_id = "rm_frame";
        output.header.stamp = msg->header.stamp;
        net_pub_->publish(output);
        pcl::toROSMsg(ori_net, output);
        output.header.stamp = msg->header.stamp;
        ori_pub_->publish(output);

        pcl::transformPointCloud(temp,temp, transform.inverse());
        for(size_t i=0;i<temp.points.size();i++){
                cv::Point3d pts_2d=lidarToCamera(temp.points[i],camera_matrix1,lidar2cam1);
                if(pts_2d.y>=0&&pts_2d.y<4096&&pts_2d.x>=0&&pts_2d.x<3000){
                    cv::circle(img,cv::Point(pts_2d.y,pts_2d.x),20,cv::Scalar(0, 255, 255),-1);
                }
        }
        pcl::toROSMsg(temp, output);
        output.header.stamp = msg->header.stamp;
        test_pub_->publish(output);
        // cv::imshow("depth",img);
        // cv::waitKey(1);
    }
    // void KalmanFilter::detect_callback(const interfaces::msg::DetectFrame::SharedPtr msg){
    //     rclcpp::Time time = msg->header.stamp;
    //     self_color=msg->self_color;
    //     detect_msg=*msg;
    //     for(int i=0;i<6;i++){
    //         pcl::PointXY red_point;
    //         red_point.x = msg->red_x[i];
    //         red_point.y = msg->red_y[i];
    //         if(red_point.x!=0&&red_point.y!=0){
    //             if(self_color==1){//己方为蓝色时将坐标反转，因为此时相机传回的是准确的，雷达聚类的是反转的
    //                 red_point.x=28-red_point.x;
    //                 red_point.y=15-red_point.y;
    //             }
    //             for(auto &kf : KFs){
    //                 // int number = i+1;
    //                 // if (number == 6)number++;
    //                 kf.camera_match(time, red_point, 0, i);
    //             }
    //         }
    //
    //         pcl::PointXY blue_point;
    //         blue_point.x = msg->blue_x[i];
    //         blue_point.y = msg->blue_y[i];
    //         if(blue_point.x!=0&&blue_point.y!=0){
    //             if(self_color==1){
    //                 blue_point.x=28-blue_point.x;
    //                 blue_point.y=15-blue_point.y;
    //             }
    //             for(auto &kf : KFs){
    //                 // int number = i+1;
    //                 // if (number == 6)number++;
    //                 kf.camera_match(time, blue_point, 1, i);
    //             }
    //         }
    //     }
    //     // std::cout<<"recieve"<<std::endl;
    //     // for(auto &kf : KFs)
    //     // {
    //     //     for(int i=kf.history.size() - 1; i >= 0; i--)
    //     //     {
    //     //         if(Kalman_filter_plus::GetTimeByRosTime(time)-kf.history[i].first > 5)
    //     //         {
    //     //             kf.history.erase(kf.history.begin() + i);
    //     //         }
    //     //     }
    //     // }
    // }
    void KalmanFilter::check(){
        for (int i=0;i<KFs.size();i++) {
            int i_color=KFs[i].get_color(),i_number=KFs[i].get_number(),max_index=i;
            int max_freq=KFs[i].get_freq(i_color,i_number);
            std::vector<int> same_lists;
            same_lists.push_back(i);
            for(int j=i+1;j<KFs.size();j++) {
                int j_color=KFs[j].get_color(),j_number=KFs[j].get_number();
                int j_freq=KFs[j].get_freq(j_color,j_number);
                if (i_color==j_color&&i_number==j_number) {
                    if (j_freq>max_freq) {
                        max_freq=j_freq;
                        max_index=j;
                    }
                    same_lists.push_back(j);
                }
            }
            for (auto index:same_lists) {
                std::set<int> remove_detect;
                if (index!=max_index) {
                    for (int i=0;i<KFs[index].detect_history.size();i++) {
                        if (KFs[index].detect_history[i].first==i_color&&KFs[index].detect_history[i].second==i_number)
                            remove_detect.insert(i);
                    }
                }
                for (auto idx:remove_detect) {
                    KFs[index].detect_history.erase(KFs[index].detect_history.begin()+idx);
                }
            }
        }
    }
    // void KalmanFilter::check(){
    //     std::set<int> remove_kfs;
    //     for (int i=0;i<KFs.size();i++) {
    //         //检查自身位置是否合法
    //         if(KFs[i].predict_point.x>=27.5||KFs[i].predict_point.x<=0.5||
    //             KFs[i].predict_point.y>=15.1||KFs[i].predict_point.y<=-0.) {
    //             remove_kfs.insert(i);
    //             continue;
    //         }
    //         //检查自身速度是否合法
    //         // int history_size=KFs[i].history.size();
    //         // if (history_size==1)continue;
    //         // double distance=Distance(KFs[i].history[history_size-1].second,KFs[i].history[history_size-2].second);
    //         // double time=KFs[i].history[history_size-1].first-KFs[i].history[history_size-2].first;
    //         double speed_x=KFs[i].KF.statePost.at<float>(1);
    //         double speed_y=KFs[i].KF.statePost.at<float>(3);
    //         double speed=sqrt(speed_x*speed_x+speed_y*speed_y);
    //         // std::cout<<"distance:"<<distance<<"   time:"<<time<<"  speed:"<<distance/time<<std::endl;
    //         if(speed>4.7) {
    //             remove_kfs.insert(i);
    //             continue;
    //         }
    //
    //         int i_color=KFs[i].get_color(),i_number=KFs[i].get_number(),max_index=i;
    //         int max_freq=KFs[i].get_freq(i_color,i_number);
    //         std::vector<int> same_lists;
    //         same_lists.push_back(i);
    //         for(int j=i+1;j<KFs.size();j++) {
    //             //检查是否距离过近
    //             if(KFs[i].Distance(KFs[i].predict_point,KFs[j].predict_point)<=0.2) {
    //                 remove_kfs.insert(j);
    //                 continue;
    //             }
    //             //检查颜色编号是否相同
    //             int j_color=KFs[j].get_color(),j_number=KFs[j].get_number();
    //             int j_freq=KFs[j].get_freq(j_color,j_number);
    //             if (i_color==j_color&&i_number==j_number) {
    //                 if (j_freq>max_freq) {
    //                     max_freq=j_freq;
    //                     max_index=j;
    //                 }
    //                 same_lists.push_back(j);
    //             }
    //         }
    //         for (auto index:same_lists) {
    //             std::set<int> remove_detect;
    //             if (index!=max_index) {
    //                 for (int i=0;i<KFs[index].detect_history.size();i++) {
    //                     if (KFs[index].detect_history[i].first==i_color&&KFs[index].detect_history[i].second==i_number)
    //                         remove_detect.insert(i);
    //                 }
    //             }
    //             for (auto idx:remove_detect) {
    //                 KFs[index].detect_history.erase(KFs[index].detect_history.begin()+idx);
    //             }
    //         }
    //     }
    //     for (auto index:remove_kfs) {
    //         KFs.erase(KFs.begin()+index);
    //     }
    // }
    //检查是否有超出地图边界的及近似重合的检测器，并将其删除predict_point
    void KalmanFilter::check_KFs(){
        std::set<int> remove_kfs;
        for(int i=0;i<KFs.size();i++){
            for(int j=i+1;j<KFs.size();j++){
                if(KFs[i].Distance(KFs[i].predict_point,KFs[j].predict_point)<=0.2)
                    remove_kfs.insert(j);
            }

            if(KFs[i].predict_point.x>=27.5||KFs[i].predict_point.x<=0.5||
                KFs[i].predict_point.y>=15.1||KFs[i].predict_point.y<=-0.)
                remove_kfs.insert(i);
            int history_size=KFs[i].history.size();
            if (history_size==1)continue;
            double distance=Distance(KFs[i].history[history_size-1].second,KFs[i].history[history_size-2].second);
            double time=KFs[i].history[history_size-1].first-KFs[i].history[history_size-2].first;
            double speed_x=KFs[i].KF.statePost.at<float>(1);
            double speed_y=KFs[i].KF.statePost.at<float>(3);
            double speed=sqrt(speed_x*speed_x+speed_y*speed_y);
            // std::cout<<"distance:"<<distance<<"   time:"<<time<<"  speed:"<<distance/time<<std::endl;
            if(speed>4.7)
                remove_kfs.insert(i);
        }
        for (auto index:remove_kfs) {
            KFs.erase(KFs.begin()+index);
        }
    }

    void KalmanFilter::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        rclcpp::Time time = msg->header.stamp;
        auto now_time = std::chrono::steady_clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        for(auto point : cloud->points){
            pcl::PointXYZ point_xy;
            point_xy.x = point.x;
            point_xy.y = point.y;
            point_xy.z = point.z;
            cloud_xy->points.push_back(point_xy);
        }
        if(cloud_xy->points.size() == 0)return;

        // std::vector<bool> is_updated(cloud_xy->points.size(), false);
        for(auto &kf : KFs){//一开始KFs中没有元素，因此此处循环不会进行
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
        //     for(int j = 0; j < this->KFs.size(); j++){
        //         if(KFs[j].match(cloud_xy->points[i])){
        //             match_kf_indexs.push_back(j);
        //         }
        //     }
        //     if(match_kf_indexs.size() == 0){
        //         Kalman_filter_plus kf(cloud_xy->points[i], time,reinterpret_cast<rclcpp::Node*>(this));//time为最后更新时间
        //         KFs.push_back(kf);
        //         // std::cout<<"new kf"<<std::endl;
        //     }
        // }
        
        check_KFs();
        // //添加对多个聚类匹配到同一个卡尔曼对象时的处理，但会导致飞的卡尔曼增多
        // std::unordered_map<int,std::vector<int>> target_list;
        // std::vector<bool> is_updated(KFs.size(),false);
        // for(int i=0;i<cloud_xy->points.size();i++){
        //     std::vector<int> match_kf_indexs;
        //     for(int j = 0; j < this->KFs.size(); j++){
        //         if(KFs[j].match(cloud_xy->points[i])){
        //             match_kf_indexs.push_back(j);
        //         }
        //     }
        //     if(match_kf_indexs.size() == 0){
        //         Kalman_filter_plus kf(cloud_xy->points[i], time,reinterpret_cast<rclcpp::Node*>(this));//time为最后更新时间
        //         KFs.push_back(kf);
        //     }
        //     else if(match_kf_indexs.size() == 1){
        //         target_list[match_kf_indexs[0]].push_back(i);
        //         is_updated[match_kf_indexs[0]] = true;
        //     }else{
        //         float min_distance = 1000000;
        //         int min_index = 0;
        //         for(auto index : match_kf_indexs){
        //             float distance = KFs[index].Distance(KFs[index].predict_point, cloud_xy->points[i]);
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
        //         KFs[it->first].update(cloud_xy->points[it->second[0]],time);
        //     }else{
        //         int min_index=-1;
        //         int min_dis=10000;
        //         for(auto index : target_list[it->first]){
        //             double dis=KFs[it->first].Distance(KFs[it->first].predict_point,cloud_xy->points[index]);
        //             if(dis<min_dis){
        //                 min_dis=dis;
        //                 min_index=index;
        //             }
        //         }
        //         if(KFs[it->first].history.size()<7){
        //             KFs[it->first].history.clear();
        //         }else{
        //             for(auto index:target_list[it->first]){
        //                 if(index==min_index) continue;
        //
        //                 std::vector<int> match_kf_indexs;
        //                 for(int j=0;j<is_updated.size();j++){
        //                     if(is_updated[j]) continue;
        //                     if(KFs[j].loose_match(cloud_xy->points[index])){
        //                         match_kf_indexs.push_back(j);
        //                     }
        //                 }
        //                 if(match_kf_indexs.size() == 0){
        //                     Kalman_filter_plus kf(cloud_xy->points[index],time,reinterpret_cast<rclcpp::Node*>(this));//time为最后更新时间
        //                     KFs.push_back(kf);
        //                 }
        //                 else if(match_kf_indexs.size() == 1){
        //                     KFs[match_kf_indexs[0]].update(cloud_xy->points[index], time);//time为加入history的时间
        //                 }else{
        //                     float min_distance = 1000000;
        //                     int min_index = 0;
        //                     for(auto i:match_kf_indexs){
        //                         float distance = KFs[i].Distance(KFs[i].predict_point,cloud_xy->points[index]);
        //                         if(distance < min_distance){
        //                             min_distance = distance;
        //                             min_index = i;
        //                         }
        //                     }
        //                     KFs[min_index].update(cloud_xy->points[index], time);//time为加入history的时间
        //                 }
        //             }
        //         }
        //         KFs[it->first].update(cloud_xy->points[min_index],time);
        //     }
        // }
        double clu_match_thres=get_parameter("clu.match_thres").as_double();
        double clu_dis_thres=get_parameter("clu.dis_thres").as_double();
        double clu_min_dis_thres=get_parameter("clu.min_dis_thres").as_double();

        std::vector<std::vector<float> > dists;
        int dist_size=0,dist_size_size=0;
        Eigen::MatrixXd cost_matrix = getCloudCost(*cloud_xy,KFs,dist_size, dist_size_size,clu_dis_thres);
        // std::cout<<cost_matrix<<std::endl;
        eigenMat2VecVec(cost_matrix,dists);
        std::vector<std::vector<int> > matches;
        std::vector<int> u_track,u_cluster;
        //进行匹配
        linear_assignment(dists, dist_size, dist_size_size,clu_match_thres ,matches,u_cluster,u_track);
        for(auto match : matches){
            KFs[match[1]].update(cloud_xy->points[match[0]],time);
        }
        if (matches.size()==0) {
            for(auto point : cloud_xy->points){
                Kalman_filter_plus kf(point, time,reinterpret_cast<rclcpp::Node*>(this));//time为最后更新时间
                KFs.push_back(kf);
            }
        }
        for (auto cluster:u_cluster) {
            int min_dis=10000;
            for (auto match : matches) {
                double distance=Distance(cloud_xy->points[cluster],cloud_xy->points[match[0]]);
                if (distance<min_dis) {
                    min_dis=distance;
                }
            }
            if (min_dis>clu_min_dis_thres) {
                Kalman_filter_plus kf(cloud_xy->points[cluster], time,reinterpret_cast<rclcpp::Node*>(this));//time为最后更新时间
                KFs.push_back(kf);
            }
        }
        //默认卡尔曼匹配策略，可能出现一辆车两个聚类导致卡尔曼飘飞，两辆车距离过近导致只有一个卡尔曼，但卡尔曼飘飞的情况减少
        // for(auto point : cloud_xy->points){//对于每个点
        // //如果遍历所有卡尔曼都没找到能够匹配的，新建一个卡尔曼
        // //若找到了1个，则更新这个卡尔曼
        // //若找到了多个，则更新距离最近的那个
        //     std::vector<int> match_kf_indexs;
        //     for(int i = 0; i < this->KFs.size(); i++){
        //         if(KFs[i].match(point)){
        //             match_kf_indexs.push_back(i);
        //         }
        //     }
        //     if(match_kf_indexs.size() == 0){
        //         Kalman_filter_plus kf(point, time,reinterpret_cast<rclcpp::Node*>(this));//time为最后更新时间
        //         KFs.push_back(kf);
        //         // std::cout<<"new kf"<<std::endl;
        //     }
        //     else if(match_kf_indexs.size() == 1){
        //         KFs[match_kf_indexs[0]].update(point, time);//time为加入history的时间
        //         // std::cout<<"update kf"<<std::endl;
        //     }else{
        //         float min_distance = 1000000;
        //         int min_index = 0;
        //         for(auto index : match_kf_indexs){
        //             float distance = KFs[index].Distance(KFs[index].predict_point, point);
        //             if(distance < min_distance){
        //                 min_distance = distance;
        //                 min_index = index;
        //             }
        //         }
        //         KFs[min_index].update(point, time);//time为加入history的时间
        //         // std::cout<<"find kf"<<std::endl;
        //     }
        // }
        check_KFs();
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        //过滤出红蓝方有位置信息的检测点
        //TODO：需要对己方为蓝方的时候作处理
        std::vector<std::array<int, 4>> blue_det,red_det;//x,y,颜色，编号
        for(int i=0;i<6;i++){
            if(detect_msg.red_x[i]!=0&&detect_msg.red_y[i]!=0) {
                if (self_color==1)
                    red_det.push_back({28-detect_msg.red_x[i],15-detect_msg.red_y[i],0,i});
                else
                    red_det.push_back({detect_msg.red_x[i],detect_msg.red_y[i],0,i});
            }
            if(detect_msg.blue_x[i]!=0&&detect_msg.blue_y[i]!=0) {
                if (self_color==1)
                    blue_det.push_back({28-detect_msg.blue_x[i],15-detect_msg.blue_y[i],1,i});
                else
                    blue_det.push_back({detect_msg.blue_x[i],detect_msg.blue_y[i],1,i});
            }
        }
        std::vector<Kalman_filter_plus> blue_kfs,red_kfs;
        for(int i = KFs.size() - 1; i >= 0; i--){
            if (KFs[i].last_time>0.7&&KFs[i].last_time<=2.5) {
                //进入特殊位置
                if(KFs[i].predict_point.x>=25.1&&KFs[i].predict_point.y>=11.1){
                    int car_num=KFs[i].get_number();
                    if(car_num==1){
                        KFs[i].predict_point.x=26.9;
                        KFs[i].predict_point.y=11.6;
                        RCLCPP_ERROR(this->get_logger(),"enter mill!!!"); //工程兑矿区
                    }else if (car_num!=-1){
                        KFs[i].predict_point.x=26.31;
                        KFs[i].predict_point.y=14.05;
                        RCLCPP_ERROR(this->get_logger(),"enter supply!!!"); //其他车补给点
                    }
                }else if(KFs[i].predict_point.x<=20.18&&KFs[i].predict_point.x>=18.96&&KFs[i].predict_point.y>=12.46&&KFs[i].predict_point.y<=13.71){
                    int car_num=KFs[i].get_number();
                    if(car_num==2||car_num==3||car_num==4){
                        KFs[i].predict_point.x=19.58;
                        KFs[i].predict_point.y=13.15;
                        RCLCPP_ERROR(this->get_logger(),"enter energy!!!"); //打符点
                    }
                }
            }else if(KFs[i].last_time>2.5&&(KFs[i].last_time)<10){
                int car_num=KFs[i].get_number();
                if (car_num==-1||KFs[i].predict_point.x<=25.1||KFs[i].predict_point.y<=11.1)
                    KFs.erase(KFs.begin() + i);

            }else if(KFs[i].last_time>=10){
                if(KFs[i].get_number()!=1){//不是工程的车辆卡尔曼删除
                    KFs.erase(KFs.begin() + i);
                }
            }else if(KFs[i].last_time<=1){
                int color=KFs[i].get_color();
                if(color==0){
                    red_kfs.push_back(KFs[i]);
                }else if(color==1){
                    blue_kfs.push_back(KFs[i]);
                }
            }
        }
        check();
        interfaces::msg::DetectResult detect_res;

        std::vector<std::vector<float> > b_dists,r_dists;
        int b_dist_size=0,b_dist_size_size=0,r_dist_size=0,r_dist_size_size=0;

        double distance_weight=get_parameter("cost.distance_weight").as_double();
        double color_weight=get_parameter("cost.color_weight").as_double();
        double distance_thres=get_parameter("cost.distance_thres").as_double();
        match_thresh=get_parameter("fusion.match_thresh").as_double();

        //得到代价矩阵
        Eigen::MatrixXd b_cost_matrix = getFusionCost(blue_kfs,blue_det,b_dist_size, b_dist_size_size,
            distance_weight,distance_thres,color_weight);
        Eigen::MatrixXd r_cost_matrix = getFusionCost(red_kfs,red_det,r_dist_size, r_dist_size_size,
            distance_weight,distance_thres,color_weight);
        // std::cout<<cost_matrix<<std::endl;
        //转换代价矩阵的形式
        eigenMat2VecVec(b_cost_matrix,b_dists);
        eigenMat2VecVec(r_cost_matrix,r_dists);
    
        std::vector<std::vector<int> > b_matches,r_matches;
        std::vector<int> b_u_track,b_u_detection,r_u_track,r_u_detection;
        //进行匹配
        linear_assignment(b_dists, b_dist_size, b_dist_size_size,match_thresh,b_matches,b_u_track,b_u_detection);
        linear_assignment(r_dists, r_dist_size, r_dist_size_size,match_thresh,r_matches,r_u_track,r_u_detection);

        
        // std::cout<<matches.size()<<std::endl;
        // std::cout<<u_track.size()<<std::endl;
        visualization_msgs::msg::MarkerArray vis_array;
        for (int i = 0; i < b_matches.size(); i++){
            Kalman_filter_plus kf = blue_kfs[b_matches[i][0]];
            std::array<int, 4> det=blue_det[b_matches[i][1]];
            // detect_res.blue_x[det[3]] = kf.predict_point.x;
            // detect_res.blue_y[det[3]] = kf.predict_point.y;

            vis_maker(0,vis_array,det[2],det[3],kf.predict_point.x,kf.predict_point.y);
        }

        for (int i = 0; i < r_matches.size(); i++){
            Kalman_filter_plus kf = red_kfs[r_matches[i][0]];
            std::array<int, 4> det=red_det[r_matches[i][1]];
            // detect_res.red_x[det[3]] = kf.predict_point.x;
            // detect_res.red_y[det[3]] = kf.predict_point.y;

            vis_maker(0,vis_array,det[2],det[3],kf.predict_point.x,kf.predict_point.y);
        }
        std::cout<<"----------------------"<<std::endl;


        for(int i = KFs.size() - 1; i >= 0; i--){
            pcl::PointXYZRGB point;
            point.x = KFs[i].predict_point.x;
            point.y = KFs[i].predict_point.y;
            point.z = 1.5;
            int color = KFs[i].get_color();
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
        std::vector<std::vector<int>> history={{0,0,0,0,0,0},{0,0,0,0,0,0}};
        for(auto kf : KFs){
            if(kf.detect_history.size()==0)continue;
            //此处还需要优化，避免出现多个卡尔曼对象对应到同一个红蓝点
            if(kf.get_color() == 1){//蓝色
                int number= kf.get_number();

                history[1][number]++;
                vis_kal_maker(1,vis_array,1,number,kf.predict_point.x,kf.predict_point.y,kf.detect_history.size(),history[1][number]);
                if (kf.last_time>1.5) {
                    if (detect_msg.blue_x[number]!=0&&detect_msg.blue_y[number]!=0) {
                        detect_res.blue_x[number] = detect_msg.blue_x[number];
                        detect_res.blue_y[number] = detect_msg.blue_y[number];
                        continue;
                    }
                }

                detect_res.blue_x[number] = kf.predict_point.x;
                detect_res.blue_y[number] = kf.predict_point.y;
                // std::cout<<"kalman index: "<<num<<" detection : "<<1<<" "<<number<<std::endl;
            }else if(kf.get_color() == 0){//红色
                int number= kf.get_number();

                history[0][number]++;
                vis_kal_maker(1,vis_array,0,number,kf.predict_point.x,kf.predict_point.y,kf.detect_history.size(),history[1][number]);
                if (kf.last_time>1.5) {
                    if (detect_msg.red_x[number]!=0&&detect_msg.red_y[number]!=0) {
                        detect_res.red_x[number] = detect_msg.red_x[number];
                        detect_res.red_y[number] = detect_msg.red_y[number];
                        continue;
                    }
                }

                detect_res.red_x[number] = kf.predict_point.x;
                detect_res.red_y[number] = kf.predict_point.y;
                // std::cout<<"kalman index: "<<num<<" detection : "<<0<<" "<<number<<std::endl;
            }
            num++;
        }

        //self_color==1为自己为蓝方
        if(self_color==1){
            //地图默认以红方的角点为原点，因此己方为蓝方时需要转换坐标系
            for(int i=0;i<6;i++){
                if(detect_res.blue_x[i]!=0&&detect_res.blue_y[i]!=0){
                    detect_res.blue_x[i]=28-detect_res.blue_x[i];
                    detect_res.blue_y[i]=15-detect_res.blue_y[i];
                }else{
                    detect_res.blue_x[i]=detect_msg.blue_x[i];
                    detect_res.blue_y[i]=detect_msg.blue_y[i];
                }
                if(detect_res.red_x[i]!=0&&detect_res.red_y[i]!=0){
                    detect_res.red_x[i]=28-detect_res.red_x[i];
                    detect_res.red_y[i]=15-detect_res.red_y[i];
                }else{
                    detect_res.red_x[i]=detect_msg.red_x[i];
                    detect_res.red_y[i]=detect_msg.red_y[i];
                }
            } 
        }else{
            for(int i=0;i<6;i++){
                if(detect_res.blue_x[i]==0||detect_res.blue_y[i]==0){
                    detect_res.blue_x[i]=detect_msg.blue_x[i];
                    detect_res.blue_y[i]=detect_msg.blue_y[i];
                }
                if(detect_res.red_x[i]==0||detect_res.red_y[i]==0){
                    detect_res.red_x[i]=detect_msg.red_x[i];
                    detect_res.red_y[i]=detect_msg.red_y[i];
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
        auto end_time = std::chrono::steady_clock::now();
        float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
        RCLCPP_WARN(this->get_logger(), "Kalman Callback time is %f ms", dur_time);
    }
}
RCLCPP_COMPONENTS_REGISTER_NODE(upc_radar::KalmanFilter)