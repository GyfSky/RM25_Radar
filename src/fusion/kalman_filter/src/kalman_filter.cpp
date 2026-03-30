#include "kalman_filter.h"

#include <opencv2/imgproc/types_c.h>

namespace upc_radar{
    KalmanFilter::KalmanFilter(const rclcpp::NodeOptions& node_options):rclcpp::Node("kalman_filter_node",node_options),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_){

        prepareParameter();

        if (is_one_lidar) {
            sub_pc = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_dynamic", 5, std::bind(&KalmanFilter::callback, this, std::placeholders::_1));
            RCLCPP_WARN(this->get_logger(), "is_one_lidar is true, only one lidar will be used.");
        }else {
            mid70_sub_.subscribe(this, "/livox/mid70");
            avia_sub_.subscribe(this, "/livox/avia");
            PC2PCPolicy sync_policy(5);
            sync_policy.setMaxIntervalDuration(rclcpp::Duration(0,100000000));
            pc_sync_=std::make_shared<message_filters::Synchronizer<PC2PCPolicy>>(std::ref(sync_policy),avia_sub_,mid70_sub_);
            pc_sync_->registerCallback(&KalmanFilter::PCTimeSynC, this);
        }

        drone1_sub_.subscribe(this, "/livox/avia/drone");
        drone2_sub_.subscribe(this, "/livox/mid70/drone");
        MySyncPolicy drone_sync_policy(5);
        drone_sync_policy.setMaxIntervalDuration(rclcpp::Duration(0,100000000));
        drone_sync_=std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(std::ref(drone_sync_policy),drone1_sub_,drone2_sub_);
        drone_sync_->registerCallback(&KalmanFilter::droneTimeSynC, this);

        cluster_sub_.subscribe(this, "/filtered_pc");
        target_sub_.subscribe(this, "/cluster_target");
        Clus2TargetPolicy sync_policy(10);
        sync_policy.setMaxIntervalDuration(rclcpp::Duration(0,100000000));
        clus_sync_=std::make_shared<message_filters::Synchronizer<Clus2TargetPolicy>>(std::ref(sync_policy),cluster_sub_,target_sub_);
        clus_sync_->registerCallback(&KalmanFilter::clusTimeSynC, this);

        lidar_enh_pub_=this->create_publisher<interfaces::msg::LidarEnhance>("/lidar_enhance", 1);
        pub_kalman = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_kalman", 10);
        pub_cluster = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", 10);
        pub_kmeans = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_kmeans", 10);
        pub_vis = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vis_point", 10);
        lidar_detect_pub = this->create_publisher<interfaces::msg::DetectResult>("/lidar_detect", 1);
        rects_pub_=this->create_publisher<interfaces::msg::ClusterRect>("/lidar_rects", 1);
        filtered_pc_pub_=this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pc", 10);
        pub_drone_=this->create_publisher<interfaces::msg::DroneLocation>("/drone_location", 1);

        // sub_robot_hp=this->create_subscription<interfaces::msg::RobotHP>("/robot_hp", 10, std::bind(&KalmanFilter::robotHPCallback, this, std::placeholders::_1));

        net_pub=this->create_publisher<sensor_msgs::msg::PointCloud2>("/net_3D", 10);
        pc_pub= this->create_publisher<sensor_msgs::msg::PointCloud2>("/pc_3D", 10);

        show_img1=cv::imread("resource/img/cam1.jpg");
        cv::namedWindow("cam1",0);
        cv::resizeWindow("cam1",640,480);
        cv::imshow("cam1",show_img1);

        show_img2=cv::imread("resource/img/cam2.jpg");
        cv::namedWindow("cam2",0);
        cv::resizeWindow("cam2",640,480);
        cv::imshow("cam2",show_img2);

        std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ori;
        mesh_ori = open3d::io::CreateMeshFromFile(get_parameter("mesh.mesh_ori").as_string());
        if (!mesh_ori) {
            RCLCPP_ERROR(get_logger(), "mesh_ori is null");
            return;
        }
        mesh_ori->ComputeVertexNormals();
        mesh_ori->ComputeTriangleNormals();

        Eigen::Vector3d init_translate(14.0, 7.5, 0.0);
        Eigen::Matrix3d R;
        R<<0.000000, 1.000000, 0.000000,
        -1.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 1.000000;
        mesh_ori->Rotate(R,Eigen::Vector3d(0,0,0));
        mesh_ori->Translate(init_translate);

        z_map = std::make_shared<Z_Map>(
            Eigen::Vector3d { 0.0, 0.0, 0.0 },
            Eigen::Vector3d { 28.0, 15.0, 3.0 },
            0.08,
            mesh_ori);

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
        declare_parameter<std::string>("mesh.mesh_ori","resource/RMUC2025_national.stl");

        is_one_lidar=declare_parameter<bool>("is_one_lidar",false);

        camera_matrix1=cv::Matx33d(2291.14722575626,0,1103.67246,0,2291.55696902824,1086.45687,0,0,1);
        lidar2cam1=cv::Matx44d(0.48328,   -0.05083 , 0.87399  , -0.32246 ,
-0.86805 , -0.15749 , 0.47084 ,  0.12201   ,
0.11371  , -0.98621 , -0.12023 , 0.08868   ,
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

        self_trapezoidal_[0]=cv::Point2f(5.0,15.0);
        self_trapezoidal_[1]=cv::Point2f(5.0,15.0);
        self_trapezoidal_[2]=cv::Point2f(8.7,15.0);
        self_trapezoidal_[3]=cv::Point2f(9.15,15.0);

        fortress_[0]=cv::Point2f(22,8.3);
        fortress_[1]=cv::Point2f(22,6.7);
        fortress_[2]=cv::Point2f(20.5,6.7);
        fortress_[3]=cv::Point2f(20.5,8.3);
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

        std::vector<open3d::geometry::PointCloud> pcs;
        open3d::geometry::PointCloud pc_noise;
        std::vector<Eigen::Vector3d> grav;
        int max_l = *std::max_element(labels.begin(), labels.end());
        // int max_l = clusters.size();

        for (int i = 0; i <= max_l; i++) {
            pcs.emplace_back(open3d::geometry::PointCloud());
        }

        for (size_t i = 0; i < in_cloud.points_.size(); i++) {
            if(labels[i] >= 0) {
                pcs[labels[i]].points_.push_back(in_cloud.points_[i]);
            }else
                pc_noise.points_.push_back(in_cloud.points_[i]);
        }

        for (int i = 0; i <=max_l; i++) {
                out.push_back(pcs[i]);
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
            // if(initornot(tunnel_slanted_,KFs_[match[0]].output_point,5)==1&&KFs_[match[0]].last_time>1.3) {
            //     //自己是红方
            //     if (self_color==0&&match[1]==5) {
            //         KFs_[match[0]].output_point.x=17.5392;
            //         KFs_[match[0]].output_point.y=4.1475;
            //         lidar_enhance_[1][match[1]-5]=4;
            //         detect_res.blue_x[match[1]-5]=KFs_[match[0]].output_point.x;
            //         detect_res.blue_y[match[1]-5]=KFs_[match[0]].output_point.y;
            //         detect_res.v_x[match[1]-5] = 0;
            //         detect_res.v_y[match[1]-5] = 0;
            //         remove_KFs_.push_back(match[0]);
            //     }else if (self_color==1&&match[1]==0) {
            //         KFs_[match[0]].output_point.x=10.4608;
            //         KFs_[match[0]].output_point.y=10.8525;
            //         lidar_enhance_[0][match[1]]=4;
            //         detect_res.red_x[match[1]]=KFs_[match[0]].output_point.x;
            //         detect_res.red_y[match[1]]=KFs_[match[0]].output_point.y;
            //         detect_res.v_x[match[1]] = 0;
            //         detect_res.v_y[match[1]] = 0;
            //         remove_KFs_.push_back(match[0]);
            //     }else if (match[1]<5){
            //         fake_kfs[0][match[1]].location=KFs_[match[0]].output_point;
            //         fake_kfs[0][match[1]].is_first=true;
            //         double speed=sqrt(pow(KFs_[match[0]].KF.statePost.at<float>(1),2)+pow(KFs_[match[0]].KF.statePost.at<float>(3),2));
            //         if (KFs_[match[0]].KF.statePost.at<float>(1)>0) {
            //             fake_kfs[0][match[1]].v_x=speed*sin(35*M_PI/180);
            //             fake_kfs[0][match[1]].v_y=speed*cos(35*M_PI/180);
            //         }else {
            //             fake_kfs[0][match[1]].v_x=-speed*sin(35*M_PI/180);
            //             fake_kfs[0][match[1]].v_y=-speed*cos(35*M_PI/180);
            //         }
            //         lidar_enhance_[0][match[1]]=2;
            //         if (self_color==1) {
            //             fake_kfs[0][match[1]].location.x=28-fake_kfs[0][match[1]].location.x;
            //             fake_kfs[0][match[1]].location.y=15-fake_kfs[0][match[1]].location.y;
            //             fake_kfs[0][match[1]].v_x=-fake_kfs[0][match[1]].v_x;
            //             fake_kfs[0][match[1]].v_y=-fake_kfs[0][match[1]].v_y;
            //             detect_res.v_x[match[1]]=fake_kfs[0][match[1]].v_x;
            //             detect_res.v_y[match[1]]=fake_kfs[0][match[1]].v_y;
            //         }
            //         remove_KFs_.push_back(match[0]);
            //     }else if (match[1]>=5){
            //         fake_kfs[1][match[1]-5].location=KFs_[match[0]].output_point;
            //         fake_kfs[1][match[1]-5].is_first=true;
            //         double speed=sqrt(pow(KFs_[match[0]].KF.statePost.at<float>(1),2)+pow(KFs_[match[0]].KF.statePost.at<float>(3),2));
            //         if (KFs_[match[0]].KF.statePost.at<float>(1)>0) {
            //             fake_kfs[1][match[1]-5].v_x=speed*sin(35*M_PI/180);
            //             fake_kfs[1][match[1]-5].v_y=speed*cos(35*M_PI/180);
            //         }else {
            //             fake_kfs[1][match[1]-5].v_x=-speed*sin(35*M_PI/180);
            //             fake_kfs[1][match[1]-5].v_y=-speed*cos(35*M_PI/180);
            //         }
            //         lidar_enhance_[1][match[1]-5]=2;
            //         if (self_color==0) {
            //             detect_res.v_x[match[1]-5]=fake_kfs[1][match[1]-5].v_x;
            //             detect_res.v_y[match[1]-5]=fake_kfs[1][match[1]-5].v_y;
            //         }else {
            //             fake_kfs[1][match[1]-5].location.x=28-fake_kfs[1][match[1]-5].location.x;
            //             fake_kfs[1][match[1]-5].location.y=15-fake_kfs[1][match[1]-5].location.y;
            //             fake_kfs[1][match[1]-5].v_x=-fake_kfs[1][match[1]-5].v_x;
            //             fake_kfs[1][match[1]-5].v_y=-fake_kfs[1][match[1]-5].v_y;
            //         }
            //         remove_KFs_.push_back(match[0]);
            //     }
            //     continue;
            // }
            // //水平隧道
            // if(initornot(tunnel_horizontal_,KFs_[match[0]].output_point,4)==1&&KFs_[match[0]].last_time>0.2) {
            //     if (match[1]<5){
            //         fake_kfs[0][match[1]].location=KFs_[match[0]].output_point;
            //         fake_kfs[0][match[1]].is_first=true;
            //         double speed=sqrt(pow(KFs_[match[0]].KF.statePost.at<float>(1),2)+pow(KFs_[match[0]].KF.statePost.at<float>(3),2));
            //         if (KFs_[match[0]].KF.statePost.at<float>(1)>0) {
            //             fake_kfs[0][match[1]].v_x=speed;
            //             fake_kfs[0][match[1]].v_y=0;
            //         }else {
            //             fake_kfs[0][match[1]].v_x=-speed;
            //             fake_kfs[0][match[1]].v_y=0;
            //         }
            //         lidar_enhance_[0][match[1]]=3;
            //         if (self_color==1) {
            //             fake_kfs[0][match[1]].location.x=28-fake_kfs[0][match[1]].location.x;
            //             fake_kfs[0][match[1]].location.y=15-fake_kfs[0][match[1]].location.y;
            //             fake_kfs[0][match[1]].v_x=-fake_kfs[0][match[1]].v_x;
            //             fake_kfs[0][match[1]].v_y=-fake_kfs[0][match[1]].v_y;
            //             detect_res.v_x[match[1]]=fake_kfs[0][match[1]].v_x;
            //             detect_res.v_y[match[1]]=fake_kfs[0][match[1]].v_y;
            //         }
            //         remove_KFs_.push_back(match[0]);
            //     }
            //     else if (match[1]>=5){
            //         fake_kfs[1][match[1]-5].location=KFs_[match[0]].output_point;
            //         fake_kfs[1][match[1]-5].is_first=true;
            //         double speed=sqrt(pow(KFs_[match[0]].KF.statePost.at<float>(1),2)+pow(KFs_[match[0]].KF.statePost.at<float>(3),2));
            //         if (KFs_[match[0]].KF.statePost.at<float>(1)>0) {
            //             fake_kfs[1][match[1]-5].v_x=speed;
            //             fake_kfs[1][match[1]-5].v_y=0;
            //         }else {
            //             fake_kfs[1][match[1]-5].v_x=-speed;
            //             fake_kfs[1][match[1]-5].v_y=0;
            //         }
            //         lidar_enhance_[1][match[1]-5]=3;
            //         if (self_color==0) {
            //             detect_res.v_x[match[1]-5]=fake_kfs[1][match[1]-5].v_x;
            //             detect_res.v_y[match[1]-5]=fake_kfs[1][match[1]-5].v_y;
            //         }else {
            //             fake_kfs[1][match[1]-5].location.x=28-fake_kfs[1][match[1]-5].location.x;
            //             fake_kfs[1][match[1]-5].location.y=15-fake_kfs[1][match[1]-5].location.y;
            //             fake_kfs[1][match[1]-5].v_x=-fake_kfs[1][match[1]-5].v_x;
            //             fake_kfs[1][match[1]-5].v_y=-fake_kfs[1][match[1]-5].v_y;
            //         }
            //         remove_KFs_.push_back(match[0]);
            //         continue;
            //     }
            // }
            //己方梯高盲区
            if(initornot(self_trapezoidal_,KFs_[match[0]].output_point,4)==1&&KFs_[match[0]].last_time>1.0){
                //自己是红方
                if (self_color==0&&match[1]>=5) {
                    lidar_enhance_[1][match[1]-5]=1;
                    detect_res.blue_x[match[1]-5]=6.513;
                    detect_res.blue_y[match[1]-5]=12.928;
                    detect_res.v_x[match[1]-5] = 0;
                    detect_res.v_y[match[1]-5] = 0;
                    remove_KFs_.push_back(match[0]);
                }else if (self_color==1&&match[1]<5) {
                    lidar_enhance_[0][match[1]]=1;
                    detect_res.red_x[match[1]]=21.487;
                    detect_res.red_y[match[1]]=2.072;
                    detect_res.v_x[match[1]] = 0;
                    detect_res.v_y[match[1]] = 0;
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

    void KalmanFilter::getRect2d(std::vector<open3d::geometry::PointCloud> pcs,Eigen::Transform<float, 3, 2> transform,std::vector<cv::Rect> &rects,std::vector<cv::Point3d> &points,int camid) {
        // std::vector<cv::Point3d> pts;
        for(auto pc:pcs){
            // cv::Mat img=show_img.clone();
            auto center=pc.GetCenter();
            center=z_map->project_ground(Eigen::Vector2d(center[0],center[1]));
            open3d::geometry::PointCloud center_pc;
            center_pc.points_.push_back(center);
            center_pc.Transform(transform.matrix().cast<double>().inverse());
            pc.Transform(transform.matrix().cast<double>().inverse());
            cv::Point3d center_2d;
            if (camid==1)
                center_2d=lidarToCamera(pcl::PointXYZ(pc.GetCenter()[0],pc.GetCenter()[1],center_pc.GetCenter()[2]),camera_matrix1,lidar2cam1);
            else if (camid==2)
                center_2d=lidarToCamera(pcl::PointXYZ(pc.GetCenter()[0],pc.GetCenter()[1],center_pc.GetCenter()[2]),camera_matrix2,lidar2cam2);

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
            points.push_back(center_2d);
        }
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

    void KalmanFilter::updateKFs(std::vector<Kalman_filter_plus> &KFs_,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xy,const std::vector<open3d::geometry::PointCloud> &pcs,
            const interfaces::msg::ClusterTarget::SharedPtr clus_msg,pcl::PointCloud<pcl::PointXYZ> &kmeans_out,rclcpp::Time time) {
        for(auto &kf : KFs_){//一开始KFs_中没有元素，因此此处循环不会进行 若有元素，则进行预测更新
            kf.update_predict_point();
            kf.has_updated = false;
        }

        double clu_match_thres=get_parameter("clu.match_thres").as_double();
        double clu_dis_thres=get_parameter("clu.dis_thres").as_double();
        double clu_min_dis_thres=get_parameter("clu.min_dis_thres").as_double();

        std::vector<int> KFs_class(KFs_.size(),-1);
        for (int i=0;i<KFs_.size();i++) {
            if (KFs_[i].get_color()==2) continue;
            int color=KFs_[i].get_color(),number=KFs_[i].get_number();
            KFs_class[i]=color*5+number;
        }
        std::vector<std::vector<float>> cost_confMatrix_vec;
        int num_strack,num_cls;
        std::vector<std::vector<int> > matches_cls;
        std::vector<int> u_strack, u_cls;
        matches_cls.clear();u_strack.clear();u_cls.clear();

        Eigen::MatrixXd cost_confMatrix = getCost_confMatrix(clus_msg,num_strack,num_cls);
        eigenMat2VecVec(cost_confMatrix, cost_confMatrix_vec);
        linear_assignment(cost_confMatrix_vec, num_strack, num_cls, 0.9, matches_cls, u_strack, u_cls);

        std::vector<int> cluster_class(pcs.size(),-1);
        for (auto match : matches_cls) {
            cluster_class[match[0]]=match[1];
        }


        // for (int i=0;i<pcs.size();i++) {
        //     int max_index=-1;
        //     double max_value=0.0;
        //     for (int j=0;j<10;j++) {
        //         if (clus_msg->clusters[i].cost_matrix[j]>max_value) {
        //             max_value=clus_msg->clusters[i].cost_matrix[j];
        //             max_index=j;
        //         }
        //     }
        //     if (max_value!=0) {
        //         if (max_index>=5)
        //             cluster_class[i]=max_index-5;
        //         else {
        //             cluster_class[i]=max_index+5;
        //         }
        //     }
        // }

        std::vector<bool> KFs_update(KFs_.size(),false);
        std::vector<bool> cluster_update(pcs.size(),false);
        for (int i=0;i<KFs_.size();i++) {
            double dis=10000.0;
            int index=-1;
            for (int j=0;j<pcs.size();j++) {
                double distance = sqrt(pow(pcs[j].GetCenter()[0]-KFs_[i].predict_point.x,2)+pow(pcs[j].GetCenter()[1]-KFs_[i].predict_point.y,2));
                if (KFs_update[i]==false&&cluster_update[j]==false
                    &&KFs_class[i]==cluster_class[j]&&distance<1.4
                    &&distance<dis&&KFs_class[i]!=-1) {
                    dis=distance;
                    index=j;
                }
            }
            if (index!=-1) {
                KFs_update[i]=true;
                cluster_update[index]=true;
                KFs_[i].update(cloud_xy->points[index],time,clus_msg->clusters[index]);
                KFs_[i].kmeans_time_=0;
            }
        }

        std::vector<std::vector<float> > dists;
        int dist_size=0,dist_size_size=0;
        Eigen::MatrixXd cost_matrix = getCloudCost(*cloud_xy,KFs_,dist_size, dist_size_size,clu_dis_thres,cluster_update,KFs_update);
        eigenMat2VecVec(cost_matrix,dists);
        std::vector<std::vector<int> > matches;
        std::vector<int> u_track,u_cluster;
        //进行匹配
        linear_assignment(dists, dist_size, dist_size_size,clu_match_thres,matches,u_cluster,u_track);
        std::vector<KUData> again_clus(pcs.size());
        std::vector<bool> is_kmeans(KFs_.size(),false);

        for (auto index:u_track) {
            if (KFs_[index].last_time>1.5||KFs_update[index]||KFs_[index].kmeans_time_>15||KFs_[index].detect_history.size()==0) continue;
            Kalman_filter_plus kf=KFs_[index];
            int i=0;
            for(;i<pcs.size();i++) {
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
                        KFs_[match[1]].update(again_clus[match[0]].kmeans_pc.points[new_match[0]],time);
                        KFs_[match[1]].kmeans_time_++;
                    }else {
                        KFs_[again_clus[match[0]].utrack_idx[new_match[1]-1]].update(again_clus[match[0]].kmeans_pc.points[new_match[0]],time);
                        KFs_[again_clus[match[0]].utrack_idx[new_match[1]-1]].kmeans_time_++;
                    }
                }
            }else {
                KFs_[match[1]].update(cloud_xy->points[match[0]],time,clus_msg->clusters[match[0]]);
                KFs_[match[1]].kmeans_time_=0;
            }
        }

        //新增kalman
        for (auto cluster:u_cluster) {
            if (cluster_update[cluster]) continue;
            float min_dis=10000.0;
            for (int i=0;i<pcs.size();i++) {
                if (i!= cluster) {
                    float distance=Distance(cloud_xy->points[cluster],cloud_xy->points[i]);
                    if (distance<min_dis) {
                        min_dis=distance;
                    }
                }
            }
            for (int i=0;i<KFs_.size();i++) {
                float distance=sqrt(pow(cloud_xy->points[cluster].x-KFs_[i].predict_point.x,2)+pow(cloud_xy->points[cluster].y-KFs_[i].predict_point.y,2));
                if (distance<min_dis) {
                    min_dis=distance;
                }
            }
            if (min_dis>clu_min_dis_thres) {
                Kalman_filter_plus kf(cloud_xy->points[cluster], time,reinterpret_cast<rclcpp::Node*>(this),clus_msg->clusters[cluster]);//time为最后更新时间
                KFs_.push_back(kf);
            }
        }
        checkKFs(KFs_);
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
            if (kf.last_time>1.5||kf.detect_history.size()==0||KFs_[index].kmeans_time_>15) continue;
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

    void KalmanFilter::clusTimeSynC(const sensor_msgs::msg::PointCloud2::SharedPtr msg1, const interfaces::msg::ClusterTarget::SharedPtr msg2) {
        rclcpp::Time time = msg1->header.stamp;
        auto now_time = std::chrono::steady_clock::now();
        this->self_color= msg2->self_color;

        std::vector<Kalman_filter_plus> KFs_;
        mtx.lock();
        KFs_= KFs;
        mtx.unlock();

        clearOutPut();
        auto points = reinterpret_cast<const ClusterPoint*>(msg1->data.data());
        if (msg1->row_step==0) return;
        std::vector<open3d::geometry::PointCloud> pcs(points[0].sum_label);

        for (size_t i = 0; i < msg1->height * msg1->width; ++i){
            Eigen::Vector3d pt(points[i].x, points[i].y, points[i].z);
            int cluster_id=points[i].clustered_label;
            pcs[cluster_id].points_.push_back(pt);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZI>);
        for(int i=0;i<pcs.size();i++){
            pcl::PointXYZI point_xy;
            point_xy.x = pcs[i].GetCenter()[0];
            point_xy.y = pcs[i].GetCenter()[1];
            point_xy.z = pcs[i].GetCenter()[2];
            point_xy.intensity = 0;
            cloud_xy->points.push_back(point_xy);
        }
        if(cloud_xy->points.size() == 0)return;

        pcl::PointCloud<pcl::PointXYZ> kmeans_out;
        updateKFs(KFs_,cloud_xy,pcs,msg2,kmeans_out,time);

        sensor_msgs::msg::PointCloud2 output_kms;
        pcl::toROSMsg(kmeans_out, output_kms);
        output_kms.header.frame_id = "rm_frame";
        output_kms.header.stamp = msg1->header.stamp;
        pub_kmeans->publish(output_kms);

        std::vector<int> remove_KFs_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int i = KFs_.size() - 1; i >= 0; i--){
            KFs_[i].forword_predict();
            if (initornot(fortress_,KFs_[i].predict_point,4)==1) {
                if(KFs_[i].last_time>5.0)
                    remove_KFs_.push_back(i);
            }else {
                if(KFs_[i].last_time>2.5)
                    remove_KFs_.push_back(i);
            }
        }
        sort(remove_KFs_.begin(),remove_KFs_.end(),std::greater<>());
        for (auto index:remove_KFs_)
            KFs_.erase(KFs_.begin() + index);

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
        eigenMat2VecVec(cost_confMatrix, cost_confMatrix_vec);
        linear_assignment(cost_confMatrix_vec, num_strack, num_cls,0.95, matches_cls, u_strack, u_cls);

        for (auto match : matches_cls) {
            if(match[1]>=5){//蓝色
                history_cost[1][match[1]-5]++;
                vis_kal_maker(1,vis_array,1,match[1]-5,KFs_[match[0]].predict_point.x,KFs_[match[0]].predict_point.y,KFs_[match[0]].detect_history.size(),history_cost[1][match[1]-5]);
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
        RCLCPP_WARN(this->get_logger(), "clusTimeSync time is %f ms", dur_time);
        span++;
        span=span%2;
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
        std::vector<cv::Point3d> points1,points2;
        getRect2d(pcs,transform,rects1,points1,1);
        getRect2d(pcs,transform,rects2,points2,2);

        for(int i=0;i<pcs.size();i++){
            pcl::PointXYZI point_xy;
            auto point = pcs[i].GetCenter();
            point_xy.x = point[0];
            point_xy.y = point[1];
            point_xy.z = point[2];
            point_xy.intensity = 0;
            cloud_xy->points.push_back(point_xy);
        }
        sensor_msgs::msg::PointCloud2 cluster_out;
        pcl::toROSMsg(*cloud_xy, cluster_out);
        cluster_out.header.frame_id = "rm_frame";
        cluster_out.header.stamp = msg1->header.stamp;
        pub_cluster->publish(cluster_out);

        interfaces::msg::ClusterRect rects_msg;
        rects_msg.header.stamp = time;
        int sum=0;
        for (int i=0;i<rects1.size();i++) {
            interfaces::msg::Rect rect1,rect2;
            rect1.cx=points1[i].x;
            rect1.cy=points1[i].y;
            rect1.x=pcs[i].GetCenter()[0];
            rect1.y=pcs[i].GetCenter()[1];
            rect1.z=pcs[i].GetCenter()[2];
            rect1.x1=rects1[i].x;
            rect1.y1=rects1[i].y;
            rect1.x2=rects1[i].x+rects1[i].width;
            rect1.y2=rects1[i].y+rects1[i].height;
            rect1.cluster_id=i;

            rect2.cx=points2[i].x;
            rect2.cy=points2[i].y;
            rect2.x=pcs[i].GetCenter()[0];
            rect2.y=pcs[i].GetCenter()[1];
            rect2.z=pcs[i].GetCenter()[2];
            rect2.x1=rects2[i].x;
            rect2.y1=rects2[i].y;
            rect2.x2=rects2[i].x+rects2[i].width;
            rect2.y2=rects2[i].y+rects2[i].height;
            rect2.cluster_id=i;

            rects_msg.rects1.push_back(rect1);
            rects_msg.rects2.push_back(rect2);

            sum+=pcs[i].points_.size();
        }

        sensor_msgs::msg::PointCloud2 pointcloud;
        sensor_msgs::PointCloud2Modifier(pointcloud).setPointCloud2Fields(
            5,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "clustered_label", 1, sensor_msgs::msg::PointField::INT32,
            "sum_label", 1, sensor_msgs::msg::PointField::INT32);
        pointcloud.header.frame_id.assign("rm_frame");
        pointcloud.header.stamp = time;
        pointcloud.height = 1;
        pointcloud.width = sum;
        pointcloud.row_step = pointcloud.width * pointcloud.point_step;
        pointcloud.is_bigendian = false;
        pointcloud.is_dense = true;
        pointcloud.data.resize(pointcloud.row_step);
        auto iter_x = sensor_msgs::PointCloud2Iterator<float>(pointcloud, "x");
        auto iter_y = sensor_msgs::PointCloud2Iterator<float>(pointcloud, "y");
        auto iter_z = sensor_msgs::PointCloud2Iterator<float>(pointcloud, "z");
        auto iter_cl = sensor_msgs::PointCloud2Iterator<int>(pointcloud, "clustered_label");
        auto iter_su = sensor_msgs::PointCloud2Iterator<int>(pointcloud, "sum_label");

        for (int i=0;i<pcs.size();i++) {
            for (auto point: pcs[i].points_) {
                *iter_x = point[0];
                *iter_y = point[1];
                *iter_z = point[2];
                *iter_cl = i;
                *iter_su = pcs.size();
                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_cl;
                ++iter_su;
            }
        }
        filtered_pc_pub_->publish(pointcloud);
        rects_pub_->publish(rects_msg);

        auto end_time = std::chrono::steady_clock::now();
        float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
        RCLCPP_WARN(this->get_logger(), "PCTimeSync time is %f ms", dur_time);
    }

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
        std::vector<cv::Point3d> points;
        getRect2d(pcs,transform,rects,points,1);
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
        }
        checkKFs(KFs_);
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
        checkKFs(KFs_);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        //过滤出红蓝方有位置信息的检测点
        //TODO：需要对己方为蓝方的时候作处理

        std::vector<Kalman_filter_plus> blue_KFs_,red_KFs_;
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
        lidar_detect_pub->publish(detect_res);
        pub_vis->publish(vis_array);

        mtx.lock();
        KFs= KFs_;
        mtx.unlock();

        auto end_time = std::chrono::steady_clock::now();
        float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
        RCLCPP_WARN(this->get_logger(), "Kalman Callback time is %f ms", dur_time);
    }
}
RCLCPP_COMPONENTS_REGISTER_NODE(upc_radar::KalmanFilter)