#include<lidar_registration.h>

namespace upc_radar {
    LidarRegistration::LidarRegistration(const rclcpp::NodeOptions& node_options) : Node("LidarRegistration", node_options) {
        
        RCLCPP_WARN(this->get_logger(), "LidarRegistration node start");

        declare_parameter<double>("cost_thres", 0.3);
        
        use_saved_T= declare_parameter<bool>("use_saved_T", false);
        std::string map_pcd_file = declare_parameter<std::string>("map_path", "resource/RM2024.pcd");
        pub_map_grid_size =declare_parameter<double>("pub_map_grid_size", 0.01);
        al_map_grid_size = declare_parameter<double>("al_map_grid_size", 0.2);
        pc_grid_size = declare_parameter<double>("pc_grid_size", 0.15);
        std::cout<<map_pcd_file<<std::endl;

        // 从pcd读取场地点云
        target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile(map_pcd_file, *target_cloud_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load %s", map_pcd_file.c_str());
            return;
        }
    
        //转换pcd点云坐标系
        Eigen::Matrix4f Tran;
        Eigen::Matrix4f Tran1;
        //Tran1 cloudcompare将场地转正
        Tran1<< 0.941037,0.338304,0.000000,-1.023722,
                -0.338304,0.941037,0.000000,-0.499722,
                0.000000,0.000000,1.000000,0.000000,
                0.000000,0.000000,0.000000,1.000000;

        //Tran的逆 cloudcompare更改坐标系 实验室场地
        // Tran<< -0.117343 ,0.993080 ,0.004741 ,-1.625149,
        //         -0.993091 ,-0.117341 ,-0.000560 ,2.972910,
        //         0.000000 ,-0.004774 ,0.999989 ,-0.268606,
        //         0.000000 ,0.000000 ,0.000000 ,1.000000;

        //RM2025
        Tran<< 0.000000, 1.000000, 0.000000, -7.502200,
                -1.000000, 0.000000, 0.000000, 14.005900,
                0.000000, 0.000000, 1.000000, 0.000000,
                0.000000, 0.000000, 0.000000, 1.000000;

        // pcl::transformPointCloud(*target_cloud_, *target_cloud_, Tran1);
        pcl::transformPointCloud(*target_cloud_, *target_cloud_, Tran.inverse());//实验室场地及RM2025用这个

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10, std::bind(&LidarRegistration::callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/map", 100);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_map;
        voxelgrid_map.setLeafSize(pub_map_grid_size, pub_map_grid_size, pub_map_grid_size);
        voxelgrid_map.setInputCloud(target_cloud_);
        voxelgrid_map.filter(*downsampled);
        timer_ = this->create_wall_timer(std::chrono::seconds(5), [this,downsampled]() {
            sensor_msgs::msg::PointCloud2 target_msg;
            pcl::toROSMsg(*downsampled, target_msg);
            target_msg.header.frame_id = "rm_frame";
            target_msg.header.stamp = this->get_clock()->now();
            publisher_->publish(target_msg);
        });

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    void LidarRegistration::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        
        if(use_saved_T){
            std::ifstream fin("resource/lidar2world.txt");
            if (!fin) {
                RCLCPP_ERROR(this->get_logger(),"file cant open!!!");
                RCLCPP_ERROR(this->get_logger(),"use mautually!!!");
                use_saved_T=false;
            }else{
                char buf[1024]={0};
                int num=0;
                std::vector<double> T_temp;
                while(fin>>buf){
                    T_temp.push_back(atof(buf));
                    num++;
                    if(num==16) break;
                }
                if(num==16){
                    manual_aligned_=true;
                    accumulate_time=1;
                    auto_aligned_=true;
                    for(int i=0;i<4;i++)
                        for(int j=0;j<4;j++)
                            T(i,j)=T_temp[i*4+j];
                }else{
                    RCLCPP_ERROR(this->get_logger(),"file broken!!!");
                    RCLCPP_ERROR(this->get_logger(),"use mautually!!!");
                    use_saved_T=false;
                }
            }
            fin.close();
        }
        cost_thres = get_parameter("cost_thres").as_double();

        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *source_cloud);
        if(accumulated_clouds_.size() < accumulate_time){
            accumulated_clouds_.push_back(source_cloud);
            return;
        }
        else{
            accumulated_clouds_.erase(accumulated_clouds_.begin());
            accumulated_clouds_.push_back(source_cloud);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        for(auto accumulated_cloud : accumulated_clouds_){
            for (const auto& point : *accumulated_cloud){
                //rm25
                // if(point.x > 3 && point.x < 30 && point.y > -6 && point.y < 12&&point.z<7)
                //     final_cloud->push_back(point);
                //rm24
                if(point.x > 5 && point.x < 30 && point.y > -10 && point.y < 8&&point.z<7)
                    final_cloud->push_back(point);
                //lab    
                // if(point.z<1)
                //     final_cloud->push_back(point);
            }
        }       

            //转换点云格式
        if(!manual_aligned_){
            auto start=std::chrono::system_clock::now();

            std::shared_ptr<open3d::geometry::PointCloud> pc2align(new open3d::geometry::PointCloud);
            std::shared_ptr<open3d::geometry::PointCloud> map_pc(new open3d::geometry::PointCloud);

            for(size_t i=0;i<final_cloud->points.size();i++)
                pc2align->points_.push_back(Eigen::Vector3d(final_cloud->points[i].x,final_cloud->points[i].y,final_cloud->points[i].z));
                        
            for(size_t i=0;i<target_cloud_->points.size();i++)
                map_pc->points_.push_back(Eigen::Vector3d(target_cloud_->points[i].x,target_cloud_->points[i].y,target_cloud_->points[i].z));
            
            auto end=std::chrono::system_clock::now();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "transform pointcloud time: %fms", std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000.0);
            
            //初始手动匹配
            RCLCPP_WARN(this->get_logger(), "manual_trans start");
            
            T = manual_trans(pc2align, map_pc);
            manual_aligned_=true;

            RCLCPP_WARN(this->get_logger(), "manual_trans end");
        }
        if(!auto_aligned_){
            auto t1 = std::chrono::system_clock::now();
            //降采样
            pcl::PointCloud<pcl::PointXYZ>::Ptr target_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

            pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_map;
            voxelgrid_map.setLeafSize(al_map_grid_size, al_map_grid_size, al_map_grid_size);
            voxelgrid_map.setInputCloud(target_cloud_);
            voxelgrid_map.filter(*downsampled);
            *target_map_cloud = *downsampled;
            
            pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_real;
            voxelgrid_real.setLeafSize(pc_grid_size, pc_grid_size, pc_grid_size);
            voxelgrid_real.setInputCloud(final_cloud);
            voxelgrid_real.filter(*downsampled);
            *final_cloud = *downsampled;

            //应用初始变换
            pcl::transformPointCloud(*final_cloud, *final_cloud, T);

            RCLCPP_WARN(this->get_logger(), "auto_trans start");
            boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> registration(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());

            std::cout<<"number of map cloud points "<<target_map_cloud->points.size()<<std::endl;
            std::cout<<"number of real cloud points "<<final_cloud->points.size()<<std::endl;

            registration->setInputTarget(target_map_cloud);
            registration->setInputSource(final_cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
            registration->align(*aligned);

            RCLCPP_WARN(this->get_logger(), "calib result : %f", registration->getFitnessScore());
                        
            if(registration->getFitnessScore()<cost_thres){
                auto_aligned_ = true;

                Eigen::Matrix4f transform;
                transform= registration->getFinalTransformation();
                T=transform.cast<double>() * T;
                std::ofstream fout("resource/lidar2world.txt");  
	            if(!fout) 
                    RCLCPP_ERROR(this->get_logger(),"file cant open!!!");
	            else {
                    for(int i=0;i<4;i++){
                        for(int j=0;j<4;j++)
                            fout<<T(i,j)<<" ";
                        fout<<std::endl;
                    }
                    fout<<"------------------"<<std::endl;
		            fout.close();           
	            }
                RCLCPP_WARN(this->get_logger(), "auto_trans end");
            }

            auto t2 = std::chrono::system_clock::now();
            std::cout << "auto_trans time :" << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()/1000 << "[s]" << std::endl;
        }
        publishTF(T);
    }

    Eigen::Matrix4d LidarRegistration::manual_trans(std::shared_ptr<open3d::geometry::PointCloud> pc2align, std::shared_ptr<open3d::geometry::PointCloud> mesh_pc){
        /// @brief 手动配准获取初始变换矩阵
        auto picked_pc = select_points(pc2align);
        auto picked_mesh = select_points(mesh_pc);
        assert(picked_pc.size() >= 3 && picked_mesh.size() >= 3);
        assert(picked_pc.size() == picked_mesh.size());

        std::vector<Eigen::Vector2i> correspondences;
        for (size_t i = 0; i < picked_pc.size(); ++i)
            correspondences.emplace_back(picked_pc[i], picked_mesh[i]);

        open3d::pipelines::registration::TransformationEstimationPointToPoint pointToPoint;
        //返回的是将实时点云转换到地图点云的变换
        return pointToPoint.ComputeTransformation(*pc2align, *mesh_pc, correspondences);
    }

    std::vector<size_t> LidarRegistration::select_points(std::shared_ptr<const open3d::geometry::PointCloud> pcd){
        RCLCPP_INFO(this->get_logger(), "Select points...");
        RCLCPP_INFO(this->get_logger(), "  Use [shift + left click] to pick points.");
        RCLCPP_INFO(this->get_logger(), "  Use [shift + right click] to undo point picking.");
        RCLCPP_INFO(this->get_logger(), "  After picking points, press 'Q' to close the window.");

        open3d::visualization::VisualizerWithEditing vis_;
        vis_.CreateVisualizerWindow("Select Points", 1920, 1080);
        vis_.AddGeometry(pcd);
        vis_.Run();
        vis_.DestroyVisualizerWindow();
        return vis_.GetPickedPoints();
    }

    void LidarRegistration::publishTF(const Eigen::Matrix4d& transform){
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "rm_frame";
        transform_stamped.child_frame_id = "livox_frame";
        transform_stamped.transform.translation.x = transform(0, 3);
        transform_stamped.transform.translation.y = transform(1, 3);
        transform_stamped.transform.translation.z = transform(2, 3);
        Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
        Eigen::Quaterniond q(rotation);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(transform_stamped);
    }
   
} // namespace upc_radar
RCLCPP_COMPONENTS_REGISTER_NODE(upc_radar::LidarRegistration)