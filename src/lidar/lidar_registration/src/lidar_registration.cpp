#include<lidar_registration.h>

namespace upc_radar {
    Lidar_Registration::Lidar_Registration(const rclcpp::NodeOptions& node_options) : Node("Lidar_Registration", node_options) 
    {
        RCLCPP_INFO(this->get_logger(), "lidar_registration start");
        std::string target_pcd_file = "resource/remove_rooftop.pcd";

        // 从pcd读取场地点云
        target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile(target_pcd_file, *target_cloud_)) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load %s", target_pcd_file.c_str());
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

        //Tran的逆 cloudcompare更改坐标系
        Tran<<  0.007337952964,-0.999863266945,-0.014817589894,3.776853322983,
                0.999883294106,0.007535010576,-0.013287514448,-2.621445655823,
                0.013397343457,-0.014718348160,0.999801933765,-0.438632011414,
                0.000000000000,0.000000000000,0.000000000000,1.000000000000;

        pcl::transformPointCloud(*target_cloud_, *target_cloud_, Tran1);
        pcl::transformPointCloud(*target_cloud_, *target_cloud_, Tran.inverse());

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10, std::bind(&Lidar_Registration::callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/map", 100);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_map;
        voxelgrid_map.setLeafSize(0.01f, 0.01f, 0.01f);
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

    void Lidar_Registration::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *source_cloud);
        if(accumulated_clouds_.size() < accumulate_time)
        {
            accumulated_clouds_.push_back(source_cloud);
            return;
        }
        else
        {
            accumulated_clouds_.erase(accumulated_clouds_.begin());
            accumulated_clouds_.push_back(source_cloud);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        for(auto accumulated_cloud : accumulated_clouds_)
        {
            for (const auto& point : *accumulated_cloud)
            {
                // if(point.x > 5 && point.x < 30 && point.y > -10 && point.y < 8&&point.z<7)
                //     final_cloud->push_back(point);    
                if(point.z<1)
                    final_cloud->push_back(point);
            }
        }       

            //转换点云格式
        if(!manual_aligned_)
        {
            auto start=std::chrono::system_clock::now();

            std::shared_ptr<open3d::geometry::PointCloud> pc2align(new open3d::geometry::PointCloud);
            std::shared_ptr<open3d::geometry::PointCloud> map_pc(new open3d::geometry::PointCloud);

            for(size_t i=0;i<final_cloud->points.size();i++)
                pc2align->points_.push_back(Eigen::Vector3d(final_cloud->points[i].x,final_cloud->points[i].y,final_cloud->points[i].z));
                        
            for(size_t i=0;i<target_cloud_->points.size();i++)
                map_pc->points_.push_back(Eigen::Vector3d(target_cloud_->points[i].x,target_cloud_->points[i].y,target_cloud_->points[i].z));
            
            auto end=std::chrono::system_clock::now();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "transform pointcloud time: %f", std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000.0);
            
            //初始手动匹配
            RCLCPP_WARN(this->get_logger(), "manual_trans start");
            
            T = manual_trans(pc2align, map_pc);
            manual_aligned_=true;

            RCLCPP_WARN(this->get_logger(), "manual_trans end");
        }
        else if(!auto_aligned_)
        {
            auto t1 = std::chrono::system_clock::now();
            //降采样
            pcl::PointCloud<pcl::PointXYZ>::Ptr target_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

            pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_map;
            voxelgrid_map.setLeafSize(0.2f, 0.2f, 0.2f);
            voxelgrid_map.setInputCloud(target_cloud_);
            voxelgrid_map.filter(*downsampled);
            *target_map_cloud = *downsampled;
            
            pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_real;
            voxelgrid_real.setLeafSize(0.15f, 0.15f, 0.15f);
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
                        
            if(registration->getFitnessScore()<0.3)
            {
                auto_aligned_ = true;

                Eigen::Matrix4f transform;
                transform= registration->getFinalTransformation();
                T=transform.cast<double>() * T;
                RCLCPP_WARN(this->get_logger(), "auto_trans end");
            }

            auto t2 = std::chrono::system_clock::now();
            std::cout << "auto_trans time   : " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()/1000 << "[s]" << std::endl;
        }
        publishTF(T);
    }

    Eigen::Matrix4d Lidar_Registration::manual_trans(std::shared_ptr<open3d::geometry::PointCloud> pc2align, std::shared_ptr<open3d::geometry::PointCloud> mesh_pc)
    {
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

    std::vector<size_t> Lidar_Registration::select_points(std::shared_ptr<const open3d::geometry::PointCloud> pcd)
    {
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

    void Lidar_Registration::publishTF(const Eigen::Matrix4d& transform)
    {
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
RCLCPP_COMPONENTS_REGISTER_NODE(upc_radar::Lidar_Registration)