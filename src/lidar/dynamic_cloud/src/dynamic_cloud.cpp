#include "dynamic_cloud.h"

namespace upc_radar{
    DynamicCloud::DynamicCloud(const rclcpp::NodeOptions& node_options):rclcpp::Node("dynamic_cloud_node",node_options),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_){
        RCLCPP_WARN(this->get_logger(), "Dynamic_cloud Node start");
        
        declare_parameter<double>("sor.thres", 1.0);
        declare_parameter<int>("sor.MeanK", 3);
        declare_parameter<int>("dynamic.accumulate", 3);
        remove_outlier=declare_parameter("sor.remove_outlier", true);

        std::string filter_mode = declare_parameter("dynamic.filter_mode", "pcd"); // option: mesh, pcd ///无
        if (filter_mode == "mesh")
            mesh_filter_mode = true;
        else if (filter_mode == "pcd")
            mesh_filter_mode = false;
        else {
            RCLCPP_WARN(get_logger(), "Unknown filter_mode: %s", filter_mode.c_str());
            mesh_filter_mode = false;
        }

        if(mesh_filter_mode){
            prepare_meshes();
            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10, std::bind(&DynamicCloud::callback_mesh, this, std::placeholders::_1));
        }else{
            declare_parameter<double>("pc.dis_thres", 0.1);
            declare_parameter<int>("pc.thread", 12);
            prepare_pcd();
            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10, std::bind(&DynamicCloud::callback_pc, this, std::placeholders::_1));
        }
        
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_dynamic", 10);
    }

    bool DynamicCloud::get_transform(std::string frame_id,Eigen::Affine3f &transform){
        geometry_msgs::msg::TransformStamped transform_stamped;
        try{
            transform_stamped=tf_buffer_.lookupTransform("rm_frame", frame_id, tf2::TimePointZero);
        }catch(tf2::TransformException &ex){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Transform error: %s",ex.what());
            return false;
        }
        trans=tf2::transformToEigen(transform_stamped);
        transform.translation() << transform_stamped.transform.translation.x,
                               transform_stamped.transform.translation.y,
                               transform_stamped.transform.translation.z;
        Eigen::Quaternionf rotation(
            transform_stamped.transform.rotation.w,
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z);
        transform.rotate(rotation);
        return true;
    }

    void DynamicCloud::prepare_pcd(){
        auto pcd = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("resource/RM2024.pcd", *pcd) == -1){
            PCL_ERROR("Couldn't read file map.pcd \n");
            return;
        }
        Eigen::Matrix4f Tran;
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

        pcl::transformPointCloud(*pcd, *pcd, Tran.inverse());//实验室场地及RM2025用这个
        //下采样
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(pcd);
        sor.setLeafSize(0.1f, 0.1f, 0.1f);
        auto result = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*result);
        map_cloud = result;
        kd_Tree.setInputCloud(map_cloud);
    }
    
    void DynamicCloud::callback_pc(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        auto time=rclcpp::Clock().now();
        auto now_time = std::chrono::steady_clock::now();
        dis_thres=get_parameter("pc.dis_thres").as_double();
        thread_num=get_parameter("pc.thread").as_int();
        accumulate_time=get_parameter("dynamic.accumulate").as_int();

        auto receive_cloud = pcl::PointCloud<pcl::PointXYZ>();
        // int num=0;
        auto tran_points = reinterpret_cast<const upc_radar::LivoxPointXyzrtl*>(msg->data.data());
        for(size_t i=0;i<msg->width;i++){
            uint8_t temp=msg->data[i*msg->point_step+16];
            if(uint8_t(temp<<6)<64){
                receive_cloud.points.push_back(pcl::PointXYZ(tran_points[i].x,tran_points[i].y,tran_points[i].z));
                // num++;
            }   
        }
        // std::cout<<"origin num:"<<msg->width<<std::endl;
        // std::cout<<"num:"<<num<<std::endl;
        receive_cloud.width = receive_cloud.points.size();
        receive_cloud.height = 1;

        auto transform = Eigen::Affine3f::Identity();
        if(!get_transform(msg->header.frame_id,transform))return;

        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud(receive_cloud, transformed_cloud, transform);

        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
        get_filtered_cloud(transformed_cloud,filtered_cloud);

        pcl::PointCloud<pcl::PointXYZ> dynamic_pointcloud;
        GetDynamicCloud(filtered_cloud,dynamic_pointcloud,dis_thres,thread_num);

        accumulated_clouds_.push_back(dynamic_pointcloud.makeShared());
        while (accumulated_clouds_.size()>accumulate_time)
            accumulated_clouds_.erase(accumulated_clouds_.begin());

        pcl::PointCloud<pcl::PointXYZ> accumulated_cloud;
        for(auto it = accumulated_clouds_.begin(); it != accumulated_clouds_.end(); ++it){
            accumulated_cloud += **it;
        }

        if(remove_outlier){
            double thres=get_parameter("sor.thres").as_double();
            int MeanK=get_parameter("sor.MeanK").as_int();
            accumulated_cloud=removeOutlier(accumulated_cloud,MeanK,thres);
        }

        sensor_msgs::msg::PointCloud2 output;
        accumulated_cloud.header.frame_id = "rm_frame";
        pcl::toROSMsg(accumulated_cloud, output);
        output.header.frame_id = "rm_frame";
        output.header.stamp = time;
        // output.header.stamp = msg->header.stamp;
        pub_->publish(output);
        auto end_time = std::chrono::steady_clock::now();
        float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
        RCLCPP_WARN(this->get_logger(), "Dynamic Callback time is %f ms", dur_time);
    }

    void DynamicCloud::GetDynamicCloud(pcl::PointCloud<pcl::PointXYZ> &input_cloud,pcl::PointCloud<pcl::PointXYZ> &output_cloud,float threshold,int thread_num){
        int K=1;
        std::vector<std::thread> threads;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds(thread_num);
        auto start=std::chrono::system_clock::now();
        int cloud_size=input_cloud.points.size();
        int step=cloud_size/thread_num;
        for(int i=0;i<thread_num;i++){
            threads.push_back(std::thread([i,step,cloud_size,&clouds,&input_cloud,this,threshold,K](){
                for(int j=i*step;j<(i+1)*step;j++){
                    std::vector<int> pointIdxNKNSearch(K);
                    std::vector<float> pointNKNSquaredDistance(K);
                    if(kd_Tree.nearestKSearch(input_cloud.points[j],K,pointIdxNKNSearch,pointNKNSquaredDistance)>0){
                        if(pointNKNSquaredDistance[0]>threshold){
                            clouds[i].points.push_back(input_cloud.points[j]);
                        }
                    }
                }
            }));            
        }
        for(auto &t:threads){
            t.join();
        }
        for(auto &cloud:clouds){
            output_cloud+=cloud;
        }
        auto end=std::chrono::system_clock::now();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "kd_tree search time: %f", std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000.0);
    }

    void DynamicCloud::callback_mesh(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        auto now_time = std::chrono::steady_clock::now();

        auto time=rclcpp::Clock().now();
        accumulate_time=get_parameter("dynamic.accumulate").as_int();
        auto transform = Eigen::Affine3f::Identity();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> trans_pc;
        pcl::PointCloud<pcl::PointXYZ> receive_cloud;

        if(!get_transform("livox_frame",transform)) return;
        if(!is_grid_prepared){
            prepare_voxel_grid();
            is_grid_prepared=true;
        }

        pcl::fromROSMsg(*msg, *cloud);
        pcl::transformPointCloud(*cloud,trans_pc,transform);

        for (int i=0;i<trans_pc.points.size();i++) {
            Eigen::Vector3d pt(trans_pc.points[i].x, trans_pc.points[i].y, trans_pc.points[i].z);
            if(!voxel_grid.is_occupied(pt)&&!(pt[1]>12&&pt[2]>1.3&&pt[0]<14)&&!(pt[1]>3&&pt[2]>1.3&&pt[0]>14))
                receive_cloud.points.push_back(pcl::PointXYZ(pt[0],pt[1],pt[2]));
        }
        pc_buffer.emplace_back(std::move(receive_cloud));
        while (pc_buffer.size()>accumulate_time)
            pc_buffer.pop_front();

        pcl::PointCloud<pcl::PointXYZ> accumulated_cloud;
        for(auto it=pc_buffer.begin();it!=pc_buffer.end();++it)
            accumulated_cloud+=*it;

        if(remove_outlier){
            double thres=get_parameter("sor.thres").as_double();
            int MeanK=get_parameter("sor.MeanK").as_int();
            accumulated_cloud=removeOutlier(accumulated_cloud,MeanK,thres);
        }

        sensor_msgs::msg::PointCloud2 output;
        accumulated_cloud.header.frame_id = "rm_frame";
        pcl::toROSMsg(accumulated_cloud, output);
        output.header.frame_id = "rm_frame";
        output.header.stamp = time;
        pub_->publish(output);
        auto end_time = std::chrono::steady_clock::now();
        float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
        RCLCPP_WARN(this->get_logger(), "Dynamic Callback time is %f ms", dur_time);
    }

    void DynamicCloud::prepare_meshes(){
        declare_parameter("mesh.mesh_ori", "24_bg2align_fix1.stl");
        declare_parameter("mesh.mesh_filter", "24_bg2filter_fix1.stl");
        declare_parameter("mesh.init_translate", std::vector<double> { 0., 0., 0. });

        auto init_translate_v = get_parameter("mesh.init_translate").as_double_array();
        Eigen::Vector3d init_translate(init_translate_v[0], init_translate_v[1], init_translate_v[2]);

        mesh_ori = open3d::io::CreateMeshFromFile("resource/"+get_parameter("mesh.mesh_ori").as_string());
        if (!mesh_ori) {
            RCLCPP_ERROR(get_logger(), "mesh_ori is null");
            return;
        }
        mesh_ori->ComputeVertexNormals();
        mesh_ori->ComputeTriangleNormals();
        mesh_ori->Scale(0.001, Eigen::Vector3d::Zero());
        mesh_ori->Translate(init_translate);


        std::string mesh_filename ="resource/"+get_parameter("mesh.mesh_filter").as_string();
        RCLCPP_INFO(get_logger(), "mesh_filter: %s", mesh_filename.c_str());
        mesh_filter = open3d::io::CreateMeshFromFile(mesh_filename);
        if (!mesh_filter) {
            RCLCPP_ERROR(get_logger(), "mesh_filter is null");
            return;
        }
        mesh_filter->Scale(0.001, Eigen::Vector3d::Zero());
        mesh_filter->Translate(init_translate);
        
        RCLCPP_INFO(get_logger(), "meshes prepared");
    }

    void DynamicCloud::prepare_voxel_grid()
    {
        static auto voxel_size = declare_parameter("voxel_grid.voxel_size", 0.06);
        static auto dilate_size = declare_parameter("voxel_grid.dilate_size", 1);
        static auto size_min = declare_parameter("voxel_grid.size.min", std::vector<double> { 0.150, 0.150, 0. });
        static auto size_max = declare_parameter("voxel_grid.size.max", std::vector<double> { 27.850, 14.850, 1.500 });
        static auto occupy_expand = declare_parameter("voxel_grid.occupy_expand", 0.17);

        voxel_grid.initialize({ { size_min[0], size_min[1], size_min[2] },
            { size_max[0], size_max[1], size_max[2] },
            voxel_size });

        voxel_grid.occupy_by_mesh_filter(mesh_filter, occupy_expand);//经过部分膨胀过后的地图stl
        // else
        //     voxel_grid.occupy_by_pc(pc_filter, dilate_size);
        voxel_grid.occupy_by_mesh(mesh_ori,trans*Eigen::Vector3d::Zero(), dilate_size);
    }

}RCLCPP_COMPONENTS_REGISTER_NODE(upc_radar::DynamicCloud)