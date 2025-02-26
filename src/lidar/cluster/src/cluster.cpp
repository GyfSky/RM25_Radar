#include "cluster.h"

namespace upc_radar{
    Cluster::Cluster(const rclcpp::NodeOptions& node_options): Node("cluster_node", node_options),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_){
        RCLCPP_WARN(this->get_logger(), "cluster_node start");

        declare_parameter<double>("cluster.min_pts_k",1800.0);
        declare_parameter<int>("cluster.min_pts",6);
        declare_parameter<double>("cluster.eps",0.3);
        declare_parameter<std::string>("cluster.mode","nor");
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_dynamic", 10, std::bind(&Cluster::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", 10);
    }

    void Cluster::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) {return;}

        open3d::geometry::PointCloud in_cloud;
        for(size_t i=0;i<cloud->points.size();i++)
            //全部转到二维
            in_cloud.points_.push_back(Eigen::Vector3d(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z));
        
        double min_points_k=get_parameter("cluster.min_pts_k").as_double();
        size_t min_points=get_parameter("cluster.min_pts").as_int();
        double eps=get_parameter("cluster.eps").as_double();

        geometry_msgs::msg::TransformStamped transform_stamped;
        try{
            transform_stamped = tf_buffer_.lookupTransform("rm_frame", msg->header.frame_id, tf2::TimePointZero);
        }catch (tf2::TransformException &ex){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Transform error: %s", ex.what());
            return;
        }

        std::vector<int> labels;
        Eigen::Vector3d zero_pos(transform_stamped.transform.translation.x,transform_stamped.transform.translation.y,transform_stamped.transform.translation.z);
        
        std::string cluster_mode=get_parameter("cluster.mode").as_string();
        if (cluster_mode == "nor")
            labels=NormalDBSCAN(in_cloud,eps,min_points);
        else if (cluster_mode == "dif")
            labels=DifferingDBSCAN(in_cloud,zero_pos,eps,min_points_k);
        else{
            RCLCPP_WARN(get_logger(), "Unknown cluster_mode: %s", cluster_mode.c_str());
            labels=NormalDBSCAN(in_cloud,eps,min_points);
        }

        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> pcs;
        open3d::geometry::PointCloud pc_noise;
        std::vector<Eigen::Vector3d> grav;
        int max_l = *std::max_element(labels.begin(), labels.end());

        for (int i = 0; i <= max_l; i++)
            pcs.emplace_back(std::make_shared<open3d::geometry::PointCloud>());
        for (size_t i = 0; i < in_cloud.points_.size(); i++)
            if(labels[i] >= 0)
                pcs[labels[i]]->points_.push_back(in_cloud.points_[i]);
            else
                pc_noise.points_.push_back(in_cloud.points_[i]);
        
        pcl::PointXYZ move_point;
        pcl::PointCloud<pcl::PointXYZ> *out_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (int i = 0; i <= max_l; i++) {
            move_point.x=pcs[i]->GetCenter()[0];
            move_point.y=pcs[i]->GetCenter()[1];
            move_point.z=pcs[i]->GetCenter()[2];
            out_cloud->points.push_back(move_point);
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*out_cloud, output);
        output.header.frame_id = "rm_frame";
        output.header.stamp = msg->header.stamp;
        pub_->publish(output);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        RCLCPP_WARN(this->get_logger(), "Cluster callback time: %f ms", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()/1000.0);
    }

    //不计入噪音点
    std::vector<int> Cluster::DifferingDBSCAN(const open3d::geometry::PointCloud &cloud,
        const Eigen::Vector3d &zero_pos,double eps,double min_points_k){

        /// DBSCAN算法 根据距离二次反比聚类 min_points
        if (cloud.points_.empty()) {
            return std::vector<int>();
        }
        open3d::geometry::KDTreeFlann kdtree(cloud);

        std::vector<std::vector<int>> nbs(cloud.points_.size());
        // #pragma omp parallel for schedule(static) num_threads(open3d::utility::EstimateMaxThreads())
        // for (int idx = 0; idx < int(cloud.points_.size()); ++idx) {
        //     std::vector<double> dists2;
        //     kdtree.SearchRadius(cloud.points_[idx], eps, nbs[idx], dists2);
        // // #pragma omp critical
        // }
        tbb::parallel_for(tbb::blocked_range<int>(0, int(cloud.points_.size())),
            [&](const tbb::blocked_range<int> &r) {
                for (int idx = r.begin(); idx < r.end(); ++idx) {
                    std::vector<double> dists2;
                    //nbs为邻域内的点，猜测记录的为索引
                    kdtree.SearchRadius(cloud.points_[idx], eps, nbs[idx], dists2);
                }
            });

        //默认标签为 -2, 未被访问过
        //为每个类分配一个标签
        //为每个点的索引添加一个聚类标签
        std::vector<int> labels(cloud.points_.size(), -2);
        int cluster_label = 0;
        for (size_t idx = 0; idx < cloud.points_.size(); ++idx) {
            // 标签已被确认, 跳过
            if (labels[idx] != -2) {
                continue;
            }
            // 令最少点数与距离的平方成反比
            //squaredNorm为距离（向量长度）的平方
            double temp =(cloud.points_[idx] - zero_pos).squaredNorm();
            // RCLCPP_INFO(this->get_logger(), "temp: %f", temp);
            size_t min_points = min_points_k / (cloud.points_[idx] - zero_pos).squaredNorm();
            // 如果邻域内点数不足, 标记为噪声
            if (nbs[idx].size() < min_points) {
                labels[idx] = -1;
                continue;
            }

            // 利用 unordered_set 去重
            //还没有添加标签的点
            BucketSet nbs_next(cloud.points_.size());
            //已经打完标签的点
            BucketSet nbs_visited(cloud.points_.size());
            //遍历当前点所确定的邻域中的点
            for (int nb : nbs[idx])
                //添加邻域中的点
                nbs_next.insert(nb);
            //添加当前点
            nbs_visited.insert(int(idx));

            //为当前点添加标签
            labels[idx] = cluster_label;
            // BFS 遍历邻域
            while (nbs_next.size > 0) {
                //将邻域中的点和当前点放在一块，即已经打完标签的点
                int nb = nbs_next.now_first;
                nbs_next.erase(nb);
                nbs_visited.insert(nb);

                // 噪声转为聚类中的点 (不用专门判断)
                // if (labels[nb] == -1)
                //     labels[nb] = cluster_label;
                // 在并行中, 有可能出现这种情况
                // if (labels[nb] != -2 && labels[nb] != -1)
                //     continue;
                // 不记入噪声点, 缩小聚类, 减少错误
                //若邻域中的当前点已经添加过标签，则跳过
                if (labels[nb] != -2)
                    continue;

                //为邻域中的点添加聚类标签
                labels[nb] = cluster_label;

                //若邻域中的点的邻域点数大于等于min_points，则加入到nbs_next中
                if (nbs[nb].size() >= min_points)
                    for (int qnb : nbs[nb])
                        ////即如果nbs_visited中没有qnb，则将qnb加入到nbs_next中
                        if (!nbs_visited.bucket[qnb])
                            nbs_next.insert(qnb);
            }

            cluster_label++;
        }

        return labels;
    }

    //记入噪音点
    std::vector<int> Cluster::NormalDBSCAN(const open3d::geometry::PointCloud& cloud,
        double eps,size_t min_points){
            
        open3d::geometry::KDTreeFlann kdtree(cloud);

        std::vector<std::vector<int>> nbs(cloud.points_.size());
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
}
RCLCPP_COMPONENTS_REGISTER_NODE(upc_radar::Cluster)