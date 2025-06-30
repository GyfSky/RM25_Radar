//
// Created by thesky on 25-4-2.
//
#include <chrono>
#include <vector>
#include <deque>

#include <open3d/Open3D.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,sensor_msgs::msg::PointCloud2> MySyncPolicy;

class CalibLidar : public rclcpp::Node{
    public:
    CalibLidar(std::string name):Node(name){
        RCLCPP_WARN(this->get_logger(), "CalibLidar start");

        mid70_sub.subscribe(this, "/livox/lidar_3GGDL82001UK381");
        avia_sub.subscribe(this, "/livox/lidar_3JEDM7A00106241");

        MySyncPolicy sync_policy(20);
        sync=std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(std::ref(sync_policy),avia_sub,mid70_sub);
        sync_policy.setMaxIntervalDuration(rclcpp::Duration(0,20000000));
        sync->registerCallback(&CalibLidar::PcTimeSynC, this);

    }
    ~CalibLidar(){}

    private:
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> mid70_sub;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> avia_sub;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;
    int accumulate_time =30;
    bool manual_aligned_=false;
    bool auto_aligned_=false;
    Eigen::Matrix4d T;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_1;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_2;

    std::vector<size_t> select_points(std::shared_ptr<const open3d::geometry::PointCloud> pcd){
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

    Eigen::Matrix4d manual_trans(std::shared_ptr<open3d::geometry::PointCloud> source, std::shared_ptr<open3d::geometry::PointCloud> target){
        auto picked_source = select_points(source);
        auto picked_target = select_points(target);
        assert(picked_source.size() >= 3 && picked_target.size() >= 3);
        assert(picked_source.size() == picked_target.size());

        std::vector<Eigen::Vector2i> correspondences;
        for (size_t i = 0; i < picked_source.size(); ++i)
            correspondences.emplace_back(picked_source[i], picked_target[i]);

        open3d::pipelines::registration::TransformationEstimationPointToPoint pointToPoint;
        return pointToPoint.ComputeTransformation(*source, *target, correspondences);
    }

    void PcTimeSynC(const sensor_msgs::msg::PointCloud2::SharedPtr msg1, const sensor_msgs::msg::PointCloud2::SharedPtr msg2) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr avia_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr mid70_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::fromROSMsg(*msg1, *avia_cloud);
        pcl::fromROSMsg(*msg2, *mid70_cloud);

        if(accumulated_clouds_1.size() < accumulate_time){
            accumulated_clouds_1.push_back(avia_cloud);
            accumulated_clouds_2.push_back(mid70_cloud);
            return;
        }else{
            accumulated_clouds_1.erase(accumulated_clouds_1.begin());
            accumulated_clouds_1.push_back(avia_cloud);
            accumulated_clouds_2.erase(accumulated_clouds_2.begin());
            accumulated_clouds_2.push_back(mid70_cloud);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        for(auto accumulated_cloud : accumulated_clouds_1){
            for (const auto& point : *accumulated_cloud){
                //rm24
                // if(point.x > 5 && point.x < 30 && point.y > -10 && point.y < 8&&point.z<7)
                //     target_cloud->push_back(point);
                //lab
                if(point.x > -35 && point.x < 35 && point.y > -35 && point.y < 35&&point.z<7.6)
                    target_cloud->push_back(point);
            }
        }

        for(auto accumulated_cloud : accumulated_clouds_2){
            for (const auto& point : *accumulated_cloud){
                //rm24
                // if(point.x > 5 && point.x < 30 && point.y > -10 && point.y < 8&&point.z<7)
                //     source_cloud->push_back(point);
                //lab
                if(point.x > -35 && point.x < 35 && point.y > -35 && point.y < 35&&point.z<7.6)
                    source_cloud->push_back(point);
            }
        }
        if(!manual_aligned_){
            std::shared_ptr<open3d::geometry::PointCloud> avia_o3d(new open3d::geometry::PointCloud);
            std::shared_ptr<open3d::geometry::PointCloud> mid70_o3d(new open3d::geometry::PointCloud);

            for(size_t i=0;i<target_cloud->points.size();i++)
                avia_o3d->points_.push_back(Eigen::Vector3d(target_cloud->points[i].x,target_cloud->points[i].y,target_cloud->points[i].z));

            for(size_t i=0;i<source_cloud->points.size();i++)
                mid70_o3d->points_.push_back(Eigen::Vector3d(source_cloud->points[i].x,source_cloud->points[i].y,source_cloud->points[i].z));

            T = manual_trans(mid70_o3d, avia_o3d);
            manual_aligned_=true;

            RCLCPP_WARN(this->get_logger(), "manual_trans end");
        }
        if(!auto_aligned_){
            //降采样
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

            pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_avia;
            voxelgrid_avia.setLeafSize(0.2, 0.2, 0.2);
            voxelgrid_avia.setInputCloud(target_cloud);
            voxelgrid_avia.filter(*downsampled);
            *target_cloud = *downsampled;

            pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_mid70;
            voxelgrid_mid70.setLeafSize(0.2, 0.2, 0.2);
            voxelgrid_mid70.setInputCloud(source_cloud);
            voxelgrid_mid70.filter(*downsampled);
            *source_cloud = *downsampled;

            //应用初始变换
            pcl::transformPointCloud(*source_cloud, *source_cloud, T);

            boost::shared_ptr<pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>> registration(new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>());

            std::cout<<"number of avia cloud points "<<target_cloud->points.size()<<std::endl;
            std::cout<<"number of mid70 cloud points "<<source_cloud->points.size()<<std::endl;

            pcl::PointCloud<pcl::PointXYZINormal>::Ptr sourceCloudNormal(new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::copyPointCloud(*source_cloud, *sourceCloudNormal);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr targetCloudNormal(new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::copyPointCloud(*target_cloud, *targetCloudNormal);

            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
            pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            ne.setInputCloud(source_cloud);		//pcl通常使用该方法来传数据
            ne.setSearchMethod(tree);
            ne.setKSearch(30);
            ne.compute(*source_normals);		//获得法向量
            pcl::copyPointCloud(*source_normals, *sourceCloudNormal);

            ne.setInputCloud(target_cloud);		//pcl通常使用该方法来传数据
            ne.compute(*target_normals);		//获得法向量
            pcl::copyPointCloud(*target_normals, *targetCloudNormal);


            registration->setInputTarget(targetCloudNormal);
            registration->setInputSource(sourceCloudNormal);
            registration->setMaxCorrespondenceDistance(0.8);
            registration->setTransformationEpsilon(1e-20);
            registration->setEuclideanFitnessEpsilon(0.00001);
            registration->setUseReciprocalCorrespondences(true);

            pcl::PointCloud<pcl::PointXYZINormal>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZINormal>());
            registration->align(*aligned);

            RCLCPP_WARN(this->get_logger(), "calib result : %f", registration->getFitnessScore());

            Eigen::Matrix4f transform;
            transform= registration->getFinalTransformation();
            T=transform.cast<double>() * T;

            if(registration->getFitnessScore()<10){
                auto_aligned_ = true;
                RCLCPP_WARN(this->get_logger(), "auto_trans end");
            }else {
                accumulated_clouds_1.clear();
                accumulated_clouds_2.clear();
            }
        }
        if(auto_aligned_)
            std::cout<<T<<std::endl;
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalibLidar>("CalibLidar");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}