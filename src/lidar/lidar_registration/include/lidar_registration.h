#include <chrono>
#include <memory>
#include <fstream> 
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>

#include <open3d/Open3D.h>
#include <open3d/visualization/visualizer/RenderOptionWithEditing.h>

namespace upc_radar{
    class LidarRegistration : public rclcpp::Node
    {
        public:
        LidarRegistration(const rclcpp::NodeOptions& node_options);
        ~LidarRegistration(){}
    
        private:
        double cost_thres_;
        double al_map_grid_size_;
        double pc_grid_size_;
        
        bool manual_aligned_=false;
        bool auto_aligned_=false;
        bool use_saved_T_=false;
        bool use_prepoints_=true;
        std::string situation_;
        std::string child_frame_id_;
        std::string sub_topic_;
        
        Eigen::Matrix4d T_;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

        int accumulate_time_ =30;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        Eigen::Matrix4d manualTrans(std::shared_ptr<open3d::geometry::PointCloud> pc2align, std::shared_ptr<open3d::geometry::PointCloud> mesh_pc);
        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        std::vector<size_t> selectPoints(std::shared_ptr<const open3d::geometry::PointCloud> pcd);
        void publishTF(const Eigen::Matrix4d& transform);
    };

}//namespace upc_radar