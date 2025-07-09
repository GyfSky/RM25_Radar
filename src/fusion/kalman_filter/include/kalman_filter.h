#pragma once

#include "utils.h"
#include "interfaces/msg/robot_hp.hpp"
#include "filter_plus.h"
#include "lapjv.h"
#include "interfaces/msg/detect_result.hpp"
#include "interfaces/msg/detect_frame.hpp"
#include "interfaces/msg/lidar_enhance.hpp"
#include "interfaces/msg/detect_res.hpp"
#include "interfaces/msg/drone_location.hpp"
#include "interfaces/msg/detect_obj.hpp"
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/ml/kmeans.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tbb/parallel_for.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>

#include <mutex>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_components/register_node_macro.hpp>
//-------------------------------------------//
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/common/transforms.h>

namespace upc_radar{

    //自定义消息同步策略
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,sensor_msgs::msg::PointCloud2> MySyncPolicy;

    class KalmanFilter :public rclcpp::Node
    {
        public:
        KalmanFilter(const rclcpp::NodeOptions& node_options);
        ~KalmanFilter(){}
    
        private:
        int self_color=0;
        double match_thresh=0.45;
        std::vector<Kalman_filter_plus> KFs;
        std::mutex mtx;
        bool is_one_lidar = false, get_lidar2world=false, use_rect2d=true;
        int min_points=3;
        double eps=0.25;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> acc_clouds;
        int span=0;
        std::array<cv::Point2f,5> tunnel_slanted_,tunnel_horizontal_,supply_,outpost_,little_engine_l_,little_engine_r_,high_way_;

        cv::Mat show_img1,show_img2;
        cv::Matx33d camera_matrix1,camera_matrix2;
        cv::Matx44d lidar2cam1,lidar2cam2;
        geometry_msgs::msg::TransformStamped transform_stamped;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        interfaces::msg::DetectFrame detect_msg;
        interfaces::msg::DetectRes cam_msg;
        interfaces::msg::RobotHP robot_hp;
        interfaces::msg::DetectResult detect_res;
        int lidar_enhance_[2][5]={};//0为不进行特殊处理，1为补给区静止，2为模拟倾斜移动，3为模拟水平移动，4为隧道处静止，5为工程猜测点+英雄前哨站猜测点，6为英雄高地猜测点
        bool first_b_change_=true,first_r_change_=true;
        std::array<std::array<FakeKF,5>,2> fake_kfs;

        //双雷达消息同步
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> mid70_sub;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> avia_sub;
        std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;
        //飞机点云同步
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> drone1_sub_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> drone2_sub_;
        std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> drone_sync_;

        //单雷达
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc;
        rclcpp::Subscription<interfaces::msg::DetectFrame>::SharedPtr sub_detect;

        rclcpp::Subscription<interfaces::msg::RobotHP>::SharedPtr sub_robot_hp;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_main_img;

        //相机检测信息
        rclcpp::Subscription<interfaces::msg::DetectRes>::SharedPtr sub_cam;

        rclcpp::Publisher<interfaces::msg::DetectResult>::SharedPtr lidar_detect_pub;
        rclcpp::Publisher<interfaces::msg::LidarEnhance>::SharedPtr lidar_enh_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_vis;//可视化
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_kalman;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_kmeans;
        rclcpp::Publisher<interfaces::msg::DroneLocation>::SharedPtr pub_drone_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr net_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub;

        void prepareParameter();
        void prepareLocation();
        void checkLocation(std::vector<Kalman_filter_plus> &KFs);

        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void PCTimeSynC(const sensor_msgs::msg::PointCloud2::SharedPtr msg1, const sensor_msgs::msg::PointCloud2::SharedPtr msg2);
        void droneTimeSynC(const sensor_msgs::msg::PointCloud2::SharedPtr msg1, const sensor_msgs::msg::PointCloud2::SharedPtr msg2);
        void camCallback(const interfaces::msg::DetectRes::SharedPtr msg);
        void clearOutPut();

        void detectCallback(const interfaces::msg::DetectFrame::SharedPtr msg);
        void robotHPCallback(const interfaces::msg::RobotHP::SharedPtr msg);

        void guessWithoutClass(std::vector<Kalman_filter_plus> &KFs_,std::vector<int> u_strack,visualization_msgs::msg::MarkerArray &vis_array,std::vector<std::vector<int>> history_cost);
        void guessWithClass(std::vector<Kalman_filter_plus> &KFs_,std::vector<std::vector<int>> matches_cls,std::vector<int> &remove_KFs_);
        void speedSimulation();

        std::vector<int> normalDBSCAN(const open3d::geometry::PointCloud cloud,double eps,size_t min_points,std::vector<std::vector<int>> &nbs);
        std::vector<int> normalDBSCAN(const open3d::geometry::PointCloud cloud,double eps,size_t min_points,std::vector<std::vector<int>> &nbs,std::vector<std::vector<int>> &clusters);
        std::vector<int> normalDBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,double eps,size_t min_points);

        void getCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector<open3d::geometry::PointCloud> &out);
        void getRect2d(std::vector<open3d::geometry::PointCloud> pcs,Eigen::Transform<float, 3, 2> transform,std::vector<cv::Rect> &rects,int camid);
        void getClusterID(std::vector<ClusPC>&clus_pcs,std::vector<open3d::geometry::PointCloud> pcs,std::vector<cv::Rect>rects);
        void updateKFs(std::vector<Kalman_filter_plus> &KFs_,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xy,const std::vector<cv::Rect> &rects1,
            const std::vector<cv::Rect> &rects2,const std::vector<open3d::geometry::PointCloud> &pcs,pcl::PointCloud<pcl::PointXYZ> &kmeans_out,rclcpp::Time time);
        void checkKFs(std::vector<Kalman_filter_plus> &KFs_);
        void checkClass(std::vector<Kalman_filter_plus> &KFs_);
        void check(std::vector<Kalman_filter_plus> &KFs_);
        void getImg1(const sensor_msgs::msg::CompressedImage::ConstPtr &rosImg_ptr);

        cv::Point3d lidarToCamera(pcl::PointXYZ lidar_point,cv::Matx33d cam_matrix,cv::Matx44d lidar2cam);
    };
}//namespace tdt_radar