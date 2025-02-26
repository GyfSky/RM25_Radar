#pragma once

#include "utils.h"
#include "filter_plus.h"
#include "lapjv.h"
#include "interfaces/msg/detect_result.hpp"
#include "interfaces/msg/detect_frame.hpp"

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

    class KalmanFilter :public rclcpp::Node
    {
        public:
        KalmanFilter(const rclcpp::NodeOptions& node_options);
        ~KalmanFilter(){}
    
        private:
        int self_color;
        double match_thresh=0.45;
        std::vector<Kalman_filter_plus> KFs;

        interfaces::msg::DetectFrame detect_msg;
        interfaces::msg::DetectResult dep_msg;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Subscription<interfaces::msg::DetectFrame>::SharedPtr sub_detect_;
        rclcpp::Subscription<interfaces::msg::DetectResult>::SharedPtr sub_dep_;
        rclcpp::Publisher<interfaces::msg::DetectResult>::SharedPtr lidar_detect_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_point_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void detect_callback(const interfaces::msg::DetectFrame::SharedPtr msg);
        void dep_callback(const interfaces::msg::DetectResult::SharedPtr msg);
        void check_KFs();
        void check();

        //-------------------------------------------//
        cv::Mat show_img;
        bool get_lidar2world=false;
        geometry_msgs::msg::TransformStamped transform_stamped;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        cv::Matx33d camera_matrix1;
        cv::Matx44d lidar2cam1;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr net_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ori_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pub_;
        cv::Point3d lidarToCamera(pcl::PointXYZ lidar_point,cv::Matx33d cam_matrix,cv::Matx44d lidar2cam);
    };
}//namespace tdt_radar