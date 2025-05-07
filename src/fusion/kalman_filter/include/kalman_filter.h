#pragma once

#include "utils.h"
#include "filter_plus.h"
#include "lapjv.h"
#include "interfaces/msg/detect_result.hpp"
#include "interfaces/msg/detect_frame.hpp"

#include "interfaces/msg/detect_res.hpp"
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

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,sensor_msgs::msg::PointCloud2> MySyncPolicy;

    struct BucketSet {
        std::vector<char> bucket;
        size_t size;
        size_t now_first;
        void insert(size_t value){
            if (!bucket[value]) {
                bucket[value] = true;
                ++size;
                if (value < now_first)
                    now_first = value;
            }
        }
        void erase(size_t value){
            if (bucket[value]) {
                bucket[value] = false;
                --size;
                if (size == 0)
                    now_first = -1;
                //当前最小值被删除, 往后遍历更新最小值
                else if (value == now_first) {
                    for (size_t i = value + 1; i < bucket.size(); ++i){
                        if (bucket[i]) {
                            now_first = i;
                            break;
                        }
                    }
                }
            }
        }
        BucketSet(size_t real_size):bucket(real_size, false),size(0),now_first(-1){}
    };

    class KalmanFilter :public rclcpp::Node
    {
        public:
        KalmanFilter(const rclcpp::NodeOptions& node_options);
        ~KalmanFilter(){}
    
        private:
        int self_color;
        double match_thresh=0.45;
        std::vector<Kalman_filter_plus> KFs;
        std::mutex mtx;
        bool is_one_lidar = false;
        int min_points=3;
        double eps=0.25;
        bool use_rect2d=true;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> acc_clouds;
        int span=0;

        interfaces::msg::DetectFrame detect_msg;
        interfaces::msg::DetectResult dep_msg;
        interfaces::msg::DetectRes cam_msg;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Subscription<interfaces::msg::DetectFrame>::SharedPtr sub_detect_;
        rclcpp::Subscription<interfaces::msg::DetectResult>::SharedPtr sub_dep_;
        rclcpp::Publisher<interfaces::msg::DetectResult>::SharedPtr lidar_detect_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_point_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_kmeans;

        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> mid70_sub;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> avia_sub;
        std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_main_img;

        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void PcTimeSynC(const sensor_msgs::msg::PointCloud2::SharedPtr msg1, const sensor_msgs::msg::PointCloud2::SharedPtr msg2);
        std::vector<int> NormalDBSCAN(const open3d::geometry::PointCloud cloud,double eps,size_t min_points,std::vector<std::vector<int>> &nbs);
        std::vector<int> NormalDBSCAN(const open3d::geometry::PointCloud cloud,double eps,size_t min_points,std::vector<std::vector<int>> &nbs,std::vector<std::vector<int>> &clusters);
        std::vector<int> NormalDBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,double eps,size_t min_points);

        void get_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector<open3d::geometry::PointCloud> &out);
        void get_2drect(std::vector<open3d::geometry::PointCloud> pcs,Eigen::Transform<float, 3, 2> transform,std::vector<cv::Rect> &rects,int camid);
        void get_cluster_id(std::vector<Clus_pc>&clus_pcs,std::vector<open3d::geometry::PointCloud> pcs,std::vector<cv::Rect>rects);
        void detect_callback(const interfaces::msg::DetectFrame::SharedPtr msg);
        void dep_callback(const interfaces::msg::DetectResult::SharedPtr msg);
        void check_KFs(std::vector<Kalman_filter_plus> &KFs_);
        void check(std::vector<Kalman_filter_plus> &KFs_);
        void getImg1(const sensor_msgs::msg::CompressedImage::ConstPtr &rosImg_ptr);

        rclcpp::Subscription<interfaces::msg::DetectRes>::SharedPtr sub_cam_;
        void cam_callback(const interfaces::msg::DetectRes::SharedPtr msg);

        //-------------------------------------------//
        cv::Mat show_img1,show_img2;
        bool get_lidar2world=false;
        geometry_msgs::msg::TransformStamped transform_stamped;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        cv::Matx33d camera_matrix1;
        cv::Matx33d camera_matrix2;
        cv::Matx44d lidar2cam1;
        cv::Matx44d lidar2cam2;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr net_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ori_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pub_;
        cv::Point3d lidarToCamera(pcl::PointXYZ lidar_point,cv::Matx33d cam_matrix,cv::Matx44d lidar2cam);
    };
}//namespace tdt_radar