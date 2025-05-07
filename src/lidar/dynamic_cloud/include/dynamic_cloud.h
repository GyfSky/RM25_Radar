#ifndef DYNAMIC_CLOUD_H
#define DYNAMIC_CLOUD_H

#include "VoxelGrid.h"

#include <deque>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp_components/register_node_macro.hpp>


namespace upc_radar{

    #pragma pack(push,1)
    typedef struct{
        float x;            /**< X axis, Unit:m */
        float y;            /**< Y axis, Unit:m */
        float z;            /**< Z axis, Unit:m */
        float reflectivity; /**< Reflectivity   */
        uint8_t tag;        /**< Livox point tag   */
        uint8_t line;       /**< Laser line id     */
    }LivoxPointXyzrtl;
    #pragma pack(pop)

    auto little_engine_filter = [](pcl::PointXYZ &point) {
        //小资源岛方程：
        // y=tan55°*x - 20.2563 短底边
        // y=tan55°*x -18.0736 长底边
        // y=-1/tan55°*x -5.9628
        // y=-1/tan55°*x -7.4988
        double xminusy = point.y - point.x*tan(55.0/180.0*M_PI);//找经过这一点的与底边平行的直线与y轴交点的距离
        double xplusy = point.y + point.x / tan(55.0/180.0*M_PI);//找经过这一点的与底边垂直的直线与y轴交点的距离  
        return (xminusy < -21.9555 && xminusy > -23.3419) &&
               (xplusy > 16.7456 && xplusy < 18.1448)&&(point.z>0&&point.z<1.2);
               //构建方程之后微调一下
    };
    //rm25
    void get_filtered_cloud25(pcl::PointCloud<pcl::PointXYZ> origin_cloud,pcl::PointCloud<pcl::PointXYZ> &filtered_cloud){
        for (size_t i = 0; i < origin_cloud.size(); i++){
            auto &point = origin_cloud.points[i];
            //point.x < 3大致为己方停机坪   point.z > 1.4
            if (point.x < 2.7 || point.x > 28 || point.y < 0 || point.y > 15 || point.z < 0 || point.z > 1.35 ||
                //己方停机坪和飞镖
                (point.y>9.9&&point.x<=3.1)||
                //己方补给区
                (point.y<4.15&&point.x<=3.9)||
                //己方基地
                (point.y>=15-8.45&&point.y<=15-6.55&&point.x<=28-24.93&&point.x>=28-26.55)||
                //或者y(0,5),x(25,28)不要 敌方停机坪和飞镖
                (point.y > 0 && point.y < 5 && point.x > 25) ||
                //敌方基地
                (point.y<=8.45&&point.y>=6.55&&point.x>=24.93&&point.x<=26.55)||
                //或者y(11,12),x(23,24)不要
                // (point.y > 11 && point.y < 12 && point.x > 23 && point.x < 24)
                //画四个直线切割大资源岛
                ((21.5-2.9/sqrt(2))<(point.x + point.y) &&(point.x + point.y) <(21.5+2.9/sqrt(2))&&
                (-6.5-0.9/sqrt(2))<(point.y-point.x)&&(point.y-point.x)<(-6.5+0.9/sqrt(2)))||
                //前哨站17<point.x&&point.x<18
                ((3.1<point.y&&point.y<4.1)&&(10.5<point.x&&point.x<11.3))||
                ((10.9<point.y&&point.y<11.9)&&(16.7<point.x&&point.x<17.5))
                //猜是为了减少r4散射点
                // ((11<point.y&&point.y<12.25)&&(23<point.x&&point.x<24.1)&&(point.z<0.535))||
                //兑换区
                // (point.x>28-2.0234&&point.x<28-1.0234)&&(point.y > 10.955+0.1 && point.y < 10.955 + 1.6 - 0.1)&&(point.z>0.4&&point.z<1.5)||
                // little_engine_filter(point)
            ///TODO: 此处代码混乱，需要重构，全部替换成Lambda表达式的过滤器形式
            ){
                continue;
            }
            filtered_cloud.push_back(point);
        }
    }
    //rm24
    void get_filtered_cloud24(pcl::PointCloud<pcl::PointXYZ> origin_cloud,pcl::PointCloud<pcl::PointXYZ> &filtered_cloud){
        for (size_t i = 0; i < origin_cloud.size(); i++){
            auto &point = origin_cloud.points[i];
            //point.x < 3大致为己方停机坪   point.z > 1.4
            if (point.x < 3 || point.x > 28 || point.y < 0 || point.y > 15 || point.z < 0 || point.z > 1.35 ||
                //或者y(0,5),x(25,28)不要 敌方停机坪和飞镖
                (point.y > 0 && point.y < 5 && point.x > 25) ||
                //己方飞机飞行区
                (point.y>12&&point.z>1.3&&point.x<14)||
                //敌方飞机飞行区
                (point.y>3&&point.z>1.3&&point.x>14)||
                //敌方基地
                (point.y<=8.5&&point.y>=6.6512&&point.x>=25.75&&point.x<=27.65)||
                //或者y(11,12),x(23,24)不要
                // (point.y > 11 && point.y < 12 && point.x > 23 && point.x < 24)
                //画四个直线切割大资源岛
                ((21.5-0.9/sqrt(2))<(point.x + point.y) &&(point.x + point.y) <(21.5+0.9/sqrt(2))&&
                (-6.5-2.9/sqrt(2))<(point.y-point.x)&&(point.y-point.x)<(-6.5+2.9/sqrt(2)))||
                //前哨站17<point.x&&point.x<18
                ((12<point.y&&point.y<13.5)&&(16.55<point.x&&point.x<17.55))||
                //猜是为了减少r4散射点
                // ((11<point.y&&point.y<12.25)&&(23<point.x&&point.x<24.1)&&(point.z<0.535))||
                //兑换区
                (point.x>28-2.0234&&point.x<28-1.0234)&&(point.y > 10.955+0.1 && point.y < 10.955 + 1.6 - 0.1)&&(point.z>0.4&&point.z<1.5)
                // little_engine_filter(point)
            ///TODO: 此处代码混乱，需要重构，全部替换成Lambda表达式的过滤器形式
            ){
                continue;
            }
                filtered_cloud.push_back(point);
        }
    }

    int initornot(const std::vector<pcl::PointXY> polygon,pcl::PointXYZ point,int polygonSize) {
        int counter = 0;
        double xinters;
        pcl::PointXY p1, p2;

        p1 = polygon[0];
        for (size_t i = 1; i <= polygonSize; i++) {
            p2 = polygon[i];
            if (point.y > std::min(p1.y, p2.y)) {
                if (point.y <= std::max(p1.y, p2.y)) {
                    if (point.x <= std::max(p1.x, p2.x)) {
                        if (p1.y != p2.y) {
                            xinters = (point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                            if (p1.x == p2.x || point.x <= xinters)
                                counter++;
                        }
                    }
                }
            }
            p1 = p2;
        }

        if ( counter % 2 == 1) return 1;
        else return -1;
    }

    void get_filtered_cloudlab(pcl::PointCloud<pcl::PointXYZ> origin_cloud,pcl::PointCloud<pcl::PointXYZ> &filtered_cloud) {
        std::vector<pcl::PointXY> test;
        test.push_back(pcl::PointXY(5.516865,4.561527));
        test.push_back(pcl::PointXY(6.028124,5.218801));
        test.push_back(pcl::PointXY(4.776117,6.045959));
        test.push_back(pcl::PointXY(4.264859,5.388684));
        test.push_back(pcl::PointXY(5.516865,4.561527));
        for (size_t i = 0; i < origin_cloud.size(); i++){
            auto &point = origin_cloud.points[i];
            if(point.x<=7.07&&point.y<=12&&point.x>=-3&&point.y>=-3&&point.z<1.2&&initornot(test,point,5)==-1)
                filtered_cloud.push_back(point);
            // if(point.x<=7.07&&point.y<=12&&point.x>=-3&&point.y>=-3&&point.z<1.2)
            //     filtered_cloud.push_back(point);
        }
    }

    pcl::PointCloud<pcl::PointXYZ> removeOutlier(pcl::PointCloud<pcl::PointXYZ> origin_cloud,int MeanK,double thres){

        pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        *origin_cloud_ptr=origin_cloud;

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(origin_cloud_ptr);
        // 设置平均距离估计的最近邻居的数量K
        sor.setMeanK(MeanK);
        // 设置标准差阈值系数
        sor.setStddevMulThresh(thres);
        // 执行过滤
        // sor.setNegative(true);
        sor.filter(cloud_filtered);

        return cloud_filtered;
    }

    class DynamicCloud: public rclcpp::Node{
    public:
        DynamicCloud(const rclcpp::NodeOptions &options);
        ~DynamicCloud(){}
        void prepare_pcd();
        void prepare_meshes();
        void prepare_voxel_grid();
        bool get_transform(std::string frame_id,Eigen::Affine3f &transform);
        void callback_pc(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void GetDynamicCloud(pcl::PointCloud<pcl::PointXYZ> &input_cloud,pcl::PointCloud<pcl::PointXYZ> &output_cloud,float threshold,int thread_num);
        void callback_mesh(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void seg_normal(pcl::PointCloud<pcl::PointXYZ> pcl2cloud,pcl::PointCloud<pcl::PointXYZ> &pcl2cloud_out);
    private:
        std::string frame_id;
        std::string sub_topic;
        std::string pub_topic;
        int thread_num = 12;
        double dis_thres=0.1;
        int accumulate_time = 3;
        bool remove_outlier=false;
        double leaf_size=0.04;
        double x_bound=0.4;
        double y_bound=0.4;
        double z_bound_high=0.7;
        double gradient_threshold=0.25;
        bool is_real =false;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
        pcl::KdTreeFLANN<pcl::PointXYZ> kd_Tree;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> accumulated_clouds_;
        std::string situation;

        bool mesh_filter_mode;
        bool is_grid_prepared=false;
        VoxelGrid voxel_grid;
        Eigen::Isometry3d trans;
        std::deque<pcl::PointCloud<pcl::PointXYZ>> pc_buffer;
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ori;
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh_filter;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_raw;
    };
}

#endif //DYNAMIC_CLOUD_H


