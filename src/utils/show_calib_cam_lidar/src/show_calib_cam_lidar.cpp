//
// Created by thesky on 25-5-10.
//
//
// Created by thesky on 25-5-5.
//
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


class ShowCalib : public rclcpp::Node {
    public:
    ShowCalib(std::string name):Node(name) {
        img=cv::imread("resource/11.png");
//         camera_matrix=cv::Matx33d(2291.14722575626,0,1103.67246,0,2291.55696902824,1086.45687,0,0,1);
//         lidar2cam=cv::Matx44d(0.46173  , -0.00534 , 0.88700  , -0.00254  ,
// -0.88300 , -0.09782  ,0.45906  , 0.09259   ,
// 0.08431  , -0.99519 , -0.04988 , -0.00688 ,
// 0.00000  , 0.00000 ,  0.00000 ,  1.00000);
        camera_matrix=cv::Matx33d(1703.57857839016,0,776.621431609538,0,1698.69114037183,558.752840063006,0,0,1);
        lidar2cam=cv::Matx44d(-0.26869 , -0.07683 , 0.96016 ,  -0.17722  ,
-0.95942 , 0.10986  , -0.25969 , -0.10152  ,
-0.08553 , -0.99097 , -0.10323 , 0.11610   ,
0.00000 ,  0.00000  , 0.00000 ,  1.00000);
        lidar2cam=lidar2cam.inv();
        cv::namedWindow("img",0);
        cv::resizeWindow("img",640,480);
        pc_sub=this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_3JEDM7A00106241", 10, std::bind(&ShowCalib::callback, this, std::placeholders::_1));
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cv::Mat image=img.clone();
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg,cloud);
        acc_pcs.push_back(cloud);
        if (acc_pcs.size()<=40) {
            return;
        }else {
            acc_pcs.erase(acc_pcs.begin());
        }
        pcl::PointCloud<pcl::PointXYZ> acc_pc;
        for (auto pcs:acc_pcs) {
            acc_pc+=pcs;
        }
        for (auto point:acc_pc.points) {
            cv::Point3d pts_2d;
            pts_2d=lidarToCamera(point,camera_matrix,lidar2cam);
            if(pts_2d.x>=0&&pts_2d.x<2448&&pts_2d.y>=0&&pts_2d.y<2048){
                cv::circle(image,cv::Point(pts_2d.x,pts_2d.y),0,cv::Scalar(0,0,255),1);
            }
        }
        cv::imshow("img",image);
        cv::waitKey(1);
    }

    cv::Point3d lidarToCamera(pcl::PointXYZ lidar_point,cv::Matx33d cam_matrix,cv::Matx44d lidar2cam){
        cv::Matx41d lidar_coor{lidar_point.x, lidar_point.y, lidar_point.z, 1.0f};
        cv::Matx31d camera_coor =cam_matrix*(lidar2cam*lidar_coor).get_minor<3, 1>(0, 0);
        double u = camera_coor(0) / camera_coor(2);
        double v = camera_coor(1) / camera_coor(2);
        double d = camera_coor(2);
        //(v, u) 是像素的坐标，其中 v 是行索引（通常对应于图像的高度），u 是列索引（通常对应于图像的宽度）
        return cv::Point3d(u,v,d);
    }

    ~ShowCalib(){}

    private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
    cv::Mat img;
    cv::Matx33d camera_matrix;
    cv::Matx44d lidar2cam;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> acc_pcs;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShowCalib>("ShowCalib");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}