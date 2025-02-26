#include <chrono>
#include <vector>
#include <deque>

#include <open3d/Open3D.h>
#include <tbb/parallel_for.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace upc_radar{

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

    class Cluster : public rclcpp::Node
    {
        public:
        Cluster(const rclcpp::NodeOptions& node_options);
        ~Cluster(){}

        std::vector<int> DifferingDBSCAN(const open3d::geometry::PointCloud& cloud_ptr,
            const Eigen::Vector3d& zero_pos,double eps,double min_points_k);

        std::vector<int> NormalDBSCAN(const open3d::geometry::PointCloud& cloud_ptr,
            double eps,size_t min_points);
    
        private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    };
}//namespace upc_radar
