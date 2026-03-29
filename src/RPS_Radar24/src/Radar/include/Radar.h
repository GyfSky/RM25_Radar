// #include "../../General/include/General.h"
#include "../../Net/include/Net.h"
// #include "../../Livox/include/Mid70ros1.h"
#include "../../Camera_hk/include/Camera_mlt.h"
#include "../../General/include/Mouse.h"
#include "../../General/include/SensorParam.h"
#include "../../Locate/include/Predict.h"
#include "../../ByteTrack/include/PretreatObjs.h"
#include "../../Port/include/Port.h"
#include "../../Image/include/Image.h"
#include "../../Hero/include/CoordSolver.h"

#include <interfaces/msg/detect_frame.hpp>
#include <interfaces/msg/net_detect.hpp>
#include <interfaces/msg/detect_result.hpp>
#include <interfaces/msg/detect_obj.hpp>
#include <interfaces/msg/detect_res.hpp>
#include <interfaces/msg/lidar_enhance.hpp>
#include <interfaces/msg/drone_location.hpp>
#include <interfaces/msg/cost_matrix.hpp>
#include <interfaces/msg/cluster_target.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

double getTimeByRosTime(rclcpp::Time ros_time);

// class MyRadar : public Livox
class MyRadar 
{
private:
    std::string node_name;
    std::string config_path;
    bool is_first = true;
    // int after_picture=1;
    cv::Mat mainCamMat;
    cv::Mat secCamMat;
    int save_count=0;

    //test
    TRTInferV1::TRTInfer myInfer;


    std::shared_ptr<SensorParam> MainCam_ptr = nullptr;
    std::shared_ptr<SensorParam> SecCam_ptr = nullptr;
    std::shared_ptr<SensorParam> Lidar_ptr = nullptr;

    std::shared_ptr<Modes> Modes_ptr = nullptr;


    std::shared_ptr<Predict> Predict_ptr = nullptr;
    std::shared_ptr<PretreatObjs> PretreatObjs_ptr = nullptr;
    std::shared_ptr<BYTETracker> BYTETracker_ptr = nullptr;

    std::shared_ptr<MatrixCoordinateSystem> CooSystem_ptr = nullptr;
    // std::shared_ptr<Image> Image_ptr = nullptr;
    // std::shared_ptr<Image> MainCam_Image_ptr = nullptr;
    // std::shared_ptr<Image> SecCam_Image_ptr  = nullptr;

    // std::shared_ptr<Livox> Livox_ptr = nullptr;
    std::shared_ptr<Net> MainCam_Net_ptr = nullptr;
    std::shared_ptr<Net> SecCam_Net_ptr = nullptr;
    std::shared_ptr<Net> Armor_Net_ptr = nullptr;
    std::shared_ptr<Mouse> Mouse_ptr = nullptr;


    std::shared_ptr<CostMatrix> costMatrix_ptr = nullptr;
    std::shared_ptr<CoordSolver> CoordSolve_ptr = nullptr;

    std::shared_ptr<Port> Port_ptr = nullptr; //

    //二维数组，第一维是相机数/输入图片数量，第二维是检测物体个数
    std::vector<std::vector<TRTInferV1::DetectionObj>> DetectionObjs;


//    std::vector<std::vector<TRTInferV1::Object>> Objects;

    std::vector<Car> cars;
    std::vector<Armor> armors;

    std::vector<cv::Mat> frames;
    std::vector<STrack> tracked_stracks, lost_stracks, lost_predict_stracks,out,out_init,to_sentry;
    std::vector<int> windmill_car;
    cv::Rect rect;
    std::vector<cv::Point2d> dart_center;
    rclcpp::Time dart_time;
    bool dart_flag=false;

    std::vector<Car> redCars,blueCars,restCars,lastCars;

    int after = 0;//图片序号
    int bafter;
    int classWithoutCar;

    std::string save_main_dir;
    std::string save_sec_dir;
    int pic_num = 0;
    int color_index;
    OurPattern ourPattern;
    double game_time_=0;

public:
    std::shared_ptr<MapGraphMtx> MainMapGraph_ptr = nullptr;
    std::shared_ptr<MapGraphMtx> SecMapGraph_ptr = nullptr;
    bool is_one_cam = false;
    rclcpp::Node::SharedPtr node;
    interfaces::msg::DetectResult lidar_det;
    interfaces::msg::DetectResult lidar_det1;
    bool is_close = false;
    bool is_init=false;
    rclcpp::Time time_now;
    rclcpp::Publisher<interfaces::msg::DetectFrame>::SharedPtr detect_pub;
    rclcpp::Publisher<interfaces::msg::DetectRes>::SharedPtr res_pub;
    rclcpp::Publisher<interfaces::msg::ClusterTarget>::SharedPtr cluster_target_pub_;
    interfaces::msg::NetDetect car_det;
    interfaces::msg::NetDetect armor_det;
    interfaces::msg::LidarEnhance lidar_enhance_;
    interfaces::msg::DroneLocation drone_location_;

    bool hero_guess_1_=false,hero_guess_2_=false,engineer_guess_1_=false,engineer_guess_2_=false;
    int hero_time_1_=0,hero_time_2_=0,engineer_time_1_=0,engineer_time_2_=0;

    cv::Point3d hero_location1_,hero_location2_,hero_location3_,buff_location_,
    engineer_location1_,engineer_location2_,fortress_location_,supply_location_;
    std::array<cv::Point2f, 25> self_central_heights_,tunnel_slanted_;
    std::array<int,5> rival_offense_time={0};
    std::array<int,5> acc_time_{};

    std::vector<cv::Mat> cam1,cam2;
    std::vector<double> time1,time2;
    std::mutex netLock,cam1Lock,cam2Lock,timeSyncLock,droneLock,lidarLock,initLock,accTimeLock;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<interfaces::msg::ClusterRect>::SharedPtr cluster_rect_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_main_img;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_sec_img;
    rclcpp::Subscription<interfaces::msg::NetDetect>::SharedPtr sub_car;
    rclcpp::Subscription<interfaces::msg::NetDetect>::SharedPtr sub_armor;
    rclcpp::Subscription<interfaces::msg::DetectResult>::SharedPtr sub_lidar_det;
    rclcpp::Subscription<interfaces::msg::LidarEnhance>::SharedPtr sub_lidar_enh;
    rclcpp::Subscription<interfaces::msg::DroneLocation>::SharedPtr sub_drone_location;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr main_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sec_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img1_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img2_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dart_pub_;
    bool flag1=false,flag2=false,first_sub_lidar_=true;
    rclcpp::CallbackGroup::SharedPtr callBackGroup_;
    rclcpp::CallbackGroup::SharedPtr timerGroup_;

    MyRadar(/* args */rclcpp::Node::SharedPtr node);
    MyRadar(rclcpp::Node::SharedPtr node,bool flag);
    ~MyRadar();

    void rectsCallBack(const interfaces::msg::ClusterRect::SharedPtr msg);
    void cam1CallBack(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void cam2CallBack(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void lidarEnhanceCallBack(const interfaces::msg::LidarEnhance::SharedPtr msg);
    void lidarDetCallBack(const interfaces::msg::DetectResult::SharedPtr msg);
    void droneCallBack(const interfaces::msg::DroneLocation::SharedPtr msg);

    std::shared_ptr<Image> MainCam_Image_ptr = nullptr;
    std::shared_ptr<Image> SecCam_Image_ptr  = nullptr;
    void Init();
    void calib();
    void STrackInit(int classWithoutCar, OurPattern ourPattern);
    void STrackGuess(int classWithoutCar);
    void STrackClear();
    void getDartWarning(cv::Mat img);

    void getDartWarning(cv::Mat img,int value);
    void getRivalOffenseWarning(vector<bool> &isWarring);
    PictureSource getPictureSource();
    void Save();
    void Spin();
    void Close();
};
