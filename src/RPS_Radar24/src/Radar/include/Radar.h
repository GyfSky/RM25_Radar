// #include "../../General/include/General.h"
#include "../../Net/include/Net.h"
// #include "../../Livox/include/Mid70ros1.h"
#include "../../Camera_hk/include/Camera_mlt.h"
#include "../../General/include/Mouse.h"
#include "../../General/include/SensorParam.h"
#include "../../Locate/include/KRepresent.h"
#include "../../Locate/include/Predict.h"
#include "../../ByteTrack/include/PretreatObjs.h"
#include "../../Port/include/Port.h"
#include "../../Image/include/ImageStitch.h"
#include "../../Image/include/Image.h"
#include "../../Hero/include/CoordSolver.h"

#include <interfaces/msg/detect_frame.hpp>
#include <interfaces/msg/net_detect.hpp>
#include <interfaces/msg/detect_result.hpp>
#include <interfaces/msg/detect_obj.hpp>
#include <interfaces/msg/detect_res.hpp>
#include <interfaces/msg/lidar_enhance.hpp>
#include <interfaces/msg/drone_location.hpp>

// class MyRadar : public Livox
class MyRadar 
{
private:
    std::string node_name;
    bool is_first = true;
    // int after_picture=1;
    cv::Mat mainCamMat;
    cv::Mat secCamMat;

    //test
    TRTInferV1::TRTInfer myInfer;


    std::shared_ptr<SensorParam> MainCam_ptr = nullptr;
    std::shared_ptr<SensorParam> SecCam_ptr = nullptr;
    std::shared_ptr<SensorParam> Lidar_ptr = nullptr;

    std::shared_ptr<Modes> Modes_ptr = nullptr;
    std::shared_ptr<MapGraphMtx> MainMapGraph_ptr = nullptr;
    std::shared_ptr<MapGraphMtx> SecMapGraph_ptr = nullptr;


    std::shared_ptr<Predict> Predict_ptr = nullptr;
    std::shared_ptr<PretreatObjs> PretreatObjs_ptr = nullptr;
    std::shared_ptr<BYTETracker> BYTETracker_ptr = nullptr;

    std::shared_ptr<MatrixCoordinateSystem> CooSystem_ptr = nullptr;
    std::shared_ptr<KRepresent> KRepresent_ptr = nullptr;
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
    int value=200;
    int classWithoutCar;

    std::string save_main_dir;
    std::string save_sec_dir;
    int pic_num = 0;
    int color_index;
    OurPattern ourPattern;
    double game_time_=0;

public:
    bool is_one_cam = false;
    rclcpp::Node::SharedPtr node;
    interfaces::msg::DetectResult lidar_det;
    interfaces::msg::DetectResult lidar_det1;
    bool is_close = false;
    bool is_init=false;
    rclcpp::Time time_now;
    rclcpp::Publisher<interfaces::msg::DetectFrame>::SharedPtr detect_pub;
    rclcpp::Publisher<interfaces::msg::DetectRes>::SharedPtr res_pub;
    interfaces::msg::NetDetect car_det;
    interfaces::msg::NetDetect armor_det;
    interfaces::msg::LidarEnhance lidar_enhance_;
    interfaces::msg::DroneLocation drone_location_;

    bool hero_guess_1_=false,hero_guess_2_=false,engineer_guess_1_=false,engineer_guess_2_=false;
    int hero_time_1_=0,hero_time_2_=0,engineer_time_1_=0,engineer_time_2_=0;

    cv::Point3d hero_location1_,hero_location2_,hero_location3_,buff_location_,
    engineer_location1_,engineer_location2_,fortress_location_,supply_location_;

    MyRadar(/* args */rclcpp::Node::SharedPtr node);
    ~MyRadar();
    std::shared_ptr<Image> MainCam_Image_ptr = nullptr;
    std::shared_ptr<Image> SecCam_Image_ptr  = nullptr;
    void Init(int argc, char **argv);
    void STrackInit(int classWithoutCar, OurPattern ourPattern);
    void STrackGuess(int classWithoutCar);
    void STrackClear();
    void getDartWarning(cv::Mat img,int value);
    PictureSource getPictureSource();
    void Save();
    void Spin(int argc, char **argv);
    void Close();
};