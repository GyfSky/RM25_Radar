#ifndef JSONCPP_TEST_PRETREATOBJS_H
#define JSONCPP_TEST_PRETREATOBJS_H
#include "../../Net/include/Inference.h"
#include "../../General/include/SensorParam.h"
#include "interfaces/msg/rect.hpp"
#include "CostMatrix.h"


class PretreatObjs {
private:
    std::shared_ptr<CostMatrix> costMatrix_ptr = std::shared_ptr<CostMatrix>(new CostMatrix());
    OurPattern ourPattern;
    int allCam = 2;
    int aGroupOfArmor = 6;
    int groupNum = 2;
    std::vector<std::vector<double>> W_of_armorConfs;

    float p_car_midpoint = 0.96;

    float p_carY_downLine = 3./4;
    float p_carY_upLine = 1./3;

    int tx, ty, result_w, result_h, img1_w, img1_h, img2_w, img2_h;
    int sec2main_w = 1920,  sec2main_h = 1440;
    bool sec_is_left;

    std::vector<int> windmill_car;
    double sum_windmill_car_conf = 0.56;
    double startupArea_car_conf = 0.6;
    double windmill_car_conf;
    Scalar redLower,redUpper,blueLower,blueUpper;
    int rl_h=10,rl_s=105,rl_v=255,rh_h=30,rh_s=155,rh_v=255;
    int bl_h=93,bl_s=55,bl_v=125,bh_h=115,bh_s=255,bh_v=255;

public:
    cv::Mat H, H2, perspective_K;
    cv::Mat main_translation = (cv::Mat_<double>(3,3) << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0 );
    cv::Mat sec_translation = (cv::Mat_<double>(3,3) << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0 );
    int num = 0;  // TODO: num = classWithoutCar;
    int half_classWithoutCar;
    int classWithoutCar;

    PretreatObjs(OurPattern ourPattern,std::string config_path);
    PretreatObjs(std::shared_ptr<SensorParam> MainCam_ptr,std::shared_ptr<SensorParam> SecCam_ptr, bool sec_is_left,std::string config_path);

    void set_windmill_car(std::vector<int> windmill_car);
    bool check_color(cv::Mat armor,int cls);

    TRTInferV1::DetectionObj objs2newMainObjs(TRTInferV1::DetectionObj objs, cv::Mat obj2Main);
    TRTInferV1::DetectionObj objs2newMainObjs(interfaces::msg::Rect objs, cv::Mat obj2Main);

    template<typename T>// T Car or Strack
    void update_classfy(T &objs, int num, std::vector<T> &outRedObjs, std::vector<T> &outBlueObjs,std::vector<T> &outRestObjs);
    void update_classfy(int &temp_bestcls,float &conf_armor ,Eigen::MatrixXd &car_armorConfMatrix);

    void set_confs_by_locate3D(double &windmill_car_conf, double &startupArea_car_conf);
    bool get_Armors_w_conf_Double_net(STrack &car, std::vector<TRTInferV1::DetectionObj> armors,cv::Mat &car_img);

};


#endif //JSONCPP_TEST_PRETREATOBJS_H
