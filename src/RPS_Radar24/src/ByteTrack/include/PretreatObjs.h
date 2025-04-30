//
// Created by plusseven on 23-12-20.
//

#ifndef JSONCPP_TEST_PRETREATOBJS_H
#define JSONCPP_TEST_PRETREATOBJS_H
//#include "STrack.h"
#include "../../Net/include/Inference.h"
#include "../../General/include/SensorParam.h"
#include "CostMatrix.h"


class PretreatObjs {
private:
    std::shared_ptr<CostMatrix> costMatrix_ptr = std::shared_ptr<CostMatrix>(new CostMatrix());
    OurPattern ourPattern;
    int allCam = 2;
    bool isBR = false;
//    bool isGuess = false;
    int maxSize;
    int aGroupOfArmor = 6;
    int groupNum = 2;
    std::vector<std::vector<double>> W_of_armorConfs;

    float p_car_midpoint = 0.96;

    float p_carY_downLine = 3./4;
    float p_carY_upLine = 1./3;

    // 2cam
    int tx, ty, result_w, result_h, img1_w, img1_h, img2_w, img2_h;
    int sec2main_w = 1920,  sec2main_h = 1440;
    bool sec_is_left;

    //
    std::vector<int> windmill_car;
    // double sum_windmill_car_conf = 0.84;
    double sum_windmill_car_conf = 0.56;
    double startupArea_car_conf = 0.6;
    double windmill_car_conf;

public:
    cv::Mat H, H2, perspective_K;
    cv::Mat main_translation = (cv::Mat_<double>(3,3) << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0 );
    cv::Mat sec_translation = (cv::Mat_<double>(3,3) << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0 );
    int num = 0;  // TODO: num = classWithoutCar;
    int half_classWithoutCar;
    int classWithoutCar;

    PretreatObjs(OurPattern ourPattern);
    PretreatObjs(std::shared_ptr<SensorParam> MainCam_ptr,std::shared_ptr<SensorParam> SecCam_ptr, bool sec_is_left);

    void getW_of_armorConfs(int maxSize);
    void set_windmill_car(std::vector<int> windmill_car);

    TRTInferV1::DetectionObj objs2newMainObjs(TRTInferV1::DetectionObj objs, cv::Mat obj2Main);

    TRTInferV1::DetectionObj allObjs2newMainObjs(TRTInferV1::DetectionObj mainObjs, TRTInferV1::DetectionObj secObjs,
         cv::Mat &img1, cv::Mat &img2);

    std::vector<TRTInferV1::DetectionObj> secObjs2mainObjs(
            std::vector<TRTInferV1::DetectionObj> mainObjs,std::vector<TRTInferV1::DetectionObj> secObjs,
            std::shared_ptr<SensorParam> MainCam_ptr,std::shared_ptr<SensorParam> SecCam_ptr,
            float match_thresh,std::vector<std::vector<int>> &list_);

    std::vector<TRTInferV1::Object> secObjs2mainObjs(
            std::vector<TRTInferV1::Object> mainObjs,std::vector<TRTInferV1::Object> secObjs,
            std::shared_ptr<SensorParam> MainCam_ptr,std::shared_ptr<SensorParam> SecCam_ptr,
            float match_thresh,std::vector<std::vector<int>> &list_);

    TRTInferV1::DetectionObj secObjs2mainObjs(TRTInferV1::DetectionObj secObjs,
            std::shared_ptr<SensorParam> MainCam_ptr,std::shared_ptr<SensorParam> SecCam_ptr);

    TRTInferV1::Object secObjs2mainObjs(TRTInferV1::Object secObjs,
            std::shared_ptr<SensorParam> MainCam_ptr,std::shared_ptr<SensorParam> SecCam_ptr);

    void Car_Armor(
            std::vector<TRTInferV1::DetectionObj> &DetectionObjs,
            std::vector<Car> &cars,std::vector<Armor> &armors);

    std::vector<std::vector<std::vector<int>>>  Car_Armor(
            std::vector<TRTInferV1::Object> DetectionObjs,std::vector<std::vector<int>> list_,
            std::vector<TRTInferV1::Object> &cars,std::vector<TRTInferV1::Object> &armors);


    void ArmorInCar(std::vector<Car> &cars,std::vector<Armor> &armors);
    std::vector<STrack> ArmorInCar(std::vector<TRTInferV1::Object> &cars,std::vector<TRTInferV1::Object> &armors);

    void getArmors_wconf(std::vector<Car> &cars,std::vector<Car> &outRedCars,std::vector<Car> &outBlueCars,std::vector<Car> &outRestCars,std::vector<Armor> &armors);
    std::vector<std::vector<int>>  getArmors_wconf(
            const std::vector<TRTInferV1::Object>& cars,std::vector<std::vector<TRTInferV1::Object>> allDetectionObjs,
            std::vector<std::vector<std::vector<int>>> allLists, std::vector<STrack> &outRestCars,std::vector<TRTInferV1::Object> &armors);
    void getLastCar(std::vector<Armor> &armors,std::vector<Car> &outLastCars);
    void getLastCar(
            std::vector<TRTInferV1::Object> &armors,std::vector<std::vector<TRTInferV1::Object>> allDetectionObjs,
            std::vector<std::vector<int>> armorLists,std::vector<STrack> &outSTracks);

    template<typename T>// T Car or Strack
    void update_classfy(T &objs, int num, std::vector<T> &outRedObjs, std::vector<T> &outBlueObjs,std::vector<T> &outRestObjs);
    void update_classfy(int &temp_bestcls,float &conf_armor ,Eigen::MatrixXd &car_armorConfMatrix);

    void reassign_cls(int num, int &temp_bestcls);

    std::vector<std::vector<STrack>> classfySTrackByCam(
            std::vector<STrack> &STracks, std::vector<std::vector<int>> newLists);

    std::vector<std::vector<STrack>> getSTrackwithArmor(
            std::vector<TRTInferV1::Object> DetectionObjs, std::vector<std::vector<TRTInferV1::Object>> allDetectionObjs,
            std::vector<std::vector<int>> list_);

    void set_confs_by_locate3D(double &windmill_car_conf, double &startupArea_car_conf);
    void get_Armors_w_conf_Double_net(STrack &car, std::vector<TRTInferV1::Object> armors);
    bool get_Armors_w_conf_Double_net(STrack &car, std::vector<TRTInferV1::DetectionObj> armors);

    //    void updataStrack_ws_confMatrixs(STrack &sTrack);
};


#endif //JSONCPP_TEST_PRETREATOBJS_H
