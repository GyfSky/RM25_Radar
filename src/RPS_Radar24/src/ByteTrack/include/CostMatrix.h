//
// Created by plusseven on 23-12-10.
//
#ifndef JSONCPP_TEST_COSTMATRIX_H
#define JSONCPP_TEST_COSTMATRIX_H
#include "STrack.h"
#include "lapjv.h"
#include "../../Net/include/Inference.h"
//#include "../../General/include/General.h"


//template<typename T>// T Car or Strack
class CostMatrix {
public:
    int classWithoutCar;
    int half_classWithoutCar;
    float w_rectDistance;
    float rectDistance_thresh;
    float w_3dDistance;
    float distance_thresh;

    OurPattern ourPattern;

    float step_3dDistance = 0.7;
    float step_rectDistance = 300;  //TODO:________________GAME

    bool is_predict = true;
    bool is_3Dplace_decide_step_rectDistance = true;
    float k;
    float max_rect_place = 7 ;
    float max_rect_step  = 600;     // 位置因素  5m
    float min_rect_place = 25.;
    float min_rect_step  = 200 ;     // 位置因素  23m

public:
    CostMatrix(OurPattern ourPattern,std::string config_path);
    CostMatrix() = default;

//    Eigen::MatrixXd getIouAndDistancetCost(vector<STrack> &atracks, vector<STrack> &btracks, bool isBR);
    Eigen::MatrixXd getIouAndDistancetCost(vector<STrack*> &atracks, vector<STrack> &btracks, int &atracks_size, int &btracks_size, bool isBR) ;
    vector<vector<float>> getIouAndDistancetCost_vec(vector<STrack*> &atracks, vector<STrack> &btracks, int &atracks_size, int &btracks_size, bool isBR);
    Eigen::MatrixXd getCost_confMatrix(std::vector<STrack*> &strack_pool, int &num_strack,int &num_cls);
    Eigen::MatrixXd getCost_confMatrix(std::vector<STrack> &strack_pool, int &num_strack,int &num_cls);
    Eigen::MatrixXd getIouAndDistancetCost(vector<TRTInferV1::DetectionObj> main_obj,
                                           vector<TRTInferV1::DetectionObj> sec_obj,int &main_obj_size,int &sec_obj_size);
    Eigen::MatrixXd getIouAndDistancetCost(vector<TRTInferV1::Object> main_obj,
                                           vector<TRTInferV1::Object> sec_obj,int &main_obj_size,int &sec_obj_size);

    Eigen::MatrixXd getIouAndDistancetCost(
            vector<STrack> main_obj,vector<STrack> sec_obj,int &main_obj_size,int &sec_obj_size);
    void linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
                           vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b);
    double lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol,
                 bool extend_cost = false, float cost_limit = LONG_MAX, bool return_cost = true);


};


#endif //JSONCPP_TEST_COSTMATRIX_H
