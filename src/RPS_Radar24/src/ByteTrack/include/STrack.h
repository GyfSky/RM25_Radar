#pragma once

#include <opencv2/opencv.hpp>
#include "kalmanFilter.h"

using namespace cv;
using namespace std;

enum TrackState { New = 0, Tracked, Lost, Removed, LostCopy, LostCopy_PredictOver };
enum PlaceType_special { flySlope, hole, ordinary, four, windmill, is_windmill,holeWarring, startupArea,steps,stepsWarring };

//class STrack : public Car
class STrack
{
public:
    STrack(int cls, float x, float y);
    STrack(vector<float> tlwh_, Car &car);
    STrack(float x1, float y1, float w, float h, int cls,float conf, float conf_armor, Eigen::MatrixXd car_armorConfMatrix);
    STrack(float x1, float y1, float w, float h, int cls,float conf, float conf_armor);
	STrack();
	~STrack();

    STrack(float x1, float y1, float w, float h, float conf,int classWithoutCar);
    void init_track(int cls, float conf_armor, Eigen::MatrixXd car_armorConfMatrix);
    void updataSTrack(OurPattern ourPattern);
    void setRectInPrimaryCam(float x1, float y1, float w, float h, float p);
//    void setRectInPrimaryCam(float p);

	vector<float> static tlbr_to_tlwh(vector<float> &tlbr);
    void updataStrack_ws_confMatrixs(double new_armorConf,Eigen::MatrixXd &ws_confMatrix ,Eigen::MatrixXd &ws_armorConfMatrix);
    void classfy_STrack_N(int &num);
    void updata_trackid(int &num);
    void static_tlwh(bool is3D = true);
	void static_tlbr();
	void static_3D_velocity();
	vector<float> tlwh_to_xyah(vector<float> tlwh_tmp);
	vector<float> to_xyah();
	void mark_lost();
	void mark_lostCopy();
	void mark_removed();
	int next_id();
	int end_frame();

	void activate(byte_kalman::KalmanFilter &kalman_filter, int frame_id, int &num,bool is3D = true);
	void re_activate(STrack &new_track, int frame_id,  OurPattern ourPattern, std::vector<int> windmill_car,bool new_id = false, bool is3D = true);
	void update(STrack &new_track, int frame_id,  OurPattern ourPattern, std::vector<int> windmill_car,bool is3D = true);
    void update_lose(int frame_id,double final_max_conf,bool is3D = true);
    void set_confs_by_locate3D(STrack &new_track,OurPattern ourPattern, std::vector<int> windmill_car);
    void push_front_vexSerialNum(int serialNum);
    void push_front_change_Locate3D_and_distance(cv::Point3d Locate3D);

//    void update_lose_track();

public:
// car
// public:
//    //param
    float conf=0.0;//车辆的置信度（第一层网络）
	float conf_armor=0.0;//装甲板的置信度（第二层网络）
//    x1=0.0,y1=0.0,x2=0.0,y2=0.0;
    cv::Point2d Locate2D;
    cv::Point3d Locate3D;
//    std::vector<Armor> ArmorsInCar; //TODO: del
//    bool isGuess= false;    // TODO：del
    // Eigen::MatrixXd  ws_armorConfMatrix = Eigen::MatrixXd::Zero(1,12);  //TODO:？？
	Eigen::MatrixXd  ws_armorConfMatrix = Eigen::MatrixXd::Zero(1,10);
    Eigen::MatrixXd  ws_armorConfMatrix_BR = Eigen::MatrixXd::Zero(1,6);
//    cv::Rect rect;
//车辆编号？
    int cls = -1;

	bool is_activated;
	int track_id;//？？
	int state;

    cv::Point3d _Locate3D;
	vector<float> _tlwh;
	vector<float> tlwh;     // tlx, tly, w, h
	vector<float> tlbr; 	// 对角x , y

	int frame_id;
    int lost_frame_ind_num=0;//丢失帧数 ,影响置信度（降低）与位置猜测
	int tracklet_len;//跟踪次数
	int start_frame;
    int cls_len_time = 0;//识别为同一类的次数

    double maxUpdataW = 1.0/3;
    int classWithoutCar;
    int half_classWithoutCar;

	KAL_MEAN mean;
	KAL_COVA covariance;

    KAL_MEAN_3d mean3D;
    KAL_COVA_3d cova3D;

    double oldH;
    std::vector<int> vexSerialNum;
    int false_Hs = 0;

    cv::Point3d old_Locate3D;
    std::vector<cv::Point3d> change_Locate3Ds;
    std::vector<double> change_distance;

    PlaceType_special placeType = PlaceType_special::ordinary;
    float vx_3d = 0.0, vy_3d = 0.0;

    int judge_radar_mark_data = 0;

    double startupArea_car_conf;
    double windmill_car_conf;
    double up_magnification = 1.25;
// predict
    double speed_vel = 0.8;
    double angle_vel = 0.0;
    double angle_acc = 0.0;
    double speed_acc = 0.0;
    double angle;

    double p_focus = 0.98;

	bool is_det=false;
	int camid=0;
	vector<float> sec_rect;
private:
	byte_kalman::KalmanFilter kalman_filter;
};