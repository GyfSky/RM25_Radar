#pragma once

#include "CostMatrix.h"
#include <interfaces/msg/detect_result.hpp>

//struct Object
//{
//    cv::Rect_<float> rect;
//    int label;
//    float prob;
//};

class BYTETracker : private CostMatrix
{
public:
	BYTETracker(OurPattern ourPattern);
	~BYTETracker();

//	vector<STrack> update(const vector<Object>& objects);
	void update(vector<STrack> &tracked_stracks, vector<STrack> &lost_stracks, vector<Car> &cars);
	void update(vector<STrack> &tracked_stracks, vector<STrack> &lost_stracks, vector<STrack> &lost_predict_stracks,vector<STrack> &detections, vector<STrack> &out);
    void update(vector<STrack> &tracked_stracks, vector<STrack> &lost_stracks, vector<STrack> &lost_predict_stracks,vector<STrack> &detections, vector<STrack> &out,interfaces::msg::DetectResult lidar_det);

private:
    void set_windmill_car(std::vector<int> windmill_car);
	vector<STrack*> joint_stracks(vector<STrack*> &tlista, vector<STrack> &tlistb,bool isRepeat= true);
	vector<STrack> joint_stracks(vector<STrack> &tlista, vector<STrack> &tlistb);

	vector<STrack> sub_stracks(vector<STrack> &tlista, vector<STrack> &tlistb,bool isRepeat= true);
    vector<STrack*> joint_stracks(vector<STrack*> &tlista);
	void remove_duplicate_stracks(vector<STrack> &resa, vector<STrack> &resb, vector<STrack> &stracksa, vector<STrack> &stracksb);

//    void classfy_STrack_N(STrack* &obj, int num);
//    void classfy_STrack_N(STrack &obj, int num);
    bool static sort_dependOntheframe(STrack obj1,STrack obj2){
        return (obj1.lost_frame_ind_num <=  obj2.lost_frame_ind_num) ;
    };
	vector<vector<float> > iou_distance(vector<STrack*> &atracks, vector<STrack> &btracks, int &dist_size, int &dist_size_size);
	vector<vector<float> > iou_distance(vector<STrack> &atracks, vector<STrack> &btracks);
	vector<vector<float> > ious(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs);

    void multi_predict(vector<STrack*> &stracks, byte_kalman::KalmanFilter &kalman_filter, bool is3D = true);
    void multi_predict(vector<STrack> &stracks, byte_kalman::KalmanFilter &kalman_filter, bool is3D = true);

    Scalar get_color(int idx);
public:
    double dt = 1.0/3;

private:
    int num = 0;//??
    double final_max_conf = 2./3.;

    int cls_lost_frame_ind_num =  10000;
    int no_lost_frame_ind_num =  100;

    float max_predict_3Ddistance = 1.5;

    float track_thresh;
	float high_thresh;
    float high_car_thresh;
	float match_thresh;
	float match_cls_thresh;

    bool is_detectionlow = true;
    bool is_cls = true;
    bool is_save_no_cls_track = true;

    int frame_id;//累积帧数，即开始运行到当前的累积帧数
	int max_time_lost;

//	vector<STrack> tracked_stracks;
//	vector<STrack> lost_stracks;
    std::vector<int> windmill_car; //打符车的标号
	vector<STrack> removed_stracks;
	byte_kalman::KalmanFilter kalman_filter;

};