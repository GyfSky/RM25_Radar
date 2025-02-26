#include <utility>

#include "../include/STrack.h"

STrack::STrack(int cls, float x, float y){
    this->cls = cls;
    this->Locate3D = cv::Point3d(x,y,-1.0);
}

STrack::STrack(){
    this->cls = -1;
    this->Locate3D = cv::Point3d(0.0,0.0,0.0);
//    this->ws_armorConfMatrix = Eigen::MatrixXd::Zero(0,12); // TODO:
}

STrack::STrack(vector<float> tlwh_, Car &car) {
//    vexSerialNum.resize(5);
    _tlwh.resize(4);
    //assign函数将tlwh_容器中的元素复制到tlwh容器中，从而使tlwh容器中的内容与tlwh_相同。
    _tlwh.assign(tlwh_.begin(), tlwh_.end());
    _Locate3D = car.Locate3D;

    is_activated = false; // default = fales
//    is_activated = true;
    track_id = -1;
    state = TrackState::New;

    tlwh.resize(4);
    tlbr.resize(4);


    frame_id = 0;
    lost_frame_ind_num = 0;
    tracklet_len = 0;
    start_frame = 0;

//    _tlwh.max_size();

    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    this->classWithoutCar = config["net"]["classWithoutCar"].as<int>();
    this->half_classWithoutCar = classWithoutCar/2;

    this->cls = car.cls;
    this->conf = car.conf;
    this->conf_armor = car.conf_armor;
    this->Locate3D = car.Locate3D;
    this->Locate2D = car.Locate2D;
//    this->rect = car.rect;

//    this->isGuess= car.isGuess;

    this->ws_armorConfMatrix = car.ws_armorConfMatrix;
    this->ws_armorConfMatrix_BR = car.ws_armorConfMatrix_BR;

//    if( -1 < this->cls && this->cls < classWithoutCar ){
//        this->track_id = this->cls%half_classWithoutCar + this->cls/half_classWithoutCar*7;
//    }

    static_tlwh();
    static_tlbr();
}


STrack::STrack(float x1, float y1, float w, float h, int cls,float conf, float conf_armor, Eigen::MatrixXd car_armorConfMatrix) {
//    vexSerialNum.resize(5);
    _tlwh.resize(4);
    _tlwh = {x1, y1, w, h};
    //assign函数将tlwh_容器中的元素复制到tlwh容器中，从而使tlwh容器中的内容与tlwh_相同。
//    _tlwh.assign(tlwh_.begin(), tlwh_.end());
//    _Locate3D = car.Locate3D; //TODO:

    is_activated = false; // default = fales
//    is_activated = true;
    track_id = -1;
    state = TrackState::New;

    tlwh.resize(4);
    tlbr.resize(4);


    frame_id = 0;
    lost_frame_ind_num = 0;
    tracklet_len = 0;
    start_frame = 0;

//    _tlwh.max_size();

    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    this->classWithoutCar = config["net"]["classWithoutCar"].as<int>();
    this->half_classWithoutCar = classWithoutCar/2;

    this->cls = cls;
    this->conf = conf;
    this->conf_armor = conf_armor;
//    this->Locate3D = car.Locate3D;  // TODO:
//    this->Locate2D = car.Locate2D;  // TODO：
//    this->rect = car.rect;
//    this->isGuess= car.isGuess;

    this->ws_armorConfMatrix = std::move(car_armorConfMatrix);
//    std::cout << "ws_armorConfMatrix.size  " << ws_armorConfMatrix << std::endl;
//    this->ws_armorConfMatrix_BR = car.ws_armorConfMatrix_BR;      // TODO:

    if( -1 < this->cls && this->cls < classWithoutCar ){
        this->track_id = cls;
    }

    static_tlwh();
    static_tlbr();
}

void STrack::updataSTrack(OurPattern ourPattern) {
    _Locate3D = this->Locate3D; //TODO:
//    if(ourPattern == blue){
//        distance_to_radar = 28. - this->Locate3D.x;
//    }
}

void STrack::setRectInPrimaryCam(float x1, float y1, float w, float h, float p) {
    Locate2D = cv::Point2d (x1 + w/2.0,y1 + h*p);
}

//void STrack::setRectInPrimaryCam(float p) {
//    Locate2D = cv::Point2d (tlwh[0] + tlwh[2]/2.0,tlwh[1] + tlwh[3]*p);
//}

STrack::STrack(float x1, float y1, float w, float h, int cls,float conf, float conf_armor) {
//    vexSerialNum.resize(5);
    _tlwh.resize(4);
    _tlwh = {x1, y1, w, h};
    //assign函数将tlwh_容器中的元素复制到tlwh容器中，从而使tlwh容器中的内容与tlwh_相同。
//    _tlwh.assign(tlwh_.begin(), tlwh_.end());
//    _Locate3D = car.Locate3D; //TODO:

    is_activated = false; // default = fales
//    is_activated = true;
    track_id = -1;
    state = TrackState::New;

    tlwh.resize(4);
    tlbr.resize(4);


    frame_id = 0;
    lost_frame_ind_num = 0;
    tracklet_len = 0;
    start_frame = 0;

//    _tlwh.max_size();

    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    this->classWithoutCar = config["net"]["classWithoutCar"].as<int>();
    this->half_classWithoutCar = classWithoutCar/2;

    this->cls = cls;
    this->conf = conf;
    this->conf_armor = conf_armor;

//    this->Locate3D = car.Locate3D;  // TODO:
//    this->Locate2D = car.Locate2D;  // TODO：
//    this->rect = car.rect;
//    this->isGuess= car.isGuess;

//    this->ws_armorConfMatrix = std::move(car_armorConfMatrix);    // TODO:
//    this->ws_armorConfMatrix = car.ws_armorConfMatrix;            // TODO:
//    this->ws_armorConfMatrix_BR = car.ws_armorConfMatrix_BR;      // TODO:

    if( -1 < this->cls && this->cls < classWithoutCar ){
        this->track_id = this->cls%half_classWithoutCar + this->cls/half_classWithoutCar*7;
    }

    static_tlwh();
    static_tlbr();
}

STrack::STrack(float x1, float y1, float w, float h, float conf) {
    //    vexSerialNum.resize(5);
    _tlwh.resize(4);
    _tlwh = {x1, y1, w, h};
    this->conf = conf;

    //assign函数将tlwh_容器中的元素复制到tlwh容器中，从而使tlwh容器中的内容与tlwh_相同。
//    _tlwh.assign(tlwh_.begin(), tlwh_.end());
//    _Locate3D = car.Locate3D; //TODO:

    is_activated = false; // default = fales
//    is_activated = true;
    track_id = -1;
    state = TrackState::New;

    tlwh.resize(4);
    tlbr.resize(4);


    frame_id = 0;
    lost_frame_ind_num = 0;
    tracklet_len = 0;
    start_frame = 0;

//    _tlwh.max_size();

    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    this->classWithoutCar = config["net"]["classWithoutCar"].as<int>();
    this->half_classWithoutCar = classWithoutCar/2;


//    this->Locate3D = car.Locate3D;  // TODO:
//    this->Locate2D = car.Locate2D;  // TODO：
//    this->rect = car.rect;
//    this->isGuess= car.isGuess;


    if( -1 < this->cls && this->cls < classWithoutCar ){
        this->track_id = cls;
    }

    static_tlwh();
    static_tlbr();
}

void STrack::init_track(int cls, float conf_armor, Eigen::MatrixXd car_armorConfMatrix) {
    this->cls = cls;
    this->conf_armor = conf_armor;
    this->ws_armorConfMatrix = std::move(car_armorConfMatrix);
}


STrack::~STrack()
{
}

void STrack::set_confs_by_locate3D(STrack &new_track,OurPattern ourPattern, std::vector<int> windmill_car){
    if(new_track.placeType == windmill){
        this->windmill_car_conf = this->windmill_car_conf * this->up_magnification;
        for(auto cls: windmill_car){
            new_track.ws_armorConfMatrix(0, cls)  = std::min(0.84, new_track.ws_armorConfMatrix(0, cls) + this->windmill_car_conf);
        }
//        if(this->windmill_car_conf > new_track.conf_armor){
//            new_track.conf_armor = this->windmill_car_conf;
//        }
    }
    else if(new_track.placeType == startupArea){
        this->startupArea_car_conf = this->startupArea_car_conf * this->up_magnification;
        if(ourPattern==red){
            new_track.ws_armorConfMatrix(0, 5) = std::min(0.84,  new_track.ws_armorConfMatrix(0, 5) + startupArea_car_conf);
//            if(new_track.ws_armorConfMatrix(0, 5) > new_track.conf_armor){
//                new_track.conf_armor = new_track.ws_armorConfMatrix(0, 5);
//            }
        }
        else if(ourPattern==blue){
            new_track.ws_armorConfMatrix(0, 11)  = std::min(0.84,  new_track.ws_armorConfMatrix(0, 11) + startupArea_car_conf);
//            if(new_track.ws_armorConfMatrix(0, 11) > new_track.conf_armor){
//                new_track.conf_armor = new_track.ws_armorConfMatrix(0, 11);
//            }
        }
//        std::cout << "this->startupArea_car_conf :" << this->startupArea_car_conf << ", " << new_track.ws_armorConfMatrix <<std::endl;
    }else{
        this->startupArea_car_conf = new_track.startupArea_car_conf;
        this->windmill_car_conf = new_track.windmill_car_conf;
    }
    this->placeType = new_track.placeType;
}



/**
 * @brief （只在新轨迹中使用）更新当前帧的新track的卡尔曼滤波， 加载当前帧的id
 * @param  frame_id 当前帧对应的id(即从目标跟踪代码运行到现在，目标跟踪一共运行了多少次)，
 *              用于记录当前跟踪器所处的时刻，便于计算该跟踪器是否丢失，以及丢失时长等等
 *      // start_frame 用于记录该跟踪器的开始时刻
 *
 * **/
void STrack::activate(byte_kalman::KalmanFilter &kalman_filter, int frame_id, int &num,bool is3D)
{
	this->kalman_filter = kalman_filter;
    classfy_STrack_N(num);
	vector<float> _tlwh_tmp(4);
	_tlwh_tmp[0] = this->_tlwh[0];
	_tlwh_tmp[1] = this->_tlwh[1];
	_tlwh_tmp[2] = this->_tlwh[2];
	_tlwh_tmp[3] = this->_tlwh[3];
	vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
    if(is3D){
        DETECTBOX_Z x2d_y2d_a_h_x3d_y3d;
        x2d_y2d_a_h_x3d_y3d[0] = xyah[0];
        x2d_y2d_a_h_x3d_y3d[1] = xyah[1];
        x2d_y2d_a_h_x3d_y3d[2] = xyah[2];
        x2d_y2d_a_h_x3d_y3d[3] = xyah[3];
        x2d_y2d_a_h_x3d_y3d[4] = _Locate3D.x;
        x2d_y2d_a_h_x3d_y3d[5] = _Locate3D.y;
        auto mc = this->kalman_filter.initiate(x2d_y2d_a_h_x3d_y3d);
        this->mean3D = mc.first;
        this->cova3D = mc.second;
//        std::cout << "this->mean3D  activate " << this->mean3D  << std::endl;
//        std::cout << "this->cova3D  activate " << this->cova3D  << std::endl;
    } else{
        DETECTBOX xyah_box;
        xyah_box[0] = xyah[0];
        xyah_box[1] = xyah[1];
        xyah_box[2] = xyah[2];
        xyah_box[3] = xyah[3];
        auto mc = this->kalman_filter.initiate(xyah_box);
        this->mean = mc.first;
        this->covariance = mc.second;
//        std::cout << "this->mean  activate " << this->mean  << std::endl;
//        std::cout << "this->cova  activate " << this->covariance  << std::endl;
    }

	static_tlwh();
	static_tlbr();
    if(is3D){
        static_3D_velocity();
    }

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	if (frame_id == 1)
	{
		this->is_activated = true;
	}
	//this->is_activated = true;
	this->frame_id = frame_id;
	this->start_frame = frame_id;
    this->lost_frame_ind_num = 0;
}

void STrack::re_activate(STrack &new_track, int frame_id, OurPattern ourPattern,std::vector<int> windmill_car,bool new_id, bool is3D)
{
    cv::Point3d old_Locate3D = this->Locate3D;
    this->_Locate3D = new_track.Locate3D;

    this->tracklet_len = 0;     // re_activate 与 update 唯一的不同
    this->state = TrackState::Tracked;
    this->is_activated = true;
    this->frame_id = frame_id;
    this->lost_frame_ind_num=0;

    this->Locate2D = new_track.Locate2D;
    this->conf       = new_track.conf;
//    this->Locate3D.z   = new_track.Locate3D.z;
//    this->rect = new_track.rect;

//    this->isGuess= new_track.isGuess;

//    this->vexSerialNum = new_track.vexSerialNum;
//    this->false_Hs = new_track.false_Hs;
//    this->change_distance = new_track.change_distance;
//    this->change_Locate3Ds = new_track.change_Locate3Ds;
//    this->old_Locate3D = new_track.old_Locate3D;
//    this->oldH = new_track.oldH;
    set_confs_by_locate3D(new_track, ourPattern, windmill_car);
//    std::cout << "1 this->ws_armorConfMatrix: " << this->ws_armorConfMatrix << std::endl;
    updataStrack_ws_confMatrixs(new_track.conf_armor,new_track.ws_armorConfMatrix,this->ws_armorConfMatrix);
//    std::cout << "2 this->ws_armorConfMatrix: " << this->ws_armorConfMatrix << std::endl;
//    classfy_STrack_N(num);


    vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
    if(is3D){
        DETECTBOX_Z x2d_y2d_a_h_x3d_y3d;
        x2d_y2d_a_h_x3d_y3d[0] = xyah[0];
        x2d_y2d_a_h_x3d_y3d[1] = xyah[1];
        x2d_y2d_a_h_x3d_y3d[2] = xyah[2];
        x2d_y2d_a_h_x3d_y3d[3] = xyah[3];
        x2d_y2d_a_h_x3d_y3d[4] = new_track.Locate3D.x;
        x2d_y2d_a_h_x3d_y3d[5] = new_track.Locate3D.y;
        auto mc = this->kalman_filter.update(this->mean3D, this->cova3D, x2d_y2d_a_h_x3d_y3d);
        this->mean3D = mc.first;
        this->cova3D = mc.second;
//        std::cout << "this->mean3D  re_activate " << this->mean3D  << std::endl;
//        std::cout << "this->cova3D  re_activate " << this->cova3D  << std::endl;
    } else{
        DETECTBOX xyah_box;
        xyah_box[0] = xyah[0];
        xyah_box[1] = xyah[1];
        xyah_box[2] = xyah[2];
        xyah_box[3] = xyah[3];
        auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
        this->mean = mc.first;
        this->covariance = mc.second;
//        std::cout << "this->mean  re_activate " << this->mean  << std::endl;
//        std::cout << "this->cova  re_activate " << this->covariance  << std::endl;
    }

	static_tlwh();
	static_tlbr();
    if(is3D){
        static_3D_velocity();
    }

//    push_front_change_Locate3D_and_distance((this->Locate3D - old_Locate3D));

}

void STrack::update(STrack &new_track, int frame_id, OurPattern ourPattern,std::vector<int> windmill_car,bool is3D)
{
    cv::Point3d old_Locate3D = this->Locate3D;
    this->_Locate3D = new_track.Locate3D;

    this->frame_id = frame_id;
    this->lost_frame_ind_num=0;
    this->tracklet_len++; // re_activate 与 update 唯一的不同
    this->state = TrackState::Tracked;
    this->is_activated = true;

    this->Locate2D = new_track.Locate2D;
    this->conf       = new_track.conf;
//    this->Locate3D.z   = new_track.Locate3D.z;
//    this->rect = new_track.rect;

//    this->isGuess= new_track.isGuess;

//    this->vexSerialNum = new_track.vexSerialNum;
//    this->false_Hs = new_track.false_Hs;
//    this->change_distance = new_track.change_distance;
//    this->change_Locate3Ds = new_track.change_Locate3Ds;
//    this->old_Locate3D = new_track.old_Locate3D;
//    this->oldH = new_track.oldH;
    set_confs_by_locate3D(new_track, ourPattern, windmill_car);
//    std::cout << "1 this->ws_armorConfMatrix: " << this->ws_armorConfMatrix << std::endl;
//根据新轨迹的置信度 更新 当前跟踪器的置信度
    updataStrack_ws_confMatrixs(new_track.conf_armor,new_track.ws_armorConfMatrix,this->ws_armorConfMatrix);
//    std::cout << "2 this->ws_armorConfMatrix: " << this->ws_armorConfMatrix << std::endl;
//    classfy_STrack_N(num);

	vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
    if(is3D){
        DETECTBOX_Z x2d_y2d_a_h_x3d_y3d;
        x2d_y2d_a_h_x3d_y3d[0] = xyah[0];
        x2d_y2d_a_h_x3d_y3d[1] = xyah[1];
        x2d_y2d_a_h_x3d_y3d[2] = xyah[2];
        x2d_y2d_a_h_x3d_y3d[3] = xyah[3];
        x2d_y2d_a_h_x3d_y3d[4] = new_track.Locate3D.x;
        x2d_y2d_a_h_x3d_y3d[5] = new_track.Locate3D.y;
//        std::cout << "x2d_y2d_a_h_x3d_y3d: " << x2d_y2d_a_h_x3d_y3d << std::endl;
        auto mc = this->kalman_filter.update(this->mean3D, this->cova3D, x2d_y2d_a_h_x3d_y3d);
        this->mean3D = mc.first;
        this->cova3D = mc.second;
//        std::cout << "this->mean3D  update " << this->mean3D  << std::endl;
//        std::cout << "this->cova3D  update " << this->cova3D  << std::endl;
    } else{
        DETECTBOX xyah_box;
        xyah_box[0] = xyah[0];
        xyah_box[1] = xyah[1];
        xyah_box[2] = xyah[2];
        xyah_box[3] = xyah[3];
        auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
        this->mean = mc.first;
        this->covariance = mc.second;
//        std::cout << "this->mean  update " << this->mean  << std::endl;
//        std::cout << "this->cova  update " << this->covariance  << std::endl;
    }

	static_tlwh();
	static_tlbr();
    if(is3D){
        static_3D_velocity();
    }


//    push_front_change_Locate3D_and_distance(Locate3D);
}



void STrack::update_lose(int frame_id,double final_max_conf,bool is3D)
{
//    cv::Point3d old_Locate3D = this->Locate3D;  //TODO:

//this->frame_id = frame_id;
    this->lost_frame_ind_num++;
    this->tracklet_len=0;
    this->is_activated = true;

    std::cout << this->cls << "  " << lost_frame_ind_num << std::endl;

//    this->Locate3D.z   = new_track.Locate3D.z;
//    this->rect = new_track.rect;

//    this->isGuess= new_track.isGuess;

//    this->vexSerialNum = new_track.vexSerialNum;
//    this->false_Hs = new_track.false_Hs;
//    this->change_distance = new_track.change_distance;
//    this->change_Locate3Ds = new_track.change_Locate3Ds;
//    this->old_Locate3D = new_track.old_Locate3D;
//    this->oldH = new_track.oldH;

//    if(lost_frame_ind_num < 10 || lost_frame_ind_num > 50){
//        Eigen::MatrixXd zero_armorConfMatrix = Eigen::MatrixXd::Ones(1,14) * 0.1;
//        updataStrack_ws_confMatrixs((1.-final_max_conf),zero_armorConfMatrix,this->ws_armorConfMatrix);
//        Eigen::MatrixXd zero_armorConfMatrix = Eigen::MatrixXd::Zero(1,14) ;
//        std::cout << "!!!!!!  " << lost_frame_ind_num << std::endl;
//    }
    if(lost_frame_ind_num < 5 || lost_frame_ind_num > 40) {
        Eigen::MatrixXd zero_armorConfMatrix = Eigen::MatrixXd::Ones(1, classWithoutCar) * 0.095;
        ws_armorConfMatrix = 1. / 4 * zero_armorConfMatrix + 3. / 4 * ws_armorConfMatrix;
    }

//    if(state == LostCopy){
//        //    vector<float> xyah = tlwh_to_xyah(new_track.tlwh);   //TODO:
//        vector<float> xyah = tlwh_to_xyah(tlwh);   //TODO:
//        if(is3D){
//            DETECTBOX_Z x2d_y2d_a_h_x3d_y3d;
//            x2d_y2d_a_h_x3d_y3d[0] = xyah[0];
//            x2d_y2d_a_h_x3d_y3d[1] = xyah[1];
//            x2d_y2d_a_h_x3d_y3d[2] = xyah[2];
//            x2d_y2d_a_h_x3d_y3d[3] = xyah[3];
//            x2d_y2d_a_h_x3d_y3d[4] = Locate3D.x;
//            x2d_y2d_a_h_x3d_y3d[5] = Locate3D.y;
////        std::cout << "x2d_y2d_a_h_x3d_y3d: " << x2d_y2d_a_h_x3d_y3d << std::endl;
//            auto mc = this->kalman_filter.update(this->mean3D, this->cova3D, x2d_y2d_a_h_x3d_y3d);
//            this->mean3D = mc.first;
//            this->cova3D = mc.second;
////        std::cout << "this->mean3D  update " << this->mean3D  << std::endl;
////        std::cout << "this->cova3D  update " << this->cova3D  << std::endl;
//        } else{
//            DETECTBOX xyah_box;
//            xyah_box[0] = xyah[0];
//            xyah_box[1] = xyah[1];
//            xyah_box[2] = xyah[2];
//            xyah_box[3] = xyah[3];
//            auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
//            this->mean = mc.first;
//            this->covariance = mc.second;
////        std::cout << "this->mean  update " << this->mean  << std::endl;
////        std::cout << "this->cova  update " << this->covariance  << std::endl;
//        }
//        static_tlwh();
//        static_tlbr();
//    }
//    if()
//
//    static_tlwh();
//    static_tlbr();

//    push_front_change_Locate3D_and_distance(Locate3D);
}




/**
 * @brief 队列
 * **/
void STrack::push_front_vexSerialNum(int serialNum) {
    if(this->vexSerialNum.size() >= 5){
        vexSerialNum.pop_back();
    }
    vexSerialNum.insert(vexSerialNum.begin(), serialNum);
}


void STrack::push_front_change_Locate3D_and_distance(cv::Point3d Locate3D) {
    int size = this->change_distance.size();
    double distance = get2Ddistance(Locate3D.x,Locate3D.y,0,0);
    if(size >= 7){
        change_distance.pop_back();
        change_Locate3Ds.pop_back();
    }
//    std::cout <<"size :  " << size << std::endl;
    if(size == 0){
        change_distance.push_back(distance);
        change_Locate3Ds.push_back(Locate3D);
    } else{
        change_distance.insert(change_distance.begin(),distance+change_distance.back());//？？
        change_Locate3Ds.insert(change_Locate3Ds.begin(), Locate3D);
    }
//    std::cout << "change_Locate3Ds:1  "  << change_Locate3Ds << std::endl;

}

/**
 * @brief 更新权重装甲版mat的权重`
 *
 *
 * @param ws_confMatrix      新轨迹（检测器）的权重
 * @param ws_armorConfMatrix 需要更新的(跟踪器)权重
 */
void STrack::updataStrack_ws_confMatrixs(double new_armorConf,Eigen::MatrixXd &ws_confMatrix ,Eigen::MatrixXd &ws_armorConfMatrix){
    //TODO:new_armorConf
//    std::cout << "------------------------------------------------" << std::endl;
//    std::cout <<  "ws_confMatrix__:" << ws_confMatrix << std::endl;
//    std::cout <<  "ws_confMatrix__  size:" << ws_confMatrix.size() << std::endl;
//    std::cout <<  "ws_armorConfMatrix__old:" << ws_armorConfMatrix << std::endl;
//    std::cout <<  "ws_armorConfMatrix__old  __size:" << ws_armorConfMatrix.size() << std::endl;
//    std::cout <<  "new_armorConf:" << new_armorConf << std::endl;
//    std::cout <<  "ws_armorConfMatrix__old:" << ws_armorConfMatrix << std::endl;

    ws_armorConfMatrix = ws_confMatrix * new_armorConf * maxUpdataW + (1.0 - new_armorConf * maxUpdataW ) * ws_armorConfMatrix;

//    if(ws_armorConfMatrix.sum() > 1e-6){
//        ws_armorConfMatrix =
//        ws_armorConfMatrix / (ws_armorConfMatrix.sum());
//    }


}



void STrack::classfy_STrack_N(int &num){//??
//    num += this->classWithoutCar*2;

    //获得ws_armorConfMatrix中最大值的标签（即armorConf最大值对应的标签）
    Eigen::MatrixXf::Index max_index;
    ws_armorConfMatrix.row(0).maxCoeff(&max_index);
    conf_armor = ws_armorConfMatrix(0,max_index);
    int temp_cls = max_index;

    if(conf_armor < 5e-5){
        temp_cls = classWithoutCar;
    }
    else if(half_classWithoutCar == 7) {
        int color_N = (max_index/half_classWithoutCar+1)*half_classWithoutCar-1;
        if(abs(conf_armor - ws_armorConfMatrix(0,color_N) < 1e-2)) {
            temp_cls = color_N;
        }else{
            ws_armorConfMatrix(0,half_classWithoutCar-1) = 0;
            ws_armorConfMatrix(0,classWithoutCar-1) = 0;
        }
    }
    else if(half_classWithoutCar != 6){
        std::cout << "here have error in BYTETracker::classfy_STrack_N22" << std::endl;
    }

    if( cls != temp_cls){
        cls = temp_cls;
        updata_trackid(num);
    }

}

void STrack::updata_trackid(int &num) {
//    cls = newcls;

//    std::cout << "cls:  " << cls << std::endl;
    std::cout << "num:  " << num << std::endl;
//    std::cout << "ws_armorConfMatrix:  " << ws_armorConfMatrix << std::endl;

    if(cls == classWithoutCar || cls == -1){
        track_id = num + 300;num ++;    //300+ unknown
        return;
    }else{
        if(half_classWithoutCar == 7){
            if (cls == half_classWithoutCar - 1) {
                track_id = num + 100;
                num++;// 100+ R
            } else if (cls == classWithoutCar - 1) {
                track_id = num + 200;
                num++;// 200+ B
            }
        }else if(half_classWithoutCar == 6){
                track_id = cls;
        }else{
            std::cout << "here have error in BYTETracker::classfy_STrack_N22" << std::endl;
        }
    }

}


void STrack::static_tlwh(bool is3D)
{
	if (this->state == TrackState::New){
		tlwh[0] = _tlwh[0];
		tlwh[1] = _tlwh[1];
		tlwh[2] = _tlwh[2];
		tlwh[3] = _tlwh[3];
//        Locate2D = cv::Point2d (tlwh[0] + tlwh[2]/2.0,tlwh[1] + tlwh[3]*0.95);
        Locate3D = _Locate3D;
        return;
	}
    if(is3D){
        tlwh[0] = mean3D[0];
        tlwh[1] = mean3D[1];
        tlwh[2] = mean3D[2];
        tlwh[3] = mean3D[3];
        Locate3D.x = mean3D[4];
        Locate3D.y = mean3D[5];
//        std::cout << "locate3d: " << Locate3D << std::endl;

    }
    else{
        tlwh[0] = mean[0];
        tlwh[1] = mean[1];
        tlwh[2] = mean[2];
        tlwh[3] = mean[3];
    }
	tlwh[2] *= tlwh[3];
	tlwh[0] -= tlwh[2] / 2;//？？
	tlwh[1] -= tlwh[3] / 2;
//    Locate2D = cv::Point2d (tlwh[0] + tlwh[2]/2.0,tlwh[1] + tlwh[3]*0.95);

}

void STrack::static_tlbr(){
	tlbr.clear();
	tlbr.assign(tlwh.begin(), tlwh.end());
	tlbr[2] += tlbr[0];
	tlbr[3] += tlbr[1];
}

void STrack::static_3D_velocity(){
    vx_3d = mean3D[10];
    vy_3d = mean3D[11];
}

vector<float> STrack::tlwh_to_xyah(vector<float> tlwh_tmp)
{
	vector<float> tlwh_output = tlwh_tmp;
	tlwh_output[0] += tlwh_output[2] / 2;
	tlwh_output[1] += tlwh_output[3] / 2;
	tlwh_output[2] /= tlwh_output[3];
	return tlwh_output;
}

vector<float> STrack::to_xyah()
{
	return tlwh_to_xyah(tlwh);
}

vector<float> STrack::tlbr_to_tlwh(vector<float> &tlbr)
{
	tlbr[2] -= tlbr[0];
	tlbr[3] -= tlbr[1];
	return tlbr;
}

void STrack::mark_lost()
{
    tracklet_len = 0;
	state = TrackState::Lost;
}

void STrack::mark_lostCopy()
{
    state = TrackState::LostCopy;
}

void STrack::mark_removed()
{
	state = TrackState::Removed;
}

int STrack::next_id()
{
	static int _count = 20;
	_count++;
	return _count;
}

int STrack::end_frame()
{
	return this->frame_id;
}


//void STrack::push_back(Eigen::MatrixXd &ws_arormConfMatrix,Eigen::MatrixXd &ws_arormConfMatrix_BR)
//{
//    if(ws_armorConfMatrix_s.size()==maxLen){
//        ws_armorConfMatrix_s.erase(ws_armorConfMatrix_s.begin());
//        ws_armorConfMatrix_BR_s.erase(ws_armorConfMatrix_BR_s.begin());
//    }
//    ws_armorConfMatrix_s.push_back(ws_arormConfMatrix);
//    ws_armorConfMatrix_BR_s.push_back(ws_arormConfMatrix_BR);
//}