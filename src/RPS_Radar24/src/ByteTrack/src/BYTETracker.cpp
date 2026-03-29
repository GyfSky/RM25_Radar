#include "../include/BYTETracker.h"
#include <fstream>
#include <rclcpp/logging.hpp>
//BYTETracker(int frame_rate = 30, int track_buffer = 30);
BYTETracker::BYTETracker(OurPattern ourPattern,std::shared_ptr<MatrixCoordinateSystem> CooSystem_ptr,std::string config_path): CostMatrix(ourPattern,config_path)
{
	track_thresh = 0.25;
	high_thresh = 0.6;
    high_car_thresh = 0.1;
	match_thresh = 0.90;  // cost低于该值才进行匹配
	match_cls_thresh = 0.90;  // cost低于该值才进行匹配
	frame_id = 0;
    this->ourPatternColor=ourPattern;
    this->CooSystem_ptr=CooSystem_ptr;
//	max_time_lost = int(frame_rate / 30.0 * track_buffer); //default = 30
	cout << "Init ByteTrack!" << endl;
}

BYTETracker::~BYTETracker()
{
}

void BYTETracker::clear_cam_accurate() {
    for (int i=0;i<2;i++) {
        for (int j=0;j<5;j++) {
            cam_accurate_[i][j]=false;
        }
    }
}

//先根据距离和iou匹配检测结果与跟踪器，再根据每个跟踪器的种类矩阵，匹配种类与跟踪器
void BYTETracker::update(vector<STrack> &tracked_stracks, vector<STrack> &lost_stracks, vector<STrack> &lost_predict_stracks,vector<STrack> &cars, vector<STrack> &out,vector<STrack> &to_sentry,interfaces::msg::DetectResult lidar_det,interfaces::msg::LidarEnhance lidar_enhance)
{
    //tracked_stracks 里面为 activated_stracks refind_stracks
    ////////////////// Step 1: Get detections //////////////////
    this->frame_id++; //TODO：
    vector<STrack> activated_stracks;//更新状态为跟踪的跟踪器 新建的跟踪器 当前帧更新过的跟踪器
    vector<STrack> refind_stracks;  // 在该帧由 !track -> track  更新状态不为跟踪的跟踪器
//    vector<STrack> removed_stracks;
    vector<STrack> current_frame_lost_stracks;//跟踪器在当前帧丢失
    vector<STrack> always_frame_lost_stracks;//上一帧和当前帧均没有匹配上
    vector<STrack> always_frame_lost_predict_stracks;
    vector<STrack> detections;//放检测到的对象，还没有进行跟踪器的初始化
    vector<STrack> detections_low;

    vector<STrack> detections_cp;  // 与 &tracked_stracks 无法匹配上的 detections//未匹配上的检测结果，暂存，最终赋给detections
    vector<STrack> tracked_stracks_swap;//没有用到
    vector<STrack> resa, resb;//没有用到
//    vector<STrack> output_stracks;

    vector<STrack*> unconfirmed;//没有用到
    vector<STrack*> current_frame_tracked_stracks;//当前帧跟踪器
    vector<STrack*> past_frame_lose_stracks;//上一帧丢失的跟踪器
    vector<STrack*> strack_pool;//跟踪器池
//    vector<STrack> strack_pool;
    vector<STrack*> r_tracked_stracks;//第一次未匹配上的跟踪结果

    if(is_detectionlow){
        if (cars.size() > 0)
        {
            for (int i = 0; i < cars.size(); i++)
            {
                cars[i].updata_trackid(num);
                if (cars[i].conf >= track_thresh)
                {
                    detections.push_back(cars[i]);
                }
                else
                {
                    detections_low.push_back(cars[i]);
                }
            }
        }
    }else{
        detections = std::move(cars);
    }

    //检测是否超出边界
    std::set<int,std::greater<>> remove_lost,remove_predict;
    for (int i=0;i<lost_stracks.size();i++) {
        if (!CooSystem_ptr->Correction(lost_stracks[i].Locate3D,ourPatternColor)) remove_lost.insert(i);
    }
    for (int i=0;i<lost_predict_stracks.size();i++) {
        if (!CooSystem_ptr->Correction(lost_predict_stracks[i].Locate3D,ourPatternColor)) remove_predict.insert(i);
    }
    if (lost_stracks.size()==lost_predict_stracks.size()) {
        remove_lost.insert(remove_predict.begin(),remove_predict.end());
        for (auto index:remove_lost) {
            lost_stracks.erase(lost_stracks.begin()+index);
            lost_predict_stracks.erase(lost_predict_stracks.begin()+index);
        }
    }else {
        for (auto index:remove_lost)
            lost_stracks.erase(lost_stracks.begin()+index);

        for (auto index:remove_predict)
            lost_predict_stracks.erase(lost_predict_stracks.begin()+index);
    }

//    // Add newly detected tracklets to tracked_stracks
//    for (int i = 0; i < tracked_stracks.size(); i++)
//    {
//        if (!tracked_stracks[i].is_activated)
//            unconfirmed.push_back(&tracked_stracks[i]);
//        else
//            current_frame_tracked_stracks.push_back(&tracked_stracks[i]);
//    }//
//
// self
    //已经跟踪的轨迹
    for (int i = 0; i < tracked_stracks.size(); i++)
    {
        current_frame_tracked_stracks.push_back(&tracked_stracks[i]);
    }

    bool is_lose_predict = false;//丢失的轨迹是否全部被预测
    int lost_stracks_size = lost_stracks.size();//上一帧的丢失轨迹的数量
    int lost_predict_stracks_size = lost_predict_stracks.size();//丢失预测的大小

    for (int i = 0; i < lost_stracks_size; i++){
        past_frame_lose_stracks.push_back(&lost_stracks[i]);
    }
    //主要是判断上一帧的always_frame_lost_stracks与always_frame_lost_predict_stracks是否相当
    if(lost_predict_stracks_size == lost_stracks_size){
//    if(lost_predict_stracks_size == lost_stracks_size ){
        is_lose_predict = true;
        for (int i = 0; i < lost_predict_stracks_size; i++){
            past_frame_lose_stracks.push_back(&lost_predict_stracks[i]);//？？
        }
    }else if(lost_predict_stracks_size != lost_stracks_size){
//        lost_stracks.clear();
        lost_predict_stracks.size();
        std::cout << "---------------------Predict have error, here will no predict-----------" << std::endl;
    }

//    is_lose_predict = false;
//    is_cls = false;+



//{
////////////////// Step 2: First association, with IoU //////////////////
//    strack_pool = joint_stracks(current_frame_tracked_stracks, lost_stracks); // tracked_stracks 已有的；lost_stracks 才有的
//    // strack_pool = joint_stracks(current_frame_tracked_stracks, lost_stracks, false); // tracked_stracks 已有的；lost_stracks 才有的
////    strack_pool = joint_stracks(tracked_stracks); //
//    std::cout << "strack_pool.size()" << strack_pool.size() << std::endl;
//    std::cout << "lost_stracks" << lost_stracks.size() << std::endl;
//
//    std::vector<std::vector<float>> cost_confMatrix_vec;
//    int num_strack,num_cls;
//
//    vector<vector<int> > matches_cls;
//    vector<int> u_strack, u_cls;
//    matches_cls.clear();u_strack.clear();u_cls.clear();
//    Eigen::MatrixXd cost_confMatrix = getCost_confMatrix(strack_pool,num_strack,num_cls);
////    std::cout << "cost_confMatrix _ cls:  " << std::endl << cost_confMatrix << std::endl;
//    eigenMat2VecVec(cost_confMatrix, cost_confMatrix_vec);
//    linear_assignment(cost_confMatrix_vec, num_strack, num_cls, match_thresh, matches_cls, u_strack, u_cls);

//    for (int i=0; i < matches_cls.size(); i++)
//    {
//        strack_pool[matches_cls[i][0]]->cls = matches_cls[i][1];
//        strack_pool[matches_cls[i][0]]->conf_armor = strack_pool[matches_cls[i][0]]->ws_armorConfMatrix(0,matches_cls[i][1]);
//
//    }
//    int num = 0;
//    for(int i=0; i < u_strack.size();i++){
//        classfy_STrack_N(strack_pool[u_strack[i]],num);
//    }
//}


    //进行预测
    multi_predict(tracked_stracks, this->kalman_filter);
    multi_predict(past_frame_lose_stracks, this->kalman_filter);


////instead ing
    //第一：先将高置信度的目标与上一帧已跟踪的轨迹进行匹配
    vector<vector<float> > dists;
    int dist_size = 0, dist_size_size = 0;//dist_s  ize = atracks.size(); dist_size_size = btracks.size();
    Eigen::MatrixXd cost_matrix = getIouAndDistancetCost(current_frame_tracked_stracks, detections, dist_size, dist_size_size, false);
    eigenMat2VecVec(cost_matrix,dists);
   std::cout << "first cost matrix: "<< std::endl << cost_matrix <<std::endl;
    vector<vector<int> > matches;
    vector<int> u_track, u_detection;
//    std::cout << "cost_matrix2\n" << cost_matrix << std::endl;
    linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_track, u_detection);

    for (int i = 0; i < matches.size(); i++)
    {
        STrack *track = current_frame_tracked_stracks[matches[i][0]];
        STrack *det = &detections[matches[i][1]];
        if (track->state == TrackState::Tracked)
        {
            track->update(*det, this->frame_id, ourPattern, windmill_car);
            activated_stracks.push_back(*track);
        }
        else//会出现这种情况吗
        {
            track->re_activate(*det, this->frame_id, ourPattern, windmill_car,false);
            refind_stracks.push_back(*track);
        }
    }

//    for (int i = 0; i < u_detection.size(); i++)
//    {
//        detections_cp.push_back(detections[u_detection[i]]);
//    }

//    detections.clear();
//    detections.assign(detections_cp.begin(), detections_cp.end());
    //第二：若is_detectionlow为true，将第一次与置信度高的检测结果未匹配上的并且为Tracked状态的跟踪结果与低置信度的检测结果进行匹配，同时将阈值放低
    //并将两次都没有匹配上的跟踪结果置为lost，并加入 current_frame_lost_stracks 中
    //若is_detectionlow为false，则将没有匹配上的跟踪结果置为lost，并加入 current_frame_lost_stracks 中
    if(is_detectionlow){
        for (int i = 0; i < u_detection.size(); i++)
        {
            detections_cp.push_back(detections[u_detection[i]]);
        }
        detections.clear();
        //将detections_low中的元素复制到detections中
        detections.assign(detections_low.begin(), detections_low.end());

        for (int i = 0; i < u_track.size(); i++)
        {
            if (current_frame_tracked_stracks[u_track[i]]->state == TrackState::Tracked)//会有不是这种的情况出现吗 有可能
                                                                                        //最后会进行全局最优的匹配更新，会更新cls
                                                                                        //并将部分跟踪器的状态设置为lost
            {
                r_tracked_stracks.push_back(current_frame_tracked_stracks[u_track[i]]);
            }
        }
////instead
        dists.clear();
        //将第一次与置信度高的检测结果未匹配上的跟踪结果与置信度较低的检测结果进行匹配
        cost_matrix = getIouAndDistancetCost(r_tracked_stracks, detections, dist_size, dist_size_size, false);
        eigenMat2VecVec(cost_matrix,dists);

        matches.clear();
        u_track.clear();
        u_detection.clear();
        std::cout << "cost_matrix_r_tracked_stracks" << std::endl << cost_matrix << std::endl;
        linear_assignment(dists, dist_size, dist_size_size, 0.95, matches, u_track, u_detection);

        for (int i = 0; i < matches.size(); i++)
        {
            STrack *track = r_tracked_stracks[matches[i][0]];
            STrack *det = &detections[matches[i][1]];
            if (track->state == TrackState::Tracked)
            {
                track->update(*det, this->frame_id, ourPattern, windmill_car);
                activated_stracks.push_back(*track);
            }
            else//应该没有这种情况，因为r_tracked_stracks是由状态为Tracked的组成的
            {
                track->re_activate(*det, this->frame_id, ourPattern, windmill_car,false);
                refind_stracks.push_back(*track);
            }
        }
        for (int i = 0; i < u_track.size(); i++)
        {
            STrack *track = r_tracked_stracks[u_track[i]];
            if (track->state != TrackState::Lost && track->state != TrackState::LostCopy && track->state != TrackState::LostCopy_PredictOver)
            {
                track->mark_lost();
                //TODO: updata-------------------------------------------------------------------LOOK----here
                current_frame_lost_stracks.push_back(*track);
            }
        }
//        detections.clear();
//        detections.assign(detections_cp.begin(), detections_cp.end());
    }
    else{
        for (int i = 0; i < u_track.size(); i++)
        {
            STrack *track = current_frame_tracked_stracks[u_track[i]];
            //三个状态是什么意思
            if (track->state != TrackState::Lost && track->state != TrackState::LostCopy && track->state != TrackState::LostCopy_PredictOver)
            {
                track->mark_lost();
                //TODO: updata-------------------------------------------------------------------LOOK----here
                current_frame_lost_stracks.push_back(*track);
            }
        }

    }


    // Deal with unconfirmed tracks, usually tracks with only one beginning frame
//    if(is_lose_predict){
    //第三：利用上一帧丢失的跟踪器进行匹配，放宽阈值
    for (int i = 0; i < u_detection.size(); i++)
    {
        detections_cp.push_back(detections[u_detection[i]]);
    }
    detections.clear();
    detections.assign(detections_cp.begin(), detections_cp.end());

    dists.clear();
    cost_matrix = getIouAndDistancetCost(past_frame_lose_stracks, detections, dist_size, dist_size_size, false);
    eigenMat2VecVec(cost_matrix,dists);

    matches.clear();
    vector<int> u_unconfirmed;
    u_detection.clear();
    std::cout << "cost_matrix_unconfirmed" << '\n' << cost_matrix << std::endl;
    linear_assignment(dists, dist_size, dist_size_size, 0.95, matches, u_unconfirmed, u_detection);

    std::vector<int> is_lose(lost_stracks_size, 1);  // default 丢失 (is_lose(是否丢失) this mode is true) ,is false 不会进入该帧的lose
    //更新丢失状态
    for (int i = 0; i < matches.size(); i++)
    {
        past_frame_lose_stracks[matches[i][0]]->update(detections[matches[i][1]], this->frame_id,ourPattern, windmill_car);
        if(matches[i][0]>=lost_stracks_size)
            past_frame_lose_stracks[matches[i][0]%lost_stracks_size]->update(detections[matches[i][1]], this->frame_id,ourPattern, windmill_car);
        int lose_track_list_index = matches[i][0]%lost_stracks_size;//如果is_lose_predict为true，取余数则为取前半段的跟踪器
        is_lose[lose_track_list_index] = 0;
        activated_stracks.push_back(*past_frame_lose_stracks[matches[i][0]]);
    }

    std::cout << "STEP1"<< std::endl;


//        for (int i = 0; i < u_unconfirmed.size(); i++)
//        {
//            STrack *track = past_frame_lose_stracks[u_unconfirmed[i]];
//            track->mark_lost(); //TODO: make other---lose----------------------------------------------------------------LOOK----here
//            //TODO: updata---lose----------------------------------------------------------------LOOK----here
//            current_frame_lost_stracks.push_back(*track);
//        }
    // for (int i = 0; i < u_unconfirmed.size(); i++){
    //     STrack *lose = past_frame_lose_stracks[u_unconfirmed[i]];
    //     if((lose->cls == classWithoutCar && lose->lost_frame_ind_num > no_lost_frame_ind_num) ||//？？
    //             lose->lost_frame_ind_num > cls_lost_frame_ind_num){
    //         lose->mark_lost();
    //         continue;
    //     }//TODO：需要更改，避免两次降低置信度发生
    //     // if (is_lose_predict) {
    //     //     if (u_unconfirmed[i]>=lost_stracks_size) {
    //     //         if (is_lose[u_unconfirmed[i]-lost_stracks_size]==1) {
    //     //             lose->mark_lost();
    //     //             lose->update_lose(frame_id, final_max_conf);
    //     //             auto temp= past_frame_lose_stracks[u_unconfirmed[i]-lost_stracks_size];
    //     //             temp->update_lose(frame_id, final_max_conf);
    //     //             temp->track_id = lose->track_id;
    //     //             always_frame_lost_stracks.push_back(*temp);
    //     //             always_frame_lost_predict_stracks.push_back(*lose);
    //     //             is_lose[u_unconfirmed[i]-lost_stracks_size]++;
    //     //         }else {
    //     //             lose->state=Tracked;
    //     //             past_frame_lose_stracks[u_unconfirmed[i]-lost_stracks_size]->state=Tracked;
    //     //         }
    //     //     }else {
    //     //         if (is_lose[u_unconfirmed[i]]==1) {
    //     //             lose->mark_lost();
    //     //             lose->update_lose(frame_id, final_max_conf);
    //     //             auto temp= past_frame_lose_stracks[u_unconfirmed[i]+lost_stracks_size];
    //     //             temp->update_lose(frame_id, final_max_conf);
    //     //             lose->track_id = temp->track_id;
    //     //             always_frame_lost_stracks.push_back(*lose);
    //     //             always_frame_lost_predict_stracks.push_back(*temp);
    //     //             is_lose[u_unconfirmed[i]]++;
    //     //         }else {
    //     //             lose->state=Tracked;
    //     //             past_frame_lose_stracks[u_unconfirmed[i]+lost_stracks_size]->state=Tracked;
    //     //         }
    //     //     }
    //     // }else {
    //         lose->mark_lost();
    //         lose->update_lose(frame_id, final_max_conf);
    //         // always_frame_lost_stracks.push_back(*lose);
    //     // }
    //         if (u_unconfirmed[i]>=lost_stracks_size) {
    //             always_frame_lost_predict_stracks.push_back(*lose);
    //         }else {
    //             always_frame_lost_stracks.push_back(*lose);
    //         }
    // }

    for (int i = 0; i < lost_stracks_size; i++)
    {
        if(is_lose[i]==1){
            STrack *lose = past_frame_lose_stracks[i];
            //代表已经丢失？？
            if((lose->cls == classWithoutCar && lose->lost_frame_ind_num > no_lost_frame_ind_num) ||//？？
                    lose->lost_frame_ind_num > cls_lost_frame_ind_num){
                // lose->mark_lost();
                // if(is_lose_predict) {
                //     STrack *predict = past_frame_lose_stracks[i+lost_stracks_size];
                //     predict->mark_lost();
                // }
                continue;
            }
            //增加丢失帧数，拉低整体的置信度
            lose->update_lose(frame_id, final_max_conf);//？？
            // lose->mark_lost();
            always_frame_lost_stracks.push_back(*lose);
            if(is_lose_predict){
                STrack *predict = past_frame_lose_stracks[i+lost_stracks_size];//取后半段的预测的跟踪器
                //增加丢失帧数，拉低整体的置信度
                predict->update_lose(frame_id, final_max_conf);
                // predict->mark_lost();
                predict->track_id = lose->track_id;
                always_frame_lost_predict_stracks.push_back(*predict);//将上一帧还没有匹配上的跟踪器加入，即上一帧和
                                                                      //当前帧均没有匹配上
            }
        }
    }
//    }

    std::cout << "STEP2"<< std::endl;

    ////////////////// Step 4: Init new stracks //////////////////
    //第四：为未匹配到的，满足条件的检测器激活
    for (int i = 0; i < u_detection.size(); i++)
    {
        STrack *track = &detections[u_detection[i]];
//        if (track->conf_armor < this->high_thresh && (track->conf_armor > 1e-6 && track->conf < this->high_car_thresh) )  //TODO
        if (track->conf < this->high_car_thresh)//？？
            continue;
        track->activate(this->kalman_filter, this->frame_id,this->num);//创建新的卡尔曼对象？
        //更新track_id
        track->updata_trackid(num);
        //TODO:
        if( -1 < track->cls && track->cls < classWithoutCar){
            track->is_activated = true;
        }
        activated_stracks.push_back(*track);
    }

    std::cout << "STEP3"<< std::endl;


    ////////////////// Step 5: Update state //////////////////

    //第五：算全局最优匹配
    int activated_stracks_size = activated_stracks.size();
    int refind_stracks_size = refind_stracks.size();
    int current_frame_lost_stracks_size = current_frame_lost_stracks.size();
    int always_frame_lost_stracks_size = always_frame_lost_stracks.size();
    int always_frame_lost_predict_stracks_size = always_frame_lost_predict_stracks.size();

    for (int i = 0; i < activated_stracks_size; i++)
        strack_pool.push_back(&activated_stracks[i]);
    for (int i = 0; i < refind_stracks_size; i++)
        strack_pool.push_back(&refind_stracks[i]);
    for (int i = 0; i < current_frame_lost_stracks_size; i++)
        strack_pool.push_back(&current_frame_lost_stracks[i]);
    for (int i = 0; i < always_frame_lost_stracks_size; i++)
        strack_pool.push_back(&always_frame_lost_stracks[i]);
//    for (int i = 0; i < always_frame_lost_predict_stracks_size; i++)
//        strack_pool.push_back(&always_frame_lost_predict_stracks[i]);

    int before_always_lose = activated_stracks_size + refind_stracks_size + current_frame_lost_stracks_size;//在always_frame_lost_stracks之前的大小

//    vector<STrack> out;
    if(is_cls){
        std::vector<std::vector<float>> cost_confMatrix_vec;
        int num_strack,num_cls;

        vector<vector<int> > matches_cls;
        vector<int> u_strack, u_cls;
        matches_cls.clear();u_strack.clear();u_cls.clear();
        Eigen::MatrixXd cost_confMatrix = getCost_confMatrix(strack_pool,num_strack,num_cls);//？？
//        std::cout << "cost_confMatrix _ cls:  " << std::endl << cost_confMatrix << std::endl;
        eigenMat2VecVec(cost_confMatrix, cost_confMatrix_vec);
        //本质都是代价最小
        linear_assignment(cost_confMatrix_vec, num_strack, num_cls, match_cls_thresh, matches_cls, u_strack, u_cls);


        std::cout << "STEP5"<< std::endl;

        std::cout << "matches_cls" << std::endl;

        std::vector<bool> is_cls_update(num_cls,false);
        int half=half_classWithoutCar;
        std::array<std::array<int,5>,2> is_lidar_det={{{{0, 0, 0, 0, 0}},{{0, 0, 0, 0, 0}}}};
        for (int i=0; i < matches_cls.size(); i++)
        {
            int index_stracks_pool = matches_cls[i][0];
            int cls_stracks_pool = matches_cls[i][1];
            is_cls_update[cls_stracks_pool]=true;
//            std::cout << index_stracks_pool << ", " << cls_stracks_pool << std::endl;
//            std::cout << "cls_of_" << ", " << strack_pool[index_stracks_pool]->cls << std::endl;

            // no gassu  //TODO:mode
            //更新装甲板置信度
            strack_pool[index_stracks_pool]->conf_armor = strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool);
            int color_N = cls_stracks_pool/half_classWithoutCar*half_classWithoutCar+6;
            if(classWithoutCar == 14 && abs(strack_pool[index_stracks_pool]->conf_armor - strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,color_N)) < 1e-3){
                if(strack_pool[index_stracks_pool]->cls != color_N ){
                    strack_pool[index_stracks_pool]->cls = color_N;
                    strack_pool[index_stracks_pool]->updata_trackid(color_N);
                }
                continue;
            }
            if(strack_pool[index_stracks_pool]->cls != cls_stracks_pool){
                //该跟踪器的类别发生变化 则更新为新的cls
                strack_pool[index_stracks_pool]->cls_len_time = 0;
                strack_pool[index_stracks_pool]->cls = cls_stracks_pool;
                strack_pool[index_stracks_pool]->updata_trackid(num);
            }else{
                strack_pool[index_stracks_pool]->cls_len_time ++;
            }

            if(classWithoutCar ==14) {
                if (cls_stracks_pool != half_classWithoutCar - 1 && cls_stracks_pool != classWithoutCar - 1) {
                    out[cls_stracks_pool] = *strack_pool[index_stracks_pool];
                }
            }else if(classWithoutCar ==12){
                // std::cout<<"----armor conf----   "<<strack_pool[index_stracks_pool]->conf_armor<<std::endl;
                if (cls_stracks_pool<=5&&lidar_det.blue_x[cls_stracks_pool]!=0&&lidar_det.blue_y[cls_stracks_pool]!=0) {
                    strack_pool[index_stracks_pool]->Locate3D.x=lidar_det.blue_x[cls_stracks_pool];
                    strack_pool[index_stracks_pool]->Locate3D.y=lidar_det.blue_y[cls_stracks_pool];
                    if (strack_pool[index_stracks_pool]->lost_frame_ind_num>3) {
                        strack_pool[index_stracks_pool]->lost_frame_ind_num-=4;
                    }
                    strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool) = std::min(0.98,strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool)*1.1);
                    strack_pool[index_stracks_pool]->conf_armor = strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool);
                }
                else if (cls_stracks_pool>5&&lidar_det.red_x[cls_stracks_pool-6]!=0&&lidar_det.red_y[cls_stracks_pool-6]!=0) {
                    strack_pool[index_stracks_pool]->Locate3D.x=lidar_det.red_x[cls_stracks_pool-6];
                    strack_pool[index_stracks_pool]->Locate3D.y=lidar_det.red_y[cls_stracks_pool-6];
                    if (strack_pool[index_stracks_pool]->lost_frame_ind_num>3) {
                        strack_pool[index_stracks_pool]->lost_frame_ind_num-=4;
                    }
                    strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool) = std::min(0.98,strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool)*1.1);
                    strack_pool[index_stracks_pool]->conf_armor = strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool);
                }
                out[cls_stracks_pool] = *strack_pool[index_stracks_pool];
            }else if(classWithoutCar ==10){
                // std::cout<<"----armor conf----   "<<strack_pool[index_stracks_pool]->conf_armor<<std::endl;
                if (cls_stracks_pool<=half_classWithoutCar-1&&lidar_det.blue_x[cls_stracks_pool]!=0&&lidar_det.blue_y[cls_stracks_pool]!=0&&lidar_enhance.blue_enhance[cls_stracks_pool]==0) {
                    strack_pool[index_stracks_pool]->Locate3D.x=lidar_det.blue_x[cls_stracks_pool];
                    strack_pool[index_stracks_pool]->Locate3D.y=lidar_det.blue_y[cls_stracks_pool];
                    if (strack_pool[index_stracks_pool]->lost_frame_ind_num>3) {
                        strack_pool[index_stracks_pool]->lost_frame_ind_num-=4;
                    }
                    strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool) = std::min(0.98,strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool)*1.1);
                    strack_pool[index_stracks_pool]->conf_armor = strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool);
                    // strack_pool[index_stracks_pool]->state = TrackState::Tracked;
                    to_sentry[cls_stracks_pool] = *strack_pool[index_stracks_pool];
                    to_sentry[cls_stracks_pool].vx_3d=lidar_det.v_x[cls_stracks_pool];
                    to_sentry[cls_stracks_pool].vy_3d=lidar_det.v_y[cls_stracks_pool];
                    to_sentry[cls_stracks_pool].is_det=true;
                    is_lidar_det[0][cls_stracks_pool]=1;
                    out[cls_stracks_pool] = *strack_pool[index_stracks_pool];
                    out[cls_stracks_pool].is_det=true;
                }
                else if (cls_stracks_pool>half_classWithoutCar-1&&lidar_det.red_x[cls_stracks_pool-half_classWithoutCar]!=0&&lidar_det.red_y[cls_stracks_pool-half_classWithoutCar]!=0&&lidar_enhance.red_enhance[cls_stracks_pool-half_classWithoutCar]==0) {
                    strack_pool[index_stracks_pool]->Locate3D.x=lidar_det.red_x[cls_stracks_pool-half_classWithoutCar];
                    strack_pool[index_stracks_pool]->Locate3D.y=lidar_det.red_y[cls_stracks_pool-half_classWithoutCar];
                    if (strack_pool[index_stracks_pool]->lost_frame_ind_num>3) {
                        strack_pool[index_stracks_pool]->lost_frame_ind_num-=4;
                    }
                    strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool) = std::min(0.98,strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool)*1.1);
                    strack_pool[index_stracks_pool]->conf_armor = strack_pool[index_stracks_pool]->ws_armorConfMatrix(0,cls_stracks_pool);
                    // strack_pool[index_stracks_pool]->state = TrackState::Tracked;
                    to_sentry[cls_stracks_pool] = *strack_pool[index_stracks_pool];
                    to_sentry[cls_stracks_pool].vx_3d=lidar_det.v_x[cls_stracks_pool-half_classWithoutCar];
                    to_sentry[cls_stracks_pool].vy_3d=lidar_det.v_y[cls_stracks_pool-half_classWithoutCar];
                    to_sentry[cls_stracks_pool].is_det=true;
                    is_lidar_det[1][cls_stracks_pool-half_classWithoutCar]=1;
                    out[cls_stracks_pool] = *strack_pool[index_stracks_pool];
                    out[cls_stracks_pool].is_det=true;
                }else {
                    if (strack_pool[index_stracks_pool]->state==Tracked||strack_pool[index_stracks_pool]->state==New) {
                        to_sentry[cls_stracks_pool] = *strack_pool[index_stracks_pool];
                        to_sentry[cls_stracks_pool].is_det=true;
                        if (cls_stracks_pool>half_classWithoutCar-1) cam_accurate_[1][half_classWithoutCar-1]=true;
                        else if (cls_stracks_pool<=half_classWithoutCar-1) cam_accurate_[0][half_classWithoutCar-1]=true;
                    }else if (strack_pool[index_stracks_pool]->lost_frame_ind_num<15) {
                        to_sentry[cls_stracks_pool] = *strack_pool[index_stracks_pool];
                    }
                    out[cls_stracks_pool] = *strack_pool[index_stracks_pool];
                    if (strack_pool[index_stracks_pool]->state==Tracked)
                        out[cls_stracks_pool].is_det=true;
                }
            }else{
                std::cout << "PLEASE CHANGE is_lose_predict with out " << std::endl;
            }

            if(is_lose_predict){
                int x = index_stracks_pool - before_always_lose;//判断是不是always_frame_lost_stracks里面的元素
                //x>-1是在always_frame_lose_stracks里面，则更新对应的always_frame_lost_predict_stracks中的跟踪器
                if(x >-1){
//                    std::cout << "in predict   "  << x  << std::endl;
                    always_frame_lost_predict_stracks[x].cls = cls_stracks_pool;
                    always_frame_lost_predict_stracks[x].track_id = strack_pool[index_stracks_pool]->track_id;//？？
//                    std::cout << "index_stracks_pool: " <<index_stracks_pool << ", " << cls_stracks_pool << std::endl;
//                    std::cout << "always_frame_lost_predict_stracks[x].Locate3D: " << always_frame_loalways_frame_lost_predict_stracksst_predict_stracks[x].Locate3D.x <<  "  " << always_frame_lost_predict_stracks[x].Locate3D.y << std::endl;
                    if(classWithoutCar ==14){
                        if(cls_stracks_pool == half_classWithoutCar -1 || cls_stracks_pool == classWithoutCar - 1){
                            continue;
                        }
                        if(cls_stracks_pool != half_classWithoutCar -1 && cls_stracks_pool != classWithoutCar - 1){
                            out[cls_stracks_pool] = always_frame_lost_predict_stracks[x];
                        }
                    }else if(classWithoutCar ==12){
                        if (cls_stracks_pool<=5&&lidar_det.blue_x[cls_stracks_pool]!=0&&lidar_det.blue_y[cls_stracks_pool]!=0) {
                            always_frame_lost_predict_stracks[x].Locate3D.x=lidar_det.blue_x[cls_stracks_pool];
                            always_frame_lost_predict_stracks[x].Locate3D.y=lidar_det.blue_y[cls_stracks_pool];
                            if (always_frame_lost_predict_stracks[x].lost_frame_ind_num>3) {
                                always_frame_lost_predict_stracks[x].lost_frame_ind_num-=4;
                            }
                            always_frame_lost_predict_stracks[x].ws_armorConfMatrix(0,cls_stracks_pool) = std::min(0.98,always_frame_lost_predict_stracks[x].ws_armorConfMatrix(0,cls_stracks_pool)*1.1);
                            always_frame_lost_predict_stracks[x].conf_armor = always_frame_lost_predict_stracks[x].ws_armorConfMatrix(0,cls_stracks_pool);
                        }
                        else if (cls_stracks_pool>5&&lidar_det.red_x[cls_stracks_pool-6]!=0&&lidar_det.red_y[cls_stracks_pool-6]!=0) {
                            always_frame_lost_predict_stracks[x].Locate3D.x=lidar_det.red_x[cls_stracks_pool-6];
                            always_frame_lost_predict_stracks[x].Locate3D.y=lidar_det.red_y[cls_stracks_pool-6];
                            if (always_frame_lost_predict_stracks[x].lost_frame_ind_num>3) {
                                always_frame_lost_predict_stracks[x].lost_frame_ind_num-=4;
                            }
                            always_frame_lost_predict_stracks[x].ws_armorConfMatrix(0,cls_stracks_pool) = std::min(0.98,always_frame_lost_predict_stracks[x].ws_armorConfMatrix(0,cls_stracks_pool)*1.1);
                            always_frame_lost_predict_stracks[x].conf_armor = always_frame_lost_predict_stracks[x].ws_armorConfMatrix(0,cls_stracks_pool);
                        }
                        out[cls_stracks_pool] = always_frame_lost_predict_stracks[x];
                    }else if(classWithoutCar ==10){
                        if (cls_stracks_pool<=half_classWithoutCar-1&&lidar_det.blue_x[cls_stracks_pool]!=0&&lidar_det.blue_y[cls_stracks_pool]!=0&&lidar_enhance.blue_enhance[cls_stracks_pool]==0) {
                            always_frame_lost_predict_stracks[x].Locate3D.x=lidar_det.blue_x[cls_stracks_pool];
                            always_frame_lost_predict_stracks[x].Locate3D.y=lidar_det.blue_y[cls_stracks_pool];
                            if (always_frame_lost_predict_stracks[x].lost_frame_ind_num>3) {
                                always_frame_lost_predict_stracks[x].lost_frame_ind_num-=4;
                            }
                            always_frame_lost_predict_stracks[x].ws_armorConfMatrix(0,cls_stracks_pool) = std::min(0.98,always_frame_lost_predict_stracks[x].ws_armorConfMatrix(0,cls_stracks_pool)*1.1);
                            always_frame_lost_predict_stracks[x].conf_armor = always_frame_lost_predict_stracks[x].ws_armorConfMatrix(0,cls_stracks_pool);
                            // always_frame_lost_predict_stracks[x].state = TrackState::Tracked;
                            to_sentry[cls_stracks_pool] = always_frame_lost_predict_stracks[x];
                            to_sentry[cls_stracks_pool].vx_3d=lidar_det.v_x[cls_stracks_pool];
                            to_sentry[cls_stracks_pool].vy_3d=lidar_det.v_y[cls_stracks_pool];
                            to_sentry[cls_stracks_pool].is_det=true;
                            is_lidar_det[0][cls_stracks_pool]=1;
                            out[cls_stracks_pool] = always_frame_lost_predict_stracks[x];
                        }
                        else if (cls_stracks_pool>half_classWithoutCar-1&&lidar_det.red_x[cls_stracks_pool-half_classWithoutCar]!=0&&lidar_det.red_y[cls_stracks_pool-half_classWithoutCar]!=0&&lidar_enhance.red_enhance[cls_stracks_pool-half_classWithoutCar]==0) {
                            always_frame_lost_predict_stracks[x].Locate3D.x=lidar_det.red_x[cls_stracks_pool-half_classWithoutCar];
                            always_frame_lost_predict_stracks[x].Locate3D.y=lidar_det.red_y[cls_stracks_pool-half_classWithoutCar];
                            if (always_frame_lost_predict_stracks[x].lost_frame_ind_num>3) {
                                always_frame_lost_predict_stracks[x].lost_frame_ind_num-=4;
                            }
                            always_frame_lost_predict_stracks[x].ws_armorConfMatrix(0,cls_stracks_pool) = std::min(0.98,always_frame_lost_predict_stracks[x].ws_armorConfMatrix(0,cls_stracks_pool)*1.1);
                            always_frame_lost_predict_stracks[x].conf_armor = always_frame_lost_predict_stracks[x].ws_armorConfMatrix(0,cls_stracks_pool);
                            // always_frame_lost_predict_stracks[x].state = TrackState::Tracked;
                            to_sentry[cls_stracks_pool] = always_frame_lost_predict_stracks[x];
                            to_sentry[cls_stracks_pool].vx_3d=lidar_det.v_x[cls_stracks_pool-half_classWithoutCar];
                            to_sentry[cls_stracks_pool].vy_3d=lidar_det.v_y[cls_stracks_pool-half_classWithoutCar];
                            to_sentry[cls_stracks_pool].is_det=true;
                            is_lidar_det[1][cls_stracks_pool-half_classWithoutCar]=1;
                            out[cls_stracks_pool] = always_frame_lost_predict_stracks[x];
                        }else{
                            if (always_frame_lost_predict_stracks[x].state==TrackState::Tracked||always_frame_lost_predict_stracks[x].state==TrackState::New) {
                                to_sentry[cls_stracks_pool] = always_frame_lost_predict_stracks[x];
                                to_sentry[cls_stracks_pool].is_det=true;
                                if (cls_stracks_pool>half_classWithoutCar-1) cam_accurate_[1][half_classWithoutCar-1]=true;
                                else if (cls_stracks_pool<=half_classWithoutCar-1) cam_accurate_[0][half_classWithoutCar-1]=true;
                            }else if (always_frame_lost_predict_stracks[x].lost_frame_ind_num<15) {
                                to_sentry[cls_stracks_pool] = always_frame_lost_predict_stracks[x];
                            }
                            out[cls_stracks_pool] = always_frame_lost_predict_stracks[x];
                            if (always_frame_lost_predict_stracks[x].state==TrackState::Tracked)
                                out[cls_stracks_pool].is_det=true;
                        }
                    }else{
                        std::cout << "PLEASE CHANGE is_lose_predict with out " << std::endl;
                    }
                }
            }
        }


        std::cout << "STEP4"<< std::endl;

        std::vector<double> max_conf(classWithoutCar,0.0);//每个类别最大置信度
        std::vector<int> max_ut_idx(classWithoutCar,-1);//每个类别最大置信度对应的下标
        std::vector<bool> remove_lost(u_strack.size(),false);
        for(int i=0;i < u_strack.size();i++) {
            for (int j=0;j<classWithoutCar;j++) {
                if (!is_cls_update[j]&&strack_pool[u_strack[i]]->ws_armorConfMatrix(0,j)>max_conf[j]) {
                    max_conf[j] = strack_pool[u_strack[i]]->ws_armorConfMatrix(0,j);
                    max_ut_idx[j] = i;
                }
            }
        }
        for (int i=0;i<half_classWithoutCar;i++) {
            if (max_ut_idx[i]==-1||lidar_det.blue_x[i]==0||lidar_det.blue_y[i]==0) continue;//TODO change lidar还有没有匹配到的情况
            is_lidar_det[0][i]=1;
            strack_pool[u_strack[max_ut_idx[i]]]->Locate3D.x=lidar_det.blue_x[i];
            strack_pool[u_strack[max_ut_idx[i]]]->Locate3D.y=lidar_det.blue_y[i];
            strack_pool[u_strack[max_ut_idx[i]]]->ws_armorConfMatrix(0,i) = std::min(0.9,strack_pool[u_strack[max_ut_idx[i]]]->ws_armorConfMatrix(0,i)*1.1);
            strack_pool[u_strack[max_ut_idx[i]]]->conf_armor = strack_pool[u_strack[max_ut_idx[i]]]->ws_armorConfMatrix(0,i);

            if(strack_pool[u_strack[max_ut_idx[i]]]->cls != i){
                //该跟踪器的类别发生变化 则更新为新的cls
                strack_pool[u_strack[max_ut_idx[i]]]->cls_len_time = 0;
                strack_pool[u_strack[max_ut_idx[i]]]->cls = i;
                strack_pool[u_strack[max_ut_idx[i]]]->updata_trackid(num);
            }else{
                strack_pool[u_strack[max_ut_idx[i]]]->cls_len_time ++;
            }
            if (strack_pool[u_strack[max_ut_idx[i]]]->lost_frame_ind_num>3) {
               strack_pool[u_strack[max_ut_idx[i]]]->lost_frame_ind_num-=4;
            }
            remove_lost[max_ut_idx[i]]=true;
            if(is_lose_predict) {
                int y = u_strack[max_ut_idx[i]] - before_always_lose ;//判断是不是always_frame_lost_stracks里面的元素
                //y>-1为在always_frame_lose_stracks里面
                if(y >-1){
                    always_frame_lost_predict_stracks[y].cls = i;
                    always_frame_lost_predict_stracks[y].track_id = strack_pool[u_strack[max_ut_idx[i]]]->track_id;

                    if(classWithoutCar ==12){
                        always_frame_lost_predict_stracks[y].Locate3D.x=lidar_det.blue_x[i];
                        always_frame_lost_predict_stracks[y].Locate3D.y=lidar_det.blue_y[i];
                        if (always_frame_lost_predict_stracks[y].lost_frame_ind_num>3) {
                            always_frame_lost_predict_stracks[y].lost_frame_ind_num-=4;
                        }
                        always_frame_lost_predict_stracks[y].ws_armorConfMatrix(0,i) = std::min(0.98,always_frame_lost_predict_stracks[y].ws_armorConfMatrix(0,i)*1.1);
                        always_frame_lost_predict_stracks[y].conf_armor = always_frame_lost_predict_stracks[y].ws_armorConfMatrix(0,i);
                    }else if (classWithoutCar ==10) {
                        always_frame_lost_predict_stracks[y].Locate3D.x=lidar_det.blue_x[i];
                        always_frame_lost_predict_stracks[y].Locate3D.y=lidar_det.blue_y[i];
                        if (always_frame_lost_predict_stracks[y].lost_frame_ind_num>3) {
                            always_frame_lost_predict_stracks[y].lost_frame_ind_num-=4;
                        }
                        always_frame_lost_predict_stracks[y].ws_armorConfMatrix(0,i) = std::min(0.98,always_frame_lost_predict_stracks[y].ws_armorConfMatrix(0,i)*1.1);
                        always_frame_lost_predict_stracks[y].conf_armor = always_frame_lost_predict_stracks[y].ws_armorConfMatrix(0,i);
                    }
                }
            }
            int enhance=lidar_enhance.blue_enhance[i];
            out[i] = *strack_pool[u_strack[max_ut_idx[i]]];
            out[i].is_det=enhance==0?true:false;
            if (enhance==0||enhance==1||enhance==5) {
                to_sentry[i] = *strack_pool[u_strack[max_ut_idx[i]]];
                to_sentry[i].vx_3d = lidar_det.v_x[i];
                to_sentry[i].vy_3d = lidar_det.v_y[i];
                to_sentry[i].is_det=enhance==0?true:false;
            }
        }
        for (int i=half_classWithoutCar;i<classWithoutCar;i++) {
            if (max_ut_idx[i]==-1||lidar_det.red_x[i-half_classWithoutCar]==0||lidar_det.red_y[i-half_classWithoutCar]==0) continue;
            is_lidar_det[1][i-half_classWithoutCar]=1;
            strack_pool[u_strack[max_ut_idx[i]]]->Locate3D.x=lidar_det.red_x[i-half_classWithoutCar];
            strack_pool[u_strack[max_ut_idx[i]]]->Locate3D.y=lidar_det.red_y[i-half_classWithoutCar];
            strack_pool[u_strack[max_ut_idx[i]]]->ws_armorConfMatrix(0,i) = std::min(0.9,strack_pool[u_strack[max_ut_idx[i]]]->ws_armorConfMatrix(0,i)*1.1);
            strack_pool[u_strack[max_ut_idx[i]]]->conf_armor = strack_pool[u_strack[max_ut_idx[i]]]->ws_armorConfMatrix(0,i);

            if(strack_pool[u_strack[max_ut_idx[i]]]->cls != i){
                //该跟踪器的类别发生变化 则更新为新的cls
                strack_pool[u_strack[max_ut_idx[i]]]->cls_len_time = 0;
                strack_pool[u_strack[max_ut_idx[i]]]->cls = i;
                strack_pool[u_strack[max_ut_idx[i]]]->updata_trackid(num);
            }else{
                strack_pool[u_strack[max_ut_idx[i]]]->cls_len_time ++;
            }
            if (strack_pool[u_strack[max_ut_idx[i]]]->lost_frame_ind_num>3) {
               strack_pool[u_strack[max_ut_idx[i]]]->lost_frame_ind_num-=4;
            }
            remove_lost[max_ut_idx[i]]=true;
            if(is_lose_predict) {
                int y = u_strack[max_ut_idx[i]] - before_always_lose ;//判断是不是always_frame_lost_stracks里面的元素
                //y>-1为在always_frame_lose_stracks里面
                if(y >-1){
                    always_frame_lost_predict_stracks[y].cls = i;
                    always_frame_lost_predict_stracks[y].track_id = strack_pool[u_strack[max_ut_idx[i]]]->track_id;

                    if(classWithoutCar ==12){
                        always_frame_lost_predict_stracks[y].Locate3D.x=lidar_det.red_x[i-6];
                        always_frame_lost_predict_stracks[y].Locate3D.y=lidar_det.red_y[i-6];
                        if (always_frame_lost_predict_stracks[y].lost_frame_ind_num>3) {
                            always_frame_lost_predict_stracks[y].lost_frame_ind_num-=4;
                        }
                        always_frame_lost_predict_stracks[y].ws_armorConfMatrix(0,i) = std::min(0.98,always_frame_lost_predict_stracks[y].ws_armorConfMatrix(0,i)*1.1);
                        always_frame_lost_predict_stracks[y].conf_armor = always_frame_lost_predict_stracks[y].ws_armorConfMatrix(0,i);
                    }else if(classWithoutCar ==10){
                        always_frame_lost_predict_stracks[y].Locate3D.x=lidar_det.red_x[i-half_classWithoutCar];
                        always_frame_lost_predict_stracks[y].Locate3D.y=lidar_det.red_y[i-half_classWithoutCar];
                        if (always_frame_lost_predict_stracks[y].lost_frame_ind_num>3) {
                            always_frame_lost_predict_stracks[y].lost_frame_ind_num-=4;
                        }
                        always_frame_lost_predict_stracks[y].ws_armorConfMatrix(0,i) = std::min(0.98,always_frame_lost_predict_stracks[y].ws_armorConfMatrix(0,i)*1.1);
                        always_frame_lost_predict_stracks[y].conf_armor = always_frame_lost_predict_stracks[y].ws_armorConfMatrix(0,i);
                    }
                }
            }
            int enhance=lidar_enhance.red_enhance[i];
            out[i] = *strack_pool[u_strack[max_ut_idx[i]]];
            out[i].is_det=enhance==0?true:false;
            if (enhance==0||enhance==1||enhance==5) {
                to_sentry[i] = *strack_pool[u_strack[max_ut_idx[i]]];
                to_sentry[i].vx_3d = lidar_det.v_x[i-half_classWithoutCar];
                to_sentry[i].vy_3d = lidar_det.v_y[i-half_classWithoutCar];
                to_sentry[i].is_det=enhance==0?true:false;
            }
        }
        //没有匹配上的
        for(int i=0; i < u_strack.size();i++){
            if (remove_lost[i]) continue;
            if(strack_pool[u_strack[i]]->cls != classWithoutCar){
                strack_pool[u_strack[i]]->cls_len_time = 0;
                strack_pool[u_strack[i]]->cls = classWithoutCar;
                strack_pool[u_strack[i]]->updata_trackid(num);
                if(is_lose_predict) {
                    int y = u_strack[i] - before_always_lose ;//判断是不是always_frame_lost_stracks里面的元素
                    //y>-1为在always_frame_lose_stracks里面
                    if(y >-1){
                        always_frame_lost_predict_stracks[y].cls = classWithoutCar;
                        num --;
                        always_frame_lost_predict_stracks[y].updata_trackid(num);
                    }
                }
            }
            if(!is_save_no_cls_track){
                strack_pool[u_strack[i]]->mark_lost();
                // current_frame_lost_stracks.push_back(*strack_pool[u_strack[i]]);
            }
        }
        for (int i=0;i<half_classWithoutCar;i++) {
            if (is_lidar_det[0][i]==0&&lidar_det.blue_x[i]!=0&&lidar_det.blue_y[i]!=0&&!cam_accurate_[0][i]) {
                int enhance=lidar_enhance.blue_enhance[i];
                out[i].Locate3D.x=lidar_det.blue_x[i];
                out[i].Locate3D.y=lidar_det.blue_y[i];
                out[i].cls = i;
                out[i].is_det=enhance==0?true:false;
                if (out[i].lost_frame_ind_num>3)
                    out[i].lost_frame_ind_num-=4;
                if (to_sentry[i].lost_frame_ind_num>3)
                    to_sentry[i].lost_frame_ind_num-=4;
                if (enhance==0||enhance==1||enhance==5) {
                    to_sentry[i].Locate3D.x=lidar_det.blue_x[i];
                    to_sentry[i].Locate3D.y=lidar_det.blue_y[i];
                    to_sentry[i].cls = i;
                    to_sentry[i].vx_3d = lidar_det.v_x[i];
                    to_sentry[i].vy_3d = lidar_det.v_y[i];
                    to_sentry[i].is_det=enhance==0?true:false;
                }
            }
            if (is_lidar_det[1][i]==0&&lidar_det.red_x[i]!=0&&lidar_det.red_y[i]!=0&&!cam_accurate_[1][i]) {
                int enhance=lidar_enhance.red_enhance[i];
                out[i+half_classWithoutCar].Locate3D.x=lidar_det.red_x[i];
                out[i+half_classWithoutCar].Locate3D.y=lidar_det.red_y[i];
                out[i+half_classWithoutCar].cls = i+half_classWithoutCar;
                out[i+half_classWithoutCar].is_det=enhance==0?true:false;
                if (out[i+half_classWithoutCar].lost_frame_ind_num>3)
                    out[i+half_classWithoutCar].lost_frame_ind_num-=4;
                if (to_sentry[i+half_classWithoutCar].lost_frame_ind_num>3)
                    to_sentry[i+half_classWithoutCar].lost_frame_ind_num-=4;
                if (enhance==0||enhance==1||enhance==5) {
                    to_sentry[i+half_classWithoutCar].Locate3D.x=lidar_det.red_x[i];
                    to_sentry[i+half_classWithoutCar].Locate3D.y=lidar_det.red_y[i];
                    to_sentry[i+half_classWithoutCar].cls = i+half_classWithoutCar;
                    to_sentry[i+half_classWithoutCar].vx_3d = lidar_det.v_x[i];
                    to_sentry[i+half_classWithoutCar].vy_3d = lidar_det.v_y[i];
                    to_sentry[i+half_classWithoutCar].is_det=enhance==0?true:false;
                }
            }
        }

        std::cout << "STEP6"<< std::endl;

//  std::cout << out
//        for(int i = 0 ; i<out.size() ; i++){
//            std::cout << i << ":  cls_" << out[i].cls << " ,trackid_ " <<  out[i].track_id << std::endl;
//        }

    }

    tracked_stracks.clear();

    if(is_save_no_cls_track){
        tracked_stracks.insert(tracked_stracks.begin(),activated_stracks.begin(),activated_stracks.end());
        tracked_stracks.insert(tracked_stracks.begin(),refind_stracks.begin(),refind_stracks.end());
    }else{
        //        for(auto _out:out){
        //            if(_out.state == TrackState::Tracked){
        //                tracked_stracks.push_back(_out);
        //            }
        //        }
        std::cout << "have error in out"  << std::endl;
    }


    lost_stracks.clear();
    lost_stracks.assign(always_frame_lost_stracks.begin(), always_frame_lost_stracks.end());
    lost_stracks.insert(lost_stracks.end(),current_frame_lost_stracks.begin(),current_frame_lost_stracks.end());

    std::cout << "STEP7"<< std::endl;

    lost_predict_stracks.clear();
    lost_predict_stracks.assign(always_frame_lost_predict_stracks.begin(), always_frame_lost_predict_stracks.end());
    for(int i = 0;i < current_frame_lost_stracks_size; i++){
        current_frame_lost_stracks[i].mark_lostCopy();//设置为copy lost 状态
        // if(is_lose_predict){
            lost_predict_stracks.push_back(current_frame_lost_stracks[i]);//？？
        // }
    }
    clear_cam_accurate();
    std::cout<<"STEP8"<<always_frame_lost_stracks.size()<<std::endl;
    std::cout<<"STEP8"<<always_frame_lost_predict_stracks.size()<<std::endl;
    std::cout<<"STEP8"<<current_frame_lost_stracks.size()<<std::endl;
    std::cout<<"STEP8"<<current_frame_lost_stracks_size<<std::endl;
//    return out;
    std::cout<<"finish track"<<std::endl;
    return;
}