#include "../include/BYTETracker.h"
#include "../include/lapjv.h"

void BYTETracker::set_windmill_car(std::vector<int> windmill_car){
    this->windmill_car.assign(windmill_car.begin(), windmill_car.end());
}


void BYTETracker::multi_predict(vector<STrack*> &stracks, byte_kalman::KalmanFilter &kalman_filter,bool is3D)
{
    for (int i = 0; i < stracks.size(); i++)
    {
        if (is3D){
            if (stracks[i]->state != TrackState::Tracked)
            {
//                stracks[i]->lost_frame_ind_num++;
                stracks[i]->mean3D[9] = 0;
            }
            int distance = get2Ddistance(stracks[i]->Locate3D.x,stracks[i]->Locate3D.y,stracks[i]->_Locate3D.x,stracks[i]->_Locate3D.y);
            // if(distance < max_predict_3Ddistance){
            if(stracks[i]->state != TrackState::Lost && stracks[i]->state != TrackState::LostCopy_PredictOver ){
                kalman_filter.predict(stracks[i]->mean3D, stracks[i]->cova3D);
            }
            // }

        } else{
            if (stracks[i]->state != TrackState::Tracked)
            {
//                stracks[i]->lost_frame_ind_num++;
                stracks[i]->mean[7] = 0;
            }
            // mode1 ： 该模式下失踪的跟踪器（this->lose_track）为静止在最后一次消失的位置--------------
//            if(stracks[i]->state != TrackState::Lost && stracks[i]->state != TrackState::LostCopy){
//                kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
//            }
            if(stracks[i]->state != TrackState::Lost && stracks[i]->state != TrackState::LostCopy_PredictOver ){
               kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
            }
            // --------------------------------------------------------------------------------
            // mode2 : 该模式下失踪的跟踪器（this->lose_track）为匀速直线模式运动，直至跟踪器的框变小的到消失或跟踪器跑出图片---------
//             kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
            //--------------------------------------------------------------------------------------------------------
        }
        stracks[i]->static_tlwh();
        stracks[i]->static_tlbr();
    }
}

void BYTETracker::multi_predict(vector<STrack> &stracks, byte_kalman::KalmanFilter &kalman_filter,bool is3D)
{
    for (int i = 0; i < stracks.size(); i++)
    {
        if (is3D){
            if (stracks[i].state != TrackState::Tracked)
            {
//                stracks[i].lost_frame_ind_num++;
            	//令h的速度为零
                stracks[i].mean3D[9] = 0;
            }
            int distance = get2Ddistance(stracks[i].Locate3D.x,stracks[i].Locate3D.y,stracks[i]._Locate3D.x,stracks[i]._Locate3D.y);
            // if(distance < max_predict_3Ddistance){
                if(stracks[i].state != TrackState::Lost && stracks[i].state != TrackState::LostCopy_PredictOver ){
                    kalman_filter.predict(stracks[i].mean3D, stracks[i].cova3D);
                }
            // }
        } else{
            if (stracks[i].state != TrackState::Tracked)
            {
//                stracks[i].lost_frame_ind_num++;
                stracks[i].mean[7] = 0;
            }
            // mode1 ： 该模式下失踪的跟踪器（this->lose_track）为静止在最后一次消失的位置--------------
//            if(stracks[i].state != TrackState::Lost && stracks[i].state != TrackState::LostCopy){
//                kalman_filter.predict(stracks[i].mean, stracks[i].covariance);
//            }
            if(stracks[i].state != TrackState::Lost && stracks[i].state != TrackState::LostCopy_PredictOver ){
                kalman_filter.predict(stracks[i].mean, stracks[i].covariance);
            }
            // --------------------------------------------------------------------------------
            // mode2 : 该模式下失踪的跟踪器（this->lose_track）为匀速直线模式运动，直至跟踪器的框变小的到消失或跟踪器跑出图片---------
//             kalman_filter.predict(stracks[i].mean, stracks[i].covariance);
            //--------------------------------------------------------------------------------------------------------
        }
        stracks[i].static_tlwh();
        stracks[i].static_tlbr();
    }
}


/**
 * @brief 向 tlista 中 不重复的加入 tlistb， 使 tlista = old_tlista 与 old_tlistb 的并集
 * **/
vector<STrack*> BYTETracker::joint_stracks(vector<STrack*> &tlista, vector<STrack> &tlistb, bool isRepeat)
{
	map<int, int> exists;
    vector<int> whiteSheet_repeat;
	vector<STrack*> res;
	for (int i = 0; i < tlista.size(); i++)
	{
		exists.insert(pair<int, int>(tlista[i]->track_id, 1));// insert 插入新的键值对
		res.push_back(tlista[i]);
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (!exists[tid] || exists.count(tid) == 0) //count函数用于返回指定键(tid)在map中出现的次数
		{
			exists[tid] = 1;
			res.push_back(&tlistb[i]);
            whiteSheet_repeat.push_back(tid);
		}else if(isRepeat){
            for(int x : whiteSheet_repeat){
                if(tid==x){
                    res.push_back(&tlistb[i]);
                }
            }
        }

	}
	return res;
}

vector<STrack*> BYTETracker::joint_stracks(vector<STrack*> &tlista)
{
    int sTrack =  tlista.size();
    if(sTrack >= 2){
        for (int i = 0; i < sTrack - 1; i++)
        {
            if (this->frame_id - tlista[i]->end_frame() > this->frame_id - tlista[i+1]->end_frame())
            {
                swap(tlista[i],tlista[i+1]);

            }
        }
    }
    int num = min_(sTrack,20);
//    std::cout << "num: " << num << std::endl;

	vector<STrack*> res(tlista.begin(),tlista.begin() + num);
	return res;
}


/**
 * @brief 向 tlista 中 不重复的加入 tlistb， 使 tlista = old_tlista 与 old_tlistb 的并集（A = A+B）（概论表达方式）
 * **/
vector<STrack> BYTETracker::joint_stracks(vector<STrack> &tlista, vector<STrack> &tlistb)
{
	map<int, int> exists;
	vector<STrack> res;
	for (int i = 0; i < tlista.size(); i++)
	{
		exists.insert(pair<int, int>(tlista[i].track_id, 1));
		res.push_back(tlista[i]);
	}
	for (int i = 0; i < tlistb.size(); i++)
	{
		int tid = tlistb[i].track_id;
		if (!exists[tid] || exists.count(tid) == 0)
		{
			exists[tid] = 1;
			res.push_back(tlistb[i]);
		}
	}
	return res;
}

/**
 * @brief 向 tlista 中 删除 tlistb， 使 tlista = old_tlista - old_tlistb （A = A-B = A-AB）概论表达方式）
 * **/
vector<STrack> BYTETracker::sub_stracks(vector<STrack> &tlista, vector<STrack> &tlistb, bool isRepeat)
{
    vector<STrack> res;
    if(!isRepeat){
        map<int, STrack> stracks;
        for (int i = 0; i < tlista.size(); i++)
        {
            stracks.insert(pair<int, STrack>(tlista[i].track_id, tlista[i]));
        }
        for (int i = 0; i < tlistb.size(); i++)
        {
            int tid = tlistb[i].track_id;
            if (stracks.count(tid) != 0)
            {
                stracks.erase(tid);
            }
        }
        std::map<int, STrack>::iterator  it;
        for (it = stracks.begin(); it != stracks.end(); ++it)
        {
            res.push_back(it->second);
        }

    }else{
        std::vector<int> erase_tid;
        for (int i = 0; i < tlista.size(); i++){
            int flag = 1;
            for (int j = 0; j < tlistb.size(); j++){
                int tid = tlistb[j].track_id;
                if(tid == tlista[i].track_id){
                    flag = -1;
                }
            }
            if(flag == 1){
                res.push_back(tlista[i]);
            }
        }
    }
	return res;
}

void BYTETracker::remove_duplicate_stracks(vector<STrack> &resa, vector<STrack> &resb, vector<STrack> &stracksa, vector<STrack> &stracksb)
{
	vector<vector<float> > pdist = iou_distance(stracksa, stracksb);
//    //TODO:_isBR______________________________________________________________________________________________
//    Eigen::MatrixXd pdist_mat = getIouAndDistancetCost(stracksa, stracksb, false);
//    vector<vector<float> > pdist;
//    eigenMat2VecVec(pdist_mat, pdist);
//    ////____________________________________________________________________________________________________
	vector<pair<int, int> > pairs;
	for (int i = 0; i < pdist.size(); i++)
	{
		for (int j = 0; j < pdist[i].size(); j++)
		{
			if (pdist[i][j] < 0.15)
			{
				pairs.push_back(pair<int, int>(i, j));
			}
		}
	}

	vector<int> dupa, dupb;
	for (int i = 0; i < pairs.size(); i++)
	{
		int timep = stracksa[pairs[i].first].frame_id - stracksa[pairs[i].first].start_frame;
		int timeq = stracksb[pairs[i].second].frame_id - stracksb[pairs[i].second].start_frame;
		if (timep > timeq)
			dupb.push_back(pairs[i].second);
		else
			dupa.push_back(pairs[i].first);
	}

	for (int i = 0; i < stracksa.size(); i++)
	{
		vector<int>::iterator iter = find(dupa.begin(), dupa.end(), i);
		if (iter == dupa.end())
		{
			resa.push_back(stracksa[i]);
		}
	}

	for (int i = 0; i < stracksb.size(); i++)
	{
		vector<int>::iterator iter = find(dupb.begin(), dupb.end(), i);
		if (iter == dupb.end())
		{
			resb.push_back(stracksb[i]);
		}
	}
}
//
//void BYTETracker::classfy_STrack_N(STrack* &obj, int num){
//    num += this->classWithoutCar*2;
//    //获得ws_armorConfMatrix中最大值的标签（即armorConf最大值对应的标签）
//    Eigen::MatrixXf::Index max_index;
//    obj->ws_armorConfMatrix.row(0).maxCoeff(&max_index);
//    obj->conf_armor = obj->ws_armorConfMatrix(0,max_index);
//    if(obj->conf_armor > 1e-6){
//        if(max_index < 7){
//            obj->cls = num + 100;num ++;//100+ B
//        }else if(max_index < 14){
//            obj->cls = num + 200;num ++;//200+ R
//        }
//    }else{
//        obj->cls = num + 300;num ++;    //300+ unknown
//    }
//}
//
//
//void BYTETracker::classfy_STrack_N(STrack &obj, int num){
////    num += this->classWithoutCar*2;
//    //获得ws_armorConfMatrix中最大值的标签（即armorConf最大值对应的标签）
//    Eigen::MatrixXf::Index max_index;
//    obj.ws_armorConfMatrix.row(0).maxCoeff(&max_index);
//    obj.conf_armor = obj.ws_armorConfMatrix(0,max_index);
//
//    if(obj.conf_armor < 1e-1){
//        obj.cls  = num + 300;num ++;    //300+ unknown
//    }
//    else if(half_classWithoutCar == 7){
//        int color_N = (max_index/half_classWithoutCar+1)*half_classWithoutCar-1;
//        if( abs(obj.conf_armor  - obj.ws_armorConfMatrix(0,color_N)) < 1e-1){
//            if(color_N == half_classWithoutCar - 1){
//                obj.cls = num + 100;num ++;// 100+ R
//            }else if(color_N == classWithoutCar - 1){
//                obj.cls = num + 200;num ++;// 200+ B
//            }else{
//                std::cout << "here have error in BYTETracker::classfy_STrack_N" << std::endl;
//            }
//        }else{
//            obj.cls  = max_index;
//            obj.ws_armorConfMatrix(0,half_classWithoutCar-1) = 0;
//            obj.ws_armorConfMatrix(0,half_classWithoutCar*2-1) = 0;
//        }
//    }
//    else if(half_classWithoutCar == 6){
//        obj.cls = max_index;
//    }else{
//        std::cout << "Please set the update_classfy  and classfy_STrack_N without T by yourself" << std::endl;
//    }
//}

Scalar BYTETracker::get_color(int idx)
{
    idx += 3;
    return Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}


////----------updata--------------------------------------------------------------------------------------------------
vector<vector<float> > BYTETracker::ious(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs)
{           //TODO: 改称rect,用rect1&rect2计算并集，rect1|rect2计算差集
	vector<vector<float> > ious;
	if (atlbrs.size()*btlbrs.size() == 0)
		return ious;

	ious.resize(atlbrs.size());
	for (int i = 0; i < ious.size(); i++)
	{
		ious[i].resize(btlbrs.size());
	}

	//bbox_ious
	for (int k = 0; k < btlbrs.size(); k++)
	{
		vector<float> ious_tmp;
		float box_area = (btlbrs[k][2] - btlbrs[k][0] + 1)*(btlbrs[k][3] - btlbrs[k][1] + 1);
		for (int n = 0; n < atlbrs.size(); n++)
		{
            //distance 2d
            double rectDistance = sqrt(pow((atlbrs[n][0]+atlbrs[n][2])/2-(btlbrs[k][0]+btlbrs[k][2])/2,2) + pow((atlbrs[n][1]+atlbrs[n][3])/2-(btlbrs[k][1]+btlbrs[k][3])/2,2));
			if(rectDistance <= rectDistance_thresh){
                ious[n][k] = (1.0 - rectDistance/rectDistance_thresh) * this->w_rectDistance;
            } else{
                ious[n][k] = 0.0;
            }
            float iw = min(atlbrs[n][2], btlbrs[k][2]) - max(atlbrs[n][0], btlbrs[k][0]) + 1;
            float ih = min(atlbrs[n][3], btlbrs[k][3]) - max(atlbrs[n][1], btlbrs[k][1]) + 1;
            if(iw > 0 && ih > 0){
                float ua = (atlbrs[n][2] - atlbrs[n][0] + 1) * (atlbrs[n][3] - atlbrs[n][1] + 1) + box_area - iw * ih;
                ious[n][k] = ious[n][k] +  iw * ih / ua * (1 - this->w_rectDistance);
            }
//            int a_cls = atracks[i]->cls;
//            int b_cls = btracks[k].cls;
//            if(isBR){
//                if(a < 6){
//                    cost_matrix(a,b) = cost_matrix(a,b) * (1.0 - btracks[b].ws_armorConfMatrix_BR(0,a)*atracks[a]->conf_armor) + btracks[b].ws_armorConfMatrix_BR(0,a)*atracks[a]->conf_armor;
//                }else if( -1 < b_cls && b_cls < 14){
//                    cost_matrix(a,b) = cost_matrix(a,b) * (1.0 - atracks[a]->ws_armorConfMatrix_BR(0,b_cls%7)*btracks[b].conf_armor) + atracks[a]->ws_armorConfMatrix_BR(0,b_cls%7)*btracks[b].conf_armor;
//                }
//            }else{
//                if( -1 < a_cls  && a_cls < 14 ) {
//                    cost_matrix(a, b) = cost_matrix(a, b) * (1.0 - btracks[b].ws_armorConfMatrix(0,a)*atracks[a]->conf_armor) + btracks[b].ws_armorConfMatrix(0,a)*atracks[a]->conf_armor;
//                }else if( -1 < b_cls && b_cls < 14){
//                    cost_matrix(a,b) = cost_matrix(a,b) * (1.0 - atracks[a]->ws_armorConfMatrix(0,b_cls)*btracks[b].conf_armor) + atracks[a]->ws_armorConfMatrix(0,b_cls)*btracks[b].conf_armor;
//                }
//            }
            ious[n][k] = 1.0 - ious[n][k];
		}
	}

	return ious;
}
vector<vector<float> > BYTETracker::iou_distance(vector<STrack*> &atracks, vector<STrack> &btracks, int &dist_size, int &dist_size_size)
{
	vector<vector<float> > cost_matrix;
	if (atracks.size() * btracks.size() == 0)
	{
		dist_size = atracks.size();
		dist_size_size = btracks.size();
		return cost_matrix;
	}
	vector<vector<float> > atlbrs, btlbrs;
	for (int i = 0; i < atracks.size(); i++)
	{
		atlbrs.push_back(atracks[i]->tlbr);
	}
	for (int i = 0; i < btracks.size(); i++)
	{
		btlbrs.push_back(btracks[i].tlbr);
	}

	dist_size = atracks.size();
	dist_size_size = btracks.size();

    cost_matrix = ious(atlbrs, btlbrs);

	return cost_matrix;
}
vector<vector<float> > BYTETracker::iou_distance(vector<STrack> &atracks, vector<STrack> &btracks)
{
	vector<vector<float> > atlbrs, btlbrs;
	for (int i = 0; i < atracks.size(); i++)
	{
		atlbrs.push_back(atracks[i].tlbr);
	}
	for (int i = 0; i < btracks.size(); i++)
	{
		btlbrs.push_back(btracks[i].tlbr);
	}

	vector<vector<float> > cost_matrix = ious(atlbrs, btlbrs);


	return cost_matrix;
}
//----------------------------------------------------------------------------------------------------------------------\

//void BYTETracker::car2object(vector<Car> &cars,vector<Object> &objects){
//    for(auto &car: cars){
//        Object obj;
//        obj.rect  = car.rect;
//        obj.label = car.cls;
//        obj.prob  = car.conf_armor;
//        objects.push_back(obj);
//    }
//}