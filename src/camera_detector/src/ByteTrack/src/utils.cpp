#include "../include/BYTETracker.h"
#include "../include/lapjv.h"

void BYTETracker::set_windmill_car(std::vector<int> windmill_car){
    this->windmill_car.assign(windmill_car.begin(), windmill_car.end());
}

void BYTETracker::multi_predict(vector<STrack*> &stracks, byte_kalman::KalmanFilter &kalman_filter,bool is3D){
    for (int i = 0; i < stracks.size(); i++){
        if (is3D){
            if (stracks[i]->state != TrackState::Tracked){
                stracks[i]->mean3D[9] = 0;
            }
            if(stracks[i]->state != TrackState::Lost && stracks[i]->state != TrackState::LostCopy_PredictOver ){
                kalman_filter.predict(stracks[i]->mean3D, stracks[i]->cova3D);
            }
        } else{
            if (stracks[i]->state != TrackState::Tracked){
                stracks[i]->mean[7] = 0;
            }
            if(stracks[i]->state != TrackState::Lost && stracks[i]->state != TrackState::LostCopy_PredictOver ){
               kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
            }
        }
        stracks[i]->static_tlwh();
        stracks[i]->static_tlbr();
    }
}

void BYTETracker::multi_predict(vector<STrack> &stracks, byte_kalman::KalmanFilter &kalman_filter,bool is3D){
    for (int i = 0; i < stracks.size(); i++){
        if (is3D){
            if (stracks[i].state != TrackState::Tracked){
                stracks[i].mean3D[9] = 0;
            }
        	if(stracks[i].state != TrackState::Lost && stracks[i].state != TrackState::LostCopy_PredictOver ){
        		kalman_filter.predict(stracks[i].mean3D, stracks[i].cova3D);
        	}
        } else{
            if (stracks[i].state != TrackState::Tracked){
                stracks[i].mean[7] = 0;
            }
            if(stracks[i].state != TrackState::Lost && stracks[i].state != TrackState::LostCopy_PredictOver ){
                kalman_filter.predict(stracks[i].mean, stracks[i].covariance);
            }
        }
        stracks[i].static_tlwh();
        stracks[i].static_tlbr();
    }
}