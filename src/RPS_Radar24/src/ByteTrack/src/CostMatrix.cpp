//
// Created by plusseven on 23-12-10.
//

#include "../include/CostMatrix.h"



CostMatrix::CostMatrix(OurPattern ourPattern){
    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    this->classWithoutCar = config["net"]["classWithoutCar"].as<int>();
    this->half_classWithoutCar = classWithoutCar/2;
    this->w_rectDistance = config["cost"]["w_rectDistance"].as<float>();
    this->rectDistance_thresh = config["cost"]["rectDistance_thresh"].as<float>();
    this->w_3dDistance = config["cost"]["w_3dDistance"].as<float>();
    this->distance_thresh = config["cost"]["distance_thresh"].as<float>();

    if(is_3Dplace_decide_step_rectDistance){
        this->k = (max_rect_step - min_rect_step)/(max_rect_place - min_rect_place);
    }
}
//
///**
// * @brief 获取跟踪器与被筛选后的检测器的代价矩阵（cost_matrix）
// *
// * @param atracks 跟踪器
// * @param btracks 被筛选后的检测器
// * @param isBR    输入的是单颜色(只有红色/只有蓝色)的检测框==true，else==false
// *
// * @return 代价矩阵（cost_matrix）
// */
//Eigen::MatrixXd CostMatrix::getIouAndDistancetCost(vector<STrack> &atracks, vector<STrack> &btracks, bool isBR){
//    //TODO: 改称rect,用rect1&rect2计算交集，rect1|rect2计算并集
//    int atracks_size = atracks.size(),btracks_size = btracks.size();
//    if (atracks_size * btracks_size == 0)
//    {
//        Eigen::MatrixXd cost_matrix;
//        return cost_matrix;
//    }
//    Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(atracks_size,btracks_size);
//    float i_area,u_area,iou;
//    for(int a = 0; a < atracks_size; a++){
//        for(int b = 0; b < btracks_size; b++){
//            //iou
//            i_area = ((atracks[a].rect)&(btracks[b].rect)).area(); //交集
//            u_area = atracks[a].rect.area() + btracks[b].rect.area() - i_area; //并集
//            iou = i_area/u_area;
//            //distance 2d
//            float rectDistance = sqrt(pow(((atracks[a].rect.x + atracks[a].rect.width/2)-(btracks[b].rect.x + btracks[b].rect.width/2)),2) + pow(((atracks[a].rect.y + atracks[a].rect.height/2)-(btracks[b].rect.y + btracks[b].rect.height/2)),2));
//            if(rectDistance <= rectDistance_thresh){
//                rectDistance = 1.0 - rectDistance/rectDistance_thresh;
//                cost_matrix(a,b) = iou * (1 - this->w_rectDistance) + rectDistance * this->w_rectDistance;
//            } else{
//                cost_matrix(a,b) = iou * (1 - this->w_rectDistance);
//            }
//
//            int a_cls = atracks[a].cls;
//            int b_cls = btracks[b].cls;
//            if(isBR){
//                if(a < 6){
//                    cost_matrix(a,b) = cost_matrix(a,b) * (1.0 - btracks[b].ws_armorConfMatrix_BR(0,a)*atracks[a].conf_armor*1/2) + btracks[b].ws_armorConfMatrix_BR(0,a)*atracks[a].conf_armor*1/2;
//                }else if( -1 < b_cls && b_cls < 14){
//                    cost_matrix(a,b) = cost_matrix(a,b) * (1.0 - atracks[a].ws_armorConfMatrix_BR(0,b_cls%7)*btracks[b].conf_armor*1/2) + atracks[a].ws_armorConfMatrix_BR(0,b_cls%7)*btracks[b].conf_armor*1/2;
//                }
//            }else{
//                if( -1 < a_cls  && a_cls < 14 ) {
//                    cost_matrix(a, b) = cost_matrix(a, b) * (1.0 - btracks[b].ws_armorConfMatrix(0,a)*atracks[a].conf_armor*1/2) + btracks[b].ws_armorConfMatrix(0,a)*atracks[a].conf_armor*1/2;
//                }else if( -1 < b_cls && b_cls < 14){
//                    cost_matrix(a,b) = cost_matrix(a,b) * (1.0 - atracks[a].ws_armorConfMatrix(0,b_cls)*btracks[b].conf_armor*1/2) + atracks[a].ws_armorConfMatrix(0,b_cls)*btracks[b].conf_armor*1/2;
//                }
//            }
//            cost_matrix(a,b) = 1.0 - cost_matrix(a,b);
//        }
//    }
//    return cost_matrix;
//}

/**
 * @brief 获取跟踪器与被筛选后的检测器的代价矩阵（cost_matrix）
 *
 * @param atracks 跟踪器
 * @param btracks 被筛选后的检测器
 * @param isBR    输入的是单颜色(只有红色/只有蓝色)的检测框==true，else==false
 *
 * @return 代价矩阵（cost_matrix）
 */
Eigen::MatrixXd CostMatrix::getIouAndDistancetCost(vector<STrack*> &atracks, vector<STrack> &btracks, int &atracks_size, int &btracks_size, bool isBR){
    //TODO: 改称rect,用rect1&rect2计算交集，rect1|rect2计算并集
    atracks_size = atracks.size();btracks_size = btracks.size();
    if (atracks_size * btracks_size == 0)
    {
        Eigen::MatrixXd cost_matrix;
        return cost_matrix;
    }
    atracks_size = atracks.size();btracks_size = btracks.size();
    Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(atracks_size,btracks_size);
    float i_area,u_area;
    for(int a = 0; a < atracks_size; a++){
        //计算a的检测框面积
        float a_box_area =(atracks[a]->tlbr[2] - atracks[a]->tlbr[0] ) * (atracks[a]->tlbr[3] - atracks[a]->tlbr[1] );

        float use_rectDistance_thresh, use_3Ddistance_thresh;
        if(!is_predict){
            use_rectDistance_thresh = min((atracks[a]->lost_frame_ind_num+1) * step_rectDistance, rectDistance_thresh);
            use_3Ddistance_thresh = min((atracks[a]->lost_frame_ind_num+1) * step_3dDistance, distance_thresh);
        }else if(is_3Dplace_decide_step_rectDistance){
            // 按照轨迹所在位置计算rect_distance
            float distance_to_radar;
            if(ourPattern == blue){
                distance_to_radar = 28. - atracks[a]->_Locate3D.x;
            }else{
                distance_to_radar = atracks[a]->_Locate3D.x;
            }
            use_rectDistance_thresh = k * (max(min_rect_place, max(max_rect_place, distance_to_radar)) - min_rect_place) + min_rect_step;
            use_3Ddistance_thresh = min((atracks[a]->lost_frame_ind_num+1) * step_3dDistance, (distance_thresh / 2));
        }else{
            use_rectDistance_thresh = rectDistance_thresh;
            use_3Ddistance_thresh = step_3dDistance;
        }



        for(int b = 0; b < btracks_size; b++){
            float iou = 0.0;
            //iou
            float iw = min(atracks[a]->tlbr[2], btracks[b].tlbr[2]) - max(atracks[a]->tlbr[0], btracks[b].tlbr[0]) ;
            float ih = min(atracks[a]->tlbr[3], btracks[b].tlbr[3]) - max(atracks[a]->tlbr[1], btracks[b].tlbr[1]) ;
            if(iw > 0 && ih > 0) {
                //计算b的检测框面积
                float b_box_area = (btracks[b].tlbr[2] - btracks[b].tlbr[0] )*(btracks[b].tlbr[3] - btracks[b].tlbr[1] );
                i_area = iw * ih; //交集
                u_area = a_box_area + b_box_area - i_area;//并集
                iou = i_area/u_area;
            }
            //distance 2d
            float rectDistance = sqrt(pow(((atracks[a]->tlwh[0] + atracks[a]->tlwh[2]/2)-(btracks[b].tlwh[0] + btracks[b].tlwh[2]/2)),2) + pow(((atracks[a]->tlwh[1] + atracks[a]->tlwh[3]/2)-(btracks[b].tlwh[1] + btracks[b].tlwh[3]/2)),2));
            rectDistance = max_((1.0 - rectDistance/use_rectDistance_thresh),0.0);
            //distance 3d
            float distance = sqrt(pow(atracks[a]->Locate3D.x-btracks[b].Locate3D.x,2)+pow(atracks[a]->Locate3D.y-btracks[b].Locate3D.y,2));
            distance = max_((1.0-distance/use_3Ddistance_thresh),0.0);

            //各个的权值之和大概为1
            if(atracks[a]->state != TrackState::New && atracks[a]->state != TrackState::Tracked) {//跟丢的情况
                //iou和二维距离的置信度降低，三维距离的置信度增加
                cost_matrix(a,b) = iou * (1 - this->w_rectDistance * (4./3) - this->w_3dDistance) +
                                   rectDistance * this->w_rectDistance * (1./2.) + distance * (this->w_3dDistance + this->w_rectDistance * (3./4.));
            }else{//已跟踪的情况及新增跟踪器的情况
                cost_matrix(a,b) = iou * (1 - this->w_rectDistance - this->w_3dDistance) +
                                   rectDistance * this->w_rectDistance + distance * this->w_3dDistance;
            }


            int a_cls = atracks[a]->cls;
            int b_cls = btracks[b].cls;
            int a_track_id = atracks[a]->cls;
            int b_track_id = btracks[b].cls;
//            std::cout << "a_cls " << a_cls << "..." << "a_track_id:  " << a_track_id << std::endl;
//            std::cout << "b_cls " << b_cls << "..." << "b_track_id:  " << b_track_id << std::endl;

            if(isBR){
                if(a < 6){
                    cost_matrix(a,b) = cost_matrix(a,b) * (1.0 - btracks[b].ws_armorConfMatrix_BR(0,a_track_id%7)*atracks[a]->conf_armor) + btracks[b].ws_armorConfMatrix_BR(0,a_track_id%7)*atracks[a]->conf_armor;
                }else if( -1 < b_track_id && b_track_id < classWithoutCar){
                    cost_matrix(a,b) = cost_matrix(a,b) * (1.0 - atracks[a]->ws_armorConfMatrix_BR(0,b_track_id%7)*btracks[b].conf_armor) + atracks[a]->ws_armorConfMatrix_BR(0,b_track_id%7)*btracks[b].conf_armor;
                }
            }else{
                if( -1 < a_track_id  && a_track_id < classWithoutCar ) {
                    cost_matrix(a, b) = cost_matrix(a, b) * (1.0 - btracks[b].ws_armorConfMatrix(0,a_track_id)*atracks[a]->conf_armor) + btracks[b].ws_armorConfMatrix(0,a_track_id)*atracks[a]->conf_armor;
                }else if( -1 < b_track_id && b_track_id < classWithoutCar){
                    cost_matrix(a,b) = cost_matrix(a,b) * (1.0 - atracks[a]->ws_armorConfMatrix(0,b_track_id)*btracks[b].conf_armor) + atracks[a]->ws_armorConfMatrix(0,b_track_id)*btracks[b].conf_armor;
                }
            }
            cost_matrix(a,b) = 1.0 - cost_matrix(a,b);
        }
    }
    return cost_matrix;
}

Eigen::MatrixXd CostMatrix::getIouAndDistancetCost(vector<TRTInferV1::DetectionObj> main_obj,vector<TRTInferV1::DetectionObj> sec_obj,int &main_obj_size,int &sec_obj_size) {
    double w_rectDistance_sec2main = 0.37, rectDistance_thresh_sec2main = 400;
    main_obj_size = main_obj.size();sec_obj_size = sec_obj.size();
    if (main_obj_size * sec_obj_size == 0)
    {
        Eigen::MatrixXd cost_matrix;
        return cost_matrix;
    }
    main_obj_size = main_obj.size();sec_obj_size = sec_obj.size();
    Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(main_obj_size,sec_obj_size);
    float i_area,u_area;
    for(int a = 0; a < main_obj_size; a++){
        float S_main = (main_obj[a].x2-main_obj[a].x1) * (main_obj[a].y2-main_obj[a].y1);
        for(int b = 0; b < sec_obj_size; b++){
            float iou = 0.0;
            //iou
            float iw = min_(main_obj[a].x2, sec_obj[b].x2) - max_(main_obj[a].x1, sec_obj[b].x1) ;
            float ih = min_(main_obj[a].y2, sec_obj[b].y2) - max_(main_obj[a].y1, sec_obj[b].y1) ;
//            std::cout << "iw: " << iw << ",ih: " << ih << std::endl;
            if(iw > 0 && ih > 0) {
                i_area = iw * ih;
                u_area = S_main + (sec_obj[b].x2 - sec_obj[b].x1) * (sec_obj[b].y2 - sec_obj[b].y1) - i_area; //并集
                iou = i_area/u_area;
//                std::cout << "u_area: " << u_area << std::endl;
//                std::cout << "iou" << iou << std::endl;
            }

            //distance 2d
            float rectDistance = sqrt(pow(((main_obj[a].x1 + main_obj[a].x2)-(sec_obj[b].x1 + sec_obj[b].x2))/2,2)
                    + pow(((main_obj[a].y1 + main_obj[a].y2)-(sec_obj[b].y2 + sec_obj[b].y2))/2,2));
            rectDistance = max_((1.0 - rectDistance/rectDistance_thresh_sec2main),0.0);

//            std::cout << "rectDistance" << rectDistance << std::endl;

            cost_matrix(a,b) = iou * (1 - w_rectDistance_sec2main) + rectDistance * w_rectDistance_sec2main ;
            cost_matrix(a,b) = 1.0 - cost_matrix(a,b);

        }
    }
    return cost_matrix;
}

Eigen::MatrixXd CostMatrix::getIouAndDistancetCost
    (vector<TRTInferV1::Object> main_obj,vector<TRTInferV1::Object> sec_obj,int &main_obj_size,int &sec_obj_size){
    double w_rectDistance_sec2main = 0.37, rectDistance_thresh_sec2main = 350;
    main_obj_size = main_obj.size();sec_obj_size = sec_obj.size();
    if (main_obj_size * sec_obj_size == 0)
    {
        Eigen::MatrixXd cost_matrix;
        return cost_matrix;
    }
    main_obj_size = main_obj.size();sec_obj_size = sec_obj.size();
    Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(main_obj_size,sec_obj_size);
    float i_area,u_area;
    for(int a = 0; a < main_obj_size; a++){
        float S_main = (main_obj[a].x2-main_obj[a].x1) * (main_obj[a].y2-main_obj[a].y1);
        for(int b = 0; b < sec_obj_size; b++){
            float iou = 0.0;
            //iou
            float iw = min_(main_obj[a].x2, sec_obj[b].x2) - max_(main_obj[a].x1, sec_obj[b].x1) ;
            float ih = min_(main_obj[a].y2, sec_obj[b].y2) - max_(main_obj[a].y1, sec_obj[b].y1) ;
//            std::cout << "iw: " << iw << ",ih: " << ih << std::endl;
            if(iw > 0 && ih > 0) {
                i_area = iw * ih;
                u_area = S_main + (sec_obj[b].x2 - sec_obj[b].x1) * (sec_obj[b].y2 - sec_obj[b].y1) - i_area; //并集
                iou = i_area/u_area;
//                std::cout << "u_area: " << u_area << std::endl;
//                std::cout << "iou" << iou << std::endl;
            }

            //distance 2d
            float rectDistance = sqrt(pow(((main_obj[a].x1 + main_obj[a].x2)-(sec_obj[b].x1 + sec_obj[b].x2))/2,2)
                    + pow(((main_obj[a].y1 + main_obj[a].y2)-(sec_obj[b].y2 + sec_obj[b].y2))/2,2));
            rectDistance = max_((1.0 - rectDistance/rectDistance_thresh_sec2main),0.0);

//            std::cout << "rectDistance" << rectDistance << std::endl;

            cost_matrix(a,b) = iou * (1 - w_rectDistance_sec2main) + rectDistance * w_rectDistance_sec2main ;
            cost_matrix(a,b) = 1.0 - cost_matrix(a,b);

        }
    }
    return cost_matrix;
}

Eigen::MatrixXd CostMatrix::getIouAndDistancetCost(
                vector<STrack> main_obj,vector<STrack> sec_obj,int &main_obj_size,int &sec_obj_size){
    double w_rectDistance_sec2main = 0.37, rectDistance_thresh_sec2main = 100;
    main_obj_size = main_obj.size();sec_obj_size = sec_obj.size();
    if (main_obj_size * sec_obj_size == 0)
    {
        Eigen::MatrixXd cost_matrix;
        return cost_matrix;
    }
    main_obj_size = main_obj.size();sec_obj_size = sec_obj.size();
    Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(main_obj_size,sec_obj_size);
    float i_area,u_area;
    for(int a = 0; a < main_obj_size; a++){
        float S_main = (main_obj[a].tlbr[2]-main_obj[a].tlbr[0]) * (main_obj[a].tlbr[3]-main_obj[a].tlbr[1]);
        for(int b = 0; b < sec_obj_size; b++){
            float iou = 0.0;
            //iou
            float iw = min_(main_obj[a].tlbr[2], sec_obj[b].tlbr[2]) - max_(main_obj[a].tlbr[0], sec_obj[b].tlbr[0]) ;
            float ih = min_(main_obj[a].tlbr[3], sec_obj[b].tlbr[3]) - max_(main_obj[a].tlbr[1], sec_obj[b].tlbr[1]) ;
//            std::cout << "iw: " << iw << ",ih: " << ih << std::endl;
            if(iw > 0 && ih > 0) {
                i_area = iw * ih;
                u_area = S_main + (sec_obj[b].tlbr[2] - sec_obj[b].tlbr[0]) * (sec_obj[b].tlbr[3] - sec_obj[b].tlbr[1]) - i_area; //并集
                iou = i_area/u_area;
//                std::cout << "u_area: " << u_area << std::endl;
//                std::cout << "iou" << iou << std::endl;
            }

            //distance 2d
            float rectDistance = sqrt(pow(((main_obj[a].tlbr[0] + main_obj[a].tlbr[2])-(sec_obj[b].tlbr[0] + sec_obj[b].tlbr[2]))/2,2)
                                      + pow(((main_obj[a].tlbr[1] + main_obj[a].tlbr[3])-(sec_obj[b].tlbr[3] + sec_obj[b].tlbr[3]))/2,2));
            rectDistance = max_((1.0 - rectDistance/rectDistance_thresh_sec2main),0.0);

//            std::cout << "rectDistance" << rectDistance << std::endl;

            cost_matrix(a,b) = iou * (1 - w_rectDistance_sec2main) + rectDistance * w_rectDistance_sec2main ;
            cost_matrix(a,b) = 1.0 - cost_matrix(a,b);

        }
    }
    return cost_matrix;
}


//
//Eigen::MatrixXd CostMatrix::getIouAndDistancetCost(vector<STrack*> &atracks, vector<STrack> &btracks, int &atracks_size, int &btracks_size, bool isBR){
//    //TODO: 改称rect,用rect1&rect2计算交集，rect1|rect2计算并集
//
//        atracks_size = atracks.size();btracks_size = btracks.size();
//        if (atracks_size * btracks_size == 0)
//        {
//            Eigen::MatrixXd cost_matrix;
//            return cost_matrix;
//        }
//    atracks_size = atracks.size();btracks_size = btracks.size();
//    Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(atracks_size,btracks_size);
//    float i_area,u_area,iou;
//    for(int b = 0; b < btracks_size; b++){
//        float box_area = (btracks[b].tlbr[2] - btracks[b].tlbr[0] + 1)*(btracks[b].tlbr[3] - btracks[b].tlbr[1] + 1);
//        for(int a = 0; a < atracks_size; a++){
//
////            //iou
////            i_area = ((atracks[a]->rect)&(btracks[b].rect)).area(); //交集
////            u_area = atracks[a]->rect.area() + btracks[b].rect.area() - i_area; //并集
////            iou = i_area/u_area;
////            //distance 2d
////            float rectDistance = sqrt(pow(((atracks[a]->rect.x + atracks[a]->rect.width/2)-(btracks[b].rect.x + btracks[b].rect.width/2)),2) + pow(((atracks[a]->rect.y + atracks[a]->rect.height/2)-(btracks[b].rect.y + btracks[b].rect.height/2)),2));
////            if(rectDistance <= rectDistance_thresh){
////                rectDistance = 1.0 - rectDistance/rectDistance_thresh;
////                cost_matrix(a,b) = iou * (1 - this->w_rectDistance) + rectDistance * this->w_rectDistance;
////            } else{
////                cost_matrix(a,b) = iou * (1 - this->w_rectDistance);
////            }
//            //distance 2d
//            double rectDistance = sqrt(pow((atracks[a]->tlbr[0]+atracks[a]->tlbr[2])/2-(btracks[b].tlbr[0]+btracks[b].tlbr[2])/2,2) + pow((atracks[a]->tlbr[1]+atracks[a]->tlbr[3])/2-(btracks[b].tlbr[1]+btracks[b].tlbr[3])/2,2));
//            if(rectDistance <= rectDistance_thresh){
//                cost_matrix(a,b) = (1.0 - rectDistance/rectDistance_thresh) * this->w_rectDistance;
//            } else{
//                cost_matrix(a,b) = 0.0;
//            }
//            float iw = min(atracks[a]->tlbr[2], btracks[b].tlbr[2]) - max(atracks[a]->tlbr[0], btracks[b].tlbr[0]) + 1;
//            float ih = min(atracks[a]->tlbr[3], btracks[b].tlbr[3]) - max(atracks[a]->tlbr[1], btracks[b].tlbr[1]) + 1;
//            if(iw > 0 && ih > 0){
//                float ua = (atracks[a]->tlbr[2] - atracks[a]->tlbr[0] + 1) * (atracks[a]->tlbr[3] - atracks[a]->tlbr[1] + 1) + box_area - iw * ih;
//                cost_matrix(a,b) = cost_matrix(a,b) +  iw * ih / ua * (1 - this->w_rectDistance);
//            }
//
//
//            int a_cls = atracks[a]->cls;
//            int b_cls = btracks[b].cls;
////            if(isBR){
////                if(a < 6){
////                    cost_matrix(a,b) = cost_matrix(a,b) * (1.0 - btracks[b].ws_armorConfMatrix_BR(0,a)*atracks[a]->conf_armor) + btracks[b].ws_armorConfMatrix_BR(0,a)*atracks[a]->conf_armor;
////                }else if( -1 < b_cls && b_cls < 14){
////                    cost_matrix(a,b) = cost_matrix(a,b) * (1.0 - atracks[a]->ws_armorConfMatrix_BR(0,b_cls%7)*btracks[b].conf_armor) + atracks[a]->ws_armorConfMatrix_BR(0,b_cls%7)*btracks[b].conf_armor;
////                }
////            }else{
////                if( -1 < a_cls  && a_cls < 14 ) {
////                    cost_matrix(a, b) = cost_matrix(a, b) * (1.0 - btracks[b].ws_armorConfMatrix(0,a)*atracks[a]->conf_armor) + btracks[b].ws_armorConfMatrix(0,a)*atracks[a]->conf_armor;
////                }else if( -1 < b_cls && b_cls < 14){
////                    cost_matrix(a,b) = cost_matrix(a,b) * (1.0 - atracks[a]->ws_armorConfMatrix(0,b_cls)*btracks[b].conf_armor) + atracks[a]->ws_armorConfMatrix(0,b_cls)*btracks[b].conf_armor;
////                }
////            }
//            cost_matrix(a,b) = 1.0 - cost_matrix(a,b);
//        }
//    }
//    return cost_matrix;
//}


vector<vector<float>> CostMatrix::getIouAndDistancetCost_vec(vector<STrack*> &atracks, vector<STrack> &btracks, int &atracks_size, int &btracks_size, bool isBR){
    atracks_size = atracks.size();btracks_size = btracks.size();
    if (atracks_size * btracks_size == 0)
    {
        vector<vector<float>>  cost_matrix;
        return cost_matrix;
    }

    vector<vector<float>>  cost_matrix(atracks_size,vector<float>(btracks_size));
    float i_area,u_area,iou;
    for(int b = 0; b < btracks_size; b++){
        float box_area = (btracks[b].tlbr[2] - btracks[b].tlbr[0] + 1)*(btracks[b].tlbr[3] - btracks[b].tlbr[1] + 1);
        for(int a = 0; a < atracks_size; a++){

//            //iou
//            i_area = ((atracks[a]->rect)&(btracks[b].rect)).area(); //交集
//            u_area = atracks[a]->rect.area() + btracks[b].rect.area() - i_area; //并集
//            iou = i_area/u_area;
//            //distance 2d
//            float rectDistance = sqrt(pow(((atracks[a]->rect.x + atracks[a]->rect.width/2)-(btracks[b].rect.x + btracks[b].rect.width/2)),2) + pow(((atracks[a]->rect.y + atracks[a]->rect.height/2)-(btracks[b].rect.y + btracks[b].rect.height/2)),2));
//            if(rectDistance <= rectDistance_thresh){
//                rectDistance = 1.0 - rectDistance/rectDistance_thresh;
//                cost_matrix(a,b) = iou * (1 - this->w_rectDistance) + rectDistance * this->w_rectDistance;
//            } else{
//                cost_matrix(a,b) = iou * (1 - this->w_rectDistance);
//            }
            //distance 2d
            double rectDistance = sqrt(pow((atracks[a]->tlbr[0]+atracks[a]->tlbr[2])/2-(btracks[b].tlbr[0]+btracks[b].tlbr[2])/2,2) + pow((atracks[a]->tlbr[1]+atracks[a]->tlbr[3])/2-(btracks[b].tlbr[1]+btracks[b].tlbr[3])/2,2));
            if(rectDistance <= rectDistance_thresh){
                cost_matrix[a][b] = (1.0 - rectDistance/rectDistance_thresh) * this->w_rectDistance;
            } else{
                cost_matrix[a][b] = 0.0;
            }
            float iw = min(atracks[a]->tlbr[2], btracks[b].tlbr[2]) - max(atracks[a]->tlbr[0], btracks[b].tlbr[0]) + 1;
            float ih = min(atracks[a]->tlbr[3], btracks[b].tlbr[3]) - max(atracks[a]->tlbr[1], btracks[b].tlbr[1]) + 1;
            if(iw > 0 && ih > 0){
                float ua = (atracks[a]->tlbr[2] - atracks[a]->tlbr[0] + 1) * (atracks[a]->tlbr[3] - atracks[a]->tlbr[1] + 1) + box_area - iw * ih;
                cost_matrix[a][b] = cost_matrix[a][b] +  iw * ih / ua * (1 - this->w_rectDistance);
            }


            int a_cls = atracks[a]->cls;
            int b_cls = btracks[b].cls;
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
            cost_matrix[a][b] = 1.0 - cost_matrix[a][b];
        }
    }
    return cost_matrix;
}

Eigen::MatrixXd CostMatrix::getCost_confMatrix(std::vector<STrack*> &strack_pool,int &num_strack,int &num_cls){

//    num_strack = strack_pool.size();num_cls = 14 + 6;////TODO:
    num_strack = strack_pool.size();num_cls = classWithoutCar;////TODO:
    Eigen::MatrixXd cost_confMatrix = Eigen::MatrixXd::Ones(num_strack,num_cls);
    for(int k=0; k<strack_pool.size(); k++){
        //从矩阵中提取一个子块
        cost_confMatrix.block(k,0,1,classWithoutCar) -= strack_pool[k]->ws_armorConfMatrix.block(0,0,1,classWithoutCar);
//        cost_confMatrix.block(k,14,1,6) = Eigen::MatrixXd::Ones(1,6) * 0.1;
    }
    return cost_confMatrix;
}

Eigen::MatrixXd CostMatrix::getCost_confMatrix(std::vector<STrack> &strack_pool,int &num_strack,int &num_cls){
    num_strack = strack_pool.size();num_cls = 14 + 6;////TODO:
    Eigen::MatrixXd cost_confMatrix = Eigen::MatrixXd::Zero(num_strack,num_cls);
    for(int k=0; k<strack_pool.size(); k++){
        cost_confMatrix.block(k,0,1,14) = strack_pool[k].ws_armorConfMatrix.block(0,0,1,14);
//        cost_confMatrix.block(k,14,1,6) = Eigen::MatrixXd::Ones(1,6) * 0.1;
    }
    return cost_confMatrix;
}

void CostMatrix::linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
                             vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b)
{
    if (cost_matrix.size() == 0)
    {
        for (int i = 0; i < cost_matrix_size; i++)
        {
            unmatched_a.push_back(i);
        }
        for (int i = 0; i < cost_matrix_size_size; i++)
        {
            unmatched_b.push_back(i);
        }
        return;
    }

    vector<int> rowsol; vector<int> colsol;
    float c = lapjv(cost_matrix, rowsol, colsol, true, thresh);
    for (int i = 0; i < rowsol.size(); i++)
    {
        if (rowsol[i] >= 0)
        {
            vector<int> match;
            match.push_back(i);
            match.push_back(rowsol[i]);
            matches.push_back(match);
        }
        else
        {
            unmatched_a.push_back(i);
        }
    }

    for (int i = 0; i < colsol.size(); i++)
    {
        if (colsol[i] < 0)
        {
            unmatched_b.push_back(i);
        }
    }
}

double CostMatrix::lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol,
                          bool extend_cost, float cost_limit, bool return_cost)
{
    vector<vector<float> > cost_c;
    cost_c.assign(cost.begin(), cost.end());

    vector<vector<float> > cost_c_extended;

    int n_rows = cost.size();
    int n_cols = cost[0].size();
    rowsol.resize(n_rows);
    colsol.resize(n_cols);

    int n = 0;
    if (n_rows == n_cols)
    {
        n = n_rows;
    }
    else
    {
        if (!extend_cost)
        {
            cout << "set extend_cost=True" << endl;
            system("pause");
            exit(0);
        }
    }

    //扩展矩阵并将其初始化，扩展成n_rows+n_cols大小的矩阵
    if (extend_cost || cost_limit < LONG_MAX)
    {
        n = n_rows + n_cols;
        cost_c_extended.resize(n);
        for (int i = 0; i < cost_c_extended.size(); i++)
            cost_c_extended[i].resize(n);

        //初始化扩展矩阵
        if (cost_limit < LONG_MAX)//初始值设为cost_limit的一半
        {
            for (int i = 0; i < cost_c_extended.size(); i++)
            {
                for (int j = 0; j < cost_c_extended[i].size(); j++)
                {
                    cost_c_extended[i][j] = cost_limit / 2.0;//？？
                }
            }
        }
        else//初始值设为最大代价值+1
        {
            float cost_max = -1;
            for (int i = 0; i < cost_c.size(); i++)
            {
                for (int j = 0; j < cost_c[i].size(); j++)
                {
                    if (cost_c[i][j] > cost_max)
                        cost_max = cost_c[i][j];
                }
            }
            for (int i = 0; i < cost_c_extended.size(); i++)
            {
                for (int j = 0; j < cost_c_extended[i].size(); j++)
                {
                    cost_c_extended[i][j] = cost_max + 1;
                }
            }
        }

        //调整代价值，将扩展的部分代价值调成0
        for (int i = n_rows; i < cost_c_extended.size(); i++)
        {
            for (int j = n_cols; j < cost_c_extended[i].size(); j++)
            {
                cost_c_extended[i][j] = 0;
            }
        }
        //调整代价值，将原矩阵的值赋给扩展矩阵未扩展的部分
        for (int i = 0; i < n_rows; i++)
        {
            for (int j = 0; j < n_cols; j++)
            {
                cost_c_extended[i][j] = cost_c[i][j];
            }
        }

        cost_c.clear();
        cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
    }

    double **cost_ptr;//声明一个指向二维数组的指针
    cost_ptr = new double *[sizeof(double *) * n];//动态分配了 n 个 double * 类型的空间
    for (int i = 0; i < n; i++)
        cost_ptr[i] = new double[sizeof(double) * n];//每个都是n个空间

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            cost_ptr[i][j] = cost_c[i][j];
        }
    }

    int* x_c = new int[sizeof(int) * n];
    int *y_c = new int[sizeof(int) * n];

    int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
    if (ret != 0)
    {
        cout << "Calculate Wrong!" << endl;
        system("pause");
        exit(0);
    }

    double opt = 0.0;

    if (n != n_rows)
    {
        for (int i = 0; i < n; i++)
        {
            if (x_c[i] >= n_cols)
                x_c[i] = -1;
            if (y_c[i] >= n_rows)
                y_c[i] = -1;
        }
        for (int i = 0; i < n_rows; i++)
        {
            rowsol[i] = x_c[i];
        }
        for (int i = 0; i < n_cols; i++)
        {
            colsol[i] = y_c[i];
        }

        if (return_cost)
        {
            for (int i = 0; i < rowsol.size(); i++)
            {
                if (rowsol[i] != -1)
                {
                    //cout << i << "\t" << rowsol[i] << "\t" << cost_ptr[i][rowsol[i]] << endl;
                    opt += cost_ptr[i][rowsol[i]];
                }
            }
        }
    }
    // else if (return_cost)
    // {
    //     for (int i = 0; i < rowsol.size(); i++)
    //     {
    //         opt += cost_ptr[i][rowsol[i]];
    //     }
    // }

    else{
        for (int i = 0; i < n_rows; i++)
            rowsol[i] = x_c[i];
        for (int i = 0; i < n_cols; i++)
            colsol[i] = y_c[i];
        if(return_cost){
            for (int i = 0; i < rowsol.size(); i++)
                opt += cost_ptr[i][rowsol[i]];
        }
    }

    for (int i = 0; i < n; i++)
    {
        delete[]cost_ptr[i];
    }
    delete[]cost_ptr;
    delete[]x_c;
    delete[]y_c;

    return opt;
}