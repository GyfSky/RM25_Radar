//
// Created by plusseven on 24-1-20.
//

#include "../include/Predict.h"

Predict::Predict(){
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
//    options.minimizer_progress_to_stdout = true;   // 输出到cout
}

std::vector<double> Predict::getAngles(std::vector<cv::Point3d> Locate3Ds) {
    std::vector<double> angles;
    for(auto &Locate3D : Locate3Ds){
        double temp_angle = -1.0;
        double x= Locate3D.x, y = Locate3D.y;
        if(abs(x) <= 1e-2 && abs(y) <= 1e-2){ // 判定为静止，只要最近期的一个angle备用，此前angles记为无效
            if(angles.size()>0)
                temp_angle = angles.back();

            angles.clear();
            if(temp_angle<-1e-6)
                angles.push_back(temp_angle);
            continue;
        }
        else if(abs(x) < 1e-2 || abs(y) < 1e-2){
            temp_angle = 0.0;
        }else{
            temp_angle = abs(atan(y/x)) / M_PI * 180.;
        }

        if( x>=1e-2 && y>-1e-2 ){
            temp_angle = temp_angle + 0.0;
        } else if( x<1e-2 && y>=1e-2 ){
            temp_angle = 180.0 - temp_angle ;
        } else if( x<=-1e-2 && y<1e-2){
            temp_angle = temp_angle + 180.0;
        } else if( x>-1e-2 && y<=-1e-2){
            temp_angle = 360.0 - temp_angle;
        }
        angles.push_back(temp_angle);
    }
    if(angles.size()>0){
        isguess = true;
    }else{
        isguess = false;
    }
    return angles;
}

void Predict::getVelAccfromPathFitting(double *vel_acc,std::vector<double> shifts) {
    for(int i = 0;i < shifts.size(); i++){
        problem.AddResidualBlock
                (
                        // 向问题中添加误差项
                        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                                new CURVE_FITTING_COST(shifts[i] , dt * i)
                        ),
                        nullptr,            // 核函数，这里不使用，为空
                        vel_acc                 // 待估计参数
                );
    }
//    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);  // 开始优化
//    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
//    std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;
//    std::cout << "solve time cost = " << 1./(time_used.count()) << " Fps " << std::endl;

    // 输出结果
//    std::cout << summary.BriefReport() << std::endl;
//    std::cout << "estimated vel, acc = ";
//    for (auto a:vel_acc) std::cout << a << " ";
//    std::cout << std::endl;
}




void Predict::loss_track_predict(std::vector<MapVertex> &vexs, std::vector<std::vector<MapEdge>> arcs, std::vector<STrack> &tracks) {
    for(auto &track:tracks){
        int vexSerialNum = track.vexSerialNum.back();

        if(track.tracklet_len<1){
            std::vector<double> angles = this->getAngles(track.change_Locate3Ds);
            track.angle = angles.back();
            if( vexs[vexSerialNum].missType == Line){
                if(track.change_Locate3Ds.size() < 1){
                    track.speed_vel = 0.0;
                }
                else if(track.change_Locate3Ds.size() < 3){
                    track.speed_vel = track.change_distance.back()/dt;
                }
                else if(track.change_Locate3Ds.size() < 5){
                    double speed_vel_acc[] = {guess_speed_vel, guess_speed_acc};
                    this->getVelAccfromPathFitting(speed_vel_acc, track.change_distance);
                    track.speed_vel = speed_vel_acc[0];
                    track.speed_acc = speed_vel_acc[1];
                }
                else if(angles.size() >= 5){
                    double angle_vel_acc[] = {guess_angle_vel, guess_angle_acc};
                    this->getVelAccfromPathFitting(angle_vel_acc, angles);
                    track.angle_vel = angle_vel_acc[0];
                    track.angle_acc = angle_vel_acc[1];
                }
            }
        }
        if(vexs[vexSerialNum].missType == Drop){
            track.Locate3D = vexs[vexSerialNum].missPoints[0];
        }else if(vexs[vexSerialNum].missType == Line){

            double diff_angle = vexs[vexSerialNum].missAngle - track.angle;
            double flow_direction_angle;
            double change_diatance = track.speed_vel * dt + track.speed_acc * dt * dt / 2.0;
            double change_angle = track.angle_vel * dt + track.angle_acc * dt * dt / 2.0;

            if(abs(diff_angle) > 83.5){ // 与flow_direction垂直的附近15度判定为无法计算得到流向
                flow_direction_angle = -1;
            }else if(abs(diff_angle) < abs(diff_angle -180)){
                flow_direction_angle = vexs[vexSerialNum].missAngle + 180;
            }else{
                flow_direction_angle = vexs[vexSerialNum].missAngle;
            }

            track.angle += change_angle;

            diff_angle = flow_direction_angle - track.angle;
            if(abs(diff_angle)< 5){ // 与flow_direction的差为10度判定为与流向方向一致
                track.angle = flow_direction_angle;
                track.angle_vel = 0.0;
                track.angle_acc = 0.0;
            }
            cv::Point3d temp_Locate3D;

            temp_Locate3D.x += change_diatance * cos(track.angle*M_PI/180.);
            temp_Locate3D.y += change_diatance * sin(track.angle*M_PI/180.);

            int in_where = -1;
            in_where = initornot3D(vexs[vexSerialNum].points_reality_3d, track.Locate3D, vexs[vexSerialNum].point_3d_number);// 判断是否在之前所在的区域
            if(in_where == -1){                                                                     // 判断是否在相邻的区域中
                for(int p=0; p<vexs.size(); p++){
                    in_where = initornot3D(vexs[p].points_reality_3d, track.Locate3D, vexs[p].point_3d_number);
                    if(in_where != -1){
                        break;
                    }
                }
            }
            if(in_where != -1){
                track.Locate3D.x = temp_Locate3D.x;
                track.Locate3D.y = temp_Locate3D.y;
            }else{
                track.Locate3D.x = change_diatance * cos(flow_direction_angle*M_PI/180.);
                track.Locate3D.y = change_diatance * sin(flow_direction_angle*M_PI/180.);
            }
        }
    }

}


void Predict::loss_track_predict(std::vector<STrack> &tracks) {
    std::vector<STrack> add_run_track;
    for(auto track:tracks){
        if(track.lost_frame_ind_num<1){
            track.state = LostCopy;
            add_run_track.push_back(track);
        }
    }
    tracks.insert(tracks.end(),add_run_track.begin(),add_run_track.end());
    std::cout << " tracks.size() : " << tracks.size() << std::endl;
}



void Predict::loss_track_predict(const cv::Mat T_Cam2World,const double fx, const double fy,
                                 const double cx, const double cy,std::vector<STrack> &tracks) {
    std::vector<STrack> add_run_track;
    for(auto &track:tracks){
        if(track.lost_frame_ind_num<1){
            STrack copy_track = track;
            std::vector<double> angles = this->getAngles(copy_track.change_Locate3Ds);
            copy_track.angle = angles.back();
            copy_track.state = LostCopy;
            if(copy_track.change_Locate3Ds.size() < 1){
                copy_track.speed_vel = 0.0;
            }
            else if(copy_track.change_Locate3Ds.size() < 3){
                copy_track.speed_vel = copy_track.change_distance.back()/dt;
            }
            else {//if(copy_track.change_Locate3Ds.size() < 5)
                double speed_vel_acc[] = {guess_speed_vel, guess_speed_acc};
                this->getVelAccfromPathFitting(speed_vel_acc, copy_track.change_distance);
                copy_track.speed_vel = speed_vel_acc[0];
                copy_track.speed_acc = speed_vel_acc[1];
            }
//            if(angles.size() >= 5){
//                double angle_vel_acc[] = {guess_angle_vel, guess_angle_acc};
//                this->getVelAccfromPathFitting(angle_vel_acc, angles);
//                track.angle_vel = angle_vel_acc[0];
//                track.angle_acc = angle_vel_acc[1];
//            }
            double change_diatance = copy_track.speed_vel * dt + copy_track.speed_acc * dt * dt / 2.0;
            double change_angle = copy_track.angle_vel * dt + copy_track.angle_acc * dt * dt / 2.0;

            copy_track.angle += change_angle;
            copy_track.Locate3D.x += change_diatance * cos(copy_track.angle*M_PI/180.);
            copy_track.Locate3D.y += change_diatance * sin(copy_track.angle*M_PI/180.);

            getRectFromLocate3D(T_Cam2World, fx, fy, cx, cy,copy_track);

            add_run_track.push_back(copy_track);
        }
        else if(track.state == LostCopy){
            double change_diatance = track.speed_vel * dt + track.speed_acc * dt * dt / 2.0;
            double change_angle = track.angle_vel * dt + track.angle_acc * dt * dt / 2.0;

            track.angle += change_angle;
            track.Locate3D.x += change_diatance * cos(track.angle*M_PI/180.);
            track.Locate3D.y += change_diatance * sin(track.angle*M_PI/180.);

            getRectFromLocate3D(T_Cam2World, fx, fy, cx, cy,track);

        }

    }
    tracks.insert(tracks.end(),add_run_track.begin(),add_run_track.end());
//    std::cout << " tracks.size() : " << tracks.size() << std::endl;
    for(auto track : tracks)
        std::cout << track.track_id << " " ;
    std::cout << "--------" << std::endl;


}


void Predict::getRectFromLocate3D(const cv::Mat T_Cam2World,const double fx, const double fy,
                                  const double cx, const double cy, STrack &track) {
    cv::Mat T_World2Cam = T_Cam2World.inv();
    Eigen::Matrix<double,4,4> Rt;
    cv::cv2eigen(T_World2Cam.inv(),Rt);
    Eigen::Vector4d temp_reality_3d(track.Locate3D.x,track.Locate3D.y,track.Locate3D.z, 1);
    Eigen::Vector4d pc = Rt * temp_reality_3d;
    Eigen::Vector2d project(fx * pc[0] / pc[2] + cx,fy * pc[1] / pc[2] + cy );
    double w = track.tlwh[2];
    double h = track.tlwh[3];
    double x = project.x() - w/2.0;
    double y = project.y() - h*track.p_focus;
    track.tlwh[0] = x;
    track.tlwh[1] = y;

}

void Predict::getRectFromLocate3D(const cv::Mat T_Cam2World,const double fx, const double fy,
                                  const double cx, const double cy, std::vector<STrack> &tracks) {
    cv::Mat T_World2Cam = T_Cam2World.inv();
    Eigen::Matrix<double,4,4> Rt;
    cv::cv2eigen(T_World2Cam.inv(),Rt);
    for(auto &track : tracks){
        Eigen::Vector4d temp_reality_3d(track.Locate3D.x,track.Locate3D.y,track.Locate3D.z, 1);
        Eigen::Vector4d pc = Rt * temp_reality_3d;
        Eigen::Vector2d project(fx * pc[0] / pc[2] + cx,fy * pc[1] / pc[2] + cy );
        double w = track.tlwh[2];
        double h = track.tlwh[3];
        double x = project.x() - w/2.0;
        double y = project.y() - h*track.p_focus;
        track.tlwh[0] = x;
        track.tlwh[1] = y;
    }

}