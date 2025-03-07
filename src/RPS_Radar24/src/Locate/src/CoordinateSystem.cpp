#include"../include/CoordinateSystem.h"


/**
 * @brief 得到xxx2world 的旋转矩阵
 * @param OutputArray cv::Mat &T 相机2世界 的旋转矩阵
 */
void MatrixCoordinateSystem::Get2world_matrix(cv::Mat &T,cv::Mat &K,std::vector<cv::Point2d> &pts_pnp_2d,std::vector<cv::Point3d> &pts_pnp_3d,cv::Mat dist){
    if(pts_pnp_2d.size() != pts_pnp_3d.size()){
        std::cerr << "can't PNP because the point number is not same" << std::endl;
    }
    cv::Mat r,t,R;
    cv::solvePnP(pts_pnp_3d, pts_pnp_2d, K ,dist, r, t, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    T = (cv::Mat_<float>(4, 4)<<
        R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0),
        R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0),
        R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0),
        0,0,0,1);
    std::cout << "T: " << T << std::endl;
    T = T.inv();
    std::cout << "T.inv: " << T << std::endl;

}

void MatrixCoordinateSystem::get_Point_2d(cv::Mat T_2self, cv::Mat K, cv::Point3f &point3f, cv::Point2f &point2f){
    cv::Mat point3f_world = (cv::Mat_<float>(4, 1) << point3f.x, point3f.y, point3f.z, 1.0);
//    cv::Mat point3f_world = Mat::ones(4,1,CV_32FC1);
//    T_2self = Mat::eye(4,4,CV_32FC1);
    std::cout << "T_2self: " << T_2self << std::endl;
    std::cout << "point3f_world: " << point3f_world << std::endl;
    cv::Mat temp_point3f_self = T_2self * point3f_world;
    cv::Mat point3f_self = (cv::Mat_<float>(3, 1) <<
            temp_point3f_self.at<float>(0), temp_point3f_self.at<float>(1), temp_point3f_self.at<float>(2));
    cv::Mat point2f_self = (1.0/point3f.z) * K * point3f_self;
    point2f = cv::Point2f(point2f_self.at<float>(0), point2f_self.at<double>(1));
}



double MatrixCoordinateSystem::getH(MapVertex vex, cv::Point2d Locate2D) {
    Eigen::Matrix<double, 2, 1> matrix_2d_xy;
    Eigen::Matrix<double, 2, 1> matrix_2d_getxy;
    double Hight, X , Y;
    matrix_2d_xy << (Locate2D.x - vex.points_predict_2d.at(1).x),
            (Locate2D.y -vex.points_predict_2d.at(1).y);

    matrix_2d_getxy = vex.matrix_change2.inverse() * matrix_2d_xy;
    matrix_2d_getxy = vex.matrix_change3 * matrix_2d_getxy + vex.matrix_2d;

    X = matrix_2d_getxy.x();
    Y = matrix_2d_getxy.y();
    Hight = vex.getH_abc_3d.at(0) * X + vex.getH_abc_3d.at(1) * Y +
            vex.getH_abc_3d.at(2);
    return Hight;
}

void MatrixCoordinateSystem::set_warring_and_place_by_locate3D(const MapVertex& vex, OurPattern ourPattern, std::vector<bool> &isWarring, STrack &track){
    if(vex.placeType==flySlope && ( ((vex.placeColor == PlaceColor::R) &&(ourPattern==blue)) || ((vex.placeColor == PlaceColor::B) &&(ourPattern==red))) ){
        isWarring[0] = true;
        track.placeType = flySlope;
    }


    if((vex.placeType==holeWarring || vex.placeType==hole) && (((vex.placeColor == PlaceColor::B)) || ((vex.placeColor == PlaceColor::R) && (ourPattern==red) ))){
        //步兵
        if(((ourPattern==red) && (track.cls < classWithoutCar/2-1 && track.cls > 1)) || ((ourPattern==blue) &&  (track.cls < classWithoutCar-1 && track.cls > classWithoutCar/2+1))){
            isWarring[1] = true;
            track.placeType = holeWarring;
        }
        else if(((ourPattern==red) && (track.cls < classWithoutCar/2 || track.cls > classWithoutCar-1)) || ((ourPattern==blue) &&  (track.cls < 0 || track.cls > classWithoutCar/2-1))){
            isWarring[2] = true;
            track.placeType = holeWarring;
        }
    }
    if(vex.placeType== windmill && ( ((vex.placeColor == PlaceColor::R) &&(ourPattern==blue)) || ((vex.placeColor == PlaceColor::B) &&(ourPattern==red))) ){
        isWarring[3] = true;
        track.placeType = windmill;
    }

    if(vex.placeType==startupArea && ( ((vex.placeColor == PlaceColor::R) &&(ourPattern==blue)) || ((vex.placeColor == PlaceColor::B) &&(ourPattern==red))) ){
        if((ourPattern==red) && (track.cls < classWithoutCar/2 || track.cls > classWithoutCar-1) ){
//            double B7_conf = std::min(0.95, track.ws_armorConfMatrix(0, 5) + startupArea_car_conf);
//            track.ws_armorConfMatrix(0, 5) = B7_conf;
            track.placeType = startupArea;
//            std::cout << "vex.placeType==startupArea: " << track.cls  << "  " << track.ws_armorConfMatrix(0, 5) << std::endl;
        }
        else if((ourPattern==blue) &&  (track.cls < 0 || track.cls > classWithoutCar/2-1)){
            track.placeType = startupArea;
        }
    }
}

void MatrixCoordinateSystem::coordinateCorrection(cv::Point3d &Locate3D, OurPattern ourPattern){
    //？？
    if(Locate3D.y < edge)
        Locate3D.y = edge;
    else if(Locate3D.y > (venue_h - edge))
        Locate3D.y = venue_h - edge;

    if(ourPattern == red){
        if(Locate3D.x > 27)
            Locate3D.x = 27;

        if(Locate3D.y > 11.05){
            //兑换站
            if(Locate3D.x > 26.5)
                Locate3D.x = 26.5;
            else if(Locate3D.x > 24.7)
                Locate3D.x = 24.0;

        }
    }
    else if(ourPattern == blue){
        if(Locate3D.x < 1.0)
            Locate3D.x = 1.0;

        if(Locate3D.y < 3.95){
            if(Locate3D.x < 1.5)
                Locate3D.x = 1.5;
            else if(Locate3D.x < 3.3)
                Locate3D.x = 4.0;
        }
    }

}

/***
 * @note new_use
 *
 * */
void  MatrixCoordinateSystem::solve_reality_3d(const cv::Mat T_, const double fx, const double fy, const double cx,
                                              const double cy, std::vector<MapVertex> &vexs,std::vector<std::vector<MapEdge>> arcs,
                                              std::vector<STrack> &tracks, OurPattern ourPattern,std::vector<bool> &isWarring) {

    cv::Mat T = T_.inv(); // 把相机2世界 转为 世界2相机
    double a = T.at<float>(0, 0);
    double b = T.at<float>(0, 1);
    double c = T.at<float>(0, 2);

    double d = T.at<float>(1, 0);
    double e = T.at<float>(1, 1);
    double f = T.at<float>(1, 2);

    double g = T.at<float>(2, 0);
    double h = T.at<float>(2, 1);
    double i = T.at<float>(2, 2);

    double tx = T.at<float>(0, 3);
    double ty = T.at<float>(1, 3);
    double tz = T.at<float>(2, 3);
    //遍历每个轨迹
    for (auto & track : tracks) {
        double Hight = 0;
        int in_where = -1;
        double A, B, Z;

        Eigen::Matrix<double, 2, 1> matrix_2d_xy;
        Eigen::Matrix<double, 2, 1> matrix_2d_getxy;
////TODO:test!!!!!!

//        if(tracks[s].vexSerialNum.size()>0){            // 判断是否为新轨迹
//            int vexSerialNum = tracks[s].vexSerialNum.back();
//            in_where = initornot(vexs[vexSerialNum].points_predict_2d,tracks[s].Locate2D);
//            if(in_where != -1){                     // 判断是否在之前所在的区域
//                tracks[s].push_front_vexSerialNum(vexSerialNum);
//                Hight = this->getH(vexs[vexSerialNum], tracks[s].Locate2D);
//            } else{                                 // 判断是否在相邻的区域中
//                for(int p=0; p<vexs.size(); p++){
//                    if( arcs[vexSerialNum][p].p > 5 || arcs[vexSerialNum][p].p < 1e-6){
//                        continue;
//                    } else {
//                        in_where = initornot(vexs[p].points_predict_2d,tracks[s].Locate2D);
//                        if(in_where != -1){
//                            tracks[s].push_front_vexSerialNum(vexSerialNum);
//                            Hight = this->getH(vexs[vexSerialNum], tracks[s].Locate2D);
//                        }
//                    }
//                }
//                if(in_where == -1){                  //  如果也不在相邻的区域中，则H
//                    tracks[s].false_Hs++;
////                    if(tracks[s].false_Hs >= 19){
////                        tracks[s].vexSerialNum.clear();
////                        for (auto & vex : vexs) {
////                            if(ourPattern == red){
////                                if(vex.seeType == Rhide || vex.seeType == hide ){
////                                    continue;
////                                }
////                            }else if(ourPattern == blue){
////                                if(vex.seeType == Bhide || vex.seeType == hide ){
////                                    continue;
////                                }
////                            }else{
////                                std::cout << "error in ourPattern" << std::endl;
////                            }
////                            in_where = initornot(vex.points_predict_2d, tracks[s].Locate2D); //cv::pointPolygonTest 点xy 是否在 xxx中
////                            if (in_where != -1) {
////                                tracks[s].push_front_vexSerialNum(vex.vertex);
////                                Hight = this->getH(vex, tracks[s].Locate2D);
////                                tracks[s].oldH = Hight;
////                                break;
////                            }
////                        }
////                    }
////                    else
//                        Hight = tracks[s].oldH;
//
//
//                }else{
//                    tracks[s].oldH = Hight;
//                }
//            }
//        }else{
            ////TODO:maybe is useless
            double is_windmill_H ;
            for (auto & vex : vexs) {
                if(ourPattern == red){
                    if(vex.seeType == Rhide || vex.seeType == hide ){
                        continue;
                    }
                }else if(ourPattern == blue){
                    if(vex.seeType == Bhide || vex.seeType == hide ){
                        continue;
                    }
                }
//                else{
//                    std::cout << "error in ourPattern" << std::endl;
//                }
                in_where = initornot(vex.points_predict_2d, track.Locate2D, vex.point_3d_number); //cv::pointPolygonTest 点xy 是否在 xxx中
                if (in_where != -1) {
                    set_warring_and_place_by_locate3D(vex, ourPattern, isWarring, track);
                    track.push_front_vexSerialNum(vex.vertex-1);
                    Hight = this->getH(vex, track.Locate2D);
                    track.oldH = Hight;
                    if(vex.placeType == is_windmill){
                        continue;
                    }
                    break;
                }
            }
//        }
//        std::cout << "tracks[s].vexSerialNum.size()" << tracks[s].vexSerialNum.size() << std::endl;
        track.oldH = Hight;
//        std::cout << "Hight:" << Hight << std::endl;

        //Xc为相机坐标系下的物体x坐标，Yc为相机坐标系下的物体y坐标，Zc为相机坐标系下的物体的深度（距离）

        // [a,b,c,tx
        //  d,e,f,ty
        //  g,h,i,tz]

        A = (track.Locate2D.x - cx) / fx;//A=Xc/Zc
        B = (track.Locate2D.y - cy) / fy;//A=Yc/Zc

        Z = (a - A * g) * (e - B * h) - (d - B * g) * (b - A * h);
        if (Z < 1e-6 && Z > -1e-6) {
            std::cout << "have one error in getLocate3D" << std::endl;
        } else { //TODO: -0.5 because map have error
            track.Locate3D.x = ((((A * i - c) * Hight + (A * tz - tx)) * (e - B * h) -
                               ((B * i - f) * Hight + (B * tz - ty)) * (b - A * h)) / Z) - 0.5;
            track.Locate3D.y = (((a - A * g) * ((B * i - f) * Hight + (B * tz - ty)) -
                               (d - B * g) * ((A * i - c) * Hight + (A * tz - tx))) / Z) - 0.5;

            coordinateCorrection(track.Locate3D, ourPattern);
            track._Locate3D = track.Locate3D;
        }
        if(track.tracklet_len>1e-6)
            track.push_front_change_Locate3D_and_distance((track.Locate3D-track.old_Locate3D));
        track.old_Locate3D = track.Locate3D;
//        tracks[s].old_Locate3D.z = 0.0;
//        std::cout << "change_Locate3Ds:2  "  << tracks[s].change_Locate3Ds << std::endl;
    }

}



/***
 * @note new_use
 *
 * */
void  MatrixCoordinateSystem::solve_reality_3d(
        const cv::Mat T_, const double fx, const double fy, const double cx,const double cy, std::vector<STrack> &tracks)
{
    cv::Mat T = T_.inv(); // 把相机2世界 转为 世界2相机
    double a = T.at<float>(0, 0);
    double b = T.at<float>(0, 1);
    double c = T.at<float>(0, 2);

    double d = T.at<float>(1, 0);
    double e = T.at<float>(1, 1);
    double f = T.at<float>(1, 2);

    double g = T.at<float>(2, 0);
    double h = T.at<float>(2, 1);
    double i = T.at<float>(2, 2);

    double tx = T.at<float>(0, 3);
    double ty = T.at<float>(1, 3);
    double tz = T.at<float>(2, 3);
    for (int s=0 ; s<tracks.size();s++) {
        double Hight = 0;
        int in_where = -1;
        double A, B, Z;

        Eigen::Matrix<double, 2, 1> matrix_2d_xy;
        Eigen::Matrix<double, 2, 1> matrix_2d_getxy;

        Hight = tracks[s].oldH;
        std::cout << "Hight:" << Hight << std::endl;
        A = (tracks[s].Locate2D.x - cx) / fx;
        B = (tracks[s].Locate2D.y - cy) / fy;

        Z = (a - A * g) * (e - B * h) - (d - B * g) * (b - A * h);
        if (Z < 1e-6 && Z > -1e-6) {
            std::cout << "have one error in getLocate3D" << std::endl;
        } else {
            tracks[s].Locate3D.x = ((((A * i - c) * Hight + (A * tz - tx)) * (e - B * h) -
                               ((B * i - f) * Hight + (B * tz - ty)) * (b - A * h)) / Z);
            tracks[s].Locate3D.y = (((a - A * g) * ((B * i - f) * Hight + (B * tz - ty)) -
                               (d - B * g) * ((A * i - c) * Hight + (A * tz - tx))) / Z);
        }

    }

}


/***
 * @note old_use
 *
 * */
void  MatrixCoordinateSystem::solve_reality_3d(
        const cv::Mat T_, const double fx, const double fy, const double cx,const double cy,
        std::vector<MapVertex> &vexs,std::vector<std::vector<MapEdge>> arcs,std::vector<Car> &tracks, OurPattern ourPattern)
{
    cv::Mat T = T_.inv(); // 把相机2世界 转为 世界2相机
    double a = T.at<float>(0, 0);
    double b = T.at<float>(0, 1);
    double c = T.at<float>(0, 2);

    double d = T.at<float>(1, 0);
    double e = T.at<float>(1, 1);
    double f = T.at<float>(1, 2);

    double g = T.at<float>(2, 0);
    double h = T.at<float>(2, 1);
    double i = T.at<float>(2, 2);

    double tx = T.at<float>(0, 3);
    double ty = T.at<float>(1, 3);
    double tz = T.at<float>(2, 3);
    for (int s=0 ; s<tracks.size();s++) {
        double Hight = 0.0;
        int in_where = -1;
        double A, B, Z;

        Eigen::Matrix<double, 2, 1> matrix_2d_xy;
        Eigen::Matrix<double, 2, 1> matrix_2d_getxy;

        ////TODO:maybe is useless
        for (auto & vex : vexs) {
            if(ourPattern == red){
                if(vex.seeType == Rhide || vex.seeType == hide ){
                    continue;
                }
            }else if(ourPattern == blue){
                if(vex.seeType == Bhide || vex.seeType == hide ){
                    continue;
                }
            }else{
                std::cout << "error in ourPattern" << std::endl;
            }
            in_where = initornot(vex.points_predict_2d, tracks[s].Locate2D,vex.point_3d_number); //cv::pointPolygonTest 点xy 是否在 xxx中
            if (in_where != -1) {
                std::cout << "---in_where: " << vex.vertex << std::endl;
                Hight = this->getH(vex, tracks[s].Locate2D);
                in_where = initornot(vex.points_predict_2d, tracks[s].Locate2D,vex.point_3d_number); //cv::pointPolygonTest 点xy 是否在 xxx中

                break;
            }
        }
//        std::cout << "tracks[s].vexSerialNum.size()" << tracks[s].vexSerialNum.size() << std::endl;
        std::cout << "Hight:" << Hight << ",cls " << tracks[s].cls << std::endl;
        A = (tracks[s].Locate2D.x - cx) / fx;
        B = (tracks[s].Locate2D.y - cy) / fy;

        Z = (a - A * g) * (e - B * h) - (d - B * g) * (b - A * h);
        if (Z < 1e-6 && Z > -1e-6) {
            std::cout << "have one error in getLocate3D" << std::endl;
        } else {
            tracks[s].Locate3D.x = ((((A * i - c) * Hight + (A * tz - tx)) * (e - B * h) -
                                     ((B * i - f) * Hight + (B * tz - ty)) * (b - A * h)) / Z);
            tracks[s].Locate3D.y = (((a - A * g) * ((B * i - f) * Hight + (B * tz - ty)) -
                                     (d - B * g) * ((A * i - c) * Hight + (A * tz - tx))) / Z);
        }

    }

}

cv::Point3d  MatrixCoordinateSystem::solve_reality_3d(
        const cv::Mat T_, const double fx, const double fy, const double cx,const double cy,
        std::vector<MapVertex> &vexs,std::vector<std::vector<MapEdge>> arcs,cv::Rect &rect, OurPattern ourPattern)
{
    cv::Mat T = T_.inv(); // 把相机2世界 转为 世界2相机
    double a = T.at<float>(0, 0);
    double b = T.at<float>(0, 1);
    double c = T.at<float>(0, 2);

    double d = T.at<float>(1, 0);
    double e = T.at<float>(1, 1);
    double f = T.at<float>(1, 2);

    double g = T.at<float>(2, 0);
    double h = T.at<float>(2, 1);
    double i = T.at<float>(2, 2);

    double tx = T.at<float>(0, 3);
    double ty = T.at<float>(1, 3);
    double tz = T.at<float>(2, 3);


    double Hight = 0.2;
    int in_where = -1;
    double A, B, Z;

    cv::Point xy = cv::Point(rect.x + rect.width/2,rect.y + rect.height);
    cv::Point3d xyz;


    Eigen::Matrix<double, 2, 1> matrix_2d_xy;
    Eigen::Matrix<double, 2, 1> matrix_2d_getxy;

    ////TODO:maybe is useless
    for (auto & vex : vexs) {
        if(ourPattern == red){
            if(vex.seeType == Rhide || vex.seeType == hide ){
                continue;
            }
        }else if(ourPattern == blue){
            if(vex.seeType == Bhide || vex.seeType == hide ){
                continue;
            }
        }else{
            std::cout << "error in ourPattern" << std::endl;
        }
        in_where = initornot(vex.points_predict_2d, xy,vex.point_3d_number); //cv::pointPolygonTest 点xy 是否在 xxx中
        if (in_where != -1) {
            std::cout << "---in_where: " << vex.vertex << std::endl;
            Hight = this->getH(vex, xy);
            in_where = initornot(vex.points_predict_2d, xy,vex.point_3d_number); //cv::pointPolygonTest 点xy 是否在 xxx中
            break;
        }
    }
//        std::cout << "tracks[s].vexSerialNum.size()" << tracks[s].vexSerialNum.size() << std::endl;
    std::cout << "Hight:" << Hight << std::endl;
    A = (xy.x - cx) / fx;
    B = (xy.y - cy) / fy;

    Z = (a - A * g) * (e - B * h) - (d - B * g) * (b - A * h);
    if (Z < 1e-6 && Z > -1e-6) {
        std::cout << "have one error in getLocate3D" << std::endl;
    } else {
        xyz.x = ((((A * i - c) * Hight + (A * tz - tx)) * (e - B * h) -
                                 ((B * i - f) * Hight + (B * tz - ty)) * (b - A * h)) / Z);
        xyz.y = (((a - A * g) * ((B * i - f) * Hight + (B * tz - ty)) -
                                 (d - B * g) * ((A * i - c) * Hight + (A * tz - tx))) / Z);
    }
    std::cout << "xyz: " << "x" << xyz.x << " y" << xyz.y << std::endl;
    return xyz;
}

// void MatrixCoordinateSystem::SetT_matrix (cv::Mat &T_World2Main,cv::Mat &T_Lidar2world){
//     ROS_INFO("1");
//     T_Lidar2world = this->T_Lidar2world;
//     ROS_INFO("2");
//     Eigen::Matrix<double,4,4> E_T_Main2world;
//     std::cout << this->T_Main2world;
//     ROS_INFO("3");
//     cv::cv2eigen(this->T_Main2world, E_T_Main2world);
//     ROS_INFO("4");
//     Eigen::Matrix<double,4,4> E_T_World2Main = E_T_Main2world.inverse();
//     ROS_INFO("5");
//     cv::eigen2cv(E_T_World2Main,T_World2Main);
    
// }
