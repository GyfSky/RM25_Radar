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
    if((vex.placeType==stepsWarring || vex.placeType==steps) && (((vex.placeColor == PlaceColor::B)&& (ourPattern==blue)) || ((vex.placeColor == PlaceColor::R) && (ourPattern==red) ))){
        //步兵
        if(((ourPattern==red) && (track.cls < classWithoutCar/2-1 && track.cls > 1)) || ((ourPattern==blue) &&  (track.cls < classWithoutCar-1 && track.cls > classWithoutCar/2+1))){
            isWarring[1] = true;
            track.placeType = stepsWarring;
        }
        else if(((ourPattern==red) && (track.cls < classWithoutCar/2 || track.cls > classWithoutCar-1)) || ((ourPattern==blue) &&  (track.cls < 0 || track.cls > classWithoutCar/2-1))){
            isWarring[2] = true;
            track.placeType = stepsWarring;
        }
    }
    if(vex.placeType== windmill && ( ((vex.placeColor == PlaceColor::R) &&(ourPattern==blue)) || ((vex.placeColor == PlaceColor::B) &&(ourPattern==red))) ){
        isWarring[3] = true;
        track.placeType = windmill;
    }

    if(vex.placeType==startupArea && ( ((vex.placeColor == PlaceColor::R) &&(ourPattern==blue)) || ((vex.placeColor == PlaceColor::B) &&(ourPattern==red))) ){
        if((ourPattern==red) && (track.cls < classWithoutCar/2 || track.cls > classWithoutCar-1) ){
            track.placeType = startupArea;
        }else if((ourPattern==blue) &&  (track.cls < 0 || track.cls > classWithoutCar/2-1)){
            track.placeType = startupArea;
        }
    }
}

bool MatrixCoordinateSystem::coordinateCorrection(cv::Point3d &Locate3D, OurPattern ourPattern){
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
        if (Locate3D.x < 0)
            return false;
    }else if(ourPattern == blue){
        if(Locate3D.x < 1.0)
            Locate3D.x = 1.0;

        if(Locate3D.y < 3.95){
            if(Locate3D.x < 1.5)
                Locate3D.x = 1.5;
            else if(Locate3D.x < 3.3)
                Locate3D.x = 4.0;
        }
        if (Locate3D.x > 28)
            return false;
    }
    return true;
}

bool MatrixCoordinateSystem::Correction(cv::Point3d &Locate3D, OurPattern ourPattern){
    if(ourPattern == red){
        if (Locate3D.x < 0.2)
            return false;
    }
    else if(ourPattern == blue){
        if (Locate3D.x > 27.8)
            return false;
    }
    return true;
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
    int index=0;
    std::vector<int> remove_lists;
    //遍历每个轨迹
    for (auto & track : tracks) {
        double Hight = 0;
        int in_where = -1;
        double A, B, Z;

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
        track.oldH = Hight;

        A = (track.Locate2D.x - cx) / fx;//A=Xc/Zc
        B = (track.Locate2D.y - cy) / fy;//A=Yc/Zc

        Z = (a - A * g) * (e - B * h) - (d - B * g) * (b - A * h);
        if (Z < 1e-6 && Z > -1e-6) {
            std::cout << "have one error in getLocate3D" << std::endl;
        } else {
            track.Locate3D.x = ((((A * i - c) * Hight + (A * tz - tx)) * (e - B * h) -
                               ((B * i - f) * Hight + (B * tz - ty)) * (b - A * h)) / Z);
            track.Locate3D.y = (((a - A * g) * ((B * i - f) * Hight + (B * tz - ty)) -
                               (d - B * g) * ((A * i - c) * Hight + (A * tz - tx))) / Z);

            bool flag=coordinateCorrection(track.Locate3D, ourPattern);
            if (!flag) remove_lists.push_back(index);
            track._Locate3D = track.Locate3D;
        }
        if(track.tracklet_len>1e-6)
            track.push_front_change_Locate3D_and_distance((track.Locate3D-track.old_Locate3D));
        track.old_Locate3D = track.Locate3D;
        index++;
    }
    sort(remove_lists.begin(),remove_lists.end(),std::greater<>());
    for(auto i:remove_lists)
        tracks.erase(tracks.begin()+i);
}