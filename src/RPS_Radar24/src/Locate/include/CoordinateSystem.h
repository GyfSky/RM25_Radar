#ifndef RADAR2023_WITHTRT_COORDINATESYSTEM_H
#define RADAR2023_WITHTRT_COORDINATESYSTEM_H
//#include "Place.h"

#include "MapAOV.h"

class MatrixCoordinateSystem{
private:
    float edge = 0.7; //0.7 约等于 0.8 * 根号3
    float venue_w = 28.0;  // m
    float venue_h = 15.0;  // m
    int classWithoutCar;
public:
//    MatrixCoordinateSystem(std::vector<std::string> Names);
    MatrixCoordinateSystem(int classWithoutCar) {
        this->classWithoutCar=classWithoutCar;
    }
//    cv::Mat GetCVMAT_matrix(std::string Name);
//    void GetCVMAT_matrix();
    void get_Point_2d(cv::Mat T_2self ,cv::Mat K ,cv::Point3f &point3f, cv::Point2f &point2f);
    void Get2world_matrix(cv::Mat &T,cv::Mat &K,std::vector<cv::Point2d> &pts_pnp_2d,std::vector<cv::Point3d> &pts_pnp_3d,cv::Mat dist=cv::Mat());
    void solve_reality_3d(const cv::Mat T_, const double fx, const double fy, const double cx, const double cy,
                          std::vector<MapVertex> &vexs ,std::vector<std::vector<MapEdge>> arcs,std::vector<STrack> &tracks, OurPattern ourPattern,std::vector<bool> &isWarring);
    void solve_reality_3d(const cv::Mat T_, const double fx, const double fy, const double cx, const double cy,
                          std::vector<MapVertex> &vexs ,std::vector<std::vector<MapEdge>> arcs,std::vector<Car> &cars, OurPattern ourPattern);
    void solve_reality_3d(const cv::Mat T_, const double fx, const double fy, const double cx,const double cy, std::vector<STrack> &tracks);
    cv::Point3d solve_reality_3d(const cv::Mat T_, const double fx, const double fy, const double cx, const double cy,
                          std::vector<MapVertex> &vexs ,std::vector<std::vector<MapEdge>> arcs,cv::Rect &rect , OurPattern ourPattern);
    void set_warring_and_place_by_locate3D(const MapVertex& vex, OurPattern ourPattern, std::vector<bool> &isWarring, STrack &track);
    bool coordinateCorrection(cv::Point3d &Locate3D, OurPattern ourPattern);
    bool Correction(cv::Point3d &Locate3D, OurPattern ourPattern);
    double getH(MapVertex vex, cv::Point2d Locate2D);

    //    void solve_reality_3d(const cv::Mat T, const double fx, const double fy, const double cx, const double cy, Place &place, Car &car);
    // void SetT_matrix(cv::Mat &T_World2Main,cv::Mat &T_Lidar2world);

};

//cv::Point3d solve_reality_3d(const cv::Mat T_, const double fx, const double fy, const double cx, const double cy, Place &place, Car &car);
//extern template void MatrixCoordinateSystem::solve_reality_3d<STrack>(const cv::Mat T_, const double fx, const double fy, const double cx, const double cy,
//                                              std::vector<MapVertex> &vexs ,std::vector<STrack> &objs);

#endif //RADAR2023_WITHTRT_COORDINATESYSTEM_H
