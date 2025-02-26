//
// Created by plusseven on 24-7-24.
//

#ifndef COORDSOLVER_COORDSOLVER_H
#define COORDSOLVER_COORDSOLVER_H

#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

//#include <Eigen/Core>
//#include <Eigen/Dense>
//#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
//#include <yaml-cpp/yaml.h>

#include "../../General/include/General.h"

using namespace std;

//struct PnPInfo
//{
//    Eigen::Vector3d armor_cam;
//    Eigen::Vector3d armor_world;
//    Eigen::Vector3d R_cam;
//    Eigen::Vector3d R_world;
//    Eigen::Vector3d euler;
//    Eigen::Matrix3d rmat;
//};


class CoordSolver {
public:
    CoordSolver(OurPattern ourPattern);
    ~CoordSolver();

//    bool loadParam(string coord_path,string param_name);
//
//
//    PnPInfo pnp(const std::vector<Point2f> &points_pic, const Eigen::Matrix3d &rmat_imu, enum TargetType type, int method);
//
//    Eigen::Vector3d camToWorld(const Eigen::Vector3d &point_camera,const Eigen::Matrix3d &rmat);
//    Eigen::Vector3d worldToCam(const Eigen::Vector3d &point_world,const Eigen::Matrix3d &rmat);
//
//    Eigen::Vector3d staticCoordOffset(Eigen::Vector3d &xyz);
//    Eigen::Vector2d staticAngleOffset(Eigen::Vector2d &angle);
//    Eigen::Vector2d getAngle(Eigen::Vector3d &xyz_cam, Eigen::Matrix3d &rmat);
//    cv::Point2f reproject(Eigen::Vector3d &xyz);
//    cv::Point2f getHeading(Eigen::Vector3d &xyz_cam);

    double dynamicCalcPitchOffset(double x, double y, double z);
    Eigen::Vector2d calcYawPitch(Eigen::Vector3d &xyz);

    inline double calcYaw(Eigen::Vector3d &xyz);
    inline double calcPitch(Eigen::Vector3d &xyz);
    bool setBulletSpeed(double speed);


private:
    int max_iter = 10;
    float stop_error = 0.001;
    int R_K_iter = 60;
    Eigen::Vector3d base_xyz;

//    YAML::Node param_node;

//    double bullet_speed = 28;
    double bullet_speed = 16;            //TODO:弹速可变
//    const double k = 0.01903;                //25°C,1atm,小弹丸
    // const double k = 0.000556;                //25°C,1atm,大弹丸
    const double k = 0.000530;                //25°C,1atm,发光大弹丸
    const double g = 9.781;
};


#endif //COORDSOLVER_COORDSOLVER_H
