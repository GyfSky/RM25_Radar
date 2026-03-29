//
// Created by plusseven on 24-4-8.
//

#include "../include/SensorParam.h"

SensorParam::SensorParam(std::string Name,CamPosition camPosition ,OurPattern ourPattern,std::string config_path) {
    std::cout << "SensorParam 1.0" << std::endl;

    cv::FileStorage cvMatMatrixFile = cv::FileStorage(config_path, cv::FileStorage::READ);
    cvMatMatrixFile[Name]["K"] >> K;
    fx = K.at<float>(0,0);
    fy = K.at<float>(1,1);
    cx = K.at<float>(0,2);
    cy = K.at<float>(1,2);
    cvMatMatrixFile[Name]["picture_size"]["input_w"] >> img_w;
    cvMatMatrixFile[Name]["picture_size"]["input_h"] >> img_h;

    std::string camPos, ourColor;
    if(camPosition == CamPosition::left)            camPos = "left";
    else if(camPosition == CamPosition::right)      camPos = "right";
    else                                        std::cout << "have error in CamPosition" << std::endl;

    if(ourPattern == OurPattern::red)               ourColor = "red";
    else if(ourPattern == OurPattern::blue)         ourColor = "blue";
    else                                        std::cout << "have error in OurPattern" << std::endl;

    int point_num = cvMatMatrixFile["pnp"][camPos]["point_num"];
    for(int i = 0;i<point_num;i++){
        cv::Point3d point =
                cv::Point3d(cvMatMatrixFile["pnp"][camPos][ourColor]["pts_pnp_2d"][i][0],
                            cvMatMatrixFile["pnp"][camPos][ourColor]["pts_pnp_2d"][i][1],
                            cvMatMatrixFile["pnp"][camPos][ourColor]["pts_pnp_2d"][i][2]);
        this->pts_pnp_3d.push_back(point);
    }
    cvMatMatrixFile.release();

    std::cout << Name << "\n" << " of pts_pnp_3d is ok" << std::endl;
}

void SensorParam::setworld2self_config(cv::Mat T_main2World){
     cv::Mat world2self = T_2world.inv();
     a = world2self.at<float>(0, 0);
     b = world2self.at<float>(0, 1);
     c = world2self.at<float>(0, 2);

     d = world2self.at<float>(1, 0);
     e = world2self.at<float>(1, 1);
     f = world2self.at<float>(1, 2);

     g = world2self.at<float>(2, 0);
     h = world2self.at<float>(2, 1);
     i = world2self.at<float>(2, 2);

     tx = world2self.at<float>(0, 3);
     ty = world2self.at<float>(1, 3);
     tz = world2self.at<float>(2, 3);

     cv::cv2eigen(T_main2World.inv(), Rt);
//     std::cout << "Rt: " << Rt << std::endl;

     is_setworld2self_config = true;
}


int SensorParam::change2main(float &x, float &y,
         const double main_fx, const double main_fy, const double main_cx, const double main_cy){

    double A, B, X, Y, Z;
    double Hight = 0.0; //TODO
    A = (x - cx) / fx;
    B = (y - cy) / fy;

    Z = (a - A * g) * (e - B * h) - (d - B * g) * (b - A * h);
    if (Z == 0) {
        std::cout << "have one error in change2mainCam" << std::endl;
        return -1;
    }

    X = ((((A * i - c) * Hight + (A * tz - tx)) * (e - B * h) - ((B * i - f) * Hight + (B * tz - ty)) * (b - A * h)) / Z);
    Y = (((a - A * g) * ((B * i - f) * Hight + (B * tz - ty)) - (d - B * g) * ((A * i - c) * Hight + (A * tz - tx))) / Z);
    std::cout << "XY: " << X << ", " << Y << std::endl;
    Eigen::Vector4d temp_reality_3d(X,Y,Hight, 1);
    Eigen::Vector4d pc = Rt * temp_reality_3d;
    std::cout << "pc: " << pc<< std::endl;
    x = main_fx * pc[0] / pc[2] + main_cx;
    y = main_fy * pc[1] / pc[2] + main_cy;
    return 1;
}