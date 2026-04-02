#include "../include/SensorParam.h"

SensorParam::SensorParam(std::string Name,OurPattern ourPattern,std::string config_path) {
    std::cout << "SensorParam 1.0" << std::endl;

    cv::FileStorage cvMatMatrixFile = cv::FileStorage(config_path, cv::FileStorage::READ);
    cvMatMatrixFile[Name]["K"] >> K;
    fx = K.at<float>(0,0);
    fy = K.at<float>(1,1);
    cx = K.at<float>(0,2);
    cy = K.at<float>(1,2);
    cvMatMatrixFile[Name]["picture_size"]["input_w"] >> img_w;
    cvMatMatrixFile[Name]["picture_size"]["input_h"] >> img_h;

    std::string ourColor;

    if(ourPattern == OurPattern::red)               ourColor = "red";
    else if(ourPattern == OurPattern::blue)         ourColor = "blue";
    else                                        std::cout << "have error in OurPattern" << std::endl;

    int point_num = cvMatMatrixFile[Name]["point_num"];
    for(int i = 0;i<point_num;i++){
        cv::Point3d point =
                cv::Point3d(cvMatMatrixFile[Name][ourColor]["pts_pnp_2d"][i][0],
                            cvMatMatrixFile[Name][ourColor]["pts_pnp_2d"][i][1],
                            cvMatMatrixFile[Name][ourColor]["pts_pnp_2d"][i][2]);
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
     is_setworld2self_config = true;
}