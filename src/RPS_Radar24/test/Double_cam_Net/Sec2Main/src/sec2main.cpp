//
// Created by plusseven on 24-1-31.
//

#include "../include/sec2main.h"
Sec2main::Sec2main(const cv::Mat T_World2Sec, const double fx, const double fy, const double cx,const double cy) {
    a = T_World2Sec.at<double>(0, 0);
    b = T_World2Sec.at<double>(0, 1);
    c = T_World2Sec.at<double>(0, 2);

    d = T_World2Sec.at<double>(1, 0);
    e = T_World2Sec.at<double>(1, 1);
    f = T_World2Sec.at<double>(1, 2);

    g = T_World2Sec.at<double>(2, 0);
    h = T_World2Sec.at<double>(2, 1);
    i = T_World2Sec.at<double>(2, 2);

    tx = T_World2Sec.at<double>(0, 3);
    ty = T_World2Sec.at<double>(1, 3);
    tz = T_World2Sec.at<double>(2, 3);

    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;

}

Sec2main::Sec2main(const cv::Mat T_World2Sec, const double fx, const double fy, const double cx,const double cy,
                   const cv::Mat T_Main2World, const double main_fx, const double main_fy, const double main_cx, const double main_cy ) {
    a = T_World2Sec.at<double>(0, 0);
    b = T_World2Sec.at<double>(0, 1);
    c = T_World2Sec.at<double>(0, 2);

    d = T_World2Sec.at<double>(1, 0);
    e = T_World2Sec.at<double>(1, 1);
    f = T_World2Sec.at<double>(1, 2);

    g = T_World2Sec.at<double>(2, 0);
    h = T_World2Sec.at<double>(2, 1);
    i = T_World2Sec.at<double>(2, 2);

    tx = T_World2Sec.at<double>(0, 3);
    ty = T_World2Sec.at<double>(1, 3);
    tz = T_World2Sec.at<double>(2, 3);

    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;

    cv::cv2eigen(T_Main2World.inv(),Rt);

    this->main_fx = main_fx;
    this->main_fy = main_fy;
    this->main_cx = main_cx;
    this->main_cy = main_cy;
}
Sec2main::Sec2main(const cv::Mat T_Sec2Main,const cv::Mat K_M,const cv::Mat K_S) {
    a = T_Sec2Main.at<double>(0, 0);
    b = T_Sec2Main.at<double>(0, 1);
    c = T_Sec2Main.at<double>(0, 2);

    d = T_Sec2Main.at<double>(1, 0);
    e = T_Sec2Main.at<double>(1, 1);
    f = T_Sec2Main.at<double>(1, 2);

    g = T_Sec2Main.at<double>(2, 0);
    h = T_Sec2Main.at<double>(2, 1);
    i = T_Sec2Main.at<double>(2, 2);

    tx = T_Sec2Main.at<double>(0, 3);
    ty = T_Sec2Main.at<double>(1, 3);
    tz = T_Sec2Main.at<double>(2, 3);

    cv::cv2eigen(T_Sec2Main.inv(),Rt);

    Eigen::Matrix<double,3,3> K;
    cv::cv2eigen(K_M,K);
    this->K_M.block(0,0,3,3) = K;
//    this->K_M(3,3) = 1;

    cv::cv2eigen(K_S,K);
    this->K_S.block(0,0,3,3) = K;
//    this->K_S(3,3) = 1;

    this->R = Rt.block(0,0,3,3);
    this->t = Rt.block(0,3,3,1);

    std::cout << R << t << std::endl;

    this->fx = this->K_S(0,0);
    this->fy = this->K_S(1,1);
    this->cx = this->K_S(0,2);
    this->cy = this->K_S(1,2);


    this->main_fx = this->K_M(0,0);
    this->main_fy = this->K_M(1,1);
    this->main_cx = this->K_M(0,2);
    this->main_cy = this->K_M(1,2);
}

int Sec2main::change2mainCam(float &x, float &y ) {
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
//    std::cout << "XY: " << X << ", " << Y << std::endl;
    Eigen::Vector4d temp_reality_3d(X,Y,0, 1);
    Eigen::Vector4d pc = Rt * temp_reality_3d;
    x = main_fx * pc[0] / pc[2] + main_cx;
    y = main_fy * pc[1] / pc[2] + main_cy;
    return 1;

}

int Sec2main::sec2mainCam(float &x, float &y) {
    double p;
    float xx = x, yy = y;
//    p = -(1.0*(a*e*fx*fy*i - 1.0*a*f*fx*fy*h - 1.0*b*d*fx*fy*i + b*f*fx*fy*g + c*d*fx*fy*h - 1.0*c*e*fx*fy*g + a*e*fx*fy*tz - 1.0*b*d*fx*fy*tz - 1.0*a*fx*fy*h*ty + b*fx*fy*g*ty + d*fx*fy*h*tx - 1.0*e*fx*fy*g*tx))/(a*fx*h*y - 1.0*b*fx*g*y - 1.0*d*fy*h*x + e*fy*g*x - 1.0*a*cy*fx*h - 1.0*a*e*fx*fy + b*cy*fx*g + b*d*fx*fy + cx*d*fy*h - 1.0*cx*e*fy*g);
//    p = (1.0*fx*fy)/(fx*fy*i - 1.0*cy*fx*h - 1.0*cx*fy*g + fy*g*x + fx*h*y);
//    Eigen::Vector3d temp_reality_3d(x,y,1);
//    auto out =  p * (this->K_M * this->R  * this->K_S.inverse() * temp_reality_3d);
//    x = out(0,0);
//    y = out(1,0);

    x = (1.0*(a*main_fx*fy*xx + b*fx*main_fx*yy + main_cx*fy*g*xx + main_cx*fx*h*yy - 1.0*a*cx*main_fx*fy - 1.0*b*cy*fx*main_fx - 1.0*cx*main_cx*fy*g - 1.0*main_cx*cy*fx*h + c*fx*main_fx*fy + main_cx*fx*fy*i))/(fx*fy*i - 1.0*cy*fx*h - 1.0*cx*fy*g + fy*g*xx + fx*h*yy);

    y = (1.0*(main_cy*fy*g*xx + d*fy*main_fy*xx + main_cy*fx*h*yy + e*fx*main_fy*yy - 1.0*cx*main_cy*fy*g - 1.0*cx*d*fy*main_fy - 1.0*cy*main_cy*fx*h - 1.0*cy*e*fx*main_fy + main_cy*fx*fy*i + f*fx*fy*main_fy))/(fx*fy*i - 1.0*cy*fx*h - 1.0*cx*fy*g + fy*g*xx + fx*h*yy);

    return 1;
}

