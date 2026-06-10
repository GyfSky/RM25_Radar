#include "../include/General.h"

int initornot(const std::array<cv::Point2f, 25> polygon,cv::Point2d point,int polygonSize) {
    // 手动闭合多边形：如果多边形的第一个点和最后一个点不同，则复制第一个点到最后一个位置
    std::array<cv::Point2f, 26> closedPolygon; // 为闭合多边形预留额外空间
    for (size_t i = 0; i < polygonSize; ++i) {
        closedPolygon[i] = polygon[i];
    }
    //手动闭合多边形
    closedPolygon[polygonSize] = polygon[0];

    int counter = 0;
    double xinters;
    cv::Point2f p1, p2;

    p1 = closedPolygon[0];
    for (size_t i = 1; i <= polygonSize; i++) {
        p2 = closedPolygon[i];
        if (point.y > std::min(p1.y, p2.y)) {
            if (point.y <= std::max(p1.y, p2.y)) {
                if (point.x <= std::max(p1.x, p2.x)) {
                    if (p1.y != p2.y) {
                        xinters = (point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                        if (p1.x == p2.x || point.x <= xinters)
                            counter++;
                    }
                }
            }
        }
        p1 = p2;
    }

    if ( counter % 2 == 1) return 1;
    else return -1;
}

/**
 * @brief 令点（x,y）与零点的连线为line1，则 angle = line1 与 x轴的夹角
 *
 * @return temp_angle 180度角度制，非弧度制
 * **/
double getAngle180(double x, double y){
    double temp_angle;
    if(abs(x) < 1e-2 || abs(y) < 1e-2){
        temp_angle = 0.0;
    }else{
        temp_angle = abs(atan(y/x)) / M_PI * 180.;//换成角度制
    }

    if( (x<1e-2 && y>=1e-2)||(x>-1e-2 && y<=-1e-2)  ){//由于上面将 atan 取绝对值，此处利用pi - theta 取反，得到原始角度
        temp_angle = 180.0 - temp_angle ;
    }
    return temp_angle;
}

/**
 * p1(p1_x, p1_y),    p2(p2_x, p2_y)     两点一线//main
 * p1_(p1_x_, p1_y_)  p2_(p2_x_, p2_y_)  两点一线//sec
 *
 * @note 该距离并不完全正确， 只是一个相对近似值
 *
 * **/
double line2line_distance(double p1_x, double p1_y,double p2_x,double p2_y,double len,
                          double p1_x_,double p1_y_,double p2_x_,double p2_y_,double len_ ){
    double x= p1_x-p2_x, x_=p1_x_-p2_x_, y=p1_y-p2_y,  y_=p1_y_-p2_y_;
    double angle1 = getAngle180(x,y);
    double angle2 = getAngle180(x_,y_);

    if(abs(angle1-angle2)>15.0&&abs(angle1-angle2)<165.0){
        return -1;//模拟两条直线不是近似平行的时候
    }

    //四个点构成的面积 * 2
    double S;
    Eigen::Vector3d p(p1_x-p2_x, p1_y-p2_y, 0);
    Eigen::Vector3d p_(p1_x_-p2_x_, p1_y_-p2_y_, 0);
    Eigen::Vector3d a(p1_x-p1_x_, p1_y-p1_y_, 0);//main和sec第一个点的连线构成的向量
    Eigen::Vector3d b(p2_x-p2_x_, p2_y-p2_y_, 0);//main和sec第二个点的连线构成的向量
    S = (p.cross(a)).norm() + (p_.cross(b)).norm();//向量叉乘

    double distance;
    distance = S/(len_+len);
    return distance;
}