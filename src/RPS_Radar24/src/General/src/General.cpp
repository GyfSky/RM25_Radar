#include "../include/General.h"


/**
 * @brief 判断点是否在给定凸多边形内(n<=10)
 *
 * @return -1 -> 该点 在   凸多边形内
 * @return  1 -> 该点 不在  凸多边形内
 */
int initornot(std::array<cv::Point2f,25> predict2d ,cv::Point xy, int pointNum){
    Eigen::Vector3d p1,p2;
    Eigen::Vector3d x1,x2,temp_x2;

    for(int i=0;i<pointNum;i++){
        if(i==0){
            p1 << (predict2d[pointNum-1].x-xy.x),(predict2d[pointNum-1].y-xy.y),0;
            p2 << (predict2d[i].x-xy.x),(predict2d[i].y-xy.y),0;
            x2 = p1.cross(p2);
            temp_x2 = x2;
            continue;
        }
        else{
            p1 = p2;
            x1 = x2;
            p2 << (predict2d[i].x-xy.x),(predict2d[i].y-xy.y),0;
            x2 = p1.cross(p2);
        }

        if(x2.z()*x1.z()<-1e-6)
            return -1;
    }

    x1 = x2;
    x2 = temp_x2;
    if(x2.z()*x1.z()<-1e-6){
        return -1;
    }
    else{
        return 1;
    }
}

/**
 * @brief 判断点是否在给定凸多边形内(n<=10)(3d模式)， 命令H
 *
 * @return -1 -> 该点 在   凸多边形内
 * @return  1 -> 该点 不在  凸多边形内
 */
int initornot3D(std::array<Eigen::Matrix<double, 3, 1>,25> predict3d ,cv::Point3d &xyz, int pointNum){
    Eigen::Vector3d p1,p2;
    Eigen::Vector3d x1,x2,temp_x2;

    for(int i=0;i<pointNum;i++){
        if(i==0){
            p1 << (predict3d[pointNum-1].x()-xyz.x),(predict3d[pointNum-1].y()-xyz.y),0;
            p2 << (predict3d[i].x()-xyz.x),(predict3d[i].y()-xyz.y),0;
            x2 = p1.cross(p2);
            temp_x2 = x2;
            continue;
        }
        else{
            p1 = p2;
            x1 = x2;
            p2 << (predict3d[i].x()-xyz.x),(predict3d[i].y()-xyz.y),0;
            x2 = p1.cross(p2);
        }

        if(x2.z()*x1.z()<0)
            return -1;
    }

    x1 = x2;
    x2 = temp_x2;
    if(x2.z()*x1.z()<0){
        return -1;
    }
    else{
        xyz.z = predict3d[0].z(); // new
        return 1;
    }

}

/**
 * @brief 令点（x,y）与零点的连线为line1，则 angle = line1 与 x轴的夹角
 *
 * @return temp_angle 360度角度制，非弧度制
 * **/
double getAngle360(double x, double y){
    double temp_angle;
    if(abs(x) < 1e-2 || abs(y) < 1e-2){
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

    return temp_angle;
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
        return -1;//此处为什么返回 模拟两条直线不是近似平行的时候
    }

    //四个点构成的面积 * 2
    double S;

    Eigen::Vector3d p(p1_x-p2_x, p1_y-p2_y, 0);
    Eigen::Vector3d p_(p1_x_-p2_x_, p1_y_-p2_y_, 0);
    Eigen::Vector3d a(p1_x-p1_x_, p1_y-p1_y_, 0);//main和sec第一个点的连线构成的向量
    Eigen::Vector3d b(p2_x-p2_x_, p2_y-p2_y_, 0);//main和sec第二个点的连线构成的向量

    S = (p.cross(a)).norm() + (p_.cross(b)).norm();//向量叉乘
//    std::cout << "S: " << S << std::endl;

    double distance;
    distance = S/(len_+len);
//    std::cout << "distance: " << distance << std::endl;
    return distance;
}


//void eigenMat2VecVec(Eigen::MatrixXd &eigen,std::vector<std::vector<double>> &vecVec){
//    int col = eigen.cols();
//    int raw = eigen.rows();
//    Eigen::RowVectorXd vec_d(col);
//    for(int i=0;i<raw;i++){
//        vec_d = eigen.block(i,0,1,col);
//        std::vector<double> vec(vec_d.data(), vec_d.data() + vec_d.size());
//        vecVec.push_back(vec);
//    }
//}
//
//void eigenMat2VecVec(Eigen::MatrixXd &eigen,std::vector<std::vector<float>> &vecVec){
//    int col = eigen.cols();
//    int raw = eigen.rows();
//    Eigen::RowVectorXd vec_d(col);
//    for(int i=0;i<raw;i++){
//        vec_d = eigen.block(i,0,1,col);
//        std::vector<float> vec(vec_d.data(), vec_d.data() + vec_d.size());
//        vecVec.push_back(vec);
//    }
//}

