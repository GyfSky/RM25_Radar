//
// Created by plusseven on 24-1-15.
//

#ifndef JSONCPP_TEST_MAPAOV_H
#define JSONCPP_TEST_MAPAOV_H

#include "../../ByteTrack/include/STrack.h"

//#include "Place.h"

enum PlaceColor { R, B };//己方？敌方？
enum SeeType { Clear, Rmist, Rhide, Bmist, Bhide, mist, hide};  //R,B 为我方颜色
enum MissType{ Line, Drop, WaitDrops, Inelse, lineDrops};//？

struct MapEdge{
public:
    double p;
    double diff_distance;
    double x1;
    double y1;
    double x2;
    double y2;
    MapEdge();
    MapEdge(double p);
    MapEdge(double x1,double y1,double x2,double y2);
};



#define maxnum 99  // 最大可存储99个点

class MapVertex {
public:
    int vertex;//顶点编号
    bool isH;
    SeeType seeType;
    PlaceType_special placeType;//特殊位置？？
    PlaceColor placeColor;
    MissType missType;
    double distance;//？？
    Eigen::MatrixXd cost_matrix;//？？

    std::vector<cv::Point3d> missPoints;//
    double missAngle;//
    int missLinesTimes = -1;//

    int point_3d_number;//3d点个数
    std::array<Eigen::Matrix<double, 3, 1>,10> points_reality_3d;//场地中的3d点
    std::array<cv::Point2f,10> points_predict_2d;//场地在图像中的2d点，数量等于3d点
    // 用来粗率定位的参数
    std::array<double,3> getH_abc_3d;//
    Eigen::Matrix<double,2,2> matrix_change2;//
    Eigen::Matrix<double,2,2> matrix_change3;//
    Eigen::Matrix<double,2,1> matrix_2d;//

public:
    MapVertex(std::array<Eigen::Matrix<double, 3, 1>,10> points_reality_3d,int point_3d_number,
              PlaceColor placeColor,SeeType seeType = Clear,PlaceType_special placeType = ordinary, bool isH = false);
    void get_predict_2d(const Eigen::Matrix<double,4,4> Rt, const double fx, const double fy, const double cx, const double cy);
    void get_roughH_config();
    void setMissType(double point_x, double point_y, double point_z = 0.0);
    void setMissType(double angel);
    void setMissType(double angel, std::vector<cv::Point3d> wait_rects);
    void setMissType(double angel, std::vector<cv::Point3d> wait_rects, int missLinesTimes,double point_x, double point_y, double point_z = 0.0);
    void setMissType();
    void get_diff_distance_and_p();
};


class MapGraphMtx{
public:
    std::vector<MapVertex> vexs; //顶点表
    std::vector<std::vector<MapEdge>> arcs; // 临接表 应该是邻接矩阵，大小为n*n
    int vexnum;  //当前点数
    int arcnum;  // 当前边数；

//    std::vector<cv::Point3d> pts_pnp_3d;
//    std::vector<cv::Point2f> pts_predict_2d_point;
    ////from mode
    OurPattern ourPatternColor;
    MapGraphMtx(OurPattern ourPatternColor);

    void get_completeMapGraphMtx();
    void print_AdjacencyMatrix();
    void push_back_MapVertex(std::vector<MapVertex> &vertexs, std::array<Eigen::Matrix<double, 3, 1>,10> points_reality_3d,int point_3d_number,
                             PlaceColor placeColor,SeeType seeType = Clear, PlaceType_special placeType = ordinary, bool isH = false);
    void creat_dir(int mainVex, int secVex, double x1,double y1,double x2,double y2,int offset = 0, bool isUndirected = true);
    void creat_dir(std::vector<MapVertex> vexs, int mainVex, int secVex, int offset = 0, bool isUndirected = true);
    void get_predict_2d(const cv::Mat T, const double fx, const double fy, const double cx, const double cy);
    void get_roughH_config();

private:
    void get_lib_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor,int offset=0);
    void get_lb_R_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);
    void get_rt_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);
    void get_lt_R_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);
    void get_rb_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);
    void get_mid_R_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);
    void get_mid_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);
    void get_hole_R_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);
    void get_hole_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);

    void get_startupArea_R_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);
    void get_startupArea_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);

    void get_midGround_R_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);
    void get_midGround_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);
    void get_behindGround_R_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);
    void get_behindGround_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset = 0);


};

#endif //JSONCPP_TEST_MAPAOV_H
