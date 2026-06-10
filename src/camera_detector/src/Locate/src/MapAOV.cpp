#include "../include/MapAOV.h"

MapEdge::MapEdge() {
    this->p = 0;
}

MapEdge::MapEdge(double p) {
    this->p = p;
}

MapEdge::MapEdge(double x1,double y1,double x2,double y2){
    this->p = 1;
    this->x1 = x1;
    this->y1 = y1;
    this->x2 = x2;
    this->y2 = y2;
}

MapVertex::MapVertex(std::array<Eigen::Matrix<double, 3, 1>, 25> points_reality_3d,int point_3d_number,
                     PlaceColor placeColor , SeeType seeType, PlaceType_special placeType, bool isH) {
    this->points_reality_3d = points_reality_3d;
    this->point_3d_number = point_3d_number;
    this->placeColor = placeColor;
    this->seeType = seeType;
    this->placeType = placeType;
    this->isH = isH;
}


/**
 * @brief 获取单个(single)区域3d坐标投影到相机坐标系的数据
 * **/
void MapVertex::get_predict_2d(const Eigen::Matrix<double,4,4> Rt, const double fx, const double fy, const double cx, const double cy) {
    for(int j=0; j<point_3d_number; j++){
        Eigen::Vector4d temp_reality_3d(points_reality_3d[j][0],points_reality_3d[j][1],points_reality_3d[j][2], 1);
        Eigen::Vector4d pc = Rt * temp_reality_3d;
        Eigen::Vector2d predict(fx * pc[0] / pc[2] + cx,fy * pc[1] / pc[2] + cy );
        points_predict_2d.at(j) = cv::Point2d((int)predict[0],(int)predict[1]);
    }
}

/**
 * @brief 获取所有(all)区域3d坐标投影到相机坐标系的数据
 * **/
void MapGraphMtx::get_predict_2d(const cv::Mat T, const double fx, const double fy, const double cx, const double cy) {
    Eigen::Matrix<double,4,4> Rt;
    cv::cv2eigen(T.inv(),Rt);
    for(int i=0; i<vexs.size(); i++){
        vexs[i].get_predict_2d(Rt, fx, fy, cx, cy);
    }
}

/**
 * @brief 获取单个(single)区域的投影（转换）矩阵s
 * **/
void MapVertex::get_roughH_config() {
    //算出3d平面的表达 //TODO：4个点的顺序必须为 h1,h1,h2,h2
    Eigen::Vector3d p1_3d,p2_3d,p3_3d;
    p1_3d << points_reality_3d.at(0);
    p2_3d << points_reality_3d.at(1);
    p3_3d << points_reality_3d.at(2);
    auto n = (p3_3d-p1_3d).cross(p2_3d-p1_3d);//得到平面的法向量

    if(n[2]==0){//区域水平
        matrix_change2 << 1,0,0,1;
        matrix_change3 << 1,0,0,1;

        getH_abc_3d.at(0) = 0 ;
        getH_abc_3d.at(1) = 0 ;
        getH_abc_3d.at(2) = double (p3_3d[2]);
        return;
    }else{
        getH_abc_3d.at(0) = -(double (n[0]))/(double (n[2]));
        getH_abc_3d.at(1) = -(double (n[1]))/(double (n[2]));
        getH_abc_3d.at(2) = double (p3_3d[2]) - getH_abc_3d.at(0)*p3_3d[0] - getH_abc_3d.at(1)*p3_3d[1];

        //2d点的投射
        matrix_change3 << (points_reality_3d.at(0)[0]- points_reality_3d.at(1)[0]) ,( points_reality_3d.at(2)[0]- points_reality_3d.at(1)[0]),
                (points_reality_3d.at(0)[1]- points_reality_3d.at(1)[1]) , ( points_reality_3d.at(2)[1]- points_reality_3d.at(1)[1]);

        matrix_change2 << ( points_predict_2d.at(0).x- points_predict_2d.at(1).x) ,( points_predict_2d.at(2).x- points_predict_2d.at(1).x),
                ( points_predict_2d.at(0).y- points_predict_2d.at(1).y) ,( points_predict_2d.at(2).y- points_predict_2d.at(1).y);

        matrix_2d <<  points_reality_3d.at(1)[0], points_reality_3d.at(1)[1];
    }
}

/**
 * @brief 获取所有(all)区域的投影（转换）矩阵ss
 * **/
void MapGraphMtx::get_roughH_config() {
    for (int i = 0; i < vexs.size(); i++) {
        vexs[i].get_roughH_config();
    }
}

/**
 * 场地围挡在红方补给
 * 站附近的交点为坐标原点，沿场地长边向蓝方为 X 轴正方向，沿场地短边向红方停机坪为 Y 轴正方向
 * 左上，顺时针
 * **/
MapGraphMtx::MapGraphMtx(OurPattern ourPatternColor) {
    this->ourPatternColor = ourPatternColor;
    this->vexnum = 100;
    std::vector<std::vector<MapEdge>> temp_arcs(vexnum, std::vector<MapEdge>(vexnum));
    swap(this->arcs,temp_arcs);
    this->get_completeMapGraphMtx();
}

inline void MapGraphMtx::push_back_MapVertex(std::vector<MapVertex> &vertexs, std::array<Eigen::Matrix<double, 3, 1>, 25> points_reality_3d, int point_3d_number,
                                             PlaceColor placeColor, SeeType seeType ,PlaceType_special placeType, bool isH) {
    MapVertex temp_mapVertex(points_reality_3d, point_3d_number,placeColor, seeType, placeType, isH);
    temp_mapVertex.vertex = vexs.size() + 1;
    vertexs.push_back(temp_mapVertex);
}

/**
 *
 * @parma isUndirected   默认设置为无向图
 * 该函数为自动版添加数据
 *
 * @note 该距离并不完全正确， 只是一个相对近似值
 * **/
void MapGraphMtx::creat_dir(std::vector<MapVertex> vexs, int mainVex, int secVex,int offset, bool isUndirected){
    MapEdge init_mapEdge(0);
    mainVex = mainVex + offset;
    secVex  = secVex + offset;
    (this->arcs.at(mainVex)).at(mainVex) = init_mapEdge;

    MapVertex main_vertex = vexs.at(mainVex);
    MapVertex sec_veertx = vexs.at(secVex);
    //找到距离最近的点及最近的距离
    double distance = 10;
    double x1 =0, y1=0, x2=0, y2=0;
    //遍历main_vertex所有点
    for(int i=0; i<main_vertex.point_3d_number;i++){
        double p1_x, p1_y, p2_x, p2_y;
        //p1,p2为待连接的点，当i=0时，point_3d_number-1为最后一个点，即最后一个点与第一个点构成一条边
        if(i==0){
             p1_x = main_vertex.points_reality_3d.at(main_vertex.point_3d_number-1).x();
             p1_y = main_vertex.points_reality_3d.at(main_vertex.point_3d_number-1).y();
        } else{
            p1_x = main_vertex.points_reality_3d.at(i-1).x();
            p1_y = main_vertex.points_reality_3d.at(i-1).y();
        }
        p2_x = main_vertex.points_reality_3d.at(i).x();
        p2_y = main_vertex.points_reality_3d.at(i).y();


        for(int j=0; j<sec_veertx.point_3d_number;j++){
            double p1_x_, p1_y_, p2_x_, p2_y_;
            if(j==0){
                p1_x_ = sec_veertx.points_reality_3d.at(sec_veertx.point_3d_number-1).x();
                p1_y_ = sec_veertx.points_reality_3d.at(sec_veertx.point_3d_number-1).y();
            } else{
                p1_x_ = sec_veertx.points_reality_3d.at(j-1).x();
                p1_y_ = sec_veertx.points_reality_3d.at(j-1).y();
            }
            p2_x_ = sec_veertx.points_reality_3d.at(j).x();
            p2_y_ = sec_veertx.points_reality_3d.at(j).y();

            double len, len_;
            len = get2Ddistance(p1_x, p1_y, p2_x, p2_y);//main距离
            len_ = get2Ddistance(p1_x_, p1_y_, p2_x_, p2_y_);//sec距离
            double temp_distance =  line2line_distance(p1_x, p1_y, p2_x, p2_y,len,
                                                  p1_x_, p1_y_, p2_x_, p2_y_, len_);

            if(temp_distance<-0.5){
                continue;
            }else if (temp_distance < distance){
                distance = temp_distance;
                if(len<=len_){
                    x1 = p1_x ;y1 = p1_y ;x2 = p2_x ;y2 = p2_y ;
                } else{
                    x1 = p1_x_;y1 = p1_y_;x2 = p2_x_;y2 = p2_y_;
                }
            }
        }
    }

    if(distance>9.9 || x1 < 1e-6){
        std::cout << "error in creat_dir: " << ",mainVex: " << mainVex << "  ,secVex:" << secVex << std::endl;
        return;
    }
    if(distance>0.4){
        std::cout << "mainVex: " << mainVex << "  ,secVex:" << secVex << std::endl;
        std::cout << "error in creat_dir: " << ",mainVex: " << mainVex << "  ,secVex:" << secVex << std::endl;
    }
    MapEdge temp_mapEdge(x1, y1, x2, y2);
    if(isUndirected){
        (this->arcs.at(mainVex)).at(secVex) = temp_mapEdge;
        (this->arcs.at(secVex)).at(mainVex) = temp_mapEdge;
    } else{
        (this->arcs.at(mainVex)).at(secVex) = temp_mapEdge;
    }
}

void MapGraphMtx::get_lib_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor,int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,25> temp_onegroup_points;

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(0+3,0+2.323,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(1.134+3,0+2.323,0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(1.134+3,-2.323+2.323,0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(0+3,-2.323+2.323,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide ,four, true);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(-3+3,0.16+2.323,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(0+3,0.16+2.323,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(0+3,-2.323+2.323,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(-3+3,-2.323+2.323,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide ,four, true);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(-1.25+3,0.16+2.323,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(-1.86+3,0.16+2.323,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(-1.86+3,1.92+2.323,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(-1.25+3,1.50+2.323,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide ,four, true);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(-3+3,0.16+2.323,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(-2+3,0.16+2.323,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(-2.0+3,1.54+2.323,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(-3.0+3,1.54+2.323,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide ,four, true);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(2.22, 5.127, 0.3)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(4.755,3.352,0.3)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.755,3.352,0.3)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(5.793, 4.026, 0.3)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(6.108, 4.476, 0.3)));
    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(7.07, 3.803, 0.3)));
    temp_onegroup_points.at(6) = ((Eigen::Matrix<double, 3, 1>(7.576, 3.803, 0.3)));
    temp_onegroup_points.at(7) = ((Eigen::Matrix<double, 3, 1>(7.576,6.511,0.3)));
    temp_onegroup_points.at(8) = ((Eigen::Matrix<double, 3, 1>(5.335,8.079,0.3)));
    temp_onegroup_points.at(9) = ((Eigen::Matrix<double, 3, 1>(3.376,8.051,0.3)));
    temp_onegroup_points.at(10) = ((Eigen::Matrix<double, 3, 1>(4.297,7.406,0.3)));
    temp_onegroup_points.at(11) = ((Eigen::Matrix<double, 3, 1>(3.982,6.955,0.3)));
    temp_onegroup_points.at(12) = ((Eigen::Matrix<double, 3, 1>(3.061,7.6,0.3)));
    temp_onegroup_points.at(13) = ((Eigen::Matrix<double, 3, 1>(2.22,7.629,0.3)));
    temp_onegroup_points.at(14) = ((Eigen::Matrix<double, 3, 1>(1.69,8.0,0.3)));

    this->push_back_MapVertex(vexs ,temp_onegroup_points, 15, placeColor, Bhide ,four, true);
}

void MapGraphMtx::get_fortress_R_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset){
    std::array<Eigen::Matrix<double, 3, 1>,25> temp_onegroup_points;

    //平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(6.9266,8.0657,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.2532,7.50,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.9266,6.9343,0.15)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(6.2734,6.9343,0.15)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(5.9468,7.50,0.15)));
    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(6.2734,8.0657,0.15)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 6, placeColor, mist,startupArea, false);

    //上坡

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.2532,7.50,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(6.9266,8.0657,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.1645,8.4778,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(7.7291,7.50,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,startupArea, false);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.2532,7.50,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(6.9266,6.9343,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.1645,6.5222,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(7.7291,7.50,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,startupArea, false);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(6.2734,6.9343,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(6.9266,6.9343,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.1645,6.5222,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(6.0355,6.5222,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,startupArea, false);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(6.2734,6.9343,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.9468,7.50,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.4709,7.50,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(6.0355,6.5222,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,startupArea, false);
    this->creat_dir(vexs, vexs.size()-5, vexs.size()-1,offset);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(6.2734,8.0657,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(6.9266,8.0657,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.1645,8.4778,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(6.0355,8.4778,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,startupArea, false);
    this->creat_dir(vexs, vexs.size()-6, vexs.size()-1,offset);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(6.2734,8.0657,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.9468,7.50,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.4709,7.50,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(6.0355,8.4778,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,startupArea, false);
    this->creat_dir(vexs, vexs.size()-7, vexs.size()-1,offset);
}

void MapGraphMtx::get_fortress_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset){
    std::array<Eigen::Matrix<double, 3, 1>,25> temp_onegroup_points;

        //平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-6.9266,15-8.0657,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-7.2532,15-7.50,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-6.9266,15-6.9343,0.15)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-6.2734,15-6.9343,0.15)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(28-5.9468,15-7.50,0.15)));
    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(28-6.2734,15-8.0657,0.15)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 6, placeColor, mist,startupArea, false);

    //上坡

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-7.2532,15-7.50,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-6.9266,15-8.0657,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-7.1645,15-8.4778,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-7.7291,15-7.50,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,startupArea, false);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-7.2532,15-7.50,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-6.9266,15-6.9343,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-7.1645,15-6.5222,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-7.7291,15-7.50,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,startupArea, false);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-6.2734,15-6.9343,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-6.9266,15-6.9343,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-7.1645,15-6.5222,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-6.0355,15-6.5222,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,startupArea, false);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-6.2734,15-6.9343,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-5.9468,15-7.50,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-5.4709,15-7.50,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-6.0355,15-6.5222,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,startupArea, false);
    this->creat_dir(vexs, vexs.size()-5, vexs.size()-1,offset);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-6.2734,15-8.0657,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-6.9266,15-8.0657,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-7.1645,15-8.4778,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-6.0355,15-8.4778,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,startupArea, false);
    this->creat_dir(vexs, vexs.size()-6, vexs.size()-1,offset);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-6.2734,15-8.0657,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-5.9468,15-7.50,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-5.4709,15-7.50,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-6.0355,15-8.4778,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,startupArea, false);
    this->creat_dir(vexs, vexs.size()-7, vexs.size()-1,offset);
}

void MapGraphMtx::get_trapezium_R_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,25> temp_onegroup_points;

    //10坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.0248,9.7987,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.0248,9.7987,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.0248,10.95,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.0248,10.95,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bmist,ordinary, false);

    //10坡后,43坡后
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.0248,10.95,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(3.0248,12.5,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.1748,12.5,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(5.1748,10.95,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,ordinary, false);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //10坡后
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.0248,12.5,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(3.0248,15,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.1748,15,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(5.1748,12.5,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rmist,ordinary, false);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //43坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(5.6190,10.95,0.60)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.6190,12.5,0.60)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.1748,12.5,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(5.1748,10.95,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,ordinary, false);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset);

    //最佳打符点
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(5.6190,10.95,0.60)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.6190,12.5,0.60)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.6647,12.5,0.60)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.29,11.8791,0.60)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(7.6394,10.95,0.60)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 5, placeColor, mist,windmill, false);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //高地平地(最佳打符点遮挡，全遮)
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(5.1748,13.0702,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.1748,12.5,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.0178,12.5,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(7.0178,13.0702,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,ordinary, false);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);

    //高地平地(最佳打符点遮挡，遮己方)
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.0178,13.0702,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.0178,12.5,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.6647,12.5,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.4474,11.9520,0.2)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(9.0234,12.7747,0.2)));
    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(9.5292,12.9247,0.2)));
    temp_onegroup_points.at(6) = ((Eigen::Matrix<double, 3, 1>(9.6303,13.0702,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 7, placeColor, Rhide,ordinary, false);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //高地平地(无遮挡)
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(5.1748,15,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.1748,13.0702,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.6303,13.0702,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(10.1823,13.8575,0.2)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(13.5797,13.8575,0.2)));
    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(13.5797,15,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 6, placeColor, Clear,ordinary, false);
    this->creat_dir(vexs, vexs.size()-6, vexs.size()-1,offset);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset);

    //20坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.6073,12.7747,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(8.8391,11.6777,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(8.4474,11.9520,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.0234,12.7747,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,ordinary, false);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset);
}

void MapGraphMtx::get_trapezium_B_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,25> temp_onegroup_points;

    //10坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-3.0248,15-9.7987,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-5.0248,15-9.7987,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-5.0248,15-10.95,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-3.0248,15-10.95,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rmist,ordinary, false);

    //10坡后,43坡后
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-3.0248,15-10.95,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-3.0248,15-12.5,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-5.1748,15-12.5,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-5.1748,15-10.95,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,ordinary, false);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //10坡后
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-3.0248,15-12.5,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-3.0248,15-15,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-5.1748,15-15,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-5.1748,15-12.5,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bmist,ordinary, false);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //43坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-5.6190,15-10.95,0.60)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-5.6190,15-12.5,0.60)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-5.1748,15-12.5,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-5.1748,15-10.95,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,ordinary, false);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset);

    //最佳打符点
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-5.6190,15-10.95,0.60)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-5.6190,15-12.5,0.60)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-7.6647,15-12.5,0.60)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-8.29,15-11.8791,0.60)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(28-7.6394,15-10.95,0.60)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 5, placeColor, mist,windmill, false);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //高地平地(最佳打符点遮挡，全遮)
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-5.1748,15-13.0702,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-5.1748,15-12.5,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-7.0178,15-12.5,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-7.0178,15-13.0702,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,ordinary, false);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);

    //高地平地(最佳打符点遮挡，遮己方)
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-7.0178,15-13.0702,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-7.0178,15-12.5,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-7.6647,15-12.5,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-8.4474,15-11.9520,0.2)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(28-9.0234,15-12.7747,0.2)));
    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(28-9.5292,15-12.9247,0.2)));
    temp_onegroup_points.at(6) = ((Eigen::Matrix<double, 3, 1>(28-9.6303,15-13.0702,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 7, placeColor, Bhide,ordinary, false);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //高地平地(无遮挡)
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-5.1748,15-15,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-5.1748,15-13.0702,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-9.6303,15-13.0702,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-10.1823,15-13.8575,0.2)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(28-13.5797,15-13.8575,0.2)));
    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(28-13.5797,15-15,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 6, placeColor, Clear,ordinary, false);
    this->creat_dir(vexs, vexs.size()-6, vexs.size()-1,offset);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset);

    //20坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-9.6073,15-12.7747,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-8.8391,15-11.6777,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-8.4474,15-11.9520,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-9.0234,15-12.7747,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,ordinary, false);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset);
}

void MapGraphMtx::get_highway_R_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,25> temp_onegroup_points;

    //10坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.0681,3.35,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.6652,2.15,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.3735,2.15,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(5.7821,3.35,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,ordinary, false);

    //10坡后
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(6.3735,2.15,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.7821,3.35,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(3.6757,3.35,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.6757,2,0.2)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(5.2,2,0.2)));
    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(5.2,2.15,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 6, placeColor, Rmist,ordinary, false);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //飞坡上坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(13.7702,0.14,0.55)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.7702,1,0.55)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.6013,1,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(12.6013,0.14,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,flySlope, false);

    //飞坡平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(12.6013,1.14,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(12.6013,0.0,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.8803,0.0,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.8803,1.14,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Clear,flySlope, false);
    this->creat_dir(vexs, vexs.size()-1, vexs.size()-2,offset);

    //公路平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.6757,2,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(3.6757,0,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.8803,0.0,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.8803,1.14,0.2)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(13.8013,1.14,0.2)));
    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(13.8013,2,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 6, placeColor, mist,ordinary, false);
    this->creat_dir(vexs, vexs.size()-1, vexs.size()-2,offset);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);

    //12坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(14.8066,1.2925,0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(14.8066,1.8444,0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(13.8013,1.8444,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(13.8013,1.14,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,ordinary, false);
    this->creat_dir(vexs, vexs.size()-1, vexs.size()-2,offset);
}

void MapGraphMtx::get_highway_B_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,25> temp_onegroup_points;

    //10坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-7.0681,15-3.35,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-7.6652,15-2.15,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-6.3735,15-2.15,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-5.7821,15-3.35,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,ordinary, false);

    //10坡后
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-6.3735,15-2.15,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-5.7821,15-3.35,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-3.6757,15-3.35,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-3.6757,15-2,0.2)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(28-5.2,15-2,0.2)));
    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(28-5.2,15-2.15,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 6, placeColor, Bmist,ordinary, false);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //飞坡上坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-13.7702,15-0.14,0.55)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-13.7702,15-1,0.55)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-12.6013,15-1,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-12.6013,15-0.14,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,flySlope, false);

    //飞坡平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-12.6013,15-1.14,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-12.6013,15-0.0,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-9.8803,15-0.0,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-9.8803,15-1.14,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Clear,flySlope, false);
    this->creat_dir(vexs, vexs.size()-1, vexs.size()-2,offset);

    //公路平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-3.6757,15-2,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-3.6757,15-0,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-9.8803,15-0.0,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-9.8803,15-1.14,0.2)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(28-13.8013,15-1.14,0.2)));
    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(28-13.8013,15-2,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 6, placeColor, mist,ordinary, false);
    this->creat_dir(vexs, vexs.size()-1, vexs.size()-2,offset);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);

    //12坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-14.8066,15-1.2925,0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-14.8066,15-1.8444,0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-13.8013,15-1.8444,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-13.8013,15-1.14,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,ordinary, false);
    this->creat_dir(vexs, vexs.size()-1, vexs.size()-2,offset);
}

void MapGraphMtx::get_central_R_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,25> temp_onegroup_points;

    //20坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(10.4461,4.9437,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(10.6650,4.4392,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(11.4287,4.7706,0.3)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(11.2098,5.2751,0.3)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,ordinary, false);

    //20坡后
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(11.4287,4.7706,0.3)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(11.7329,4.7390,0.3)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(11.4184,5.5188,0.3)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(11.2098,5.2751,0.3)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Clear,ordinary, false);

    //10.5坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.7486,2.0,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(15.4879,2.0,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(15.7375,2.5303,0.3)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(10.1199,2.5303,0.3)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Clear,ordinary, false);

    //一级台阶
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.5350,6.6999,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.5350,8.3001,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(10.0,8.3001,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(10.0,6.6999,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,steps, false);

    //双极台阶跨越预警1
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.5350,6.6999,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.5350,8.3001,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(8.715,8.3001,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.715,6.6999,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,stepsWarring, false);

    //二级台阶+双极台阶跨越预警2
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(10.0,6.6999,0.3)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(10.0,8.3001,0.3)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(11.2404,8.3001,0.3)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(11.2404,6.6999,0.3)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Clear,stepsWarring, false);
}

void MapGraphMtx::get_central_B_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,25> temp_onegroup_points;

    //20坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-10.4461,15-4.9437,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-10.6650,15-4.4392,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-11.4287,15-4.7706,0.3)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-11.2098,15-5.2751,0.3)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,ordinary, false);

    //20坡后
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-11.4287,15-4.7706,0.3)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-11.7329,15-4.7390,0.3)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-11.4184,15-5.5188,0.3)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-11.2098,15-5.2751,0.3)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Clear,ordinary, false);

    //10.5坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-9.7486,15-2.0,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-15.4879,15-2.0,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-15.7375,15-2.5303,0.3)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-10.1199,15-2.5303,0.3)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Clear,ordinary, false);

    //一级台阶
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-9.5350,15-6.6999,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-9.5350,15-8.3001,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-10.0,15-8.3001,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-10.0,15-6.6999,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,steps, false);

    //双极台阶跨越预警1
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-9.5350,15-6.6999,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-9.5350,15-8.3001,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-8.715,15-8.3001,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-8.715,15-6.6999,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,stepsWarring, false);

    //二级台阶+双极台阶跨越预警2
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(28-10.0,15-6.6999,0.3)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(28-10.0,15-8.3001,0.3)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28-11.2404,15-8.3001,0.3)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28-11.2404,15-6.6999,0.3)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Clear,stepsWarring, false);
}

void MapGraphMtx::get_central_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,25> temp_onegroup_points;

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(15.7375,2.5303,0.3)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(10.1199,2.5303,0.3)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(10.6189,3.2429,0.3)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(10.5412,4.2219,0.3)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(11.7329,4.7390,0.3)));
    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(11.4184,5.5188,0.3)));
    temp_onegroup_points.at(6) = ((Eigen::Matrix<double, 3, 1>(10.2029,5.0017,0.3)));
    temp_onegroup_points.at(7) = ((Eigen::Matrix<double, 3, 1>(9.85,6.6999,0.3)));
    temp_onegroup_points.at(8) = ((Eigen::Matrix<double, 3, 1>(11.2404,6.6999,0.3)));
    temp_onegroup_points.at(9) = ((Eigen::Matrix<double, 3, 1>(11.2404,8.3001,0.3)));
    temp_onegroup_points.at(10) = ((Eigen::Matrix<double, 3, 1>(9.85,8.3001,0.3)));
    temp_onegroup_points.at(11) = ((Eigen::Matrix<double, 3, 1>(9.85,9.4527,0.3)));

    temp_onegroup_points.at(12) = ((Eigen::Matrix<double, 3, 1>(28-15.7375,15-2.5303,0.3)));
    temp_onegroup_points.at(13) = ((Eigen::Matrix<double, 3, 1>(28-10.1199,15-2.5303,0.3)));
    temp_onegroup_points.at(14) = ((Eigen::Matrix<double, 3, 1>(28-10.6189,15-3.2429,0.3)));
    temp_onegroup_points.at(15) = ((Eigen::Matrix<double, 3, 1>(28-10.5412,15-4.2219,0.3)));
    temp_onegroup_points.at(16) = ((Eigen::Matrix<double, 3, 1>(28-11.7329,15-4.7390,0.3)));
    temp_onegroup_points.at(17) = ((Eigen::Matrix<double, 3, 1>(28-11.4184,15-5.5188,0.3)));
    temp_onegroup_points.at(18) = ((Eigen::Matrix<double, 3, 1>(28-10.2029,15-5.0017,0.3)));
    temp_onegroup_points.at(19) = ((Eigen::Matrix<double, 3, 1>(28-9.85,15-6.6999,0.3)));
    temp_onegroup_points.at(20) = ((Eigen::Matrix<double, 3, 1>(28-11.2404,15-6.6999,0.3)));
    temp_onegroup_points.at(21) = ((Eigen::Matrix<double, 3, 1>(28-11.2404,15-8.3001,0.3)));
    temp_onegroup_points.at(22) = ((Eigen::Matrix<double, 3, 1>(28-9.85,15-8.3001,0.3)));
    temp_onegroup_points.at(23) = ((Eigen::Matrix<double, 3, 1>(28-9.85,15-9.4527,0.3)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 24, placeColor, Clear,ordinary, false);

}

void MapGraphMtx::get_completeMapGraphMtx() {
    std::vector<MapVertex> R_vexs, B_vexs;
    // get_lib_placeConfig(this->vexs, B);

    //RM2025
    //堡垒
    get_fortress_R_placeConfig(this->vexs, R);
    get_fortress_B_placeConfig(this->vexs, B);

    //梯高
    get_trapezium_R_placeConfig(this->vexs, R);
    get_trapezium_B_placeConfig(this->vexs, B);

    get_highway_R_placeConfig(this->vexs, R);
    get_highway_B_placeConfig(this->vexs, B);

    get_central_R_placeConfig(this->vexs, R);
    get_central_B_placeConfig(this->vexs, B);
    get_central_placeConfig(this->vexs, R);
    std::cout << "vexs.size() " <<vexs.size() << std::endl;
}


