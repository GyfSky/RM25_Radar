//
// Created by plusseven on 24-1-15.
//

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

MapVertex::MapVertex(std::array<Eigen::Matrix<double, 3, 1>, 10> points_reality_3d,int point_3d_number,
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

    }
    else{
        getH_abc_3d.at(0) = -(double (n[0]))/(double (n[2]));
        getH_abc_3d.at(1) = -(double (n[1]))/(double (n[2]));
        getH_abc_3d.at(2) = double (p3_3d[2]) - getH_abc_3d.at(0)*p3_3d[0] - getH_abc_3d.at(1)*p3_3d[1];


        //2d点的投射
        matrix_change3 << (points_reality_3d.at(0)[0]- points_reality_3d.at(1)[0]) ,( points_reality_3d.at(2)[0]- points_reality_3d.at(1)[0]),
                (points_reality_3d.at(0)[1]- points_reality_3d.at(1)[1]) , ( points_reality_3d.at(2)[1]- points_reality_3d.at(1)[1]);

        matrix_change2 << ( points_predict_2d.at(0).x- points_predict_2d.at(1).x) ,( points_predict_2d.at(2).x- points_predict_2d.at(1).x),
                ( points_predict_2d.at(0).y- points_predict_2d.at(1).y) ,( points_predict_2d.at(2).y- points_predict_2d.at(1).y);

        matrix_2d <<  points_reality_3d.at(1)[0], points_reality_3d.at(1)[1];

//            matrix_change2 = matrix_change3 * matrix_change2.inverse();
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

void MapVertex::setMissType(double point_x, double point_y, double point_z) {
        this->missType = Drop;
        missPoints.push_back(cv::Point3d(point_x,point_y,point_z));
}

void MapVertex::setMissType(double angel) {
    this->missType = Line;
    this->missAngle = angel;
}

void MapVertex::setMissType(double angel, std::vector<cv::Point3d> wait_rects) {
    this->missType = WaitDrops;
    this->missAngle = angel;
    this->missPoints.clear();
    this->missPoints.insert(missPoints.end(),wait_rects.begin(),wait_rects.end());
}

void MapVertex::setMissType(double angel, std::vector<cv::Point3d> wait_rects, int missLinesTimes,double point_x, double point_y, double point_z){
    this->missType = lineDrops;
    this->missAngle = angel;
    this->missPoints.clear();
    this->missPoints.emplace_back(point_x,point_y,point_z);
    this->missPoints.insert(missPoints.end(),wait_rects.begin(),wait_rects.end());
    this->missLinesTimes = missLinesTimes;
}

void MapVertex::setMissType() {
    this->missType = Inelse;
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

    ////-------------正式场地---- start -------

//    if(ourPatternColor == red){
//        pts_pnp_3d.emplace_back(cv::Point3d(8.82197, 8.63024, 0.615));
//        pts_pnp_3d.emplace_back(cv::Point3d(8.82198, 9.29024, 0.615));
////        place.pts_pnp_3d.emplace_back(cv::Point3d(17.30477-0.649, 13.06757-0.433, 1.4724));
//        pts_pnp_3d.emplace_back(cv::Point3d(21.48525, 15.05599, 0.2));
//
//        pts_pnp_3d.emplace_back(cv::Point3d(25.22355, 15.05558, 0.4));
////        place.pts_pnp_3d.emplace_back(cv::Point3d(26.67680-0.649, 7.98879-0.433, 1.02094));
//        pts_pnp_3d.emplace_back(cv::Point3d(23.52354, 1.67667, 0.4));
//        pts_pnp_3d.emplace_back(cv::Point3d(10.25343, 7.1711, 0.6));
//    }else if(ourPatternColor == blue){
//        pts_pnp_3d.emplace_back(cv::Point3d(19.21432, 6.40551, 0.615));
//        pts_pnp_3d.emplace_back(cv::Point3d(19.21432, 5.74551, 0.615));
////        place.pts_pnp_3d.emplace_back(cv::Point3d(11.99050-0.649,2.91068-0.433, 1.47244));
//        pts_pnp_3d.emplace_back(cv::Point3d(6.51202, 0.05626, 0.2));
//
//        pts_pnp_3d.emplace_back(cv::Point3d(2.77356, 0.15667, 0.4));
////        pts_pnp_3d.emplace_back(cv::Point3d(1.3232-0.649, 7.01121-0.433, 1.02094));////
//        pts_pnp_3d.emplace_back(cv::Point3d(4.47374,13.43557,0.4));
//        pts_pnp_3d.emplace_back(cv::Point3d(17.74384, 7.94115, 0.6));
//    }else{
//        std::cout << "error in ourPatternColor" << std::endl;
//    }


    this->get_completeMapGraphMtx();
////场地围挡在红方补给站附近的交点为坐标原点，沿场地长边向蓝方为 X 轴正方向，沿场地短边向红方停机坪为 Y 轴正方向
// 左上，顺时针

}

inline void MapGraphMtx::push_back_MapVertex(std::vector<MapVertex> &vertexs, std::array<Eigen::Matrix<double, 3, 1>, 10> points_reality_3d, int point_3d_number,
                                             PlaceColor placeColor, SeeType seeType ,PlaceType_special placeType, bool isH) {
    MapVertex temp_mapVertex(points_reality_3d, point_3d_number,placeColor, seeType, placeType, isH);
    temp_mapVertex.vertex = vexs.size() + 1;
    vertexs.push_back(temp_mapVertex);
}


/**
 *
 * @parma isUndirected   默认设置为无向图
 * 该函数为手动版添加数据
 *
 * **/
void MapGraphMtx::creat_dir(int mainVex, int secVex, double x1,double y1,double x2,double y2,
                                int offset, bool isUndirected){
    MapEdge temp_mapEdge(x1, y1, x2, y2);
    MapEdge init_mapEdge(0);
    mainVex = mainVex + offset;
    secVex  = secVex + offset;
    (this->arcs.at(mainVex)).at(mainVex) = init_mapEdge;

    if(isUndirected){
        (this->arcs.at(mainVex)).at(secVex) = temp_mapEdge;
        (this->arcs.at(secVex)).at(mainVex) = temp_mapEdge;
    } else{
        (this->arcs.at(mainVex)).at(secVex) = temp_mapEdge;
    }
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
//            std::cout << "p1_x, p1_y, p2_x, p2_y    " << p1_x << "," << p1_y << "," <<  p2_x << "," <<  p2_y << std::endl;
//            std::cout << "p1_x_, p1_y_, p2_x_, p2_y_  " << p1_x_ << "," << p1_y_ << "," <<  p2_x_ << "," <<  p2_y_ << std::endl;
//            std::cout << "len: " << len << "   len_: " << len_ << std::endl;
            double temp_distance =  line2line_distance(p1_x, p1_y, p2_x, p2_y,len,
                                                  p1_x_, p1_y_, p2_x_, p2_y_, len_);

            if(temp_distance<-0.5){
                continue;
            }
            //TODO:
            else if (temp_distance < distance){
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
    // TODO:
    // std::cout << "```````distance:  " << distance << std::endl;
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
    // //pts_reality_3d
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////左下  红方---------------------------------------------------------------------------------------------------------------
    //1.上坡  TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(0,0,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(1.134,0,0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(1.134,-2.323,0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(0,-2.323,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide ,four, true);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(-3,0.16,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(0,0.16,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(0,-2.323,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(-3,-2.323,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide ,four, true);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(-1.25,0.16,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(-1.86,0.16,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(-1.86,1.92,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(-1.25,1.50,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide ,four, true);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(-3,0.16,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(-2,0.16,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(-2.0,1.54,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(-3.0,1.54,0.2)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide ,four, true);
}

/**
 * 设置左下角右上角的地图参数
 * lbrt : leftBottom and rightTop
 * **/
void MapGraphMtx::get_lb_R_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor,int offset) {
    // //pts_reality_3d
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////左下  红方---------------------------------------------------------------------------------------------------------------
    //1.上坡  TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.82370,4.00119,0  )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.02370,4.00185,0  )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.21061,2.12000,0.4)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.82370,2.12000,0.4))); //0
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide ,four, true);


    //2. 坡后小矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.82370,2.12000,0.40)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.21061,2.12000,0.40))); //0
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.21061,1.63990,0.40))); //1
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.82370,1.63990,0.40))); //1
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist ,four, false);

    this->creat_dir(vexs.size()-2, vexs.size()-1, 3.82370, 2.12000, 5.21061, 2.12000, offset,true);

    //3. 坡后后矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.82370,1.63990,0.40))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(6.61970,1.63990,0.40))); //4
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.61970,0.49900,0.40))); //5
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.82370,0.49900,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist ,is_windmill, false);

    this->creat_dir(vexs.size()-2, vexs.size()-1, 3.82370,1.63990, 5.21061,1.63990,offset,true);

    std::cout << "asc "  << vexs.back().vertex << std::endl;


    //4. 高地上坡前小方块
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(5.21061,3.04000,0.40))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(6.61970,3.04000,0.40))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.61970,1.63990,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(5.21061,1.63990,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist ,four, false); //TODO: bmist

    this->creat_dir(vexs.size()-2, vexs.size()-1, 6.61970,1.63990, 5.21061,1.63990,offset,true);



    //5. 飞坡开始的下坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.55271,1.63990,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.55271,0.49900,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.61970,0.49900,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(6.61970,1.63990,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset,true); // (3,5)

    //6. 上打符点的坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(8.56886,3.04000,0.85)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(8.56886,1.63990,0.85)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.61970,1.63990,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(6.61970,3.04000,0.4 )));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bhide ,windmill, true);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset,true); //  (4,6)


    //7. 可以看见的平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(5.21061,3.04000,0.4)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.49870,4.70600,0.4)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.67070,4.7810 ,0.4)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.45056,3.04000,0.4)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear,ordinary, false);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset,true);   // (4,7)


    //8. 飞坡平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.55271,1.63990,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(11.98718,1.4999,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(11.98718,0.6399,0.15)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(7.55271,0.49900,0.15)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, mist,flySlope, false);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (5,8)


    //9. 打符点
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(8.56886,3.04000,0.85)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.78670,3.04000,0.85)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.78670,1.63990,0.85)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.56886,1.63990,0.85)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist,windmill, false);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (6,9)


    //10. 到地面的陡坡     // TODO：
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(8.45056,3.55858,0.40)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.80516,4.57024,0.40)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(8.27792,4.87180,0.00)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.92332,3.86018,0.00)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rhide,ordinary, true);

//    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1, offset,false);   // (7,10)


    //11. 飞坡上坡  //TODO：
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(13.13198,1.4999,0.50)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.13198,0.6399,0.50)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(11.98718,0.6399,0.15)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(11.98718,1.4999,0.15)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bhide,flySlope, true);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (8,11)
}

//取的都是相对于官方地图系的绝对坐标，不存在红蓝方混淆的情况
void MapGraphMtx::get_rt_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset) {
    // //pts_reality_3d
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////右上  蓝方---------------------------------------------------------------------------------------------------------------
    //1.上坡  TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.2763,11.99815	,0  )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(23.9763,11.99815	,0  )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(23.7893,13.880	,0.4)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.2763,13.880	,0.4))); //0
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide ,four, true);//不可见区域

    //2. 坡后小矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.17630,13.880,0.40)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(23.78939,13.880,0.40))); //0
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(23.78939,14.360,0.40))); //1
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.17630,14.360,0.40))); //1
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist ,four, false);//模糊区域

    this->creat_dir(vexs,vexs.size()-2, vexs.size()-1,offset);  //(1,2)//对邻接矩阵进行赋值，为以上两个区域相邻最近的点

    //3. 坡后后矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.1763,15.5001,0.40))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(22.3803,15.5001,0.40))); //4
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(22.3803,14.3600,0.40))); //5
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.1763,14.3600,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist ,is_windmill, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);  // (2,3)


    //4. 高地上坡前小方块
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(22.3803 ,12.810,0.40))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(23.78939,12.810,0.40))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(23.78939,14.360,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(22.3803 ,14.360,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist ,four, false); //TODO:

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //5. 飞坡开始的下坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(21.44729,14.3600,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(21.44729,15.5001,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(22.38030,15.5001,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(22.38030,14.3600,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); // (3,5) 找连通区域？因此设置为-3？

    //6. 上打符点的坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.43114,12.9600,0.85)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(20.43114,14.3601,0.85)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(22.3803,14.210,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(22.3803,12.810,0.4 )));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rhide ,windmill, true);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (4,6)


    //7. 可以看见的平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(23.78939,12.810,0.4)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(23.78939,11.294,0.4)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.32930,11.219,0.4)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(20.43114,12.810,0.4)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear,ordinary, false);//清晰区域

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (4,7)


    //8. 飞坡平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(21.44729,14.3600,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(17.01282,14.3600,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.01282,15.3601,0.15)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(21.44729,15.3601,0.15)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, mist,flySlope, false);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (5,8)


    //9. 打符点
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.43114,12.9600,0.85 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(19.21330,12.9600,0.85 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.21330,14.3601,0.85 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(20.43114,14.3601,0.85 )));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist,windmill, false);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (6,9)


    //10. 到地面的陡坡     // TODO：
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.722080,11.12816,0.00)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(20.007668,12.13982,0.00)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(20.549440,12.44142,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(21.329300,11.21900,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bhide,ordinary, true);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1, offset,false);   // (7,10)  // TODO: error


    //11. 飞坡上坡  //TODO：
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(15.86802,14.5001,0.5)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(15.86802,15.3601,0.5)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.20202,15.3601,0.15)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(17.20202,14.5001,0.15)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bhide,flySlope, true);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (8,11)
}

void MapGraphMtx::get_lt_R_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////左上 红 -----------------------------------------------------------------------------------------

    //1.上R3的坡    TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.42576,11.9170,0.4)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(6.00764,11.9170,0.4)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.24163,10.8180,0  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.42576,10.8180,0  ))); //0
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide ,ordinary, true);

    //2. 坡后小矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.42576,14.3600,0.40)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.72176,14.3600,0.40))); //0
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.00764,11.9170,0.40))); //1
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.42576,11.9170,0.40))); //1
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //3. 矩形“ROBOMASTER”
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.42476,15.5000,0.40))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.72176,15.5000,0.40))); //4
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.72176,14.3600,0.40))); //5
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.42476,14.3600,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //4. 下坡（至0.2飞坡后敌方矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(8.15478,15.500,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(8.15478,14.360,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.72176,14.360,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(7.72176,15.500,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); // (3,4)


    //5. R3
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(8.12468,15.500,0.15))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(15.2190,15.500,0.15))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3,
            1>(15.2190,14.360,0.15)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.12468,14.360,0.15)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs,vexs.size()-2, vexs.size()-1,offset);   // （4，5）

    //6. 大平坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.72176,14.360,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(12.5156,14.360,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(11.5291,13.215,0.00)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(7.40833,13.215,0.00)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (5,6)


//    //7. 梯形三角       TODO
//    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(8.97550-0.649,12.52386-0.433	,0.00))); //7
//    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(8.28719-0.649,11.54089-0.433	,0.00))); //7
//    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.82601-0.649,11.86358-0.433	,0.40)));
//    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.51678-0.649,12.84507-0.433	,0.40)));
//    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, hide ,ordinary, true);
//
//    this->creat_dir(vexs,vexs.size()-3, vexs.size()-1,offset, false);   // （5，7） //TODO：error
//
//    //8. 低地下坡（飞坡后敌方矩形后下坡   TODO
//    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.946400-0.649,14.34921-0.433	,0.20)));
//    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(12.58992-0.649,14.34972-0.433	,0.20)));
//    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(11.83377-0.649,13.10547-0.433	,0.00)));
//    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.190250-0.649,13.10547-0.433	,0.00)));
//    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bhide ,ordinary, true);
//
//    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (6,8)

}

void MapGraphMtx::get_rb_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////右下 蓝 -----------------------------------------------------------------------------------------
    //1.上R3的坡    TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.5724,4.08301,0.4)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(22.9924,4.08301,0.4)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(23.7584,5.18200,0  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.5724,5.18200,0  ))); //0
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide ,ordinary, true);

    //2. 坡后小矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.5724,1.6400,0.40)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(21.2782,1.6400,0.40))); //0
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(22.9927,4.0830,0.40))); //1
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.5724,4.0830,0.40))); //1
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //3. 矩形“ROBOMASTER”
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.57524,0.5000,0.40))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(21.27824,0.5000,0.40))); //4
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.27824,1.6400,0.40))); //5
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.57524,1.6400,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //4. 下坡（至0.2飞坡后敌方矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.84522,0.5000,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(20.84522,1.6400,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.27824,1.6400,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(21.27824,0.5000,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); // (3,4)


    //5. R3
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.84522,0.5000,0.15))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.78098,0.5000,0.15))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(13.78098,1.6400,0.15)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(20.84522,1.6400,0.15)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs,vexs.size()-2, vexs.size()-1,offset);   // （4，5）

    //6. 大平坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.84522,1.6400,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(16.48438,1.6400,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.47087,2.7850,0.00)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(21.59167,2.7850,0.00)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (4,6)
//    std::cout << vexs.back().vertex << std::endl;  // 23
//    //7. 陡坡       TODO
//    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.31977-0.649,3.45439-0.433	,0.00))); //7
//    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(21.008081 -0.649,4.43736-0.433	,0.00))); //7
//    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.46680-0.649,4.11615-0.433	,0.40)));
//    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(20.78095-0.649,3.13145-0.433	,0.40)));
//    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, hide ,ordinary, true);
//
//    this->creat_dir(vexs,vexs.size()-3, vexs.size()-1,offset, false);   // （5，7）//TODO: error
//
//    //8. 低地下坡（飞坡后敌方矩形后下坡   TODO
//    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.34507-0.649,1.62439-0.433	,0.20)));
//    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(16.88820-0.649,1.88986,0.20)));
//    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.68912-0.649,2.77282-0.433	,0.00)));
//    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(20.04322-0.649,2.77282-0.433	,0.00)));
//    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rhide ,ordinary, true);
//
//    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (6,8)
}

void MapGraphMtx::get_mid_R_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////左中 红中 -----------------------------------------------------------------------------------------

    //1.平地的上坡  TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.589820,6.91503,0.6)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(10.81848,7.77549,0.6)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.10300,5.94133,0  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(10.87434,5.08086,0  ))); //0
        this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist ,ordinary, true);

    //2. 坡后小三角
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.318520,7.30243,0.60)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(10.81848,7.77549,0.60))); //0
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.589820,6.91503,0.60))); //1
    this->push_back_MapVertex(vexs, temp_onegroup_points, 3, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //3. 正梯形高地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.318320,9.79302,0.60))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(10.81836,9.32019,0.60))); //4
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(10.81848,7.77549,0.60))); //5
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.318520,7.30243,0.60)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);


    //4. 斜梯形高地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(11.22283,12.51339,0.60))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(12.45163,11.65313,0.60))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(10.81836,9.320190,0.60)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.318320,9.793020,0.60)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs,vexs.size()-2, vexs.size()-1,offset);

    //5. 下坡至0.15小梯形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(12.3407,14.11014,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.5695,13.24988,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.45163,11.65313,0.60)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(11.22283,12.51339,0.60)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); // (4,5)

    //6. 0.15小梯形 TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(12.5156,14.36000,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(14.4664,14.35900,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(13.5695,13.24988,0.15)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(12.3407,14.11014,0.15)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (5,6)

}

void MapGraphMtx::get_mid_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor,  int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////右中 蓝中 -----------------------------------------------------------------------------------------

    //1.平地的上坡  TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.41018,9.08497,0.6)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(18.18152,8.22451,0.6)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(16.89700,10.0586,0  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(18.12566,11.0052,0  ))); //0
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist ,ordinary, true);

    //2. 坡后小三角 （6，4，5）
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.83148,8.74488,0.60))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(18.18152,8.22451,0.60)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.41018,9.08497,0.60))); //0
    this->push_back_MapVertex(vexs, temp_onegroup_points, 3, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //3. 正梯形高地    （7，3，4，6）
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.83168,6.1597,0.60))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(18.18164,6.67981,0.60))); //4
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(18.18152,8.22451,0.60))); //5
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.83148,8.74488,0.60)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);


    //4. 斜梯形高地    1，2，3，7
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(17.77717,3.48661,0.60))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(16.54837,4.34687,0.60))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(18.18164,6.67981,0.60)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.83168,6.15970,0.60)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs,vexs.size()-2, vexs.size()-1,offset);

    //5. 下坡至0.15
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(16.65936,1.88986,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(15.43050,2.75012,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(16.54837,4.34687,0.60)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(17.77717,3.48661,0.60)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); // (4,5)

    //6. 0.2小三角     TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(16.48438,1.64000,0.15)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(14.53363,1.64000,0.15)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(15.43050,2.75012,0.15)));
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(16.65936,1.88986,0.15)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 3, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (5,6)

}

void MapGraphMtx::get_hole_R_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

    //5.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(12.03569,6.60000,0.09)));//TODO:
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(12.03569,10.2663,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(13.42348,8.87852,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(13.42348,7.37000,0.09)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,holeWarring, false);

    //7.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(11.27298,7.91920,0.09)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(11.27298,9.17706,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.03569,10.2663,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(12.03569,6.60000,0.09)));//TODO:
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,holeWarring, false);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.678860,8.27275,0.09)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.678820,8.82275,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(11.27298,9.17706,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(11.27298,7.91920,0.09)));//TODO:
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,hole, false);
}

void MapGraphMtx::get_hole_B_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

    //5.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(15.57652,7.1315,0.09)));//TODO:
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(15.57652,8.2245,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(16.96120,9.1675,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(16.96128,5.7293,0.09)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,holeWarring, false);

    //7.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(16.96123,5.72935,0.09)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(16.96123,9.16750,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.72702,8.08080,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(17.72710,6.99308,0.09)));//TODO:
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,holeWarring, false);

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(17.72710,6.99308,0.09)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(17.72702,8.08080,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.32118,7.72714,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.32118,7.17725,0.09)));//TODO:
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,hole, false);
}


void MapGraphMtx::get_startupArea_R_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset){
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(4.35+0.5,4.950+0.5,0.00)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(4.35+0.5,10.05+0.5,0.00)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.95+0.5,10.05+0.5,0.00)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(6.95+0.5,4.950+0.5,0.00)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,startupArea, false);

}
void MapGraphMtx::get_startupArea_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset){
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(21.05+0.5,4.950+0.5,0.00)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(21.05+0.5,10.05+0.5,0.00)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(23.65+0.5,10.05+0.5,0.00)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(23.65+0.5,4.950+0.5,0.00)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,startupArea, false);

}

void MapGraphMtx::get_midGround_R_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////地面 红中-----------------------------------------------------------------------------------------

    //1  TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(13.92018,4.743880,0.09)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(15.58022,7.117770,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(16.96431,5.733680,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(15.33394,3.405280,0.09)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bmist ,ordinary, false);
    this->vexs.back().setMissType(55);



    //2.  TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(13.42348,7.370000,0.09)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.42348,8.878520,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(15.58022,7.117770,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(14.79715,5.996330,0.09)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bmist ,ordinary, false);
    this->vexs.back().setMissType(180-55.07);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (1,2)


    //3.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(12.03569,6.60000,0.09)));//TODO:
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.42348,7.37000,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(14.79715,5.99633,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(13.92018,4.74388,0.09)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rmist ,ordinary, false);
    this->vexs.back().setMissType(180-55.07);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (1,3)
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (2,3)


    //4.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(12.39134,5.53041,0.00)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(12.76562,5.79248,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(15.33394,3.40528,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(15.02442,3.03731,0.00)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bmist,ordinary, true);
    this->vexs.back().setMissType(55);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (1,4)
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (3,4)

    //5.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(12.03569,6.60000,0.09)));//TODO:
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(12.03569,10.2663,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(13.42348,8.87852,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(13.42348,7.37000,0.09)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,ordinary, false);
    this->vexs.back().setMissType(90);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (2,5)
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (3,5)

    //6.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(8.56880,3.04000,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(8.90360,3.70071,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.1030,5.94133,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(15.0233,3.0353,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rmist,ordinary, false);
    this->vexs.back().setMissType(0.0);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (4,6)

    //7.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(11.27298,7.91920,0.09)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(11.27298,9.17706,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.03569,10.2663,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(12.03569,6.60000,0.09)));//TODO:
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,holeWarring, false);
    this->vexs.back().setMissType(90.0);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (5,7)

    //8.TODO:
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.7867,1.7900,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.7867,3.0400,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(15.0233,3.0353,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(15.4305,2.7501,0.0)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(14.5336,1.6410,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 5, placeColor, mist,ordinary, false);
    this->vexs.back().setMissType(10.532,2.915);//TODO 0
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (6,8)

    //9.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.41200,5.18100,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.41200,5.39300,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(8.46922,6.83057,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(10.8743,5.08086,0.0)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(8.9036,3.7007,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 5, placeColor, Bmist,ordinary, false);
    this->vexs.back().setMissType(180-55.07);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (6,9)


}

void MapGraphMtx::get_midGround_B_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////地面 蓝中-----------------------------------------------------------------------------------------

    //1  TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(12.05569,10.26632,0.09)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.0502,12.57208,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(15.0845,11.25968,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(13.41978,8.882230,0.09)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rmist ,ordinary, false);
    this->vexs.back().setMissType(55);


    //2.  TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(13.41978,8.88223,0.09)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(14.01698,9.73512,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(15.57652,8.2245,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(15.55765,7.1315,0.09)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rmist ,ordinary, false);
    this->vexs.back().setMissType(180-55);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (1,2)


    //3.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(14.01698,9.73512,0.09)));//TODO:
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(15.08449,11.25968,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(16.961,9.16750,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(15.57652,8.22451,0.09)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bmist ,ordinary, false);
    this->vexs.back().setMissType(180-55.07);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (1,3)
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (2,3)


    //4.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(13.66606,12.59472,0.09)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.97664,12.96840,0.00)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(16.60866,10.46959,0.00)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(16.23438,10.20752,0.09)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rmist,ordinary, true);
    this->vexs.back().setMissType(55);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (1,4)
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (3,4)

    //5.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(15.57652,7.1315,0.09)));//TODO:
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(15.57652,8.2245,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(16.96120,9.1675,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(16.96128,5.7293,0.09)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,ordinary, false);
    this->vexs.back().setMissType(90);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (2,5)
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (3,5)

    //6.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(13.97664,12.96484,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.65020,12.57208,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.83983,12.81000,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(16.89770,10.0586,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bmist,ordinary, false);
    this->vexs.back().setMissType(0.0);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (4,6)

    //7.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(16.96123,5.72935,0.09)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(16.96123,9.16750,0.09)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.72702,8.08080,0.09)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(17.72710,6.99308,0.09)));//TODO:
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,holeWarring, false);
    this->vexs.back().setMissType(90.0);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (5,7)

    //8.TODO:
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(13.97664,12.96484,0.0)));
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(13.6502,12.5720,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(14.4663,14.359,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.2133,14.2100,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.2133,12.810,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 5, placeColor, mist,ordinary, false);
    this->vexs.back().setMissType(10.532,2.915);//TODO 0
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (6,8)

    //9.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.41018,9.08497,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(18.24853,11.00518,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(20.14411,12.2245,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(21.58800,10.70491,0.0)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(21.58800,10.60491,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 5, placeColor, Rmist,ordinary, false);
    this->vexs.back().setMissType(180-55.07);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (6,9)

}

void MapGraphMtx::get_behindGround_R_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;
    std::vector<cv::Point3d> temp_wait_points;

    //1.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.412,5.1810,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.412,7.3024,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.318,7.3024,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.5892,6.9150,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor,Bhide ,ordinary, false);
    this->vexs.back().setMissType(180-55);
//    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (6,9)

    //2.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.4120,7.3024,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.4120,9.8403,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.3180,9.8403,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.3180,7.3024,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bmist,ordinary, false);
    this->vexs.back().setMissType(90);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (1,2)

    //3.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.7237,5.97763,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(3.2747,10.8180,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.4120,10.8180,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(7.4120,5.97763,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,ordinary, false);
    this->vexs.back().setMissType(90);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (2,3)
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (1,3)

    //.4
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.8237,5.1810,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(3.7237,5.9776,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.4120,5.3963,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(5.0237,5.1810,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Clear,ordinary, false);
    this->vexs.back().setMissType(0);
//    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (1,4) Todo:::::::
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (3,4)

    //5.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.4120,9.8403,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.4120,10.8180,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.8528,10.8180,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.3183,9.79302,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,ordinary, false);
    this->vexs.back().setMissType(55);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (2,5)
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (3,5)

    //6.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(0.0000,5.97763,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(0.0000,10.8180,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(3.7237,10.8180,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.7237,5.97763,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,ordinary, false);
    this->vexs.back().setMissType(90);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (3,6)
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (4,6)

    //7.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(2.0915,0.0000,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(2.0915,5.9776,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(3.7237,5.9776,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.7237,0.0000,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bmist,ordinary, false);
    temp_wait_points.clear();
    this->vexs.back().setMissType(90,temp_wait_points,4,2.091,1.670,0);
    this->creat_dir(vexs, vexs.size()-5, vexs.size()-1,offset); //  (3,7)
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (4,7)
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (6,7)

    //8.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(5.27494,10.8180,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.40833,13.2150,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(11.5291,13.2150,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.8528,10.8180,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,ordinary, false);
    this->vexs.back().setMissType(45.5);
    this->creat_dir(vexs, vexs.size()-6, vexs.size()-1,offset); //  (3,8)
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (5,8)






}

void MapGraphMtx::get_behindGround_B_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;
    std::vector<cv::Point3d> temp_wait_points;

    //1.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.8310,8.7448,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(19.5351,9.1675,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.5880,10.6049,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(21.5880,8.74480,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor,Rhide ,ordinary, false);
    this->vexs.back().setMissType(180-55);

    //2.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.83168,5.61084,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(19.83148,8.74488,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.58800,8.74488,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(21.58800,5.61084,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rmist,ordinary, false);
    this->vexs.back().setMissType(90);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (1,2)

    //3.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(21.5880,5.26833,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(21.5880,10.0049,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(25.2763,10.0223,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.2763,5.18200,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide,ordinary, false);
    this->vexs.back().setMissType(90);
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (2,3)
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (1,3)

    //.4
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(21.5880,10.0049,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(21.5880,10.7049,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(25.2763,10.7049,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.2763,10.0223,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Clear,ordinary, false);
    this->vexs.back().setMissType(0);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (1,4)
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (3,4)

    //5.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.22960,5.26833,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(19.83168,5.61084,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.58800,6.15970,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(21.5880,5.26833,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,ordinary, false);
    this->vexs.back().setMissType(55);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (2,5)
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (3,5)

    //6.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.2763,5.1820,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(25.2763,10.022,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(28.0000,10.022,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(28.0000,5.1820,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,ordinary, false);
    this->vexs.back().setMissType(90);
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (3,6)
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (4,6)

    //7.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.2763,10.0224,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(25.2763,15.0000,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(26.8950,14.3300,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(26.8950,10.0224,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rmist,ordinary, false);
    temp_wait_points.clear();
    this->vexs.back().setMissType(90,temp_wait_points,4,26.895,14.3300,0);
    this->creat_dir(vexs, vexs.size()-5, vexs.size()-1,offset); //  (3,7)
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (4,7)
    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (6,7)

    //8.
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(17.4708,2.7850,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(19.2296,5.2683,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(23.2700,5.1820,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(21.5916,2.7850,0.0)));
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide,ordinary, false);
    this->vexs.back().setMissType(45.5);
    this->creat_dir(vexs, vexs.size()-6, vexs.size()-1,offset); //  (3,8)
    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset); //  (5,8)

}

/**
 *
 * @brief 邻接矩阵可视化
 * **/
void MapGraphMtx::print_AdjacencyMatrix() {
    for(int i=0; i<this->vexnum; i++){
        for(int j=0; j<this->vexnum; j++){
            std::cout<<  arcs[i][j].p << "  ";
        }
        std::cout << std::endl;
    }
}

void MapGraphMtx::get_completeMapGraphMtx() {
    std::vector<MapVertex> R_vexs, B_vexs;
    // get_lib_placeConfig(this->vexs, B);
    get_rt_B_placeConfig(this->vexs, B); //TODO: (2,4)
    get_mid_B_placeConfig(this->vexs, B);
    get_rb_B_placeConfig(this->vexs, B);
    get_lb_R_placeConfig(this->vexs, R);
    get_mid_R_placeConfig(this->vexs, R);
    get_lt_R_placeConfig(this->vexs, R);

    get_hole_R_placeConfig(this->vexs, R);
    get_hole_B_placeConfig(this->vexs, B);

    get_startupArea_R_placeConfig(this->vexs, R);
    get_startupArea_B_placeConfig(this->vexs, B);
//    get_midGround_R_placeConfig(this->vexs, R);
//    get_midGround_B_placeConfig(this->vexs, B);
//    get_behindGround_R_placeConfig(this->vexs, R);
//    get_behindGround_B_placeConfig(this->vexs, B);
    std::cout << "vexs.size() " <<vexs.size() << std::endl;
    // TODO：
    // print_AdjacencyMatrix();
}


