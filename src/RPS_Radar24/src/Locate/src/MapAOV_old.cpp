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
//        double inv_z = 1.0 / pc[2];
//        double inv_z2 = 1.0 / (inv_z * inv_z);
        Eigen::Vector2d predict(fx * pc[0] / pc[2] + cx,fy * pc[1] / pc[2] + cy );
        points_predict_2d.at(j) = cv::Point2d((int)predict[0],(int)predict[1]);
    }
}

/**
 * @brief 获取所有(all)区域3d坐标投影到相机坐标系的数据
 * **/
void MapGraphMtx::get_predict_2d(const cv::Mat T, const double fx, const double fy, const double cx, const double cy) {
    Eigen::Matrix<double,4,4> Rt;
    cv::cv2eigen(T.inv(),Rt);//得到世界到相机的转换
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
    auto n = (p3_3d-p1_3d).cross(p2_3d-p1_3d);

    if(n[2]==0){
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
    double distance = 10;
    double x1 =0, y1=0, x2=0, y2=0;
    for(int i=0; i<main_vertex.point_3d_number;i++){
        double p1_x, p1_y, p2_x, p2_y;
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
            len = get2Ddistance(p1_x, p1_y, p2_x, p2_y);
            len_ = get2Ddistance(p1_x_, p1_y_, p2_x_, p2_y_);
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


/**
 * 设置左下角右上角的地图参数
 * lbrt : leftBottom and rightTop
 * **/
void MapGraphMtx::get_lb_R_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor,int offset) {
    // //pts_reality_3d
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////左下  红方---------------------------------------------------------------------------------------------------------------
    //1.上坡  TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.42256-0.649,3.99189-0.433	,0  )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(4.91978-0.649,3.99187-0.433	,0  )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(4.90874-0.649,2.11001-0.433	,0.4)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.42256-0.649,2.11001-0.433	,0.4))); //0
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bhide ,ordinary, true);


    //2. 坡后小矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.42256-0.649,2.11079-0.433	,0.40)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.12269-0.649,2.07211-0.433	,0.40))); //0
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.12269-0.649,1.77921-0.433	,0.40))); //1
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.42256-0.649,1.77923-0.433	,0.40))); //1
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist ,ordinary, false);

    this->creat_dir(vexs.size()-2, vexs.size()-1, 3.42256-0.649, 2.11079-0.433, 5.12269-0.649, 2.07211-0.433, offset,true);

    //3. 坡后后矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.42256-0.649,1.77923-0.433	,0.40))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(6.41836-0.649,1.77927-0.433	,0.40))); //4
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.41834-0.649,0.48927-0.433	,0.40))); //5
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.42256-0.649,0.48967-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist ,ordinary, false);

    this->creat_dir(vexs.size()-2, vexs.size()-1, 3.42256-0.649,1.77923-0.433, 5.12269-0.649,1.77923-0.433,offset,true);

    std::cout << "asc "  << vexs.back().vertex << std::endl;


    //4. 高地上坡前小方块
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(5.12269-0.649,3.17923-0.433	,0.40))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(6.41836-0.649,3.17923-0.433	,0.40))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.41836-0.649,1.77921-0.433	,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(5.12269-0.649,1.77923-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist ,ordinary, false); //TODO: bmist

    this->creat_dir(vexs.size()-2, vexs.size()-1, 6.41836-0.649,1.77921-0.433, 5.12269-0.649,1.77923-0.433,offset,true);



    //5. 飞坡开始的下坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.16104-0.649,1.62926-0.433	,0.20)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.16102-0.649,0.48926-0.433	,0.20)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.41834-0.649,0.48927-0.433	,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(6.41836-0.649,1.62927-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist ,flySlope, true);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset,true); // (3,5)

    //6. 上打符点的坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(8.36755-0.649,3.02923-0.433	,0.85)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(8.36755-0.649,1.77923-0.433	,0.85)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.41836-0.649,1.77927-0.433	,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(6.41836-0.649,3.02927-0.433	,0.4 )));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bhide ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset,true); //  (4,6)


    //7. 可以看见的平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(5.12269-0.649,3.17923-0.433	,0.4)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.21271-0.649,4.671935-0.433,0.4)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.36948-0.649,4.77172-0.433	,0.4)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.48452-0.649,3.17923-0.433	,0.4)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear,ordinary, false);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset,true);   // (4,7)


    //8. 飞坡平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.161040-0.649,1.62926-0.433	,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(12.13563-0.649,1.48917-0.433,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.13563-0.649,0.62917-0.433	,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(7.161020-0.649,0.48926-0.433	,0.2)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, mist,flySlope, false);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (5,8)


    //9. 打符点
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(8.36755-0.649,3.02923-0.433	,0.85 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.93594-0.649,3.02921-0.433	,0.85 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.93592-0.649,1.77921-0.433	,0.85 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.36755-0.649,1.77923-0.433	,0.85 )));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist,ordinary, false);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (6,9)


    //10. 到地面的陡坡     // TODO：
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.97161-0.649,4.88812-0.433	,0.00)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(8.65988-0.649,3.90513-0.433	,0.00)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(8.48452-0.649,3.17923-0.433	,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(7.36948-0.649,4.77172-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rhide,ordinary, true);

//    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1, offset,false);   // (7,10)


    //11. 飞坡上坡  //TODO：
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(13.17833-0.649,1.48917-0.433	,0.55)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.17833-0.649,0.62915-0.433	,0.55)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.13563-0.649,0.62917-0.433	,0.2 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(12.13563-0.649,1.48917-0.433	,0.2 )));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bhide,ordinary, true);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (8,11)
}

void MapGraphMtx::get_rt_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset) {
    // //pts_reality_3d
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////右上  蓝方---------------------------------------------------------------------------------------------------------------
    //1.上坡  TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.87249-0.649,11.98601-0.433	,0  )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(24.57527-0.649,11.98603-0.433	,0  )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(24.38631-0.649,13.86789-0.433	,0.4)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.87253-0.649,13.86786-0.433	,0.4))); //0
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rhide ,ordinary, true);

    //2. 坡后小矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.87253-0.649,13.86786-0.433	,0.40)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(24.17248-0.649,13.86789-0.433	,0.40))); //0
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(24.17248-0.649,14.34899-0.433	,0.40))); //1
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.87253-0.649,14.34899-0.433	,0.40))); //1
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist ,ordinary, false);

    this->creat_dir(vexs,vexs.size()-2, vexs.size()-1,offset);  //(1,2)

    //3. 坡后后矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.87255-0.649,15.48898-0.433	,0.40))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(22.87693-0.649,15.48898-0.433	,0.40))); //4
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(22.87693-0.649,14.34899-0.433	,0.40))); //5
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.87255-0.649,14.34899-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);  // (2,3)


    //4. 高地上坡前小方块
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(22.87693-0.649,12.79867-0.433	,0.40))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(24.17248-0.649,12.79867-0.433	,0.40))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(24.17248-0.649,14.34899-0.433	,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(22.87693-0.649,14.34899-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist ,ordinary, false); //TODO:

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //5. 飞坡开始的下坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(22.13423-0.649,14.34899-0.433	,0.20)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(22.13425-0.649,15.48899-0.433	,0.20)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(22.87693-0.649,15.48898-0.433	,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(22.87693-0.649,14.34899-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist ,flySlope, true);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); // (3,5)

    //6. 上打符点的坡
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.92753-0.649,12.94866-0.433	,0.85)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(20.92753-0.649,14.19866-0.433	,0.85)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(22.87693-0.649,14.19899-0.433	,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(22.87693-0.649,12.94899-0.433	,0.4 )));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rhide ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (4,6)


    //7. 可以看见的平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(24.17248-0.649,12.79867-0.433	,0.4)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(24.17248-0.649,11.27614-0.433,0.4)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.92557-0.649,11.20618-0.433	,0.4)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(20.81054-0.649,12.79867-0.433	,0.4)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear,ordinary, false);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (4,7)


    //8. 飞坡平地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(22.13423-0.649,14.34899-0.433	,0.2)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(17.28173-0.649,14.48908-0.433,0.2)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.26175-0.649,15.34908-0.433	,0.2)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(22.13425-0.649,15.48899-0.433	,0.2)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, mist,flySlope, false);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (5,8)


    //9. 打符点
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.92753-0.649,12.94866-0.433	,0.85 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(19.35914-0.649,12.94866-0.433	,0.85 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.35914-0.649,14.19866-0.433	,0.85 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(20.92753-0.649,14.19866-0.433	,0.85 )));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist,ordinary, false);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (6,9)


    //10. 到地面的陡坡     // TODO：
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(21.32664-0.649,11.09169-0.433	,0.00)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(20.63591-0.649,12.07296-0.433	,0.00)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.09391-0.649,12.39396-0.433	,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(21.78464-0.649,11.41269-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bhide,ordinary, true);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1, offset,false);   // (7,10)  // TODO: error


    //11. 飞坡上坡  //TODO：
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(16.11695-0.649,14.48910-0.433	,0.55)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(16.11695-0.649,15.34910-0.433	,0.55)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.26175-0.649,15.34908-0.433	,0.2 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(17.28173-0.649,14.48908-0.433	,0.2 )));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bhide,ordinary, true);

    this->creat_dir(vexs, vexs.size()-4, vexs.size()-1,offset);   // (8,11)
}

void MapGraphMtx::get_lt_R_placeConfig(std::vector<MapVertex> &vexs, PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////左上 红 -----------------------------------------------------------------------------------------

    //1.上R3的坡    TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.42247-0.649,13.86932-0.433	,0.4)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(4.90896-0.649,13.86858-0.433	,0.4)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(4.80853-0.649,11.98675-0.433	,0  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.42247-0.649,11.98675-0.433	,0  ))); //0
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Bmist ,ordinary, true);

    //2. 坡后小矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.42247-0.649,15.48934-0.433	,0.40)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(4.90896-0.649,15.48934-0.433	,0.40))); //0
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(4.90896-0.649,13.86858-0.433	,0.40))); //1
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.42247-0.649,13.86932-0.433	,0.40))); //1
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //3. 矩形“ROBOMASTER”
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(4.90896-0.649,15.48934-0.433	,0.40))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.57003-0.649,15.48921-0.433	,0.40))); //4
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.57003-0.649,14.34921-0.433	,0.40))); //5
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(4.90896-0.649,13.86858-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //4. 下坡（至0.2飞坡后敌方矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.94640-0.649,15.48921-0.433	,0.20)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.94640-0.649,14.34921-0.433	,0.20)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.57003-0.649,14.34921-0.433	,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.57003-0.649,15.48921-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); // (3,4)


    //5. R3
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(5.12274-0.649,13.86857-0.433	,0.40))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.57003-0.649,14.34921-0.433	,0.40))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.36900-0.649,11.20678-0.433	,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(5.12274-0.649,11.20678-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs,vexs.size()-3, vexs.size()-1,offset);   // （3，5）

    //6. 0.2飞坡后敌方矩形  R1 TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.946400-0.649,15.48921-0.433	,0.20)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(15.46695-0.649,15.48911-0.433	,0.20)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(15.46693-0.649,14.34911-0.433	,0.20)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.946400-0.649,14.34921-0.433	,0.20)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (4,6)


    //7. 陡坡       TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(8.97550-0.649,12.52386-0.433	,0.00))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(8.28719-0.649,11.54089-0.433	,0.00))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.82601-0.649,11.86358-0.433	,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.51678-0.649,12.84507-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, hide ,ordinary, true);

    this->creat_dir(vexs,vexs.size()-3, vexs.size()-1,offset, false);   // （5，7） //TODO：error

    //8. 低地下坡（飞坡后敌方矩形后下坡   TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.946400-0.649,14.34921-0.433	,0.20)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(12.58992-0.649,14.34972-0.433	,0.20)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(11.83377-0.649,13.10547-0.433	,0.00)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.190250-0.649,13.10547-0.433	,0.00)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bhide ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (6,8)

}

void MapGraphMtx::get_rb_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////右下 蓝 -----------------------------------------------------------------------------------------
    //1.上R3的坡    TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.87277-0.649,2.10964-0.433	,0.4)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(24.38632-0.649,2.10967-0.433	,0.4)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(24.54700-0.649,3.99150-0.433	,0  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.87280-0.649,3.99150-0.433	,0  ))); //0
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, Rmist ,ordinary, true);

    //2. 坡后小矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.87277-0.649,0.48892-0.433	,0.40)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(24.38632-0.649,0.48892-0.433	,0.40))); //0
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(24.38632-0.649,2.10967-0.433	,0.40))); //1
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.87277-0.649,2.10964-0.433	,0.40))); //1
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //3. 矩形“ROBOMASTER”
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(24.38632-0.649,0.48892-0.433	,0.40))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(19.72524-0.649,0.48903-0.433	,0.40))); //4
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.72524-0.649,1.62903-0.433	,0.40))); //5
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(24.38632-0.649,2.10967-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //4. 下坡（至0.2飞坡后敌方矩形
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.34507-0.649,0.48904-0.433	,0.20)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(19.34507-0.649,1.62439-0.433	,0.20)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.72524-0.649,1.62903-0.433	,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.72524-0.649,0.48903-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); // (3,4)


    //5. R3
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(24.38632-0.649,2.10967-0.433	,0.40))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(19.72524-0.649,1.62903-0.433	,0.40))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.92567-0.649,4.77146-0.433	,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(24.17258-0.649,4.62142-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs,vexs.size()-3, vexs.size()-1,offset);   // （3，5）

    //6. 0.2飞坡后敌方矩形  R1 TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.34507-0.649,0.48904-0.433	,0.20)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.82731-0.649,0.48914-0.433	,0.20)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(13.82700-0.649,1.62914-0.433	,0.20)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.34507-0.649,1.62439-0.433	,0.20)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (4,6)
//    std::cout << vexs.back().vertex << std::endl;  // 23

    //7. 陡坡       TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.31977-0.649,3.45439-0.433	,0.00))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(21.008081 -0.649,4.43736-0.433	,0.00))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.46680-0.649,4.11615-0.433	,0.40)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(20.78095-0.649,3.13145-0.433	,0.40)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, hide ,ordinary, true);

    this->creat_dir(vexs,vexs.size()-3, vexs.size()-1,offset, false);   // （5，7）//TODO: error

    //8. 低地下坡（飞坡后敌方矩形后下坡   TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.34507-0.649,1.62439-0.433	,0.20)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(16.88820-0.649,1.62909-0.433	,0.20)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.68912-0.649,2.77282-0.433	,0.00)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(20.04322-0.649,2.77282-0.433	,0.00)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rhide ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (6,8)
}

void MapGraphMtx::get_mid_R_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor, int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////左中 红中 -----------------------------------------------------------------------------------------

    //1.平地的上坡  TODO

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.837520-0.649,6.85846-0.433	,0.6)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(10.90243-0.649,7.60410-0.433	,0.6)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.18677-0.649,5.76981-0.433	,0  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(11.21860-0.649,5.02417-0.433	,0  ))); //0
        this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist ,ordinary, true);

    //2. 坡后小三角
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.602420-0.649,7.19423-0.433	,0.60)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(10.90243-0.649,7.60410-0.433	,0.60))); //0
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.837520-0.649,6.85846-0.433	,0.60))); //1
    this->push_back_MapVertex(vexs, temp_onegroup_points, 3, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //3. 正梯形高地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.602470-0.649,10.08419-0.433	,0.60))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(10.90246-0.649,9.674280-0.433	,0.60))); //4
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(10.90243-0.649,7.604100-0.433	,0.60))); //5
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.602420-0.649,7.194230-0.433	,0.60)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Bmist ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);


    //4. 斜梯形高地
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(11.59548-0.649,12.93047-0.433	,0.60))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(12.66037-0.649,12.18481-0.433	,0.60))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(10.90246-0.649,9.674280-0.433	,0.60)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.602470-0.649,10.08419-0.433	,0.60)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs,vexs.size()-2, vexs.size()-1,offset);

    //5. 下坡至0.2小三角
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(12.58992-0.649,14.34972-0.433	,0.20)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.65415-0.649,13.60405-0.433	,0.20)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.66037-0.649,12.18481-0.433	,0.60)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(11.59548-0.649,12.93047-0.433	,0.60)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); // (4,5)

    //6. 0.2小三角 TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(12.58992-0.649,14.34972-0.433	,0.20)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(14.29466-0.649,14.34913-0.433	,0.20)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(13.65415-0.649,13.60405-0.433	,0.20)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 3, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (5,6)

}

void MapGraphMtx::get_mid_B_placeConfig(std::vector<MapVertex> &vexs,PlaceColor placeColor,  int offset) {
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

////右中 蓝中 -----------------------------------------------------------------------------------------

    //1.平地的上坡  TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.45776-0.649,9.119780-0.433	,0.6)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(18.39284-0.649,8.374100-0.433	,0.6)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.10850-0.649,10.20844-0.433	,0  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(18.17341-0.649,10.95407-0.433	,0  ))); //0
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist ,ordinary, true);

    //2. 坡后小三角 （6，4，5）
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.69285-0.649,8.78402-0.433	,0.60))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(18.39284-0.649,8.37415-0.433	,0.60)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.45775-0.649,9.11978-0.433	,0.60))); //0
    this->push_back_MapVertex(vexs, temp_onegroup_points, 3, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);

    //3. 正梯形高地    （7，3，4，6）
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.69208-0.649,5.89405-0.433	,0.60))); //1
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(18.39280-0.649,6.30397-0.433	,0.60))); //4
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(18.39284-0.649,8.37415-0.433	,0.60))); //5
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.69285-0.649,8.78402-0.433	,0.60)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Rmist ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset);


    //4. 斜梯形高地    1，2，3，7
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(17.69978-0.649,3.04775-0.433	,0.60))); //7
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(16.63489-0.649,3.79345-0.433	,0.60))); //7
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(18.39280-0.649,6.30397-0.433	,0.60)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.69208-0.649,5.89405-0.433	,0.60)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs,vexs.size()-2, vexs.size()-1,offset);

    //5. 下坡至0.2小三角
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(16.70520-0.649,1.62909-0.433	,0.20)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(15.82818-0.649,2.37419-0.433	,0.20)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(16.63489-0.649,3.79345-0.433	,0.60)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(17.69978-0.649,3.04775-0.433	,0.60)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 4, placeColor, Clear ,ordinary, true);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); // (4,5)

    //6. 0.2小三角     TODO
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(16.70520-0.649,1.62909-0.433	,0.20)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(14.91789-0.649,1.62914-0.433	,0.20)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(15.82818-0.649,2.37419-0.433	,0.20)));
    this->push_back_MapVertex(vexs, temp_onegroup_points, 3, placeColor, Clear ,ordinary, false);

    this->creat_dir(vexs, vexs.size()-2, vexs.size()-1,offset); //  (5,6)

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
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,ordinary, false);
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
    this->push_back_MapVertex(vexs ,temp_onegroup_points, 4, placeColor, mist,ordinary, false);
    this->vexs.back().setMissType(90.0);
    this->creat_dir(vexs, vexs.size()-3, vexs.size()-1,offset); //  (5,7)

    //8.TODO:
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(13.97664,12.96484,0.0)));
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(13.6502,12.5720,0.0)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(14.4663,14.359,0.0)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.2133,14.2100,0.0)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.2133,12.960,0.0)));
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
    get_rt_B_placeConfig(this->vexs, B); //TODO: (2,4)
    get_mid_B_placeConfig(this->vexs, B);
    get_rb_B_placeConfig(this->vexs, B);
    get_lb_R_placeConfig(this->vexs, R);
    get_mid_R_placeConfig(this->vexs, R);
    get_lt_R_placeConfig(this->vexs, R);
//    get_midGround_R_placeConfig(this->vexs, R);
//    get_midGround_B_placeConfig(this->vexs, B);
//    get_behindGround_R_placeConfig(this->vexs, R);
//    get_behindGround_B_placeConfig(this->vexs, B);
    std::cout << "vexs.size() " <<vexs.size() << std::endl;
    // TODO：
    // print_AdjacencyMatrix();
}


