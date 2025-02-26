#include "../include/Place.h"

void Place::get_predict_2d(const cv::Mat T, const double fx, const double fy, const double cx, const double cy){
    Eigen::Matrix<double,4,4> Rt;
    cv::cv2eigen(T.inv(),Rt);
    for (int i = 0; i < all_point_3d_number.size(); i++) {
        std::array<cv::Point2f,10> temp_onegroup_predict_2d;
        for(int j=0; j < all_point_3d_number[i];j++){
            Eigen::Vector4d temp_reality_3d(all_point_reality_3d[i][j][0],all_point_reality_3d[i][j][1],all_point_reality_3d[i][j][2], 1);
            Eigen::Vector4d pc = Rt * temp_reality_3d;
            double inv_z = 1.0 / pc[2];
            double inv_z2 = 1.0 / (inv_z * inv_z);
            Eigen::Vector2d predict(fx * pc[0] / pc[2] + cx,fy * pc[1] / pc[2] + cy );
            temp_onegroup_predict_2d.at(j) = cv::Point2d((int)predict[0],(int)predict[1]);
        }
        all_point_predict_2d.push_back(temp_onegroup_predict_2d);
    }
}


void Place::get_roughH_config(){
    for(int i=0;i<all_point_reality_3d.size();i++){

        std::array<double,3> temp_abc_3d;

        //算出3d平面的表达 //TODO：4个点的顺序必须为 h1,h1,h2,h2
        Eigen::Vector3d p1_3d,p2_3d,p3_3d;
        p1_3d << all_point_reality_3d[i].at(0);
        p2_3d << all_point_reality_3d[i].at(1);
        p3_3d << all_point_reality_3d[i].at(2);
        auto n = (p3_3d-p1_3d).cross(p2_3d-p1_3d);

        Eigen::Matrix<double,2,2> temp_matrix_3d22 ,temp_matrix_2d22;
        Eigen::Matrix<double,2,1> temp_matrix_2d;


        if(n[2]==0){
            temp_matrix_2d22 << 1,0,0,1;
            temp_matrix_3d22 << 1,0,0,1;

            temp_abc_3d.at(0) = 0 ;
            temp_abc_3d.at(1) = 0 ;
            temp_abc_3d.at(2) = double (p3_3d[2]);
            break;

        }
        else{
            temp_abc_3d.at(0) = -(double (n[0]))/(double (n[2]));
            temp_abc_3d.at(1) = -(double (n[1]))/(double (n[2]));
            temp_abc_3d.at(2) = double (p3_3d[2]) - temp_abc_3d.at(0)*p3_3d[0] - temp_abc_3d.at(1)*p3_3d[1];


            //2d点的投射
            temp_matrix_3d22 << (all_point_reality_3d[i].at(0)[0]- all_point_reality_3d[i].at(1)[0]) ,( all_point_reality_3d[i].at(2)[0]- all_point_reality_3d[i].at(1)[0]),
                    (all_point_reality_3d[i].at(0)[1]- all_point_reality_3d[i].at(1)[1]) , ( all_point_reality_3d[i].at(2)[1]- all_point_reality_3d[i].at(1)[1]);

            temp_matrix_2d22 << ( all_point_predict_2d[i].at(0).x- all_point_predict_2d[i].at(1).x) ,( all_point_predict_2d[i].at(2).x- all_point_predict_2d[i].at(1).x),
                    ( all_point_predict_2d[i].at(0).y- all_point_predict_2d[i].at(1).y) ,( all_point_predict_2d[i].at(2).y- all_point_predict_2d[i].at(1).y);

            temp_matrix_2d <<  all_point_reality_3d[i].at(1)[0], all_point_reality_3d[i].at(1)[1];

//            temp_matrix_2d22 = temp_matrix_3d22 * temp_matrix_2d22.inverse();
        }

        getH_abc_3d.push_back(temp_abc_3d);
        matrix_change2.push_back(temp_matrix_2d22);
        matrix_change3.push_back(temp_matrix_3d22);
        matrix_2d.push_back(temp_matrix_2d);
    }

}

Place::Place(){
//     //TODO: 添加PNP所需的3D点
//     // pts_pnp_3d.emplace_back(cv::Point3d(9.47097-0.649, 9.06324-0.433, 0.615));
//     // pts_pnp_3d.push_back((Eigen::Matrix<double, 3, 1>(10.95, 8.63, 0.40)));
//      //pts_pnp_3d  number=5
//     pts_pnp_3d.emplace_back(cv::Point3d(3.70, 3.65, 0.765));
//     pts_pnp_3d.emplace_back(cv::Point3d(5.76, 3.65, 0.765));
//     pts_pnp_3d.emplace_back(cv::Point3d(8.67, 5.10, 0.108));
//     pts_pnp_3d.emplace_back(cv::Point3d(8.67, 8.63, 0.53));
//     pts_pnp_3d.emplace_back(cv::Point3d(3.51, 8.86, 0.135));
//     pts_pnp_3d.emplace_back(cv::Point3d(3.28, 4.97, 0.765));

// //pts_reality_3d
    std::array<Eigen::Matrix<double, 3, 1>,10> temp_onegroup_points;

    // //pts_reality_3d
    // //a.飞坡
    // all_point_3d_number.push_back(4);
    // temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(10.95, 8.63, 0.40)));
    // temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.94, 8.63, 0.40)));
    // temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.94, 3.82, 0.532)));
    // temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(10.94, 3.82, 0.53)));
    // all_point_reality_3d.push_back(temp_onegroup_points);

    // //b.高地
    // all_point_3d_number.push_back(4);
    // temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.93, 6.68, 1.10)));
    // temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(8.68, 6.68, 1.10)));
    // temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(8.67, 5.10, 1.10)));
    // temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.93, 5.11, 1.10)));
    // all_point_reality_3d.push_back(temp_onegroup_points);

    // //c.斜坡
    // all_point_3d_number.push_back(4);
    // temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.51, 10.165, 0.00)));
    // temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(1.63, 10.165, 0.40)));
    // temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(1.63, 8.86, 0.40)));
    // temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.51, 8.86, 0.00)));
    // all_point_reality_3d.push_back(temp_onegroup_points);

    // //d.平台
    // all_point_3d_number.push_back(6);
    // temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(6.51, 4.70, 0.60)));
    // temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(6.18, 4.97, 0.60)));
    // temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(3.28, 4.97, 0.60)));
    // temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(2.575, 4.466, 0.60)));
    // temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(3.70, 3.65, 0.60)));
    // temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(5.76, 3.65, 0.60)));
    // all_point_reality_3d.push_back(temp_onegroup_points);

        pts_pnp_3d.emplace_back(cv::Point3d(9.47097-0.649, 9.06324-0.433, 0.615));
        pts_pnp_3d.emplace_back(cv::Point3d(9.47098-0.649, 9.72324-0.433, 0.615));
//        pts_pnp_3d.emplace_back(cv::Point3d(17.30477-0.649, 13.06757-0.433, 1.4724));
        pts_pnp_3d.emplace_back(cv::Point3d(22.13425-0.649,15.48899-0.433, 0.2));
        pts_pnp_3d.emplace_back(cv::Point3d(25.87255-0.649, 15.48858-0.433, 0.4));
//        pts_pnp_3d.emplace_back(cv::Point3d(26.67680-0.649, 7.98879-0.433, 1.02094));
        pts_pnp_3d.emplace_back(cv::Point3d(24.17254-0.649, 2.10967-0.433, 0.4));
        pts_pnp_3d.emplace_back(cv::Point3d(10.90243-0.649, 7.60410-0.433, 0.6));
    
    // pts_pnp_3d.emplace_back(cv::Point3d(10.46, 4.54, 0.0));
    // pts_pnp_3d.emplace_back(cv::Point3d(15.92, 11.6, .70));
    // pts_pnp_3d.emplace_back(cv::Point3d(22.45, 15.0, 0.2));
    // pts_pnp_3d.emplace_back(cv::Point3d(16.77, 12.57, 1.54));
    // pts_pnp_3d.emplace_back(cv::Point3d(26.20, 7.5, 1.08));
    // pts_pnp_3d.emplace_back(cv::Point3d(11.89, 4.94, 0.70));


////场地围挡在红方补给站附近的交点为坐标原点，沿场地长边向蓝方为 X 轴正方向，沿场地短边向红方停机坪为 Y 轴正方向
// 左上，顺时针


//左下
    //4.高地      1
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(8.36755-0.649,3.02923-0.433	,0.85)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.93594-0.649,3.02921-0.433	,0.85)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.93592-0.649,1.77921-0.433	,0.85)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.36755-0.649,1.77923-0.433	,0.85)));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //3.高地上坡
//    all_point_3d_number.push_back(4);
//    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.42256-0.649,2.11079-0.433	,0.4)));
//    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.12269-0.649,2.07211-0.433	,0.4)));
//    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.12269-0.649,4.61001-0.433	,0.4)));
//    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(5.30273-0.649,4.73386-0.433	,0.4)));
//    all_point_reality_3d.push_back(temp_onegroup_points);


    //2.平地      2
    all_point_3d_number.push_back(5);
//    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.42256-0.649,2.11079-0.433	,0.4)));
//    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.12269-0.649,2.07211-0.433	,0.4)));

    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(5.12269-0.649,3.17923-0.433	,0.4)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(5.12269-0.649,4.61001-0.433	,0.4)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.30273-0.649,4.73386-0.433	,0.4)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(7.36948-0.649,4.77172-0.433	,0.4)));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(8.48452-0.649,3.17923-0.433	,0.4)));
//    temp_onegroup_points.at(5) = ((Eigen::Matrix<double, 3, 1>(6.43811-0.649,3.14137-0.433	,0.4)));
//    temp_onegroup_points.at(7) = ((Eigen::Matrix<double, 3, 1>(6.41834-0.649,0.48927-0.433	,0.4)));
//    temp_onegroup_points.at(8) = ((Eigen::Matrix<double, 3, 1>(3.42256-0.649,0.58967-0.433	,0.4)));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //1.上坡      4
//    all_point_3d_number.push_back(4);  // 点的个数
//    temp_onegroup_points.at(0) << ((Eigen::Matrix<double, 3, 1>{3.42256-0.649,3.99189-0.433	,0   }));
//    temp_onegroup_points[1] =     ((Eigen::Matrix<double, 3, 1>(4.91978-0.649,3.99187-0.433	,0   )));
//    temp_onegroup_points.at(2) =  ((Eigen::Matrix<double, 3, 1>(4.90874-0.649,2.11001-0.433	,0.4 )));
//    temp_onegroup_points[3] <<    ((Eigen::Matrix<double, 3, 1>(3.42256-0.649,2.1101-0.433	,0.4 )));
//    all_point_reality_3d.push_back(temp_onegroup_points);



    //5.平地下坡    3
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.16104-0.649,1.62926-0.433	,0.2 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.16102-0.649,0.48926-0.433	,0.2 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(6.41834-0.649,0.48927-0.433	,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(6.41836-0.649,1.62927-0.433	,0.4 )));
    all_point_reality_3d.push_back(temp_onegroup_points);
    // place.bluefly[0]= all_point_3d_number.size() -1;

    //6.平地平坡    4
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.16104	-0.649 ,1.62926	-0.433,0.2   )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(12.13563 -0.649,1.48917	-0.433    ,0.2   )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.13563 -0.649,0.62917	-0.433    ,0.2   )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(7.16102	-0.649 ,0.48926	-0.433,0.2   )));
    all_point_reality_3d.push_back(temp_onegroup_points);
//    place.flynumber = all_point_3d_number.size();
    // place.bluefly[1]= all_point_3d_number.size() -1;

    //7.飞坡上坡
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(13.17833-0.649,1.48917-0.433	,0.55  )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.17833-0.649,0.62915-0.433	,0.55  )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.13563-0.649,0.62917-0.433	,0.2   )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(12.13563-0.649,1.48917-0.433	,0.2   )));
    all_point_reality_3d.push_back(temp_onegroup_points);
    // place.bluefly[2]= all_point_3d_number.size() -1;

    //8.平地陡坡    5
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(7.97161-0.649,4.88812	-0.433,0   )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(8.65988-0.649,3.90513	-0.433,0   )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(8.48452-0.649,3.17923	-0.433,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(7.36948-0.649,4.77172	-0.433,0.4 )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //4.高地         6
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.92753	-0.649,12.94866-0.433,0.85)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(19.35914	-0.649,12.94866-0.433,0.85)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.35914	-0.649,14.19866-0.433,0.85)));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(20.92753	-0.649,14.19866-0.433,0.85)));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //3.高地上坡      24
//    all_point_3d_number.push_back(4);
//    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.92753	-0.649,12.94866-0.433,0.85)));
//    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(20.92753	-0.649,14.19866-0.433,0.85)));
//    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(22.87693	-0.649,14.19899-0.433,0.4 )));
//    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(22.87693	-0.649,12.94899-0.433,0.4 )));
//    all_point_reality_3d.push_back(temp_onegroup_points);


    //2.平地         7
    all_point_3d_number.push_back(5);
//    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.87253	-0.649,13.86786-0.433,0.4 )));
//    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(24.17248	-0.649,13.86789-0.433,0.4 )));
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(24.17248	-0.649,12.79867-0.433,0.4 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(24.17248	-0.649,11.35614-0.433,0.4 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(24.02248	-0.649,11.20614-0.433,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(21.92557	-0.649,11.20618-0.433,0.4 )));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(20.81054	-0.649,12.79867-0.433,0.4 )));
//    temp_onegroup_points.at(6) = ((Eigen::Matrix<double, 3, 1>(22.87667	-0.649,12.79863-0.433,0.4 )));
//    temp_onegroup_points.at(7) = ((Eigen::Matrix<double, 3, 1>(22.87693	-0.649,15.48898-0.433,0.4 )));
//    temp_onegroup_points.at(8) = ((Eigen::Matrix<double, 3, 1>(25.87255	-0.649,15.48858-0.433,0.4 )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //1.上坡         26
//    all_point_3d_number.push_back(4);  // 点的个数
//    temp_onegroup_points.at(0) << ((Eigen::Matrix<double, 3, 1>{25.87249-0.649	,11.98601-0.433	,0  }));
//    temp_onegroup_points[1] =     ((Eigen::Matrix<double, 3, 1>(24.57527-0.649	,11.98603-0.433	,0  )));
//    temp_onegroup_points.at(2) =  ((Eigen::Matrix<double, 3, 1>(24.38631-0.649	,13.86789-0.433	,0.4)));
//    temp_onegroup_points[3] <<    ((Eigen::Matrix<double, 3, 1>(25.87253-0.649	,13.86786-0.433	,0.4)));
//    all_point_reality_3d.push_back(temp_onegroup_points);

//左上

    //2.平地1        8
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.42247-0.649,15.48934 -0.433  ,0.4 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.57003-0.649,15.48921 -0.433  ,0.4 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.57003-0.649,14.34921 -0.433  ,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.42247-0.649,13.86932 -0.433  ,0.4 )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //2.平地2         9
    all_point_3d_number.push_back(5);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.57003-0.649,14.34921 -0.433  ,0.4 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(7.3696 -0.649,11.20678 -0.433  ,0.4 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(5.30273-0.649,11.20678 -0.433  ,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(5.12269-0.649,11.38686 -0.433  ,0.4 )));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(5.12274-0.649,13.86857 -0.433  ,0.4 )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //1.上坡
//    all_point_3d_number.push_back(4);
//    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(3.42247-0.649,13.86932 -0.433 ,0.4  )));
//    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(4.90896-0.649,13.86858 -0.433 ,0.4  )));
//    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(4.80853-0.649,11.98675 -0.433 ,0    )));
//    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(3.42247-0.649,11.98675 -0.433 ,0    )));
//    all_point_reality_3d.push_back(temp_onegroup_points);

    //3.平地下坡        10
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.9464-0.649,15.48921 -0.433 ,0.2  )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(9.9464-0.649,14.34921 -0.433 ,0.2  )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.57003-0.649,14.34921-0.433  ,0.4  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.57003-0.649,15.48921-0.433  ,0.4  )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //4.低地          11
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.9464	-0.649,15.48921-0.433	,0.2  )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(15.46695	-0.649,15.48911-0.433	,0.2  )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(15.46693	-0.649,14.34911-0.433	,0.2  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.9464	-0.649,14.34921-0.433	,0.2  )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //5.低地下坡        12
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.9464	-0.649	,14.34921-0.433	,0.2 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(12.58992	-0.649 ,14.34972-0.433	,0.2 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(11.83377	-0.649 ,13.10547-0.433	,0   )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.19025	-0.649 ,13.10547-0.433	,0   )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //6.平地陡坡？       22
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(8.9755	-0.649	,12.52386-0.433	,0    )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(8.28719	-0.649 ,11.54089-0.433	,0    )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(7.82601	-0.649 ,11.86358-0.433	,0.4  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(8.51678	-0.649 ,12.84507-0.433	,0.4  )));
    all_point_reality_3d.push_back(temp_onegroup_points);

//右上

    //5.平地下坡      13
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(22.13423	-0.649,14.34899-0.433,0.2 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(22.13425	-0.649,15.48899-0.433,0.2 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(22.87693	-0.649,15.48898-0.433,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(22.87693	-0.649,14.34899-0.433,0.4 )));
    all_point_reality_3d.push_back(temp_onegroup_points);
    // place.redfly[0]= all_point_3d_number.size() -1;

    //6.平地平坡      14
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(22.13423	-0.649,14.34899-0.433,0.2 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(17.28173	-0.649,14.48908-0.433,0.2 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.26175	-0.649,15.34908-0.433,0.2 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(22.13425	-0.649,15.48899-0.433,0.2 )));
    all_point_reality_3d.push_back(temp_onegroup_points);
    // place.redfly[1]= all_point_3d_number.size() -1;
//    place.flynumber = all_point_3d_number.size();

    //7.飞坡上坡      27
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(16.11695-0.649	,14.4891-0.433	,0.55)));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(16.11695-0.649	,15.3491-0.433	,0.55)));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.26175-0.649	,15.34908-0.433	,0.2 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(17.28173-0.649	,14.48908-0.433	,0.2 )));
    all_point_reality_3d.push_back(temp_onegroup_points);
    // place.redfly[2]= all_point_3d_number.size() -1;

    //8.平地陡坡      15
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(21.32664-0.649,11.09169-0.433,0    )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(20.63591-0.649,12.07296-0.433,0    )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.09391-0.649,12.39396-0.433,0.4  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(21.78464-0.649,11.41269-0.433,0.4  )));
    all_point_reality_3d.push_back(temp_onegroup_points);

//右下

    //2.平地 1         16
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.87277-0.649,2.10964-0.433,0.4 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(25.87274-0.649,0.48892-0.433,0.4 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.72524-0.649,0.48903-0.433,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.72524-0.649,1.62903-0.433,0.4 )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //2.平地 2        17
    all_point_3d_number.push_back(5);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.72524-0.649,1.62903-0.433,0.4 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(21.92567-0.649,4.77146-0.433,0.4 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(24.02258-0.649,4.77142-0.433,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(24.17258-0.649,4.62142-0.433,0.4 )));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(24.17254-0.649,2.10967-0.433,0.4 )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //1.上坡          32
//    all_point_3d_number.push_back(4);
//    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(25.87277	-0.649    ,2.10964 -0.433   ,0.4 )));
//    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(24.38632	-0.649    ,2.10967 -0.433   ,0.4 )));
//    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(24.547	-0.649	  ,3.9915-0.433	,0   )));
//    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(25.8728	-0.649    ,3.9915-0.433		,0   )));
//    all_point_reality_3d.push_back(temp_onegroup_points);

    //3.平地下坡       18
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.34507-0.649,0.48904-0.433,0.2 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(19.34507-0.649,1.62439-0.433,0.2 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.72524-0.649,1.62903-0.433,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.72524-0.649,0.48903-0.433,0.4 )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //4.低地           19
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.34507-0.649	,0.48904-0.433	,0.2  )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.82731-0.649	,0.48914-0.433	,0.2  )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(13.827-0.649	    ,1.62914-0.433	,0.2  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.34507-0.649	,1.62439-0.433	,0.2  )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //5.低地下坡         20
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.34507-0.649,1.62439-0.433	,0.2 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(16.8882-0.649,1.62909-0.433	,0.2 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.68912-0.649,2.77282-0.433	,0   )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(20.04322-0.649,2.77282-0.433	,0   )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //6.平地陡坡？       34
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(20.31977	-0.649,3.45439-0.433,0   )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(21.00808	-0.649,4.43736-0.433,0   )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(21.4668	-0.649,4.11615-0.433,0.4 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(20.78095	-0.649,3.13145-0.433,0.4 )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //左中
    //1.低地     1
    all_point_3d_number.push_back(3);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(12.58992-0.649	,14.34972-0.433,0.2 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(14.29466-0.649	,14.34913-0.433,0.2 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(13.65415-0.649	,13.60405-0.433,0.2 )));
    all_point_reality_3d.push_back(temp_onegroup_points);
//    if(place.selfColor == "blue"){
        //2.上坡      2
        all_point_3d_number.push_back(4);
        temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(12.58992-0.649,14.34972-0.433,0.2  )));
        temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(13.65415-0.649,13.60405-0.433,0.2  )));
        temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.66037-0.649,12.18481-0.433,0.6  )));
        temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(11.59548-0.649,12.93047-0.433,0.6  )));
        all_point_reality_3d.push_back(temp_onegroup_points);

        //4.下坡      5
        all_point_3d_number.push_back(4);
        temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.83752	-0.649   ,6.85846-0.433	,0.6   )));
        temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(10.90243	-0.649   ,7.6041-0.433	,0.6   )));
        temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.18677	-0.649   ,5.76981-0.433	,0     )));
        temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(11.2186	-0.649   ,5.02417-0.433	,0     )));
        all_point_reality_3d.push_back(temp_onegroup_points);
//    }

    //3.高地1        8
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.69208-0.649	,5.89405-0.433 ,0.6 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(17.69978-0.649	,3.04775-0.433 ,0.6 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(16.63489-0.649	,3.79345-0.433 ,0.6 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(18.3928-0.649	,6.30397-0.433 ,0.6 )));
    all_point_reality_3d.push_back(temp_onegroup_points);
    //3.高地2        9
    all_point_3d_number.push_back(5);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(18.3928-0.649	,6.30397-0.433 ,0.6 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(18.39284-0.649	,8.37415-0.433 ,0.6 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(19.45775-0.649	,9.11978-0.433 ,0.6 )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(19.69285-0.649	,8.78402-0.433 ,0.6 )));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(19.69208-0.649	,5.89405-0.433 ,0.6 )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //3.高地1      3
    all_point_3d_number.push_back(4);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(9.60247-0.649	 ,10.08419-0.433	,0.6  )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(11.59548-0.649	 ,12.93047-0.433	,0.6  )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(12.66037-0.649	 ,12.18481-0.433	,0.6  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(10.90246-0.649	 ,9.67428 -0.433	,0.6  )));
    all_point_reality_3d.push_back(temp_onegroup_points);

    //3.高地2      4
    all_point_3d_number.push_back(5);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(10.90246-0.649	 ,9.67428 -0.433	,0.6  )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(10.90243-0.649	 ,7.6041-0.433		,0.6  )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(9.83752-0.649	 ,6.85846-0.433	    ,0.6  )));
    temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(9.60242-0.649	 ,7.19423-0.433	    ,0.6  )));
    temp_onegroup_points.at(4) = ((Eigen::Matrix<double, 3, 1>(9.60247-0.649	 ,10.08419-0.433	,0.6  )));
    all_point_reality_3d.push_back(temp_onegroup_points);

//右下，顺时针
//右中
    //1.低地        6
    all_point_3d_number.push_back(3);
    temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(16.7052	-0.649,1.62909 -0.433,0.2 )));
    temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(14.91789	-0.649,1.62914 -0.433,0.2 )));
    temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(15.82818	-0.649,2.37419 -0.433,0.2 )));
    all_point_reality_3d.push_back(temp_onegroup_points);

//    if(place.selfColor == "red"){
        //2.上坡        7
        all_point_3d_number.push_back(4);
        temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(16.7052-0.649	,1.62909-0.433	,0.2  )));
        temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(15.82818-0.649	,2.37419-0.433	,0.2  )));
        temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(16.63489-0.649	,3.79345-0.433	,0.6  )));
        temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(17.69978-0.649	,3.04775-0.433	,0.6  )));
        all_point_reality_3d.push_back(temp_onegroup_points);

        //4.下坡        10
        all_point_3d_number.push_back(4);
        temp_onegroup_points.at(0) = ((Eigen::Matrix<double, 3, 1>(19.45776	-0.649,9.11978	-0.433,0.6  )));
        temp_onegroup_points.at(1) = ((Eigen::Matrix<double, 3, 1>(18.39284	-0.649,8.37415	-0.433,0.6  )));
        temp_onegroup_points.at(2) = ((Eigen::Matrix<double, 3, 1>(17.1085	-0.649,10.20844	-0.433,0    )));
        temp_onegroup_points.at(3) = ((Eigen::Matrix<double, 3, 1>(18.17341	-0.649,10.95407	-0.433,0    )));
        all_point_reality_3d.push_back(temp_onegroup_points);
//    }
}