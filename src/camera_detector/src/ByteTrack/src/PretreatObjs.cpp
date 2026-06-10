#include "../include/PretreatObjs.h"
#include <rclcpp/logging.hpp>

PretreatObjs::PretreatObjs(OurPattern ourPattern,std::string config_path) {
    this->ourPattern = ourPattern;
    YAML::Node config = YAML::LoadFile(config_path);
    this->classWithoutCar = config["general"]["classWithoutCar"].as<int>();
    this->half_classWithoutCar = classWithoutCar/2;
}

void PretreatObjs::set_windmill_car(std::vector<int> windmill_car){
    this->windmill_car.assign(windmill_car.begin(), windmill_car.end());
    windmill_car_conf = sum_windmill_car_conf/windmill_car.size();
}

PretreatObjs::PretreatObjs(std::shared_ptr<SensorParam> MainCam_ptr,std::shared_ptr<SensorParam> SecCam_ptr, bool sec_is_left,std::string config_path){
    YAML::Node config = YAML::LoadFile(config_path);
    this->classWithoutCar = config["general"]["classWithoutCar"].as<int>();
    this->half_classWithoutCar = classWithoutCar/2;

    redLower=Scalar(10, 105, 105);
    redUpper=Scalar(30, 155, 255);
    blueLower=Scalar(93, 125, 125);
    blueUpper=Scalar(115, 255, 255);

    this->sec_is_left = sec_is_left;
    cv::FileStorage mats = cv::FileStorage(config_path, cv::FileStorage::READ);
    mats["Mat"]["H"] >> H;
    mats["Mat"]["H2"] >> H2;
    mats.release();
    cv::Mat org_K = (cv::Mat_<double>(3,3) << SecCam_ptr->fx,0.0,SecCam_ptr->img_w/2,
                                                        0.0,SecCam_ptr->fy,SecCam_ptr->img_h/2,
                                                        0.0, 0.0, 1.0 );

    cv::Mat goal_K = (cv::Mat_<double>(3,3) << MainCam_ptr->fx,0.0,MainCam_ptr->img_w/2,
                                                        0.0,MainCam_ptr->fy,MainCam_ptr->img_h/2,
                                                        0.0, 0.0, 1.0 );
    cv::Mat inv_org_K;
    cv::invert(org_K, inv_org_K);
    perspective_K = goal_K * inv_org_K;
    std::cout << "perspective_K: " << perspective_K << std::endl;

    img1_w = MainCam_ptr->img_w;
    img1_h = MainCam_ptr->img_h;
    img2_w = SecCam_ptr->img_w;
    img2_h = SecCam_ptr->img_h;

    if(sec_is_left){
        std::vector<cv::Point2f>corners(4);
        std::vector<cv::Point2f>corners2(4);
        std::vector<cv::Point2f>corners3(4);
        corners[0] = cv::Point(0, 0);
        corners[1] = cv::Point(0, sec2main_h);
        corners[2] = cv::Point(sec2main_w, sec2main_h);
        corners[3] = cv::Point(sec2main_w, 0);

        perspectiveTransform(corners, corners2, H);  //仿射变换对应端点

        std::cout << corners2[0].x << ", " << corners2[0].y << std::endl;
        std::cout << corners2[1].x << ", " << corners2[1].y << std::endl;

        this->tx = int(-std::min(corners2[0].x,std::min(corners2[3].x, corners2[1].x)));
        this->ty = int(-std::min(corners2[0].y,std::min(corners2[3].y, corners2[1].y)));

        this->main_translation = (cv::Mat_<double>(3,3) << 1.0, 0.0, tx,
                0.0, 1.0, ty,
                0.0, 0.0, 1.0 );
        this->sec_translation = main_translation*H;
        perspectiveTransform(corners, corners3, sec_translation);
        std::cout << corners3[0].x << ", " << corners3[0].y << std::endl;
        std::cout << corners3[1].x << ", " << corners3[1].y << std::endl;
        std::cout << corners3[2].x << ", " << corners3[2].y << std::endl;
        std::cout << corners3[3].x << ", " << corners3[3].y << std::endl;
        int w_img2 = std::max(corners2[2].x,std::max(corners2[3].x, corners2[1].x));
        int h_img2 = std::max(corners2[2].y,std::max(corners2[3].y, corners2[1].y));
        this->result_w = std::max(tx + img1_w, w_img2);
        this->result_h = std::max(ty + img1_h, h_img2);
    }else{
        std::vector<cv::Point2f>corners(4);
        std::vector<cv::Point2f>corners2(4);
        std::vector<cv::Point2f>corners3(4);
        corners[0] = cv::Point(0, 0);
        corners[1] = cv::Point(0, sec2main_h);
        corners[2] = cv::Point(sec2main_w, sec2main_h);
        corners[3] = cv::Point(sec2main_w, 0);

        perspectiveTransform(corners, corners2, H2);  //仿射变换对应端点

        this->tx = int(std::max(corners2[0].x,std::min(corners2[3].x, corners2[1].x)));
        this->ty = int(std::max(corners2[0].y,std::min(corners2[3].y, corners2[1].y)));

        this->sec_translation = main_translation*H2;
        perspectiveTransform(corners, corners3, sec_translation);
        int w_img2 = std::max(corners2[2].x,std::max(corners2[3].x, corners2[1].x));
        int h_img2 = std::max(corners2[2].y,std::max(corners2[3].y, corners2[1].y));
        this->result_w = std::max(tx + img1_w, w_img2);
        this->result_h = std::max(ty + img1_h, h_img2);
    }
}

TRTInferV1::DetectionObj PretreatObjs::objs2newMainObjs(TRTInferV1::DetectionObj objs, cv::Mat obj2Main){
    float cx = (objs.x1 + objs.x2)/2. *1.333,
          y2 = objs.y2  * 1.333,
          w  = (objs.x2 - objs.x1) * 1.333,
          h  = (objs.y2 - objs.y1) * 1.333;

    std::vector<cv::Point2f>corners(1);
    std::vector<cv::Point2f>corners1(1);
    std::vector<cv::Point2f>corners2(1);
    corners[0] = cv::Point(cx, y2);
    perspectiveTransform(corners, corners2, obj2Main);
    objs.x1 = corners2[0].x - w/2;
    objs.x2 = corners2[0].x + w/2;
    objs.y1 = corners2[0].y - h;
    objs.y2 = corners2[0].y;

    return objs;
}

TRTInferV1::DetectionObj PretreatObjs::objs2newMainObjs(interfaces::msg::Rect objs, cv::Mat obj2Main){
    float cx = (objs.x1 + objs.x2)/2. *1.333,
          y2 = objs.y2  * 1.333,
          w  = (objs.x2 - objs.x1) * 1.333,
          h  = (objs.y2 - objs.y1) * 1.333;

    TRTInferV1::DetectionObj res;

    std::vector<cv::Point2f>corners(1);
    std::vector<cv::Point2f>corners1(1);
    std::vector<cv::Point2f>corners2(1);
    corners[0] = cv::Point(cx, y2);
    perspectiveTransform(corners, corners2, obj2Main);
    res.x1 = corners2[0].x - w/2;
    res.x2 = corners2[0].x + w/2;
    res.y1 = corners2[0].y - h;
    res.y2 = corners2[0].y;
    res.confidence=0.9;

    return res;
}

/**
 * @brief 更新obj的conf_armor
 *
 * @param obj Car/Strack
 */
void PretreatObjs::update_classfy(int &temp_bestcls, float &conf_armor, Eigen::MatrixXd &car_armorConfMatrix){
    //获得car_armorConfMatrix中最大值的标签
    Eigen::MatrixXf::Index max_index;
    car_armorConfMatrix.row(0).maxCoeff(&max_index);
    conf_armor = car_armorConfMatrix(0,max_index);

    if(conf_armor < 5e-2){
        temp_bestcls = classWithoutCar ; //  classWithoutCar+1-1
    }
    else if(half_classWithoutCar == 7){
        int color_N = (max_index/half_classWithoutCar+1)*half_classWithoutCar-1;
        if( abs(conf_armor - car_armorConfMatrix(0,color_N)) < 1e-1){
            temp_bestcls = color_N;
        }else{
            temp_bestcls = max_index;
            car_armorConfMatrix(0,half_classWithoutCar-1) = 0;
            car_armorConfMatrix(0,half_classWithoutCar*2-1) = 0;
        }
    }
    else if(half_classWithoutCar == 6){
        temp_bestcls = max_index;
    }else if(half_classWithoutCar == 5){
        temp_bestcls = max_index;
    }else{
        std::cout << "Please set the update_classfy without T by yourself" << std::endl;
    }
}

/**
 * @brief 更新obj的conf_armor,cls,对obj进行分类（3类，红色，蓝色，unknown）
 *
 * @param obj Car/Strack
 */
template<typename T>
void PretreatObjs::update_classfy(T &obj, int num, std::vector<T> &outRedObjs, std::vector<T> &outBlueObjs,std::vector<T> &outRestObjs){
    num += this->classWithoutCar*2;
    //获得ws_armorConfMatrix中最大值的标签（即armorConf最大值对应的标签）
    Eigen::MatrixXf::Index max_index;
    obj.ws_armorConfMatrix.row(0).maxCoeff(&max_index);
    obj.conf_armor = obj.ws_armorConfMatrix(0,max_index);

    if(obj.cls < -1e-6){ //obj.ws_armorConfMatrix(max_index) == 0)
        obj.cls = num + 300;num ++;    //300+ unknown
        outRestObjs.push_back(obj);
    }
    else if(max_index < half_classWithoutCar ){
        if(max_index == 6){
            obj.cls = num + 100;num ++;//100+ B
        }
        obj.ws_armorConfMatrix_BR.block(0,0,1,6) = obj.ws_armorConfMatrix.block(0,0,1,6);
        outBlueObjs.push_back(obj);
    }
    else if(max_index < 2*half_classWithoutCar){
        if(max_index == 13){
            obj.cls = num + 200;num ++;//200+ R
        }
        obj.ws_armorConfMatrix_BR.block(0,0,1,6) = obj.ws_armorConfMatrix.block(0,6,1,6);
        outRedObjs.push_back(obj);
    }
}

void PretreatObjs::set_confs_by_locate3D(double &windmill_car_conf, double &startupArea_car_conf){
    windmill_car_conf = this->windmill_car_conf;
    startupArea_car_conf = this->startupArea_car_conf;
}

bool PretreatObjs::get_Armors_w_conf_Double_net(STrack &car, vector<TRTInferV1::DetectionObj> armors,cv::Mat &car_img) {
    auto trackStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    double w_getAllArea = 0.0;  // 得到所有在car里面装甲版的总面积
    double carY_downLine = car.tlwh[3] * p_carY_downLine;  // 装甲板3/4处（沿y(v)）//？？
    double carY_upLine = car.tlwh[3] * p_carY_upLine;      // 装甲板1/3处（沿y(v)）//？？
    std::vector<double> tempBest_w_armorConf(this->classWithoutCar,0.0);
    Eigen::MatrixXd car_armorConfMatrix = Eigen::MatrixXd::Zero(1,12); ////TODO:
    for(auto armor : armors){
        //找装甲板的中点
        float armorLocate2D[2] = {(armor.x1+armor.x2)/2, (armor.y1+armor.y2)/2};
        //area
        double w_armorArea = (armor.y2-armor.y1) * (armor.x2-armor.x1);
        w_getAllArea += w_armorArea;
        // 1/3y - 3/4y
        // double w_armorY = 1.0 - min_(abs((armorLocate2D[1] - carY_downLine)/(carY_upLine - carY_downLine)),1.0);//纯相机识别用
        // 1/2y
        double w_armorY = 1.0 - min_(abs((armorLocate2D[1] - car.tlwh[3]*0.5)/(car.tlwh[3]*0.5)),1.0);//点云第一层网络用
        double w_armorConf = w_armorArea * w_armorY * armor.confidence;
        if (armor.classId==4||armor.classId==10)
            w_getAllArea -= w_armorArea;
        car_armorConfMatrix(0,armor.classId) += w_armorConf;
    }
    int temp_bestcls = -1;  float conf_armor = 0.0;
    Eigen::MatrixXd car_ConfMatrix = Eigen::MatrixXd::Zero(1,groupNum*half_classWithoutCar);
    if(w_getAllArea>1e-6) {
        car_armorConfMatrix /= w_getAllArea; // (conf1*S1*y1 +...+confn*Sn*yn)/(S1+...+Sn)
    }
    set_confs_by_locate3D(car.windmill_car_conf, car.startupArea_car_conf);
    update_classfy(temp_bestcls, conf_armor,car_armorConfMatrix);
    double conf=0.0;
    int index =-1;
    for (int i=0;i<armors.size();i++) {
        if (armors[i].classId==temp_bestcls&&armors[i].confidence>conf) {
            conf=armors[i].confidence;
            index=i;
        }
        cv::Rect r = cv::Rect(armors[i].x1, armors[i].y1, armors[i].x2 - armors[i].x1, armors[i].y2 - armors[i].y1);
        if (!check_color(car_img(r),armors[i].classId))
            return true;
    }
    if (half_classWithoutCar==5) {
        if (temp_bestcls>=5&&temp_bestcls<=9) {
            temp_bestcls-=1;
        }else if (temp_bestcls==11) {
            temp_bestcls-=2;
        }
        for (int i=0;i<12;i++) {
            if (i<=3)
                car_ConfMatrix(0,i) = car_armorConfMatrix(0,i);
            else if (i>=5&&i<=9)
                car_ConfMatrix(0,i-1) = car_armorConfMatrix(0,i);
            else if (i==11)
                car_ConfMatrix(0,i-2) = car_armorConfMatrix(0,i);
        }
    }
    car.init_track(temp_bestcls, conf_armor, car_ConfMatrix);
    auto trackEndTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//    std::cout << "get_Armors_w_conf_Double_net: " << 1000./(trackEndTime - trackStartTime) << std::endl;
    return false;
}

bool PretreatObjs::check_color(cv::Mat armor,int cls) {
    cv::Mat hsvImage;
    int rl_h1=rl_h,rl_s1=rl_s,rl_v1=rl_v,rh_h1=rh_h,rh_s1=rh_s,rh_v1=rh_v;
    rl_h=rl_h1;rl_s=rl_s1;rl_v=rl_v1;rh_h=rh_h1;rh_s=rh_s1;rh_v=rh_v1;
    cvtColor(armor, hsvImage, COLOR_BGR2HSV);
    Mat redMask,blueMask;
    // redLower= Scalar(rl_h, rl_s, rl_v);
    // redUpper= Scalar(rh_h, rh_s, rh_v);
    // blueLower= Scalar(bl_h, bl_s, bl_v);
    // blueUpper = Scalar(bh_h, bh_s, bh_v);
    inRange(hsvImage, redLower, redUpper, redMask);//偏黄
    inRange(hsvImage, blueLower, blueUpper, blueMask);

    // 寻找轮廓
    vector<vector<Point>> contoursRed, contoursBlue;
    findContours(redMask, contoursRed, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    findContours(blueMask, contoursBlue, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    bool red=false,blue=false;
    if (contoursRed.size()!=0)
        red=true;
    if (contoursBlue.size()!=0)
        blue=true;
    if (red||blue) {
        return true;
    }else{
        return false;
    }
}