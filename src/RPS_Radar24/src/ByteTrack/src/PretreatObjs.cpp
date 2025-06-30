//
// Created by plusseven on 23-12-20.
//

#include "../include/PretreatObjs.h"

#include <rclcpp/logging.hpp>

PretreatObjs::PretreatObjs(OurPattern ourPattern) {
    this->ourPattern = ourPattern;
    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    this->classWithoutCar = config["net"]["classWithoutCar"].as<int>();
    this->half_classWithoutCar = classWithoutCar/2;
    this->isBR = config["pretreatObjs"]["isBR"].as<bool>();
//    this->isGuess = config["pretreatObjs"]["isGuess"].as<bool>();
    this->maxSize = config["pretreatObjs"]["maxSize"].as<int>();


//    std::cout << "isGuess   " << this->isGuess << std::endl;
//    getW_of_armorConfs(maxSize);
}

void PretreatObjs::set_windmill_car(std::vector<int> windmill_car){
    this->windmill_car.assign(windmill_car.begin(), windmill_car.end());
    windmill_car_conf = sum_windmill_car_conf/windmill_car.size();
}


PretreatObjs::PretreatObjs(std::shared_ptr<SensorParam> MainCam_ptr,std::shared_ptr<SensorParam> SecCam_ptr, bool sec_is_left){
    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    this->classWithoutCar = config["net"]["classWithoutCar"].as<int>();
    this->half_classWithoutCar = classWithoutCar/2;
    this->isBR = config["pretreatObjs"]["isBR"].as<bool>();
//    this->isGuess = config["pretreatObjs"]["isGuess"].as<bool>();
    this->maxSize = config["pretreatObjs"]["maxSize"].as<int>();

    redLower=Scalar(10, 105, 105);
    redUpper=Scalar(30, 155, 255);
    blueLower=Scalar(93, 125, 125);
    blueUpper=Scalar(115, 255, 255);


    //2cam
    this->sec_is_left = sec_is_left;
    cv::FileStorage mats = cv::FileStorage(STITCH_CONFIC_PATH, cv::FileStorage::READ);
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

//    //计算仿射变换后的四个端点
//    std::vector<cv::Point2f>corners(4);
//    std::vector<cv::Point2f>corners2(4);
//    corners[0] = cv::Point(0, 0);
//    corners[1] = cv::Point(0, img1_h);
//    corners[2] = cv::Point(img1_w, img1_h);
//    corners[3] = cv::Point(img1_w, 0);

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
        //imwrite("temp.jpg", stitchedImage);

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
//        warpPerspective(img2, stitchedImage, translation*H, cv::Size(sec2main_w + img1_w, mRows));
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
//        warpPerspective(img2, stitchedImage, translation*H, cv::Size(sec2main_w + img1_w, mRows));
        int w_img2 = std::max(corners2[2].x,std::max(corners2[3].x, corners2[1].x));
        int h_img2 = std::max(corners2[2].y,std::max(corners2[3].y, corners2[1].y));
        this->result_w = std::max(tx + img1_w, w_img2);
        this->result_h = std::max(ty + img1_h, h_img2);
    }

}

TRTInferV1::DetectionObj PretreatObjs::allObjs2newMainObjs(
    TRTInferV1::DetectionObj mainObjs, TRTInferV1::DetectionObj secObjs,cv::Mat &img1, cv::Mat &img2)
{
    warpPerspective(img2, img2, perspective_K, cv::Size(sec2main_w, sec2main_h));
    cv::Mat stitchedImage;  //定义仿射变换后的图像(也是拼接结果图像)
    cv::Mat stitchedImage2;  //定义仿射变换后的图像(也是拼接结果图像)
    int mRows = sec2main_h;
    if (img1_h > sec2main_h)
    {
        mRows = img1_h;
    }

    int count = 0;
    if(sec_is_left){
        std::cout << "img1 should be left" << std::endl;

        stitchedImage = cv::Mat::zeros(sec2main_w + img1_w, mRows, CV_8UC3);
//        warpPerspective(img2, stitchedImage, H, cv::Size(sec2main_w + img1_w, mRows));

        warpPerspective(img2, stitchedImage, sec_translation,
                        cv::Size(result_w ,result_h));

        cv::namedWindow("temp", cv::WINDOW_NORMAL);
        imshow("temp", stitchedImage);

        cv::Mat half(stitchedImage, cv::Rect(tx, ty, img1_w, img1_h));
        img1.copyTo(half);
        cv::namedWindow("result", cv::WINDOW_NORMAL);
        imshow("result", stitchedImage);
    }
    else{
        std::cout << "img2 should be left" << std::endl;
        stitchedImage = cv::Mat::zeros(sec2main_w + img1_w, mRows, CV_8UC3);
        warpPerspective(img1, stitchedImage, H2, cv::Size(img1_w + sec2main_w, mRows));
        cv::namedWindow("temp", cv::WINDOW_NORMAL);
        imshow("temp", stitchedImage);

        cv::Mat half(stitchedImage, cv::Rect(0, 0, sec2main_w, sec2main_h));
        img2.copyTo(half);
        cv::namedWindow("result", cv::WINDOW_NORMAL);
        cv::imshow("result", stitchedImage);
    }
}

TRTInferV1::DetectionObj PretreatObjs::objs2newMainObjs(TRTInferV1::DetectionObj objs, cv::Mat obj2Main)
{
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



std::vector<TRTInferV1::DetectionObj> PretreatObjs::secObjs2mainObjs(
        std::vector<TRTInferV1::DetectionObj> mainObjs,std::vector<TRTInferV1::DetectionObj> secObjs,
        std::shared_ptr<SensorParam> MainCam_ptr,std::shared_ptr<SensorParam> SecCam_ptr, float match_thresh,
        std::vector<std::vector<int>> &list_)
{
    int main_num = mainObjs.size();
    for(int i=0;i<main_num;i++)
        list_.push_back(std::vector<int>{0,i});

    for(int i=0;i<secObjs.size();i++){
        float x = (secObjs[i].x2+secObjs[i].x1)/2.,
                y = (secObjs[i].y1+secObjs[i].y2)/2.,
                w = (secObjs[i].x2-secObjs[i].x1)/SecCam_ptr->fx*MainCam_ptr->fx,
                h = (secObjs[i].y2-secObjs[i].y1)/SecCam_ptr->fy*MainCam_ptr->fy;
        SecCam_ptr->change2main(x,y,MainCam_ptr->fx,MainCam_ptr->fy,MainCam_ptr->cx,MainCam_ptr->cy);
//                test_ptr->sec2mainCam(x,y);

        secObjs[i].x1 = x - w/2;
        secObjs[i].x2 = x + w/2;
        secObjs[i].y1 = y - h/2;
        secObjs[i].y2 = y + h/2;
    }
    vector<vector<float> > dists;
    int dist_size, dist_size_size;
    vector<vector<int> > matches;
    vector<int> u_track, u_detection;


    Eigen::MatrixXd cost_matrix = costMatrix_ptr->getIouAndDistancetCost(mainObjs,secObjs,dist_size, dist_size_size);
    eigenMat2VecVec(cost_matrix,dists);
    costMatrix_ptr->linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_track, u_detection);
//    std::cout << "cost_matrix: \n"  <<cost_matrix << std::endl;

    for(int j=0;j<dist_size_size;j++){
        int flag = 1;
        for(int i=0;i<matches.size();i++ ){
            if(j==matches[i][1]){
                float
                        s_w = (secObjs[j].x2-secObjs[j].x1)/SecCam_ptr->fx*MainCam_ptr->fx,
                        s_h = (secObjs[j].y2-secObjs[j].y1)/SecCam_ptr->fy*MainCam_ptr->fy,
                        m_w = mainObjs[matches[i][0]].x2-mainObjs[matches[i][0]].x1,
                        m_h = mainObjs[matches[i][0]].y2-mainObjs[matches[i][0]].y1;

                //TODO:
                if(((m_h * m_w) < (s_h * s_w)) && (m_w/m_h < 0.4) ){
                    list_[matches[i][0]] = std::vector<int>{1,j};
                    mainObjs[matches[i][0]] = secObjs[j];
                }
                flag = -1;
                break;
            }
        }
        if(flag == 1){
            list_.push_back(std::vector<int>{1,j});
            mainObjs.push_back(secObjs[j]);
        }
    }

    return mainObjs;
}

std::vector<TRTInferV1::Object> PretreatObjs::secObjs2mainObjs(
        std::vector<TRTInferV1::Object> mainObjs,std::vector<TRTInferV1::Object> secObjs,
        std::shared_ptr<SensorParam> MainCam_ptr,std::shared_ptr<SensorParam> SecCam_ptr, float match_thresh,
        std::vector<std::vector<int>> &list_)
{
    int main_num = mainObjs.size();
    for(int i=0;i<main_num;i++)
        list_.push_back(std::vector<int>{0,i});

    for(int i=0;i<secObjs.size();i++){
        float   cx = (secObjs[i].x2+secObjs[i].x1)/2.,
                cy = (secObjs[i].y1+secObjs[i].y2)/2.,
                w = secObjs[i].w/SecCam_ptr->fx*MainCam_ptr->fx,
                h = secObjs[i].h/SecCam_ptr->fy*MainCam_ptr->fy;
        SecCam_ptr->change2main(cx,cy,MainCam_ptr->fx,MainCam_ptr->fy,MainCam_ptr->cx,MainCam_ptr->cy);
//                test_ptr->sec2mainCam(x,y);

        secObjs[i].x1 = cx - w/2;
        secObjs[i].x2 = cx + w/2;
        secObjs[i].y1 = cy - h/2;
        secObjs[i].y2 = cy + h/2;
        secObjs[i].w = w;
        secObjs[i].h = h;
    }
    vector<vector<float> > dists;
    int dist_size, dist_size_size;
    vector<vector<int> > matches;
    vector<int> u_track, u_detection;


    Eigen::MatrixXd cost_matrix = costMatrix_ptr->getIouAndDistancetCost(mainObjs,secObjs,dist_size, dist_size_size);
    eigenMat2VecVec(cost_matrix,dists);
    costMatrix_ptr->linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_track, u_detection);
//    std::cout << "cost_matrix: \n"  <<cost_matrix << std::endl;

    for(int j=0;j<dist_size_size;j++){
        int flag = 1;
        for(int i=0;i<matches.size();i++ ){
            if(j==matches[i][1]){
                if(((mainObjs[matches[i][0]].h * mainObjs[matches[i][0]].w) < (secObjs[j].h * secObjs[j].w)) && (mainObjs[matches[i][0]].w/mainObjs[matches[i][0]].h < 0.2)){
                    list_[matches[i][0]] = std::vector<int>{1,j};
                    mainObjs[matches[i][0]] = secObjs[j];
                }
                flag = -1;
                break;
            }
        }
        if(flag == 1){
            list_.push_back(std::vector<int>{1,j});
            mainObjs.push_back(secObjs[j]);
        }
    }

    return mainObjs;
}

TRTInferV1::DetectionObj PretreatObjs::secObjs2mainObjs(TRTInferV1::DetectionObj secObjs,
        std::shared_ptr<SensorParam> MainCam_ptr,std::shared_ptr<SensorParam> SecCam_ptr) {


    TRTInferV1::DetectionObj mainObjs;
    float   cx = (secObjs.x2+secObjs.x1)/2.,
            y2 = (secObjs.y1+secObjs.y2)/2.,
            w = (secObjs.x2-secObjs.x1)/SecCam_ptr->fx*MainCam_ptr->fx,
            h = (secObjs.y2-secObjs.y1)/SecCam_ptr->fy*MainCam_ptr->fy;
    SecCam_ptr->change2main(cx,y2,MainCam_ptr->fx,MainCam_ptr->fy,MainCam_ptr->cx,MainCam_ptr->cy);
//                test_ptr->sec2mainCam(x,y);

    mainObjs.x1 = cx - w/2;
    mainObjs.x2 = cx + w/2;
    mainObjs.y1 = y2 - h;
    mainObjs.y2 = y2 ;

    return mainObjs;

}


TRTInferV1::Object PretreatObjs::secObjs2mainObjs(TRTInferV1::Object secObjs,
        std::shared_ptr<SensorParam> MainCam_ptr,std::shared_ptr<SensorParam> SecCam_ptr){

    TRTInferV1::Object mainObjs;
    float   cx = (secObjs.x2+secObjs.x1)/2.,
            cy = (secObjs.y1+secObjs.y2)/2.,
            w = secObjs.w/SecCam_ptr->fx*MainCam_ptr->fx,
            h = secObjs.h/SecCam_ptr->fy*MainCam_ptr->fy;
    SecCam_ptr->change2main(cx,cy,MainCam_ptr->fx,MainCam_ptr->fy,MainCam_ptr->cx,MainCam_ptr->cy);
//                test_ptr->sec2mainCam(x,y);

    mainObjs.x1 = cx - w/2;
    mainObjs.x2 = cx + w/2;
    mainObjs.y1 = cy - h/2;
    mainObjs.y2 = cy + h/2;
    mainObjs.w = w;
    mainObjs.h = h;

    return mainObjs;

}

std::vector<STrack> PretreatObjs::ArmorInCar(std::vector<TRTInferV1::Object> &cars,std::vector<TRTInferV1::Object> &armors){
    int num = this->classWithoutCar;  //TODO
    std::vector<STrack> outRestCars;
    for(TRTInferV1::Object &car : cars){
        //这里将数组大小定义成10是为了方便调用函数initornot， 实际上大小定义成4就足够了
        std::array<cv::Point2f,25> car_4angle ;
        car_4angle[0] = cv::Point2f (float (car.x1),float (car.y1));
        car_4angle[1] = cv::Point2f (float (car.x2),float (car.y1));
        car_4angle[2] = cv::Point2f (float (car.x2),float (car.y2));
        car_4angle[3] = cv::Point2f (float (car.x1),float (car.y2));

        int state;int temp_cls = num;float temp_armorConf = 0.0;
        std::vector<TRTInferV1::Object> ArmorsInCar;
        ArmorsInCar.clear();

        for(auto &armor : armors){
            state = -1;
            state = initornot(car_4angle,cv::Point ((armor.x1+armor.x2)/2,(armor.y1+armor.y2)/2),4);
            if(state == 1 ){
                ArmorsInCar.push_back(armor);
                if (armor.confidence > temp_armorConf){
                    temp_armorConf = armor.confidence;
                    temp_cls = armor.classId;
                }
            }
        }
        if(temp_cls >= this->classWithoutCar){
            num ++;
        }
        STrack track(car.x1,car.y1,car.w,car.h,temp_cls,car.confidence,temp_armorConf);

    }
    return outRestCars;
};


/**
 * @brief 将神经网络的输出分类，分为车和装甲板两种
 *
 * @return newLists[0] all car 对应的原本 main,secDetectionObjs的标号s
 * @return newLists[1] all armor 对应的原本 main,secDetectionObjs的标号s
 *
 * @param newLists[0][1] = std::vector<TRTInferV1::Object> &cars中 第一个car 对应的原本 main,secDetectionObjs的标号s
 * 若  newLists[0][1] = {0,1} 则代表位于std::vector<TRTInferV1::Object> &cars中 第一个car 原本在 mainDectectionObjs 的 第2个
 *
 */
std::vector<std::vector<std::vector<int>>> PretreatObjs::Car_Armor(
        std::vector<TRTInferV1::Object> DetectionObjs,std::vector<std::vector<int>> list_,
        std::vector<TRTInferV1::Object> &cars,std::vector<TRTInferV1::Object> &armors)
{
    std::vector<std::vector<std::vector<int>>> newLists(2);
    for(int j = 0;j < DetectionObjs.size();j++) {
        if (DetectionObjs[j].classId >= this->classWithoutCar) {
            cars.push_back(DetectionObjs[j]);
            newLists[0].push_back(list_[j]);
        } else {
            armors.push_back(DetectionObjs[j]);
            newLists[1].push_back(list_[j]);
        }
    }
    return newLists;
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

    if(conf_armor < 1e-1){
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

void PretreatObjs::reassign_cls(int num, int &temp_bestcls) {
    if(temp_bestcls == classWithoutCar){
        temp_bestcls = num + 300;num ++;    //300+ unknown
    }
    else if(half_classWithoutCar == 7 ){
        if(temp_bestcls == 6){                   //RN
            temp_bestcls = num + 100;num ++;// 100+ R
        }else if(temp_bestcls == 13){            //BN
            temp_bestcls = num + 200;num ++;// 200+ B
        }
    }
}


/**
 * @brief 计算armor_w,分配出car
 *
 * @param cars 输入为所有车辆检测框
 * @param outRestCars 输出车的跟踪器
 * @param armors 输入时为所有装甲板，输出时为没有被包含的装甲版（即分配剩下的装甲版）
 */
std::vector<std::vector<int>> PretreatObjs::getArmors_wconf(
        const std::vector<TRTInferV1::Object>& cars,std::vector<std::vector<TRTInferV1::Object>> allDetectionObjs,
        std::vector<std::vector<std::vector<int>>> allLists,std::vector<STrack> &outRestCars,std::vector<TRTInferV1::Object> &armors)
{
    std::vector<bool> isOutArmor(armors.size(), false); // 判断该装甲版被车（Car）包含
    int car_num = 0;
    for(TRTInferV1::Object car : cars){
        auto trackStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//        float carLocate2D[2] = {(car.x1+car.x2)/2, (car.y1+car.y2)/2};
        double w_getAllArea = 0.0;  // 得到所有在car里面装甲版的总面积
//        Eigen::MatrixXd ws_armorAreaMatrix = Eigen::MatrixXd::Zero(1,14);
//        Eigen::MatrixXd ws_armorYMatrix = Eigen::MatrixXd::Zero(1,14);
        double carY_downLine = car.y1 + car.h * p_carY_downLine;  // 装甲板3/4处（沿y(v)）
        double carY_upLine = car.y1 + car.h * p_carY_upLine;      // 装甲板1/3处（沿y(v)）
        std::array<cv::Point2f,25> car_4angle ; // 这里将数组大小定义成10是为了方便调用函数initornot， 实际上大小定义成4就足够了
        car_4angle[0] = cv::Point2f (float (car.x1),float (car.y1));
        car_4angle[1] = cv::Point2f (float (car.x2),float (car.y1));
        car_4angle[2] = cv::Point2f (float (car.x2),float (car.y2));
        car_4angle[3] = cv::Point2f (float (car.x1),float (car.y2));
        int i=0;

        std::vector<std::vector<int>>whereIsArmor(this->classWithoutCar);
        std::vector<double> tempBest_w_armorConf(this->classWithoutCar,0.0);
        Eigen::MatrixXd car_armorConfMatrix = Eigen::MatrixXd::Zero(1,groupNum * half_classWithoutCar); ////TODO:
//        std::vector<int> temp_bestindexs_BRN;
        for(auto &armor : armors){
            float armorLocate2D[2] = {(armor.x1+armor.x2)/2, (armor.y1+armor.y2)/2};
            int state = -1;
            state = initornot(car_4angle,cv::Point (armorLocate2D[0],armorLocate2D[1]),4);
            if(state == 1 ){
                isOutArmor[i] = true;

                //area
                double w_armorArea = armor.w * armor.h;
                w_getAllArea += w_armorArea;
                // 1/3y - 3/4y
                double w_armorY = 1.0 - min_(abs((armorLocate2D[1] - carY_downLine)/(carY_upLine - carY_downLine)),1.0);
                //add
                double w_armorConf = w_armorArea * w_armorY * armor.confidence;
//                Eigen::MatrixXd  car_armorConfMatrix = Eigen::MatrixXd::Zero(1,20); ///TODO:
                if(half_classWithoutCar==6){                       // G, 1，2，3，4，5
                    car_armorConfMatrix(0,armor.classId) += w_armorConf;
                    whereIsArmor[armor.classId].push_back(i);
                }
                else if(half_classWithoutCar==7){               // G,1,2,3,4,5,N    //TODO： 装甲版是否为同一车辆计算（通过两个装甲板的2D距离，设置上下阈值）
                    whereIsArmor[armor.classId].push_back(i);
                    if((armor.classId+1) % half_classWithoutCar!=0){  // cls != BN/RN
                        car_armorConfMatrix(0,armor.classId) += w_armorConf;
                    }else{
                        w_armorConf =  w_armorConf / aGroupOfArmor;
                        car_armorConfMatrix.block(0,(armor.classId/half_classWithoutCar)*half_classWithoutCar,1,half_classWithoutCar) += Eigen::MatrixXd::Ones(1,half_classWithoutCar) * w_armorConf;
                    }
                } else{
                    std::cout << "here are error in half_classWithoutCar, please config by yourself" << std::endl;
                }
            }
            i++;
        }

        int temp_bestcls = -1;  float conf_armor = 0.0;
        if(w_getAllArea>1e-6) {
            car_armorConfMatrix /= w_getAllArea; // (conf1*S1*y1 +...+confn*Sn*yn)/(S1+...+Sn)
        }
        update_classfy(temp_bestcls, conf_armor,car_armorConfMatrix);


//        std::cout << "car_armorConfMatrix:  _" << car_armorConfMatrix << std::endl;
        // armor
//        if(temp_bestcls < classWithoutCar){
//            if(half_classWithoutCar==6){
//                for(int j=0;j<whereIsArmor[temp_bestcls].size();j++){
//                    isOutArmor[whereIsArmor[temp_bestcls][j]] = true;
//                }
//            }
//            else if(half_classWithoutCar==7){
//                int color_N = (temp_bestcls/half_classWithoutCar+1)*half_classWithoutCar-1;
//                for(int j=0;j<whereIsArmor[color_N].size();j++){
//                    isOutArmor[whereIsArmor[color_N][j]] = true;
//                }
//                if(temp_bestcls%half_classWithoutCar!=6){ // temp_bestcls != BN/RN
//                    for(int j=0;j<whereIsArmor[temp_bestcls].size();j++){
//                        isOutArmor[whereIsArmor[temp_bestcls][j]] = true;
//                    }
//                }
//            }
//        }

//        reassign_cls(num, temp_bestcls);
        std::vector<int> whereIsCar = allLists[0][car_num];
        TRTInferV1::Object detectionObj2car = allDetectionObjs[whereIsCar[0]][whereIsCar[1]];
        std::cout << "car: "  << car.x1 << " " << car.y1 << " " << car.w << " " <<car.h << std::endl;
        STrack track(car.x1,car.y1,car.w,car.h,temp_bestcls,car.confidence,conf_armor,car_armorConfMatrix);
        std::cout << "detectionObj2car: "  << detectionObj2car.x1 << " " << detectionObj2car.y1 << " " << detectionObj2car.w << " " <<detectionObj2car.h << std::endl;
        track.setRectInPrimaryCam(detectionObj2car.x1,detectionObj2car.y1,detectionObj2car.w,detectionObj2car.h,p_car_midpoint);
        //TODO: setRectInPrimaryCam;
//        std::cout << "car_armorConfMatrix:::: " << car_armorConfMatrix.size() << std::endl;
        outRestCars.push_back(track);
        car_num++;
        auto trackEndTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::cout << "fps_car"<< car_num  << ": " << 1000. / (trackEndTime - trackStartTime) << std::endl;
    }
    // armor
    std::vector<TRTInferV1::Object> temp_armors;
    std::vector<std::vector<int>> list;
    for(int j=0;j<isOutArmor.size();j++){
        if(!isOutArmor[j]){
            temp_armors.push_back(armors[j]);
            list.push_back(allLists[1][j]);
        }
    }
    std::swap(temp_armors,armors);
    return list;
};

std::vector<std::vector<STrack>> PretreatObjs::classfySTrackByCam(
        vector<STrack> &STracks, std::vector<std::vector<int>> newLists)
{
    std::vector<std::vector<STrack>> outSTracks(allCam);
    for(int i=0; i<STracks.size(); i++){
        outSTracks[newLists[i][0]].push_back(STracks[i]);
    }
    return outSTracks;
}




/**
 *
 * @param DetectionObjs     已经转到主相机坐标系下的所有检测框
 * @param allDetectionObjs  在原本相机坐标系的检测框， e.g. allDetectionObjs[0] 为主相机的检测结果， allDetectionObjs[1] 为副相机的检测结果
 * @param list_             DetectionObjs 与  allDetectionObjs的对应关系 e.g. 若 list[3] = {0,2} 则 DetectionObjs中的第(3+1)个 来自 主相机（0）的第（2+1）个检测结果；
 * @return
 */
std::vector<std::vector<STrack>> PretreatObjs::getSTrackwithArmor(
        std::vector<TRTInferV1::Object> DetectionObjs, std::vector<std::vector<TRTInferV1::Object>> allDetectionObjs,
        std::vector<std::vector<int>> list_)
{
    std::vector<STrack> tracked_stracks;
    std::vector<TRTInferV1::Object> car,armor;
    std::vector<std::vector<std::vector<int>>> allLists = Car_Armor(DetectionObjs, list_,car,armor);
//    std::vector<std::vector<int>> armorLists = getArmors_wconf(car,std::move(allDetectionObjs),allLists, tracked_stracks, armor);

    auto trackStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::vector<std::vector<int>> armorLists = getArmors_wconf(car,allDetectionObjs,allLists, tracked_stracks, armor);
    auto trackEndTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::cout << "fps_getArmors_wconf: " << 1000. / (trackEndTime - trackStartTime) << std::endl;

    getLastCar(armor,allDetectionObjs,armorLists,tracked_stracks);
    std::vector<std::vector<int>> newList;
    newList.insert(newList.end(),allLists[0].begin(),allLists[0].end());
    newList.insert(newList.end(),armorLists.begin(),armorLists.end());
    std::vector<std::vector<STrack>> outStracks = classfySTrackByCam(tracked_stracks, newList);
    return outStracks;
}



/**
 * @brief 根据别筛选后的装甲版生成车辆
 *
 * @param armors 没有被包含的装甲版（即分配剩下的装甲版）
 * @param outLastObjs 根据装甲版生成的车辆
 */
void PretreatObjs::getLastCar(
        std::vector<TRTInferV1::Object> &armors,std::vector<std::vector<TRTInferV1::Object>> allDetectionObjs,
        std::vector<std::vector<int>> armorLists,std::vector<STrack> &outSTracks) {
    int armor_num = 0;
    for (auto &armor: armors) {
        std::vector<int> whereIsCar = armorLists[armor_num];
        TRTInferV1::Object detectionObj2armor = allDetectionObjs[whereIsCar[0]][whereIsCar[1]];
        double width, height, cx, cy;
        width = detectionObj2armor.w;
        height = detectionObj2armor.h;
        cx = detectionObj2armor.x1 + width / 2.0;
        cy = detectionObj2armor.y1 + height / 2.0;
        int cls = armor.classId;
        double conf_armor = armor.confidence;
        double conf = armor.confidence * 0.3;
        Eigen::MatrixXd  car_armorConfMatrix = Eigen::MatrixXd::Zero(1,half_classWithoutCar * groupNum);
        if(half_classWithoutCar == 7 && (cls == 6 || cls == 13)){
            conf_armor =  conf_armor / aGroupOfArmor;
            car_armorConfMatrix.block(0,(cls/half_classWithoutCar)*half_classWithoutCar,1,half_classWithoutCar) += Eigen::MatrixXd::Ones(1,half_classWithoutCar) * conf_armor;
        }else{
            car_armorConfMatrix(0,cls) = conf_armor;
        }
//        reassign_cls(num, cls);
//        car.rect = cv::Rect((cx - 3.0 * height), (cy - 4.0 * height), (6.0 * height), (6.0 * height));
        STrack car(((armor.x1+armor.x2)/2. - 3.0 * armor.h), ((armor.y1+armor.y2)/2.  - 4.0 * armor.h), (6.0 * armor.h), (6.0 * armor.h),cls, conf,conf_armor,car_armorConfMatrix);
        car.setRectInPrimaryCam((cx - 3.0 * height), (cy - 4.0 * height), (6.0 * height), (6.0 * height),p_car_midpoint);
//        }
//        car.Locate2D = cv::Point2d (cx,car.rect.y + (6.0 * height)*0.95);
        outSTracks.push_back(car);
        armor_num++;
    }
};

/**
 * @brief 更新obj的conf_armor,cls,对obj进行分类（3类，红色，蓝色，unknown）
 *
 * @param obj Car/Strack
 */
template<typename T>
void PretreatObjs::update_classfy(T &obj, int num, std::vector<T> &outRedObjs, std::vector<T> &outBlueObjs,std::vector<T> &outRestObjs){
    // std::cout << "update_classfy 0" << std::endl;
    num += this->classWithoutCar*2;
    //获得ws_armorConfMatrix中最大值的标签（即armorConf最大值对应的标签）
    Eigen::MatrixXf::Index max_index;
    obj.ws_armorConfMatrix.row(0).maxCoeff(&max_index);
    obj.conf_armor = obj.ws_armorConfMatrix(0,max_index);

    // std::cout << "update_classfy 1" << std::endl;

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
    // std::cout << "update_classfy 2" << std::endl;
}




/**
 * @brief 将神经网络的输出分类，分为车（std::vector<Car> &cars）和装甲板（std::vector<Armor> &armors)两种
 */
void PretreatObjs::Car_Armor(std::vector<TRTInferV1::DetectionObj> &DetectionObjs,std::vector<Car> &cars,std::vector<Armor> &armors){
    for(int j = 0;j < DetectionObjs.size();j++) {
        if (DetectionObjs[j].classId >= this->classWithoutCar) {
            Car car(DetectionObjs[j].confidence, DetectionObjs[j].x1, DetectionObjs[j].y1, DetectionObjs[j].x2,
                    DetectionObjs[j].y2);
            cars.push_back(car);
        } else {
            Armor armor(DetectionObjs[j].confidence, DetectionObjs[j].x1, DetectionObjs[j].y1, DetectionObjs[j].x2,
                        DetectionObjs[j].y2, DetectionObjs[j].classId);
            armors.push_back(armor);
        }
    }
}


void PretreatObjs::ArmorInCar(std::vector<Car> &cars,std::vector<Armor> &armors){
    int num = this->classWithoutCar;
    for(Car &car : cars){
        //这里将数组大小定义成10是为了方便调用函数initornot， 实际上大小定义成4就足够了
        std::array<cv::Point2f,25> car_4angle ;
        car_4angle[0] = cv::Point2f (float (car.x1),float (car.y1));
        car_4angle[1] = cv::Point2f (float (car.x2),float (car.y1));
        car_4angle[2] = cv::Point2f (float (car.x2),float (car.y2));
        car_4angle[3] = cv::Point2f (float (car.x1),float (car.y2));
        int state;float temp_conf = 0.0;car.cls = num;
        for(auto &armor : armors){
            state = -1;
            state = initornot(car_4angle,cv::Point (armor.Locate2D.x,armor.Locate2D.y),4);
            if(state == 1 ){
                car.ArmorsInCar.push_back(armor);
                if (armor.conf > temp_conf){
                    temp_conf = armor.conf;
                    car.conf_armor = armor.conf;
                    car.cls = armor.cls;
                }
            }
        }
        if(car.cls >= this->classWithoutCar){
            num ++;
        }
    }
};

void PretreatObjs::set_confs_by_locate3D(double &windmill_car_conf, double &startupArea_car_conf){
    windmill_car_conf = this->windmill_car_conf;
    startupArea_car_conf = this->startupArea_car_conf;
}

//void PretreatObjs::set_confs_by_locate3D(Eigen::MatrixXd &car_armorConfMatrix,PlaceType_special placeType){
//    if(placeType == windmill){
//        for(auto cls: windmill_car){
//            car_armorConfMatrix(0, cls)  = std::min(0.95, car_armorConfMatrix(0, cls) + windmill_car_conf);
//        }
//    }else if(placeType == startupArea){
//        if(ourPattern==red){
//            car_armorConfMatrix(0, 5) = std::min(0.95, car_armorConfMatrix(0, 5) + startupArea_car_conf);
//        }
//        else if(ourPattern==blue){
//            car_armorConfMatrix(0, 11)  = std::min(0.95, car_armorConfMatrix(0, 11) + startupArea_car_conf);
//        }
//    }
//}


void PretreatObjs::get_Armors_w_conf_Double_net(STrack &car, vector<TRTInferV1::Object> armors) {
        auto trackStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//        float carLocate2D[2] = {(car.x1+car.x2)/2, (car.y1+car.y2)/2};
        double w_getAllArea = 0.0;  // 得到所有在car里面装甲版的总面积
//        Eigen::MatrixXd ws_armorAreaMatrix = Eigen::MatrixXd::Zero(1,14);
//        Eigen::MatrixXd ws_armorYMatrix = Eigen::MatrixXd::Zero(1,14);
        double carY_downLine = car.tlwh[3] * p_carY_downLine;  // 装甲板3/4处（沿y(v)）
        double carY_upLine = car.tlwh[3] * p_carY_upLine;      // 装甲板1/3处（沿y(v)）
        std::vector<double> tempBest_w_armorConf(this->classWithoutCar,0.0);
        Eigen::MatrixXd car_armorConfMatrix = Eigen::MatrixXd::Zero(1,groupNum * half_classWithoutCar); ////TODO:
//        std::vector<int> temp_bestindexs_BRN;
        for(auto armor : armors){
            float armorLocate2D[2] = {(armor.x1+armor.x2)/2, (armor.y1+armor.y2)/2};
                //area
                double w_armorArea = armor.w * armor.h;
                w_getAllArea += w_armorArea;
                // 1/3y - 3/4y
                double w_armorY = 1.0 - min_(abs((armorLocate2D[1] - carY_downLine)/(carY_upLine - carY_downLine)),1.0);
                //add
                double w_armorConf = w_armorArea * w_armorY * armor.confidence;
//                Eigen::MatrixXd  car_armorConfMatrix = Eigen::MatrixXd::Zero(1,20); ///TODO:
                if(half_classWithoutCar==6){    // G, 1，2，3，4，5
//                    std::cout << "armor.w_armorConf: " << w_armorConf << std::endl;
//                    std::cout << "armor.classId: " << armor.classId << std::endl;
                    car_armorConfMatrix(0,armor.classId) += w_armorConf;
                }
        }
        int temp_bestcls = -1;  float conf_armor = 0.0;
        if(w_getAllArea>1e-6) {
            car_armorConfMatrix /= w_getAllArea; // (conf1*S1*y1 +...+confn*Sn*yn)/(S1+...+Sn)
            car_armorConfMatrix /= w_getAllArea; // (conf1*S1*y1 +...+confn*Sn*yn)/(S1+...+Sn)
        }
        set_confs_by_locate3D(car.windmill_car_conf, car.startupArea_car_conf);
        update_classfy(temp_bestcls, conf_armor,car_armorConfMatrix);
        car.init_track(temp_bestcls, conf_armor, car_armorConfMatrix);
        auto trackEndTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}


bool PretreatObjs::get_Armors_w_conf_Double_net(STrack &car, vector<TRTInferV1::DetectionObj> armors,cv::Mat &car_img) {
    auto trackStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//        float carLocate2D[2] = {(car.x1+car.x2)/2, (car.y1+car.y2)/2};
    double w_getAllArea = 0.0;  // 得到所有在car里面装甲版的总面积
//        Eigen::MatrixXd ws_armorAreaMatrix = Eigen::MatrixXd::Zero(1,14);
//        Eigen::MatrixXd ws_armorYMatrix = Eigen::MatrixXd::Zero(1,14);
    double carY_downLine = car.tlwh[3] * p_carY_downLine;  // 装甲板3/4处（沿y(v)）//？？
    double carY_upLine = car.tlwh[3] * p_carY_upLine;      // 装甲板1/3处（沿y(v)）//？？
    std::vector<double> tempBest_w_armorConf(this->classWithoutCar,0.0);
    Eigen::MatrixXd car_armorConfMatrix = Eigen::MatrixXd::Zero(1,12); ////TODO:
//        std::vector<int> temp_bestindexs_BRN;
    for(auto armor : armors){
        //找装甲板的中点
        float armorLocate2D[2] = {(armor.x1+armor.x2)/2, (armor.y1+armor.y2)/2};
        //area
        double w_armorArea = (armor.y2-armor.y1) * (armor.x2-armor.x1);
        w_getAllArea += w_armorArea;
        // 1/3y - 3/4y
        double w_armorY = 1.0 - min_(abs((armorLocate2D[1] - carY_downLine)/(carY_upLine - carY_downLine)),1.0);
        //add
        double w_armorConf = w_armorArea * w_armorY * armor.confidence;
//                Eigen::MatrixXd  car_armorConfMatrix = Eigen::MatrixXd::Zero(1,20); ///TODO:
//         if(half_classWithoutCar==6){    // G, 1，2，3，4，5
// //            std::cout << "armor.w_armorConf: " << w_armorConf << std::endl;
//             std::cout << "armor.classId: " << armor.classId << std::endl;
//             car_armorConfMatrix(0,armor.classId) += w_armorConf;
//         }else if (half_classWithoutCar==5) {
//             std::cout << "new armor.classId: " << armor.classId << std::endl;
//             if (armor.classId<=3)
//                 car_armorConfMatrix(0,armor.classId) += w_armorConf;
//             else if (armor.classId>=5&&armor.classId<=9)
//                 car_armorConfMatrix(0,armor.classId-1) += w_armorConf;
//             else if (armor.classId==11)
//                 car_armorConfMatrix(0,armor.classId-2) += w_armorConf;
//             else
//                 w_getAllArea-=w_armorArea;
//         }
        std::cout << "armor.classId: " << armor.classId << std::endl;
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
    // if (index!=-1) {
    //     cv::Rect r = cv::Rect(armors[index].x1, armors[index].y1, armors[index].x2 - armors[index].x1, armors[index].y2 - armors[index].y1);
    //     if (!check_color(car_img(r),armors[index].classId)) {
    //         cv::Mat test=car_img(r);
    //         return true;
    //     }
    // }
    if (half_classWithoutCar==5) {
        if (temp_bestcls>=5&&temp_bestcls<=9) {
            temp_bestcls-=1;
        }else if (temp_bestcls==11) {
            temp_bestcls-=2;
        }else if (temp_bestcls==4||temp_bestcls==10) {
            return true;
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
    // cv::createTrackbar("rl_h", "Hik30", &rl_h1, 255);
    // cv::createTrackbar("rl_s", "Hik30", &rl_s1, 255);
    // cv::createTrackbar("rl_v", "Hik30", &rl_v1, 255);
    // cv::createTrackbar("rh_h", "Hik30", &rh_h1, 255);
    // cv::createTrackbar("rh_s", "Hik30", &rh_s1, 255);
    // cv::createTrackbar("rh_v", "Hik30", &rh_v1, 255);
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
    // if (red&&blue) {
    //     return true;
    // }else if(red&&cls>5) {
    //     return true;
    // }else if (blue&&cls<=5) {
    //     return true;
    // }else{
    //     return false;
    // }
    if (red||blue) {
        return true;
    }else{
        return false;
    }
}

/**
 * @brief 计算armor_w,分配出car,carR,carB...
 *
 * @param cars 输入为所有车辆检测框
 * @param outRedCars  输出为红色装甲版为主的车
 * @param outBlueCars 输出为蓝色装甲板为主的车
 * @param outRestCars 输出不知道什么颜色的车（即不含装甲版（即剩下的cars）/r_conf==b_conf/）
 * @param armors 没有被包含的装甲版（即分配剩下的装甲版）
 */
void PretreatObjs::getArmors_wconf(std::vector<Car> &cars,std::vector<Car> &outRedCars,std::vector<Car> &outBlueCars,std::vector<Car> &outRestCars,std::vector<Armor> &armors){
    std::vector<bool> isOutArmor(armors.size(), true); // 判断该装甲版被车（Car）包含
    int num = 0;
    for(Car &car : cars){
        double w_getAllArea = 0.0;  // 得到所有在car里面装甲版的总面积
//        Eigen::MatrixXd ws_armorAreaMatrix = Eigen::MatrixXd::Zero(1,14);
//        Eigen::MatrixXd ws_armorYMatrix = Eigen::MatrixXd::Zero(1,14);
        double carY_downLine = car.rect.y + car.rect.height * 3/4;  // 装甲板3/4处（沿y(v)）
        double carY_upLine = car.rect.y + car.rect.height / 3;      // 装甲板1/3处（沿y(v)）
        std::array<cv::Point2f,25> car_4angle ; // 这里将数组大小定义成10是为了方便调用函数initornot， 实际上大小定义成4就足够了
        car_4angle[0] = cv::Point2f (float (car.x1),float (car.y1));
        car_4angle[1] = cv::Point2f (float (car.x2),float (car.y1));
        car_4angle[2] = cv::Point2f (float (car.x2),float (car.y2));
        car_4angle[3] = cv::Point2f (float (car.x1),float (car.y2));
        int i=0;
        std::vector<int> temp_bestindexs;std::vector<int> temp_bestindexs_BRN;
        int temp_bestcls = -1;double tempBest_w_armorConf = 0.0;
        for(auto &armor : armors){
            int state = -1;
            state = initornot(car_4angle,cv::Point (armor.Locate2D.x,armor.Locate2D.y),4);
            if(state == 1 ){
                //area
                double w_armorArea = armor.rect.area();
                w_getAllArea += w_armorArea;
                // 1/3y - 3/4y
                double w_armorY = 1.0 - min_(abs((armor.Locate2D.y - carY_downLine)/(carY_upLine - carY_downLine)),1.0);
                //add
                double w_armorConf = w_armorArea * w_armorY * armor.conf;
                car.ws_armorConfMatrix(0,armor.cls%(this->half_classWithoutCar) + armor.cls/half_classWithoutCar*7) += w_armorConf;
                if(half_classWithoutCar!=7){
                    if(temp_bestcls == armor.cls) {
                        tempBest_w_armorConf += w_armorConf;
                        isOutArmor[i] = false;
                        temp_bestindexs.push_back(i);
                    }else if(w_armorConf > tempBest_w_armorConf){  //w_armorConf/w_getAllArea > tempBest_w_armorConf/w_getAllArea
                        temp_bestcls = armor.cls;
                        tempBest_w_armorConf = w_armorConf;
                        for(auto temp_index:temp_bestindexs){
                            isOutArmor[temp_index] = true;
                        }
                        temp_bestindexs.clear();
                        isOutArmor[i] = false;
                        temp_bestindexs.push_back(i);
                    }
                } else{                                         // 当前逻辑下忽略（无法处理）一个车辆检测框错误包含两个车，这两个车的装甲版，且其中一个只有是BN/RN，或两个都是BN/RN ；//TODO： 装甲版是否为同一车辆计算（通过两个装甲板的2D距离，设置上下阈值）
                    if(temp_bestcls == -1){                     // first time
                        temp_bestcls = armor.cls;
                        tempBest_w_armorConf = w_armorConf;
                        if((armor.cls/7)!=6){
                            isOutArmor[i] = false;
                            temp_bestindexs.push_back(i);
                        } else{
                            isOutArmor[i] = false;
                            temp_bestindexs_BRN.push_back(i);
                        }
                    }
                    else if(armor.cls == temp_bestcls){                                        // a == t
                        tempBest_w_armorConf += w_armorConf;
                        if((armor.cls/7)!=6){
                            isOutArmor[i] = false;
                            temp_bestindexs.push_back(i);
                        } else{
                            isOutArmor[i] = false;
                            temp_bestindexs_BRN.push_back(i);
                        }
                    } else{                                                // a != t
                        if((temp_bestcls%7)==6){                             // t == RN/BN -> a != RN/BN
                            if((temp_bestcls/7) == (armor.cls/7)){             // a != RN/BN;t and a is same color(t&&a)
                                temp_bestcls = armor.cls;
                                tempBest_w_armorConf = w_armorConf;
                                isOutArmor[i] = false;
                                temp_bestindexs.push_back(i);
                            } else if(w_armorConf > tempBest_w_armorConf) {    // a != RN/BN;t and a is not same color(t||a);a_conf > t_conf
                                temp_bestcls = armor.cls;
                                tempBest_w_armorConf = w_armorConf;
                                if(temp_bestindexs.size()>0){
                                    for(auto temp_index:temp_bestindexs){
                                        isOutArmor[temp_index] = true;
                                    }
                                    temp_bestindexs.clear();
                                } else{
                                    for(auto temp_index:temp_bestindexs_BRN){
                                        isOutArmor[temp_index] = true;
                                    }
                                    temp_bestindexs_BRN.clear();
                                }
                                isOutArmor[i] = false;
                                temp_bestindexs.push_back(i);
                            }
                        } else{                                                // t != RN/BN -> a == RN/BN
                            if((temp_bestcls/7) == (armor.cls/7)){               // a == RN/BN;t and a is same color(t&&a)
                                isOutArmor[i] = false;
                                temp_bestindexs_BRN.push_back(i);
                            } else if(w_armorConf > tempBest_w_armorConf){       // a == RN/BN; t != RN/BN; t and a is not same color(t||a); a_conf > t_conf
                                temp_bestcls = armor.cls;
                                tempBest_w_armorConf = w_armorConf;
                                for(auto temp_index:temp_bestindexs_BRN){
                                    isOutArmor[temp_index] = true;
                                }
                                temp_bestindexs_BRN.clear();
                                isOutArmor[i] = false;
                                temp_bestindexs.push_back(i);
                            }
                        }
                    }
                }
            }
            i++;
        }
//        car.cls = temp_bestcls%half_classWithoutCar + temp_bestcls/half_classWithoutCar*7;
        car.cls = temp_bestcls;

//        car.ws_armorConfMatrix = ws_armorAreaMatrix/w_getAllArea + ws_armorYMatrix;
        if(w_getAllArea>1e-6){
            car.ws_armorConfMatrix /= w_getAllArea; // (conf1*S1*y1 +...+confn*Sn*yn)/(S1+...+Sn)
        }

//TODO:
//        if(isGuess){
//            double w_armorConfMatrix_BN = car.ws_armorConfMatrix(0,6),w_armorConfMatrix_RN = car.ws_armorConfMatrix(0,13);
//            if(w_armorConfMatrix_BN>1e-5){
//                car.ws_armorConfMatrix(0,6) -= w_armorConfMatrix_BN;
//                car.ws_armorConfMatrix.block(0,0,1,7) += Eigen::MatrixXd::Ones(1,7)/6.0*w_armorConfMatrix_BN;
//                car.isGuess = true;
//            }
//            if(w_armorConfMatrix_RN>1e-5){
//                car.ws_armorConfMatrix(0,13) -= w_armorConfMatrix_RN;
//                car.ws_armorConfMatrix.block(0,7,1,7) += Eigen::MatrixXd::Ones(1,7)/6.0*w_armorConfMatrix_RN;
//                car.isGuess = true;
//            }
//        }
        // std::cout << "getArmors_wconf n3" << std::endl;
        update_classfy(car,num,outRedCars,outBlueCars,outRestCars);
        // std::cout << "getArmors_wconf n4" << std::endl;


    }
    // armor
    std::vector<Armor> temp_armors;
    for(int j=0;j<isOutArmor.size();j++){
        if(isOutArmor[j])
            temp_armors.push_back(armors[j]);
    }
    std::swap(temp_armors,armors);

};



/**
 * @brief 根据别筛选后的装甲版生成车辆
 *
 * @param armors 没有被包含的装甲版（即分配剩下的装甲版）
 * @param outLastObjs 根据装甲版生成的车辆
 */
void PretreatObjs::getLastCar(std::vector<Armor> &armors,std::vector<Car> &outLastCars) {
    for (auto &armor: armors) {
        double width, height, cx, cy;
        Car car;
        width = armor.rect.width;
        height = armor.rect.height;
        cx = armor.rect.x + width / 2.0;
        cy = armor.rect.y + height / 2.0;
        car.rect = cv::Rect((cx - 3.0 * height), (cy - 4.0 * height), (6.0 * height), (6.0 * height));
        car.cls = armor.cls;
        car.conf_armor = armor.conf * 0.3;
//        if (isGuess && half_classWithoutCar == 7 && ((armor.cls) % 7 == 6)) {
//            car.ws_armorConfMatrix.block(0, ((armor.cls) / 7 * 7), 1, 7) +=
//                    Eigen::MatrixXd::Ones(1, 7) / 6.0 * car.conf_armor;
//            car.isGuess = true;
//        } else {
            if(car.cls>100){
                //TODO:
                int index = car.cls%100-this->classWithoutCar;
            } else{
                std::cout << "car.cls " << car.cls << std::endl;
                car.ws_armorConfMatrix(0, car.cls) = car.conf_armor;
            }
//        }
        car.Locate2D = cv::Point2d (cx,car.rect.y + (6.0 * height)*0.95);
        outLastCars.push_back(car);
    }
};


/**
 * @brief 获取多帧状态下装甲版s的权重 //maybe is useless
 * @note 权重计算公式； 当有n帧的数据时, 第x帧的权重为 [3^(n-x)*2^(x-1)]/[（3^n - 2^n）]
 *
 * @param maxSize 需要加权的装甲板一共是几帧的
 * @return 多帧状态下装甲版s的权重()
 */
void PretreatObjs::getW_of_armorConfs(int maxSize){
    for(int n=1;n < maxSize+1; n++){
        std::vector<double> w_of_armorConf;
        for(int x=1;x<=n;x++){
            double w = (pow(3,(n-x))*pow(2,(x-1)))/(pow(3,n)-pow(2,n));
            w_of_armorConf.push_back(w);
        }
        W_of_armorConfs.push_back(w_of_armorConf);
    }
}

