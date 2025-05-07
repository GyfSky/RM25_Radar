#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>

#include <rclcpp/time.hpp>
#include <rclcpp/rclcpp.hpp>
#pragma once

float Distance(pcl::PointXYZI &a, pcl::PointXYZI &b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

float Distance(Eigen::Vector3d &a, Eigen::Vector3d &b) {
    return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2));
}

class Kalman_filter_plus {

    public:
    cv::KalmanFilter KF;
    int point_size=0;

    rclcpp::Node* node;
    float last_time = 0;//丢失的总时间，即没有观测到目标，只进行预测
    std::chrono::steady_clock::time_point timer;//最后更新时间
    std::vector<std::pair<double, pcl::PointXYZI>> history;//放最佳估计点，时间，坐标
    std::vector<cv::Rect> rect_2d1,rect_2d2;
    std::vector<std::pair<int ,int>> detect_history;//放相机匹配结果，第一个是颜色，第二个是编号
    std::vector<double> detect_time;//相机匹配的时间戳
    int max_history = 40;
    int max_detect_history = 15;

    pcl::PointXY predict_point;//预测和纠正都会更新
    float detect_r = 2;//1.7
    float car_speed = 2;//2
    float car_max_speed = 2;//1.5
    //opencv中表示颜色或多个数值的类
    bool has_updated = false;
    cv::Mat Q= cv::Mat::zeros(4, 4, CV_32F);
    cv::Mat R= cv::Mat::zeros(2, 2, CV_32F);
    float dt_=0.1f;
    float sigma_q_x=50.0f;//越小相信模型
    float sigma_q_y=50.0f;
    float sigma_r_x=0.1f;//越小相信观测
    float sigma_r_y=0.1f;
    float Distance(pcl::PointXY &a, pcl::PointXY &b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }
    float get_time() {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timer);
        // std::cout << duration.count()/1000.0 <<"s"<< std::endl;
        return duration.count() / 1000.0;
    }

    Kalman_filter_plus(pcl::PointXYZI input,rclcpp::Time time,rclcpp::Node* node,cv::Rect rect) {

        this->node=node;
        dt_=node->get_parameter("kalman.dt_").as_double();
        sigma_q_x=node->get_parameter("kalman.sigma_q_x").as_double();
        sigma_q_y=node->get_parameter("kalman.sigma_q_y").as_double();
        sigma_r_x=node->get_parameter("kalman.sigma_r_x").as_double();
        sigma_r_y=node->get_parameter("kalman.sigma_r_y").as_double();
        
        predict_point = pcl::PointXY(input.x, input.y);
        point_size = input.intensity;
        history.push_back(std::make_pair(GetTimeByRosTime(time), input));
        rect_2d1.push_back(rect);
        timer = std::chrono::steady_clock::now();
        int stateSize = 4;//状态的维数
        int measSize = 2;//测量的维数
        int contrSize = 0;//控制量的维数
        unsigned int type = CV_32F;//创建的矩阵类型
        KF.init(stateSize, measSize, contrSize, type);

        // 状态矩阵：[x, vx, y, vy]
        cv::Mat state(stateSize, 1, type);
        // 测量矩阵：[z_x, z_y]//z_x, z_y为测量的x，y 
        cv::Mat meas(measSize, 1, type);
        meas.at<float>(0) = input.x;
        meas.at<float>(1) = input.y;

        state.at<float>(0) = meas.at<float>(0);
        state.at<float>(2) = meas.at<float>(1);
        KF.statePost = state;//系统初始状态
        KF.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, dt_, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, dt_,
        0, 0, 0, 1);//F矩阵//状态转移矩阵
//         KF.transitionMatrix = (cv::Mat_<float>(6, 6) <<
// 1, dt_, 0, 0, dt_*dt_/2, 0,
// 0, 1, 0, 0, dt_, 0,
// 0, 0, 1, dt_, 0, dt_*dt_/2,
// 0, 0, 0, 1, 0, dt_,
// 0, 0, 0, 0, 1, 0,
// 0, 0, 0, 0, 0, 1
// );//F矩阵//状态转移矩阵

//         KF.measurementMatrix = (cv::Mat_<float>(2, 6) <<
// 1, 0, 0, 0, 0, 0,
// 0, 0, 1, 0, 0, 0);

        KF.measurementMatrix = (cv::Mat_<float>(2, 4) <<
        1, 0, 0, 0,
        0, 0, 1, 0);//H矩阵//测量状态矩阵，即测量值和状态之间的转换关系
        //Z(测量值)=H(测量状态矩阵)*X(状态值)
        
        KF.processNoiseCov = (cv::Mat_<float>(4, 4) <<
        sigma_q_x*pow(dt_, 3) / 3, sigma_q_x*pow(dt_, 2) / 2, 0,0,
        sigma_q_x*pow(dt_, 2) / 2, sigma_q_x*pow(dt_, 1), 0,0,
        0, 0, sigma_q_y*pow(dt_, 3) / 3, sigma_q_y*pow(dt_, 2) / 2,
        0, 0, sigma_q_y*pow(dt_, 2) / 2, sigma_q_y*pow(dt_, 1));
        // KF.processNoiseCov = cv::Mat::zeros(6, 6, CV_32F);
        // KF.processNoiseCov.at<float>(0, 0) = sigma_q_x * pow(dt_, 5)/20;
        // KF.processNoiseCov.at<float>(0, 1) = sigma_q_x * pow(dt_, 4)/8;
        // KF.processNoiseCov.at<float>(0, 4) = sigma_q_x * pow(dt_, 3)/6;
        //
        // KF.processNoiseCov.at<float>(1, 0) = KF.processNoiseCov.at<float>(0, 1);
        // KF.processNoiseCov.at<float>(1, 1) = sigma_q_x * pow(dt_, 3)/3;
        // KF.processNoiseCov.at<float>(1, 4) = sigma_q_x * pow(dt_, 2)/2;
        //
        // KF.processNoiseCov.at<float>(4, 0) = KF.processNoiseCov.at<float>(0, 4);
        // KF.processNoiseCov.at<float>(4, 1) = KF.processNoiseCov.at<float>(1, 4);
        // KF.processNoiseCov.at<float>(4, 4) = sigma_q_x * dt_;
        //
        // // y 轴部分
        // KF.processNoiseCov.at<float>(2, 2) = sigma_q_y * pow(dt_, 5)/20;
        // KF.processNoiseCov.at<float>(2, 3) = sigma_q_y * pow(dt_, 4)/8;
        // KF.processNoiseCov.at<float>(2, 5) = sigma_q_y * pow(dt_, 3)/6;
        //
        // KF.processNoiseCov.at<float>(3, 2) = KF.processNoiseCov.at<float>(2, 3);
        // KF.processNoiseCov.at<float>(3, 3) = sigma_q_y * pow(dt_, 3)/3;
        // KF.processNoiseCov.at<float>(3, 5) = sigma_q_y * pow(dt_, 2)/2;
        //
        // KF.processNoiseCov.at<float>(5, 2) = KF.processNoiseCov.at<float>(2, 5);
        // KF.processNoiseCov.at<float>(5, 3) = KF.processNoiseCov.at<float>(3, 5);
        // KF.processNoiseCov.at<float>(5, 5) = sigma_q_y * dt_;

        //越大越相信卡尔曼的预测值，收敛速度越快
        //过程噪声Q

        KF.measurementNoiseCov = (cv::Mat_<float>(2, 2) <<
        sigma_r_x, 0,
        0, sigma_r_y);
        //越大越不相信卡尔曼预测值
        //观测噪声R
        setIdentity(KF.errorCovPost, cv::Scalar::all(1));//初始p（误差的协方差矩阵）
        has_updated = true;
    }

    ~Kalman_filter_plus() {}

    //红是0,蓝是1
    int get_color() {
        if(detect_history.size() == 0) {
            return 2;
        }
        //比较队列中的颜色，返回最多的颜色
        int red = 0;
        int blue= 0;
        for(auto &color : detect_history) {
            if(color.first == 1) {
                blue++;
            }
            else {
                red++;
            }
        }
        if(red > blue) {
            return 0;
        }
        else {
            return 1;
        }
    }

    int get_number() {
        if(detect_history.size() == 0) return -1;

        int color = get_color();
        std::map<int, int> number_map;//得到对应颜色的各编号数量
        for(auto &number : detect_history) {
            if(number.first == color) {
                number_map[number.second]++;
            }
        }
        // 初始化最大计数和对应的数字
        int max_count = 0;
        int max_number = 0; // 假设-1为无效数字，或根据实际情况调整
        for(auto &entry : number_map) {
            if(entry.second > max_count) {
                max_count = entry.second;
                max_number = entry.first;
            }
        }
        return max_number; // 返回出现次数最多的数字
    }

    //得到给定颜色和编号的检测数量和最近时间
    std::pair<int,double> get_freq(int colcor,int number) {
        int count=0;
        double max_time=0;
        for (int i=0;i<detect_history.size();i++) {
            if (detect_history[i].first==colcor && detect_history[i].second==number) {
                if (detect_time[i]>max_time) max_time=detect_time[i];
                count++;
            }
        }
        return std::pair(count,max_time);
    }

    void update(pcl::PointXYZI input, rclcpp::Time time,cv::Rect rect) {
        timer = std::chrono::steady_clock::now();
        point_size = input.intensity;
        cv::Mat meas = cv::Mat::zeros(2, 1, CV_32F);
        meas.at<float>(0) = input.x;
        meas.at<float>(1) = input.y;
        KF.correct(meas);//根据测量值更新状态值
        predict_point.x = KF.statePost.at<float>(0);//得到最终的状态值
        predict_point.y = KF.statePost.at<float>(2);
        has_updated = true;
        last_time = 0;
        auto temp_point = input;
        history.push_back(std::make_pair(GetTimeByRosTime(time), temp_point));
        rect_2d1.push_back(rect);
        if(history.size() > max_history){
            history.erase(history.begin());
            rect_2d1.erase(rect_2d1.begin());
        }
    }

    void update_predict_point() {//基于最后的点和速度方向和车的速度，以及预测下一个点
        dt_ = get_time();
        timer = std::chrono::steady_clock::now();
        last_time += dt_;
        // KF.transitionMatrix.at<float>(0,1)=dt_;
        // KF.transitionMatrix.at<float>(2,3)=dt_;
        auto result = KF.predict();//计算预测的状态值
        predict_point.x = result.at<float>(0);
        predict_point.y = result.at<float>(2);
    }

    void forword_predict() {
        auto result = KF.predict();//计算预测的状态值
        predict_point.x = result.at<float>(0);
        predict_point.y = result.at<float>(2);
    }

    bool match(pcl::PointXY &input) {
        detect_r=node->get_parameter("kalman.detect_r").as_double();
        car_max_speed=node->get_parameter("kalman.car_max_speed").as_double();
        if(Distance(predict_point, input) < car_max_speed * dt_+detect_r){
            return true;
        }else {
            return false;
        }
    }

    bool loose_match(pcl::PointXY &input) {
        detect_r=node->get_parameter("kalman.detect_r").as_double();
        car_max_speed=node->get_parameter("kalman.car_max_speed").as_double();
        if(Distance(predict_point, input) < car_max_speed * dt_+1.5*detect_r){
            return true;
        }else {
            return false;
        }
    }
    //从最佳估计点中找
    double camera_find_match(rclcpp::Time &time,double offset) {
        const double TIME_THRESHOLD = 1.0f;
        double input_time = GetTimeByRosTime(time);
        double differ_time = 1000;
        double return_time =0;
        for(auto &point : history) {
            //update更新history
            // std::cout<<"compare"<<std::to_string(point.first)<<"and"<<std::to_string(input_time)<<std::endl;
            auto differ = fabs(point.first - input_time+offset);
            // std::cout<<"differ"<<differ<<std::endl;
            if(differ < differ_time) {
                differ_time = differ;
                return_time=point.first;
            }
        }
        // std::cout<<"------------"<<std::endl;
        // std::cout<<std::to_string(return_time)<<std::endl;
        // std::cout<<"differ"<<differ_time<<std::endl;
        // std::cout<<"last time"<<history.back().first-input_time<<std::endl;

        if(differ_time>TIME_THRESHOLD) {
            // RCLCPP_ERROR(node->get_logger(),"differ_time is too large");
            return 0;
        }//首先找到离相机取帧时间戳最近的点
        return return_time;
    }
    //从最佳估计点中找
    void camera_match(rclcpp::Time &time, pcl::PointXY &input,int color,int number) {
        const double TIME_THRESHOLD = 1.0f;
        double input_time = GetTimeByRosTime(time);
        double differ_time = 1000;
        pcl::PointXY match_point;
        for(auto &point : history) {//直接利用最后一个？应该按时间顺序存储的 不能利用最后一个，因为网络推理延迟，时间戳不同步
                                    //update更新history
            // std::cout<<"compare"<<point.first<<"and"<<input_time<<std::endl;
            auto differ = abs(point.first+0.1 - input_time);
            // std::cout<<"differ"<<differ<<std::endl;
            if(differ < differ_time) {
                differ_time = differ;
                match_point = pcl::PointXY(point.second.x,point.second.y);
            }
        }
        std::cout<<"differ"<<differ_time<<std::endl;
        // std::cout<<"last time"<<history.back().first-input_time<<std::endl;

        if(differ_time>TIME_THRESHOLD) {
            // RCLCPP_ERROR(node->get_logger(),"differ_time is too large");
            return ;
        }//首先找到离相机取帧时间戳最近的点

        if(Distance(match_point, input) < detect_r){
            // std::cout<<"match success"<<std::endl;
            detect_history.push_back(std::make_pair(color, number));
            if(detect_history.size() > max_detect_history){
                detect_history.erase(detect_history.begin());
            }
        }//如果距离小于检测半径，认为匹配成功
    }

    static double GetTimeByRosTime(rclcpp::Time& ros_time){
        double ros_time_value =ros_time.nanoseconds()/1e9;
        // std::cout<<"ros_time_value"<<ros_time_value<<std::endl;
        return ros_time_value;
    }
}; 
