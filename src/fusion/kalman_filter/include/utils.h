#include "lapjv.h"
#include "filter_plus.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <open3d/Open3D.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <random>
#include <opencv2/core/eigen.hpp>
#include <pcl/point_cloud.h>


template <typename T>
void eigenMat2VecVec(Eigen::MatrixXd &eigen,std::vector<std::vector<T>> &vecVec){
    int col = eigen.cols();//列
    int raw = eigen.rows();//行
    Eigen::RowVectorXd vec_d(col);
    for(int i=0;i<raw;i++){
        vec_d = eigen.block(i,0,1,col);//行向量
        std::vector<float> vec(vec_d.data(), vec_d.data() + vec_d.size());
        vecVec.push_back(vec);
    }
}

double f(double x) {
    return 0.0067*x*x*x-0.1368*x*x+1.3636*x+2.7665;
}
double GetTimeByRosTime(rclcpp::Time ros_time){
    double ros_time_value =ros_time.nanoseconds()/1e9;
    // std::cout<<"ros_time_value"<<ros_time_value<<std::endl;
    return ros_time_value;
}

struct One_{
    int mutli_in=0;//在范围内的未匹配kalman数量
    std::vector<int> I_utrack;//未匹配上的kalman索引
    pcl::PointCloud<pcl::PointXYZI> kmeans_pc;//经过kmeans后的聚类中心点
};

struct Clus_pc{
    double time;
    int color=2;
    int num=-1;
    Eigen::Vector3d center;
};

int initornot(std::array<cv::Point2f,5> AABB ,pcl::PointXY xy, int pointNum){
    Eigen::Vector3d p1,p2;
    Eigen::Vector3d x1,x2,temp_x2;

    for(int i=0;i<pointNum;i++){
        if(i==0){
            p1 << (AABB[pointNum-1].x-xy.x),(AABB[pointNum-1].y-xy.y),0;
            p2 << (AABB[i].x-xy.x),(AABB[i].y-xy.y),0;
            x2 = p1.cross(p2);
            temp_x2 = x2;
            continue;
        }
        else{
            p1 = p2;
            x1 = x2;
            p2 << (AABB[i].x-xy.x),(AABB[i].y-xy.y),0;
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

Eigen::MatrixXd getCost_confMatrix(std::vector<Kalman_filter_plus> &KFs,int &num_strack,int &num_cls) {
    num_strack=KFs.size();
    num_cls=10;
    Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Ones(num_strack,num_cls);

    for (int index=0;index<num_strack;index++) {
        std::array<std::array<int,6>,2> color_number={{
            {{0, 0, 0, 0, 0, 0}},  // 第一个子数组 红
            {{0, 0, 0, 0, 0, 0}}  // 第二个子数组 蓝
        }};
        for(int i=0;i<KFs[index].detect_history.size();i++){
            color_number[KFs[index].detect_history[i].first][KFs[index].detect_history[i].second]++;
        }
        for (int cls=0;cls<num_cls;cls++) {
            if (cls<5) {
                cost_matrix(index, cls)=double(color_number[0][cls])/double(KFs[index].max_detect_history);
            }else {
                cost_matrix(index, cls)=double(color_number[1][cls-5])/double(KFs[index].max_detect_history);
            }
            cost_matrix(index, cls)=1-cost_matrix(index, cls);
        }
    }
    return cost_matrix;
}

Eigen::MatrixXd getCloudCost(std::vector<Clus_pc> atracks, std::vector<Kalman_filter_plus>btracks,
    int &atracks_size, int &btracks_size,double distance_thres){
    atracks_size=atracks.size();btracks_size=btracks.size();
    if (atracks_size*btracks_size==0){
        Eigen::MatrixXd cost_matrix;
        return cost_matrix;
    }
    double distance_cost;
    double mah_cost;
    double history_cost;
    std::vector<Eigen::Matrix2d> conv(btracks_size);
    std::vector<Eigen::Vector2d> mean(btracks_size);
    std::vector<std::array<std::array<int,6>,2>> color_numbers (btracks.size(),{{
        {{0, 0, 0, 0, 0, 0}},  // 第一个子数组 蓝
        {{0, 0, 0, 0, 0, 0}}  // 第二个子数组 红
    }});

    for (int b=0;b<btracks_size;b++) {
        int his_size=btracks[b].history.size();
        double meanX = 0.0, meanY = 0.0,cov00=0,cov01=0,cov11=0;
        for (auto pair: btracks[b].history) {
            meanX += pair.second.x;
            meanY += pair.second.y;
        }
        meanX /= his_size;
        meanY /= his_size;
        mean[b]<<meanX,meanY;

        for (auto pair: btracks[b].history) {
            cov00 += (pair.second.x - meanX) * (pair.second.x - meanX);
            cov01 += (pair.second.x - meanX) * (pair.second.y - meanY);
            cov11 += (pair.second.y - meanY) * (pair.second.y - meanY);
        }
        for (auto pair: btracks[b].detect_history) {
            color_numbers[b][pair.first][pair.second]++;
        }

        cov00 /= (his_size - 1);
        cov01 /= (his_size - 1);
        cov11 /= (his_size - 1);
        conv[b]<<cov00,cov01,cov01,cov11;
    }
    Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(atracks_size,btracks_size);
    for(int a = 0; a<atracks_size; a++){
        for(int b=0;b<btracks_size;b++){
            double distance=sqrt(pow(atracks[a].center[0]-btracks[b].predict_point.x,2)+pow(atracks[a].center[1]-btracks[b].predict_point.y,2));
            distance_cost=distance<distance_thres?distance/distance_thres:1;
            Eigen::Matrix2d cov_eigen;
            cov_eigen<<btracks[b].KF.errorCovPost.at<float>(0,0),btracks[b].KF.errorCovPost.at<float>(0,2),
                                        btracks[b].KF.errorCovPost.at<float>(2,0),btracks[b].KF.errorCovPost.at<float>(2,2);

            // std::cout<<"cov: "<<cov_eigen<<std::endl;
            Eigen::Vector2d kal_pos,pc_pos;
            kal_pos<<btracks[b].predict_point.x,btracks[b].predict_point.y;
            pc_pos<<atracks[a].center[0],atracks[a].center[1];
            Eigen::Vector2d d = pc_pos - mean[b];
            mah_cost=sqrt(d.transpose() * conv[b].inverse() * d);
            mah_cost=mah_cost<distance_thres?mah_cost/distance_thres:1;
            if (btracks[b].detect_history.size()==0||atracks[a].color==2) history_cost=0;
            else history_cost=double(color_numbers[b][atracks[a].color][atracks[a].num])/f(double(btracks[b].detect_history.size()))*1.3>1?0:1-double(color_numbers[b][atracks[a].color][atracks[a].num])/f(double(btracks[b].detect_history.size()))*1.3;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"%f",history_cost);
            // std::cout<<"mah dis: "<<mah_cost<<"  dis_cost: "<<distance<<std::endl;
            cost_matrix(a,b)=distance_cost*0.7+history_cost*0.3;
            // cost_matrix(a,b)=distance_cost;
        }
    }
    return cost_matrix;
}

Eigen::MatrixXd getCloudCost(pcl::PointCloud<pcl::PointXYZI>atracks, std::vector<Kalman_filter_plus>btracks,
    int &atracks_size, int &btracks_size,double distance_thres){
    atracks_size=atracks.points.size();btracks_size=btracks.size();
    if (atracks_size*btracks_size==0){
        Eigen::MatrixXd cost_matrix;
        return cost_matrix;
    }
    double distance_cost;
    double mah_cost;
    double size_cost;
    std::vector<Eigen::Matrix2d> conv(btracks_size);
    std::vector<Eigen::Vector2d> mean(btracks_size);
    for (int b=0;b<btracks_size;b++) {
        int his_size=btracks[b].history.size();
        double meanX = 0.0, meanY = 0.0,cov00=0,cov01=0,cov11=0;
        for (auto pair: btracks[b].history) {
            meanX += pair.second.x;
            meanY += pair.second.y;
        }
        meanX /= his_size;
        meanY /= his_size;
        mean[b]<<meanX,meanY;

        for (auto pair: btracks[b].history) {
            cov00 += (pair.second.x - meanX) * (pair.second.x - meanX);
            cov01 += (pair.second.x - meanX) * (pair.second.y - meanY);
            cov11 += (pair.second.y - meanY) * (pair.second.y - meanY);
        }

        cov00 /= (his_size - 1);
        cov01 /= (his_size - 1);
        cov11 /= (his_size - 1);
        conv[b]<<cov00,cov01,cov01,cov11;
    }
    Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(atracks_size,btracks_size);
    for(int a = 0; a<atracks_size; a++){
        for(int b=0;b<btracks_size;b++){
            double distance=sqrt(pow(atracks.points[a].x-btracks[b].predict_point.x,2)+pow(atracks.points[a].y-btracks[b].predict_point.y,2));
            double speed=sqrt(pow(btracks[b].KF.statePost.at<float>(1),2)+pow(btracks[b].KF.statePost.at<float>(3),2));
            // if (distance>btracks[b].last_time*speed*3)
            //     distance=distance_thres;
            distance_cost=distance<distance_thres?distance/distance_thres:1;
            size_cost=atracks.points[a].intensity>btracks[b].point_size?btracks[b].point_size/atracks.points[a].intensity:atracks.points[a].intensity/btracks[b].point_size;
            size_cost=1-size_cost;
            Eigen::Matrix2d cov_eigen;
            cov_eigen<<btracks[b].KF.errorCovPost.at<float>(0,0),btracks[b].KF.errorCovPost.at<float>(0,2),
                                        btracks[b].KF.errorCovPost.at<float>(2,0),btracks[b].KF.errorCovPost.at<float>(2,2);

            // std::cout<<"cov: "<<cov_eigen<<std::endl;

            Eigen::Vector2d kal_pos,pc_pos;
            kal_pos<<btracks[b].predict_point.x,btracks[b].predict_point.y;
            pc_pos<<atracks.points[a].x,atracks.points[a].y;
            Eigen::Vector2d d = pc_pos - mean[b];
            mah_cost=sqrt(d.transpose() * conv[b].inverse() * d);
            mah_cost=mah_cost<distance_thres?mah_cost/distance_thres:1;
            // std::cout<<"mah dis: "<<mah_cost<<"  dis_cost: "<<distance<<std::endl;
            cost_matrix(a,b)=mah_cost*0.4+distance_cost*0.6;
            // cost_matrix(a,b)=distance_cost;
        }
        // std::cout<<"--------------"<<std::endl;
    }
    return cost_matrix;
}

Eigen::MatrixXd getFusionCost(std::vector<Kalman_filter_plus> atracks,std::vector<std::array<double, 4>> btracks,
    int &atracks_size, int &btracks_size,double distance_thres,double time_id,std::vector<bool> is_det,double dis_wight,double his_wight){
    atracks_size=atracks.size();btracks_size=btracks.size();
    if (atracks_size*btracks_size==0){
        Eigen::MatrixXd cost_matrix;
        return cost_matrix;
    }
    double distance_cost,history_cost;
    Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(atracks_size,btracks_size);
    for(int a = 0; a<atracks_size; a++){
        if (is_det[a]) {
            for(int b=0;b<btracks_size;b++) {
                cost_matrix(a,b)=1;
            }
            continue;
        }
        bool flag=false;
        double x=0,y=0,min_time=10;
        std::array<std::array<int,6>,2> color_number={{
            {{0, 0, 0, 0, 0, 0}},  // 第一个子数组 蓝
            {{0, 0, 0, 0, 0, 0}}  // 第二个子数组 红
        }};
        for (auto pair: atracks[a].history) {
            if (fabs(pair.first-time_id)<=min_time) {
                // std::cout<<"time_id:"<<std::to_string(time_id)<<"  "<<"pair.first:"<<std::to_string(pair.first)<<std::endl;
                min_time=fabs(pair.first-time_id);
                // std::cout<<"min_time:"<<std::to_string(min_time)<<std::endl;
                flag=true;
                x=pair.second.x;
                y=pair.second.y;
            }
        }
        if (min_time<0.5) flag=true;
        for(int i=0;i<atracks[a].detect_history.size();i++){
            color_number[atracks[a].detect_history[i].first][atracks[a].detect_history[i].second]++;
        }
        for(int b=0;b<btracks_size;b++){
            if (!flag) {
                cost_matrix(a,b)=1;
                continue;
            }
            double distance=sqrt(pow(x-btracks[b][0],2)+pow(y-btracks[b][1],2));
            distance_cost=distance<distance_thres?1-distance/distance_thres:0;
            if (atracks[a].detect_history.size()==0) history_cost=0;
            else history_cost=double(color_number[btracks[b][2]][btracks[b][3]])/double(atracks[a].detect_history.size());
            cost_matrix(a,b)=dis_wight*distance_cost+his_wight*history_cost;
            cost_matrix(a,b)=1-cost_matrix(a,b);
        }
    }
    return cost_matrix;
}

Eigen::MatrixXd getFusionCost(std::vector<open3d::geometry::PointCloud> atracks,std::vector<std::array<double, 8>> btracks,std::vector<cv::Rect> rects,
    int &atracks_size, int &btracks_size,double distance_thres,std::vector<bool> is_det,double dis_wight,double his_wight){
    atracks_size=atracks.size();btracks_size=btracks.size();
    if (atracks_size*btracks_size==0){
        Eigen::MatrixXd cost_matrix;
        return cost_matrix;
    }
    Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(atracks_size,btracks_size);
    for(int a = 0; a<atracks_size; a++){
        if (is_det[a]) {
            for(int b=0;b<btracks_size;b++) {
                cost_matrix(a,b)=1;
            }
            continue;
        }
        for(int b=0;b<btracks_size;b++){
            double distance_3D=sqrt(pow(atracks[a].GetCenter()[0]-btracks[b][0],2)+pow(atracks[a].GetCenter()[1]-btracks[b][1],2));
            cv::Rect b_rect(cv::Point(btracks[b][4],btracks[b][5]),cv::Point(btracks[b][6],btracks[b][7]));
            cv::Rect Intersection = rects[a] & b_rect;
            cv::Rect Union = rects[a]|b_rect;
            double iou=1-double(Intersection.area())/double(Union.area());
            double dis=distance_3D<distance_thres?distance_3D/distance_thres:1;
            cost_matrix(a,b)=dis;
        }
    }
    return cost_matrix;
}

// Eigen::MatrixXd getFusionCost(std::vector<Kalman_filter_plus> atracks,std::vector<std::array<int, 4>> btracks,
//     int &atracks_size, int &btracks_size,double distance_weight,double distance_thres,double color_weight){
//     atracks_size=atracks.size();btracks_size=btracks.size();
//     if (atracks_size*btracks_size==0){
//         Eigen::MatrixXd cost_matrix;
//         return cost_matrix;
//     }
//     double last_time_cost,distance_cost,history_cost;
//     Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(atracks_size,btracks_size);
//     for(int a = 0; a<atracks_size; a++){
//         //last_time_cost= 1-atracks[a].last_time/1.5;//根据丢失时间设置权值
//         std::array<std::array<int,6>,2> color_number={{
//             {{0, 0, 0, 0, 0, 0}},  // 第一个子数组 蓝
//             {{0, 0, 0, 0, 0, 0}}  // 第二个子数组 红
//         }};
//         for(int i=0;i<atracks[a].detect_history.size();i++){
//             color_number[atracks[a].detect_history[i].first][atracks[a].detect_history[i].second]++;
//         }
//         for(int b=0;b<btracks_size;b++){
//             double distance=sqrt(pow(atracks[a].predict_point.x-btracks[b][0],2)+pow(atracks[a].predict_point.y-btracks[b][1],2));
//             distance_cost=distance<distance_thres?1-distance/distance_thres:0;
//             // history_cost=color_number[btracks[b][2]][btracks[b][3]]/20;
//             history_cost=color_number[btracks[b][2]][btracks[b][3]]/atracks[a].detect_history.size();//改进一下，未测试
//
//             cost_matrix(a,b)=distance_cost*distance_weight+history_cost*color_weight;
//             cost_matrix(a,b)=1-cost_matrix(a,b);
//         }
//     }
//     return cost_matrix;
// }

double lapjv(const std::vector<std::vector<float>> &cost,std::vector<int> &rowsol,std::vector<int> &colsol,
                          bool extend_cost,double match_thresh,bool return_cost){
    std::vector<std::vector<float> > cost_c;
    cost_c.assign(cost.begin(), cost.end());

    std::vector<std::vector<float> > cost_c_extended;

    int n_rows = cost.size();
    int n_cols = cost[0].size();
    rowsol.resize(n_rows);
    colsol.resize(n_cols);

    int n = 0;
    // if (n_rows == n_cols){
    //     n = n_rows;
    // }else{
    //     if (!extend_cost){
    //         std::cout << "set extend_cost=True" << std::endl;
    //         system("pause");
    //         exit(0);
    //     }else{
    //         n = n_rows+n_cols;
    //     }
    // }
    n = n_rows+n_cols;
    cost_c_extended.resize(n);
    for (int i = 0; i < cost_c_extended.size(); i++)
        cost_c_extended[i].resize(n);
    
    for (int i = 0; i < cost_c_extended.size(); i++){
        for (int j = 0; j < cost_c_extended[i].size(); j++){
            cost_c_extended[i][j] = match_thresh/2;//？？
        }
    }

    //调整代价值，将部分扩展的代价值调成0
    for (int i = n_rows; i < cost_c_extended.size(); i++){
        for (int j = n_cols; j < cost_c_extended[i].size(); j++){
            cost_c_extended[i][j] = 0;
        }
    }
    //调整代价值，将原矩阵的值赋给扩展矩阵未扩展的部分
    for (int i = 0; i < n_rows; i++){
        for (int j = 0; j < n_cols; j++){
            cost_c_extended[i][j] = cost_c[i][j];
        }
    }

    cost_c.clear();
    cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());

    double **cost_ptr;//声明一个指向二维数组的指针
    cost_ptr = new double *[sizeof(double *) * n];//动态分配了 n 个 double * 类型的空间
    for (int i = 0; i < n; i++){
        cost_ptr[i] = new double[sizeof(double) * n];//每个都是n个空间
    }

    for (int i = 0; i < n; i++){
        for (int j = 0; j < n; j++){
            cost_ptr[i][j] = cost_c[i][j];
        }
    }

    int* x_c = new int[sizeof(int) * n];
    int *y_c = new int[sizeof(int) * n];

    int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
    if (ret != 0){
        std::cout << "Calculate Wrong!" << std::endl;
        system("pause");
        exit(0);
    }

    double opt = 0.0;

    // if (n != n_rows){
        for (int i = 0; i < n; i++){
            if (x_c[i] >= n_cols)
                x_c[i] = -1;
            if (y_c[i] >= n_rows)
                y_c[i] = -1;
        }
        for (int i = 0; i < n_rows; i++){
            rowsol[i] = x_c[i];
        }
        for (int i = 0; i < n_cols; i++){
            colsol[i] = y_c[i];
        }

        if (return_cost){
            for (int i = 0; i < rowsol.size(); i++){
                if (rowsol[i] != -1){
                    //cout << i << "\t" << rowsol[i] << "\t" << cost_ptr[i][rowsol[i]] << endl;
                    opt += cost_ptr[i][rowsol[i]];
                }
            }
        }
    // }else{
    //     for (int i = 0; i < n_rows; i++){
    //         rowsol[i] = x_c[i];
    //     }
    //     for (int i = 0; i < n_cols; i++){
    //         colsol[i] = y_c[i];
    //     }
    //     if(return_cost){
    //         for (int i = 0; i < rowsol.size(); i++){
    //             opt += cost_ptr[i][rowsol[i]];
    //         }
    //     }
    // }

    for (int i = 0; i < n; i++){
        delete[]cost_ptr[i];
    }
    delete[]cost_ptr;
    delete[]x_c;
    delete[]y_c;

    return opt;
}
void linear_assignment(std::vector<std::vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size,double match_thresh,
                             std::vector<std::vector<int> > &matches, std::vector<int> &unmatched_a, std::vector<int> &unmatched_b){
    if (cost_matrix.size() == 0){
        for (int i = 0; i < cost_matrix_size; i++){
            unmatched_a.push_back(i);
        }
        for (int i = 0; i < cost_matrix_size_size; i++){
            unmatched_b.push_back(i);
        }
        return;
    }

    std::vector<int> rowsol; std::vector<int> colsol;
    float c = lapjv(cost_matrix,rowsol,colsol,true,match_thresh,true);

    for (int i = 0; i < rowsol.size(); i++){
        // std::cout<<rowsol[i]<<std::endl;
        if (rowsol[i] >= 0){
            std::vector<int> match;
            match.push_back(i);
            match.push_back(rowsol[i]);
            matches.push_back(match);
        }else{
            unmatched_a.push_back(i);
        }
    }
    for (int i = 0; i < colsol.size(); i++){
        if (colsol[i] < 0){
            unmatched_b.push_back(i);
        }
    }
}
void vis_kal_maker(int type,visualization_msgs::msg::MarkerArray &vis_array,int color,int num,double x,double y,int history,int offset){
    visualization_msgs::msg::Marker central_point;
    visualization_msgs::msg::Marker text;

    std::string temp_name ="";
    if(type==0) temp_name="cost_";
    else if(type==1) temp_name="pc_";
    else if (type==2) temp_name="img_";
    else if (type==3) temp_name="res_";
    else if (type==4) temp_name="dep_";

    text.header.frame_id=central_point.header.frame_id="rm_frame";
    text.header.stamp=rclcpp::Clock().now();
    central_point.header.stamp=rclcpp::Clock().now();
    text.ns=temp_name+"txts";
    central_point.ns=temp_name+"points";
    central_point.action = visualization_msgs::msg::Marker::ADD;
    text.action = visualization_msgs::msg::Marker::ADD;
    central_point.pose.orientation.w = 1.0;

    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(0.1);
    duration.nanosec = static_cast<uint32_t>((0,01 - duration.sec) * 1e9);

    central_point.lifetime=duration;
    text.lifetime=duration;
    text.pose.orientation.w = 1.0;
    text.pose.position.x=x;
    text.pose.position.y=y;
    text.pose.position.z=1.5;
    if(type==0) text.pose.position.z=1.4;
    else if(type==1) text.pose.position.z=1.6;
    else if (type==2) text.pose.position.z=1.8;
    else if (type==3) text.pose.position.z=2.0;
    else if (type==4) text.pose.position.z=2.2;

    central_point.id = color*60+10*offset+num;
    text.id = color*60+10*offset+num;
    central_point.type = visualization_msgs::msg::Marker::POINTS;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    central_point.scale.x = central_point.scale.y = 0.5;
    text.scale.z=0.3;

    if(color==0){
        central_point.color.r = 1.0f;
        text.color.r = 1.0f;
        text.color.g = 0.0f;
        text.color.b = 0.0f;
        text.text=temp_name+"red"+std::to_string(num+1)+" "+std::to_string(history);
    }else{
        central_point.color.b = 1.0f;
        text.color.r = 0.0f;
        text.color.g = 0.0f;
        text.color.b = 1.0f;
        text.text=temp_name+"blue"+std::to_string(num+1)+" "+std::to_string(history);
    }

    central_point.color.a = 1.0;
    text.color.a = 0.5;

    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 1.2;
    central_point.points.push_back(p);
    vis_array.markers.push_back(central_point);
    vis_array.markers.push_back(text);
}

void vis_maker(int type,visualization_msgs::msg::MarkerArray &vis_array,int color,int num,double x,double y){
    visualization_msgs::msg::Marker central_point;
    visualization_msgs::msg::Marker text;

    std::string temp_name =""; 
    if(type==0) temp_name="cost_";
    else if(type==1) temp_name="pc_";
    else if (type==2) temp_name="img_";
    else if (type==3) temp_name="res_";
    else if (type==4) temp_name="dep_";

    text.header.frame_id=central_point.header.frame_id="rm_frame";
    text.header.stamp=central_point.header.stamp=rclcpp::Clock().now();
    text.ns=temp_name+"txts";
    central_point.ns=temp_name+"points";
    central_point.action = visualization_msgs::msg::Marker::ADD;
    text.action = visualization_msgs::msg::Marker::ADD;
    central_point.pose.orientation.w = 1.0;

    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(0.1);
    duration.nanosec = static_cast<uint32_t>((0,01 - duration.sec) * 1e9);

    central_point.lifetime=duration;
    text.lifetime=duration;
    text.pose.orientation.w = 1.0;
    text.pose.position.x=x;
    text.pose.position.y=y;
    text.pose.position.z=1.5;
    if(type==0) text.pose.position.z=1.4;
    else if(type==1) text.pose.position.z=1.6;
    else if (type==2) text.pose.position.z=1.8;
    else if (type==3) text.pose.position.z=2.0;
    else if (type==4) text.pose.position.z=2.2;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, 100);
    int random_number = dis(gen);

    central_point.id = 10*color+num;
    text.id = 10*color+num;
    central_point.type = visualization_msgs::msg::Marker::POINTS;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    central_point.scale.x = central_point.scale.y = 0.5;
    text.scale.z=0.3;
    
    if(color==0){
        central_point.color.r = 1.0f;
        text.color.r = 1.0f;
        text.color.g = 0.0f;
        text.color.b = 0.0f;
        text.text=temp_name+"red"+std::to_string(num+1);
    }else{
        central_point.color.b = 1.0f;
        text.color.r = 0.0f;
        text.color.g = 0.0f;
        text.color.b = 1.0f;
        text.text=temp_name+"blue"+std::to_string(num+1);
    }
        
    central_point.color.a = 1.0;
    text.color.a = 0.5;

    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 1.2;
    central_point.points.push_back(p);
    vis_array.markers.push_back(central_point);
    vis_array.markers.push_back(text);
}