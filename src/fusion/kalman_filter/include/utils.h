#include "lapjv.h"
#include "filter_plus.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include<random>


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

Eigen::MatrixXd getCloudCost(pcl::PointCloud<pcl::PointXYZ>atracks, std::vector<Kalman_filter_plus>btracks,
    int &atracks_size, int &btracks_size,double distance_thres){
    atracks_size=atracks.points.size();btracks_size=btracks.size();
    if (atracks_size*btracks_size==0){
        Eigen::MatrixXd cost_matrix;
        return cost_matrix;
    }
    double distance_cost;
    Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(atracks_size,btracks_size);
    for(int a = 0; a<atracks_size; a++){
        for(int b=0;b<btracks_size;b++){
            double distance=sqrt(pow(atracks.points[a].x-btracks[b].predict_point.x,2)+pow(atracks.points[a].y-btracks[b].predict_point.y,2));
            distance_cost=distance<distance_thres?distance/distance_thres:1;
            cost_matrix(a,b)=distance_cost;
            // cost_matrix(a,b)=1-cost_matrix(a,b);
        }
    }
    return cost_matrix;
}

Eigen::MatrixXd getFusionCost(std::vector<Kalman_filter_plus> atracks,std::vector<std::array<double, 4>> btracks,
    int &atracks_size, int &btracks_size,double distance_thres,double time_id,double dis_wight,double his_wight){
    atracks_size=atracks.size();btracks_size=btracks.size();
    if (atracks_size*btracks_size==0){
        Eigen::MatrixXd cost_matrix;
        return cost_matrix;
    }
    double distance_cost,history_cost;
    Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(atracks_size,btracks_size);
    for(int a = 0; a<atracks_size; a++){
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
            else history_cost=color_number[btracks[b][2]][btracks[b][3]]/atracks[a].detect_history.size();
            cost_matrix(a,b)=dis_wight*distance_cost+his_wight*history_cost;
            cost_matrix(a,b)=1-cost_matrix(a,b);
        }
    }
    return cost_matrix;
}

Eigen::MatrixXd getFusionCost(std::vector<Kalman_filter_plus> atracks,std::vector<std::array<int, 4>> btracks, 
    int &atracks_size, int &btracks_size,double distance_weight,double distance_thres,double color_weight){
    atracks_size=atracks.size();btracks_size=btracks.size();
    if (atracks_size*btracks_size==0){
        Eigen::MatrixXd cost_matrix;
        return cost_matrix;
    }
    double last_time_cost,distance_cost,history_cost;
    Eigen::MatrixXd  cost_matrix = Eigen::MatrixXd::Zero(atracks_size,btracks_size);
    for(int a = 0; a<atracks_size; a++){
        //last_time_cost= 1-atracks[a].last_time/1.5;//根据丢失时间设置权值
        std::array<std::array<int,6>,2> color_number={{
            {{0, 0, 0, 0, 0, 0}},  // 第一个子数组 蓝
            {{0, 0, 0, 0, 0, 0}}  // 第二个子数组 红
        }};
        for(int i=0;i<atracks[a].detect_history.size();i++){
            color_number[atracks[a].detect_history[i].first][atracks[a].detect_history[i].second]++;
        }
        for(int b=0;b<btracks_size;b++){
            double distance=sqrt(pow(atracks[a].predict_point.x-btracks[b][0],2)+pow(atracks[a].predict_point.y-btracks[b][1],2));
            distance_cost=distance<distance_thres?1-distance/distance_thres:0;
            // history_cost=color_number[btracks[b][2]][btracks[b][3]]/20;
            history_cost=color_number[btracks[b][2]][btracks[b][3]]/atracks[a].detect_history.size();//改进一下，未测试
            
            cost_matrix(a,b)=distance_cost*distance_weight+history_cost*color_weight;
            cost_matrix(a,b)=1-cost_matrix(a,b);
        }
    }
    return cost_matrix;
}

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