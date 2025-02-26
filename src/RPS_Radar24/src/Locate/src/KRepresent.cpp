#include"../include/KRepresent.h"


KRepresent::KRepresent(){
    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    back_range_thresh = config["tempKRepresent"]["back_range_thresh"].as<double>();
    new_range_thresh = config["tempKRepresent"]["new_range_thresh"].as<double>();
    cluster_thresh = config["tempKRepresent"]["cluster_thresh"].as<double>();
    cluster_times = config["tempKRepresent"]["cluster_times"].as<double>();
}

// KRepresent::~KRepresent()
// {
// }
//

void KRepresent::Run(cv::Mat &depthMat, cv::Mat &T_Main2world, std::vector<Car> &cars){
    this->depthMat = depthMat;
    for(auto &car:cars){
        std::cout << "XY" << (car.rect.x + car.rect.width/2) << "," << (car.rect.y + car.rect.height/2)  << std::endl;
        car.Locate3D = cv::Point3d(KRepresent_(car.rect));
        cv::Mat coordinate = (cv::Mat_<double>(4, 1)<<car.Locate3D.x, car.Locate3D.y, car.Locate3D.z, 1);
        cv::Mat result_ = T_Main2world * coordinate;
        // cv::Mat result_ = coordinate;
        std::cout   << "car:"  << result_ << std::endl;
        
    }
}

void KRepresent::Run(cv::Mat &depthMat, cv::Mat &T_Main2world, cv::Rect &test){
    this->depthMat = depthMat;
    cv::Point3d test_point3D = cv::Point3d(KRepresent_(test));
    cv::Mat coordinate = (cv::Mat_<double>(4, 1)<<test_point3D.x, test_point3D.y, test_point3D.z, 1);
    cv::Mat result_ = T_Main2world * coordinate;
    std::cout   << "car:"  << result_ << std::endl;
        
    
}

cv::Point3d KRepresent::KRepresent_(cv::Rect &imgLocation){
    /******************先定三个初始点（计算平均值，以平均值前后0.5m和平均值为三个点）******************/

    //先计算所有深度的平均值
    //所有深度的总和
    double x_all_world = 0;
    double y_all_world = 0;
    double z_all_world = 0;
    //点云个数
    double range_num = 0;
    for (int row = imgLocation.y + (int)(imgLocation.height / 2); row < imgLocation.y + imgLocation.height; row++){
        for (int col = imgLocation.x; col < imgLocation.x + imgLocation.width; col++){
            //获取深度值
            double x_world = (*(depthMat.ptr<cv::Vec3d>(row, col)))[0];
            double y_world = (*(depthMat.ptr<cv::Vec3d>(row, col)))[1];
            double z_world = (*(depthMat.ptr<cv::Vec3d>(row, col)))[2];
            if (x_world == 0 || y_world == 0 || z_world == 0 ){
                continue;
            }
            // std::cout << std::to_string(x_world) << "," << std::to_string(y_world) << "," << std::to_string(z_world) << "," << std::endl;
            x_all_world += x_world;
            y_all_world += y_world;
            z_all_world += z_world;
            range_num++;
        }
    }
    //所有深度的平均值
    double x_range_mean = x_all_world / range_num;
    double y_range_mean = y_all_world / range_num;
    double z_range_mean = z_all_world / range_num;
    // std::cout << "range_mean:" << x_range_mean << "," << y_range_mean << "," << z_range_mean << std::endl;

    //定义三个聚类中心
    double x_clusters[3] = {x_range_mean - 1.5, x_range_mean, x_range_mean + 1.5};
    double y_clusters[3] = {y_range_mean - 1.5, y_range_mean, y_range_mean + 1.5};
    double z_clusters[3] = {z_range_mean - 1.5, z_range_mean, z_range_mean + 1.5};

    /**********************************进入循环（超越次数和符合最低标准则退出）*************************/
    int times = 0;
    bool continueFlag = true;
    while (times <= cluster_times && continueFlag)
    {
        /****************************************分配三个线程任务*******************************************/
        x_clusterAll[0] = 0;
        x_clusterAll[1] = 0;
        x_clusterAll[2] = 0;

        y_clusterAll[0] = 0;
        y_clusterAll[1] = 0;
        y_clusterAll[2] = 0;

        z_clusterAll[0] = 0;
        z_clusterAll[1] = 0;
        z_clusterAll[2] = 0;

        clusterNum[0] = 0;
        clusterNum[1] = 0;
        clusterNum[2] = 0;

        //不同线程的距离
        int threadColDistance = (int)(imgLocation.width / 3);
        //线程1
        std::thread thread1 = std::thread([&]
                                {
                                    int leftEdge[4] = {0};
                                    leftEdge[0] = imgLocation.x;
                                    leftEdge[1] = imgLocation.x + threadColDistance;
                                    leftEdge[2] = imgLocation.y + (int)(imgLocation.height / 2);
                                    leftEdge[3] = imgLocation.y + imgLocation.height;
                                    this->KMeans(leftEdge, x_clusters, y_clusters, z_clusters);
                                });
        //线程2
        std::thread thread2 = std::thread([&]
                                {
                                    int middleEdge[4] = {0};
                                    middleEdge[0] = imgLocation.x + threadColDistance;
                                    middleEdge[1] = imgLocation.x + threadColDistance * 2;
                                    middleEdge[2] = imgLocation.y;
                                    middleEdge[3] = imgLocation.y + imgLocation.height;
                                    this->KMeans(middleEdge,x_clusters, y_clusters, z_clusters);
                                });
        //线程2
        std::thread thread3 = std::thread([&]
                                {
                                    int rightEdge[4] = {0};
                                    rightEdge[0] = imgLocation.x + threadColDistance * 2;
                                    rightEdge[1] = imgLocation.x + imgLocation.width;
                                    rightEdge[2] = imgLocation.y + (int)(imgLocation.height / 2);
                                    rightEdge[3] = imgLocation.y + imgLocation.height;
                                    this->KMeans(rightEdge, x_clusters, y_clusters, z_clusters);
                                });
        thread1.join();
        thread2.join();
        thread3.join();
        /****************************************计算新的聚类中心*******************************************/
        double nowClusterFront[3] = {0};
        double nowClusterMiddle[3] = {0};
        double nowClusterBack[3] = {0};
        if (clusterNum[0] != 0)
        {
            nowClusterFront[0] = x_clusterAll[0] / clusterNum[0];
            nowClusterFront[1] = y_clusterAll[0] / clusterNum[0];
            nowClusterFront[2] = z_clusterAll[0] / clusterNum[0];
        }
        if (clusterNum[1] != 0)
        {
            nowClusterMiddle[0] = x_clusterAll[1] / clusterNum[1];
            nowClusterMiddle[1] = y_clusterAll[1] / clusterNum[1];
            nowClusterMiddle[2] = z_clusterAll[1] / clusterNum[1];
        }
        if (clusterNum[2] != 0)
        {
            nowClusterBack[0] = x_clusterAll[2] / clusterNum[2];
            nowClusterBack[1] = y_clusterAll[2] / clusterNum[2];
            nowClusterBack[2] = z_clusterAll[2] / clusterNum[2];
        }
        // if (fabs(nowClusterFront[2] - z_clusters[0]) < cluster_thresh &&
        //     fabs(nowClusterMiddle[2] - z_clusters[1]) < cluster_thresh &&
        //     fabs(nowClusterBack[2] - z_clusters[2]) < cluster_thresh)
        if( GetDisstance(nowClusterFront,x_clusterAll[0],y_clusterAll[0],z_clusterAll[0]) < cluster_thresh &&
            GetDisstance(nowClusterMiddle,x_clusterAll[1],y_clusterAll[1],z_clusterAll[1]) < cluster_thresh &&
            GetDisstance(nowClusterBack,x_clusterAll[2],y_clusterAll[2],z_clusterAll[2]))
        {
            continueFlag = false;
            //cout << "因为时间符合条件退出" << endl;
        }
        x_clusters[0] = nowClusterFront[0];
        x_clusters[1] = nowClusterMiddle[0];
        x_clusters[2] = nowClusterBack[0];
        y_clusters[0] = nowClusterFront[1];
        y_clusters[1] = nowClusterMiddle[1];
        y_clusters[2] = nowClusterBack[1];
        z_clusters[0] = nowClusterFront[2];
        z_clusters[1] = nowClusterMiddle[2];
        z_clusters[2] = nowClusterBack[2];
        times++;
    }
    // cout << "cluster_front:" << cluster_front << endl;
    // cout << "cluster_middle:" << cluster_middle << endl;
    // cout << "cluster_back:" << cluster_back << endl;
    // cout << "cluster_front_num:" << cluster_front_num << endl;
    // cout << "cluster_middle_num:" << cluster_middle_num << endl;
    // cout << "cluster_back_num:" << cluster_back_num << endl;
    //以中间的聚类中心作为zRepresent

    int allNum = 0;
    double xRepresent = 0;
    double yRepresent = 0;
    double zRepresent = 0;

    for (int i = 0; i < 3; i++)
    {
        if (clusterNum[i] == 0)
        {
            continue;
        }
        allNum += clusterNum[i];
        xRepresent += (clusterNum[i] * x_clusters[i]);
        yRepresent += (clusterNum[i] * y_clusters[i]);
        zRepresent += (clusterNum[i] * z_clusters[i]);

    }
    xRepresent = xRepresent / allNum ;
    yRepresent = yRepresent / allNum ;
    zRepresent = zRepresent / allNum ;

    // std::cout << "xRepresent:" << xRepresent << std::endl;
    // std::cout << "yRepresent:" << yRepresent << std::endl;
    // std::cout << "zRepresent:" << zRepresent << std::endl;

    return  cv::Point3d(xRepresent,yRepresent,zRepresent);
}



void KRepresent::KMeans(int edge[4], double x_clusters[3], double y_clusters[3] ,double z_clusters[3]){
    // 設定邊界
    int leftEdge = edge[0];
    int rightEdge = edge[1];
    int topEdge = edge[2];
    int bottomEdge = edge[3];
    // 設定簇中心
    for (int row = topEdge; row < bottomEdge; row++){
        for (int col = leftEdge; col < rightEdge; col++){
            //获取深度值
            double world[3];
            world[0] = (*(depthMat.ptr<cv::Vec3d>(row, col)))[0];
            world[1] = (*(depthMat.ptr<cv::Vec3d>(row, col)))[1];
            world[2] = (*(depthMat.ptr<cv::Vec3d>(row, col)))[2];
            if (world[2] == 0){
                continue;
            }
            if (GetDisstance(world,x_clusters[0],y_clusters[0],z_clusters[0])< GetDisstance(world,x_clusters[1],y_clusters[1],z_clusters[1])){
                if (GetDisstance(world,x_clusters[0],y_clusters[0],z_clusters[0])< GetDisstance(world,x_clusters[2],y_clusters[2],z_clusters[2])){
                    //前面最小??????TODO
                    x_clusterAll[0] += world[0];
                    y_clusterAll[0] += world[1];
                    z_clusterAll[0] += world[2];
                    clusterNum[0]++;
                }else{
                    //后面最小
                    x_clusterAll[2] += world[0];
                    y_clusterAll[2] += world[1];
                    z_clusterAll[2] += world[2];
                    clusterNum[2]++;
                }
            }else{
                if (GetDisstance(world,x_clusters[1],y_clusters[1],z_clusters[1])< GetDisstance(world,x_clusters[2],y_clusters[2],z_clusters[2])){
                    //中间最小
                    x_clusterAll[1] += world[0];
                    y_clusterAll[1] += world[1];
                    z_clusterAll[1] += world[2];
                    clusterNum[1]++;
                }else{
                    //后面最小
                    x_clusterAll[2] += world[0];
                    y_clusterAll[2] += world[1];
                    z_clusterAll[2] += world[2];
                    clusterNum[2]++;
                }
            }
        }
    }
}

void KRepresent::setDepthMat(cv::Mat &depthCompetitionMat){
    depthMat = depthCompetitionMat;
}

double GetDisstance(double nowCluster[3],double x_world,double y_world,double z_world){
    double dis = std::abs(std::sqrt(std::pow((nowCluster[0]-x_world),2)+std::pow((nowCluster[1]-y_world),2)+std::pow((nowCluster[2]-z_world),2)));
    return dis;
}

