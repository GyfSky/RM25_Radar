#include"../../General/include/General.h"
class KRepresent
{
private:
    double clusterNum[3] = {0}; // 聚类点数
    double x_clusterAll[3] = {0}; // 聚类总值
    double y_clusterAll[3] = {0}; // 聚类总值
    double z_clusterAll[3] = {0}; // 聚类总值
    double back_range_thresh; // 背景深度阈值
    double new_range_thresh;  // 最后的前景深度阈值
    double cluster_thresh;    // 聚类阈值
    int cluster_times;        // 聚类的次数
    cv::Mat depthMat;
public:
    KRepresent();
    // ~KRepresent();
    void Run(cv::Mat &depthMat, cv::Mat &T_Main2world, std::vector<Car> &cars);
    void Run(cv::Mat &depthMat, cv::Mat &T_Main2world, cv::Rect &test);
    cv::Point3d KRepresent_(cv::Rect &imgLocation);
    void KMeans(int edge[4], double x_clusters[3], double y_clusters[3] ,double z_clusters[3]);
    void setDepthMat(cv::Mat &depthCompetitionMat);
};
double GetDisstance(double nowCluster[3],double x_world,double y_world,double z_world);