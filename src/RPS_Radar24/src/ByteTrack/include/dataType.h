#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "../../General/include/General.h"
//#include "../../../../Locate/include/MapAOV.h"

// 2d
//1.由于使用了RowMajor存储顺序，这些值将按照行主序存储在内存中。
//2.Eigen::Dynamic 创建动态大小的矩阵和向量
//3.访问 pair 中的元素 std::cout << "First element: " << KAL_HDATA.first << std::endl;

typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> DETECTBOX;
typedef Eigen::Matrix<float, -1, 4, Eigen::RowMajor> DETECTBOXSS;
//typedef Eigen::Matrix<float, 1, 128, Eigen::RowMajor> FEATURE;
typedef Eigen::Matrix<float, Eigen::Dynamic, 128, Eigen::RowMajor> FEATURESS;
//typedef std::vector<FEATURE> FEATURESS;

//Kalmanfilter
//typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_FILTER;
typedef Eigen::Matrix<float, 1, 8, Eigen::RowMajor> KAL_MEAN; //卡尔曼滤波器中的状态量向量(A)(系统状态X)
typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_COVA; //卡尔曼滤波器中的协方差(P)
typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> KAL_HMEAN;//卡尔曼滤波器中的观察向量（Xk）
typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> KAL_HCOVA;//卡尔曼滤波器中的观测矩阵（H）
using KAL_DATA = std::pair<KAL_MEAN, KAL_COVA>;
using KAL_HDATA = std::pair<KAL_HMEAN, KAL_HCOVA>;

//main
using RESULT_DATA = std::pair<int, DETECTBOX>;

//tracker:
using TRACKER_DATA = std::pair<int, FEATURESS>;
using MATCH_DATA = std::pair<int, int>;
typedef struct t {
	std::vector<MATCH_DATA> matches;
	std::vector<int> unmatched_tracks;
	std::vector<int> unmatched_detections;
}TRACHER_MATCHD;

////linear_assignment:
//typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> DYNAMICM;


//3d  zk and zks
typedef Eigen::Matrix<float, 1, 6, Eigen::RowMajor> DETECTBOX_Z;
typedef Eigen::Matrix<float, -1, 6, Eigen::RowMajor> DETECTBOXS_ZS;

//Kalmanfilter
/*
 * KAL_MEAN_3d 与xk相关的一些量，形如xk的一些量，如xk的噪声
 *
 */
typedef Eigen::Matrix<float, 1 , 12, Eigen::RowMajor> KAL_MEAN_3d; //卡尔曼滤波器中的状态量向量(A)(系统状态X)
typedef Eigen::Matrix<float, 12, 12, Eigen::RowMajor> KAL_COVA_3d; //卡尔曼滤波器中的协方差(P)
typedef Eigen::Matrix<float, 1, 6, Eigen::RowMajor> KAL_HMEAN_Z;//卡尔曼滤波器中的观察向量（Xk）
typedef Eigen::Matrix<float, 6, 6, Eigen::RowMajor> KAL_HCOVA_R;//卡尔曼滤波器中的观测矩阵（H）
using KAL_DATA_3d = std::pair<KAL_MEAN_3d, KAL_COVA_3d >;
using KAL_HDATA_3d = std::pair<KAL_HMEAN_Z , KAL_HCOVA_R>;
