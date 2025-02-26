#pragma once

#include "dataType.h"

namespace byte_kalman
{
	class KalmanFilter
	{
	public:
		static const double chi2inv95[10];
        KalmanFilter(double dt = 1.0/3,bool is3D = true);

    // common
    public:
		KAL_DATA initiate(const DETECTBOX& measurement);
		void predict(KAL_MEAN& mean, KAL_COVA& covariance);
		KAL_HDATA project(const KAL_MEAN& mean, const KAL_COVA& covariance);
		KAL_DATA update(const KAL_MEAN& mean,
			const KAL_COVA& covariance,
			const DETECTBOX& measurement);

		Eigen::Matrix<float, 1, -1> gating_distance(
			const KAL_MEAN& mean,
			const KAL_COVA& covariance,
			const std::vector<DETECTBOX>& measurements,
			bool only_position = false);

	private:
		Eigen::Matrix<float, 8, 8, Eigen::RowMajor> _motion_mat;
        Eigen::Matrix<float, 4, 8, Eigen::RowMajor> _update_mat;
		float _std_weight_position;
		float _std_weight_velocity;

   // add 3d
    public:
        KAL_DATA_3d initiate(const DETECTBOX_Z & measurement);
        void predict(KAL_MEAN_3d & mean, KAL_COVA_3d& covariance);
        KAL_HDATA_3d project(const KAL_MEAN_3d & mean, const KAL_COVA_3d & covariance);
        KAL_DATA_3d update(const KAL_MEAN_3d & mean,
                        const KAL_COVA_3d & covariance,
                        const DETECTBOX_Z & measurement);
    private:
        Eigen::Matrix<float, 12, 12, Eigen::RowMajor> _motion_mat_A ; // x_2d,y_2d,area,
        Eigen::Matrix<float,  6, 12, Eigen::RowMajor> _update_mat_H;
        float _R_weight_position;
        float _R_weight_velocity2d;
        float _R_weight_position3d;
        float _R_weight_velocity3d;

    };
}