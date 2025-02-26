#include "../include/kalmanFilter.h"
#include <Eigen/Cholesky>

//1.Eigen::MatrixXf::Identity(4, 8)是一个4x8的浮点数矩阵，其中对角线上的元素为1，其他元素为0。
//它是Eigen库中用于创建单位矩阵的函数。
// 1 0 0 0 0 0 0 0
// 0 1 0 0 0 0 0 0
// 0 0 1 0 0 0 0 0
// 0 0 0 1 0 0 0 0



namespace byte_kalman
{
	const double KalmanFilter::chi2inv95[10] = {
	0,
	3.8415,
	5.9915,
	7.8147,
	9.4877,
	11.070,
	12.592,
	14.067,
	15.507,
	16.919
	};
	KalmanFilter::KalmanFilter(double dt,bool is3D)
	{
        if(is3D){
            int ndim = 6;
//            dt = 1.0/3; //TODO:
            _motion_mat_A = Eigen::MatrixXf::Identity(12, 12);
            _motion_mat_A.block(0,ndim,ndim,ndim) =  Eigen::MatrixXf::Identity(6, 6) * dt;
//            std::cout << "A" << _motion_mat_A << std::endl;
            _update_mat_H = Eigen::MatrixXf::Identity(6, 12);
            _R_weight_position = 1. / 100;
            _R_weight_velocity2d = 1. / 35;
            _R_weight_position3d = 0.01;
            _R_weight_velocity3d = 1. / 100; //TODO：
//            _R_weight_position = 1. / 1000;
//            _R_weight_velocity2d = 1. / 350;
//            _R_weight_position3d = 0.001;
//            _R_weight_velocity3d = 1. / 1000; //TODO：

//            _R_weight_position = 40;
//            _R_weight_velocity2d = 30;
//            _R_weight_position3d = 0.;
//            _R_weight_velocity3d = 2; //TODO：
        }
        else{
            int ndim = 4;
            dt = 1.;

            _motion_mat = Eigen::MatrixXf::Identity(8, 8);
            for (int i = 0; i < ndim; i++) {
                _motion_mat(i, ndim + i) = dt;
            }
            _update_mat = Eigen::MatrixXf::Identity(4, 8);

            this->_std_weight_position = 1. / 20;
            this->_std_weight_velocity = 1. / 40;
        }

	}

	KAL_DATA KalmanFilter::initiate(const DETECTBOX &measurement)
	{
		DETECTBOX mean_pos = measurement;
		DETECTBOX mean_vel;
		for (int i = 0; i < 4; i++) mean_vel(i) = 0;

		KAL_MEAN mean;
		for (int i = 0; i < 8; i++) {
			if (i < 4) mean(i) = mean_pos(i);
			else mean(i) = mean_vel(i - 4);
		}

		KAL_MEAN std;
		std(0) = 2 * _std_weight_position * measurement[3];
		std(1) = 2 * _std_weight_position * measurement[3];
		std(2) = 1e-2;
		std(3) = 2 * _std_weight_position * measurement[3];
		std(4) = 10 * _std_weight_velocity * measurement[3];
		std(5) = 10 * _std_weight_velocity * measurement[3];
		std(6) = 0.05;
		std(7) = 10 * _std_weight_velocity * measurement[3];

		KAL_MEAN tmp = std.array().square(); //将数组中的元素进行平方操作
		KAL_COVA var = tmp.asDiagonal();	 //将结果构造成对角矩阵((状态转移矩阵A))
		return std::make_pair(mean, var);
	}

	void KalmanFilter::predict(KAL_MEAN &mean, KAL_COVA &covariance)
	{
		//revise the data;
		DETECTBOX std_pos;
		std_pos << _std_weight_position * mean(3),
			_std_weight_position * mean(3),
			1e-2,
			_std_weight_position * mean(3);
		DETECTBOX std_vel;
		std_vel << _std_weight_velocity * mean(3),
			_std_weight_velocity * mean(3),
			0.05,
			_std_weight_velocity * mean(3);
		KAL_MEAN tmp;
		tmp.block<1, 4>(0, 0) = std_pos;
		tmp.block<1, 4>(0, 4) = std_vel;
		tmp = tmp.array().square();
		KAL_COVA motion_cov = tmp.asDiagonal();
		KAL_MEAN mean1 = this->_motion_mat * mean.transpose();
		KAL_COVA covariance1 = this->_motion_mat * covariance *(_motion_mat.transpose());
		covariance1 += motion_cov;

		mean = mean1;
		covariance = covariance1;
	}

	KAL_HDATA KalmanFilter::project(const KAL_MEAN &mean, const KAL_COVA &covariance)
	{
		DETECTBOX std;
		std << _std_weight_position * mean(3), _std_weight_position * mean(3),
			1e-1, _std_weight_position * mean(3);
		KAL_HMEAN mean1 = _update_mat * mean.transpose();
		KAL_HCOVA covariance1 = _update_mat * covariance * (_update_mat.transpose());
		Eigen::Matrix<float, 4, 4> diag = std.asDiagonal();
		diag = diag.array().square().matrix();
		covariance1 += diag;
		//    covariance1.diagonal() << diag;
		return std::make_pair(mean1, covariance1);
	}

	KAL_DATA KalmanFilter::update(
			const KAL_MEAN &mean,
			const KAL_COVA &covariance,
			const DETECTBOX &measurement)
	{
		KAL_HDATA pa = project(mean, covariance);
		KAL_HMEAN projected_mean = pa.first;
		KAL_HCOVA projected_cov = pa.second;

		//chol_factor, lower =
		//scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)
		//kalmain_gain =
		//scipy.linalg.cho_solve((cho_factor, lower),
		//np.dot(covariance, self._upadte_mat.T).T,
		//check_finite=False).T
		Eigen::Matrix<float, 4, 8> B = (covariance * (_update_mat.transpose())).transpose();
		Eigen::Matrix<float, 8, 4> kalman_gain = (projected_cov.llt().solve(B)).transpose(); // eg.8x4
		Eigen::Matrix<float, 1, 4> innovation = measurement - projected_mean; //eg.1x4
		auto tmp = innovation * (kalman_gain.transpose());
		KAL_MEAN new_mean = (mean.array() + tmp.array()).matrix();
		KAL_COVA new_covariance = covariance - kalman_gain * projected_cov*(kalman_gain.transpose());
		return std::make_pair(new_mean, new_covariance);
	}

	Eigen::Matrix<float, 1, -1>
		KalmanFilter::gating_distance(
			const KAL_MEAN &mean,
			const KAL_COVA &covariance,
			const std::vector<DETECTBOX> &measurements,
			bool only_position)
	{
		KAL_HDATA pa = this->project(mean, covariance);
		if (only_position) {
			printf("not implement!");
			exit(0);
		}
		KAL_HMEAN mean1 = pa.first;
		KAL_HCOVA covariance1 = pa.second;

		//    Eigen::Matrix<float, -1, 4, Eigen::RowMajor> d(size, 4);
		DETECTBOXSS d(measurements.size(), 4);
		int pos = 0;
		for (DETECTBOX box : measurements) {
			d.row(pos++) = box - mean1;
		}
	// hn  ->  (H * P * H.T + R) * z =  (z - H * x)  -> z = ;//''/;//
		// covariance1 = A * A.T; factor = A
		Eigen::Matrix<float, -1, -1, Eigen::RowMajor> factor = covariance1.llt().matrixL();

		// 使用下三角视图的因子来求解线性方程组，并将结果转置后存储在矩阵z中
		// triangularView<Eigen::Lower>()表示对factor进行下三角视图的操作
		// solve<Eigen::OnTheRight>(d)表示使用下三角视图的因子来求解线性方程组，其中d是另一个已知的向量。
		Eigen::Matrix<float, -1, -1> z = factor.triangularView<Eigen::Lower>().solve<Eigen::OnTheRight>(d).transpose();
		auto zz = ((z.array())*(z.array())).matrix();
		auto square_maha = zz.colwise().sum();
		return square_maha;
	}

////------------------------------  3d  ----------------------------------------------------------------

    KAL_DATA_3d KalmanFilter::initiate(const DETECTBOX_Z &measurement)
    {
        DETECTBOX_Z mean_pos = measurement;
        DETECTBOX_Z mean_vel;
        for (int i = 0; i < 6; i++) mean_vel(i) = 0;

        KAL_MEAN_3d mean;
        for (int i = 0; i < 12; i++) {
            if (i < 6) mean(i) = mean_pos(i);
            else mean(i) = mean_vel(i - 6);
        }

        KAL_MEAN_3d std_R;
        std_R(0) = 2 * _R_weight_position * measurement[3];
        std_R(1) = 2 * _R_weight_position * measurement[3];
        std_R(2) = 1e-2;
        std_R(3) = 2 * _R_weight_position * measurement[3];
        std_R(4) = _R_weight_position3d;
        std_R(5) = _R_weight_position3d;
        std_R(6) = 10 * _R_weight_velocity2d * measurement[3];
        std_R(7) = 10 * _R_weight_velocity2d * measurement[3];
        std_R(8) = 0.05;
        std_R(9) = 10 * _R_weight_velocity2d * measurement[3];
        std_R(10) =  _R_weight_velocity3d;
        std_R(11) =  _R_weight_velocity3d;


        KAL_MEAN_3d tmp = std_R.array().square(); //将数组中的元素进行平方操作
        KAL_COVA_3d var = tmp.asDiagonal();	 //将结果构造成对角矩阵((状态转移矩阵A))
        return std::make_pair(mean, var);
    }

    void KalmanFilter::predict(KAL_MEAN_3d &mean, KAL_COVA_3d &covariance)
    {
        //revise the data;
        DETECTBOX_Z std_R_pos;
        std_R_pos << _R_weight_position * mean(3),
                _R_weight_position * mean(3),
                1e-2,
                _R_weight_position * mean(3),
                _R_weight_position3d,
                _R_weight_position3d;
        DETECTBOX_Z std_R_vel;
        std_R_vel << _R_weight_velocity2d * mean(3),
                _R_weight_velocity2d * mean(3),
                0.05,
                _R_weight_velocity2d * mean(3),
                _R_weight_velocity3d,
                _R_weight_velocity3d;
        KAL_MEAN_3d tmp;
        tmp.block<1, 6>(0, 0) = std_R_pos;
        tmp.block<1, 6>(0, 6) = std_R_vel;
        tmp = tmp.array().square();
        KAL_COVA_3d motion_cov = tmp.asDiagonal();
        KAL_MEAN_3d mean1 = this->_motion_mat_A * mean.transpose();
        KAL_COVA_3d covariance1 = this->_motion_mat_A * covariance *(_motion_mat_A.transpose());
        covariance1 += motion_cov;

        mean = mean1;
        covariance = covariance1;
    }

    KAL_HDATA_3d KalmanFilter::project(const KAL_MEAN_3d &mean, const KAL_COVA_3d &covariance)
    {
        DETECTBOX_Z std_R;
        std_R << _R_weight_position * mean(3), _R_weight_position * mean(3),
                1e-1, _R_weight_position * mean(3), _R_weight_position3d, _R_weight_position3d;
        KAL_HMEAN_Z mean1 = _update_mat_H * mean.transpose();
        KAL_HCOVA_R covariance1 = _update_mat_H * covariance * (_update_mat_H.transpose());
        Eigen::Matrix<float, 6, 6> diag = std_R.asDiagonal();
        diag = diag.array().square().matrix();
        covariance1 += diag;
        //    covariance1.diagonal() << diag;
        return std::make_pair(mean1, covariance1);
    }

    KAL_DATA_3d KalmanFilter::update(
            const KAL_MEAN_3d &mean,
            const KAL_COVA_3d &covariance,
            const DETECTBOX_Z &measurement)
    {
        KAL_HDATA_3d pa = project(mean, covariance);
        KAL_HMEAN_Z projected_mean = pa.first;
        KAL_HCOVA_R projected_cov = pa.second;

        //chol_factor, lower =
        //scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)
        //kalmain_gain =
        //scipy.linalg.cho_solve((cho_factor, lower),
        //np.dot(covariance, self._upadte_mat.T).T,
        //check_finite=False).T
        Eigen::Matrix<float, 6, 12> B = (covariance * (_update_mat_H.transpose())).transpose();
        Eigen::Matrix<float, 12, 6> kalman_gain = (projected_cov.llt().solve(B)).transpose(); // eg.8x4
        Eigen::Matrix<float, 1, 6> innovation = measurement - projected_mean; //eg.1x4
        auto tmp = innovation * (kalman_gain.transpose());
        KAL_MEAN_3d new_mean = (mean.array() + tmp.array()).matrix();
        KAL_COVA_3d new_covariance = covariance - kalman_gain * projected_cov*(kalman_gain.transpose());
        return std::make_pair(new_mean, new_covariance);
    }

}