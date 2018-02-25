#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
	x_ = F_ * x_ ; //+ u;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;  
}

void KalmanFilter::Update(const VectorXd &z) {
		int size_x = x_.size();
		MatrixXd I = MatrixXd::Identity(size_x,size_x);
		VectorXd y = z - H_ * x_;
		MatrixXd Ht = H_.transpose();
		MatrixXd S = H_ * P_ * Ht + R_;
		MatrixXd Si = S.inverse();
		MatrixXd K =  P_ * Ht * Si;

		//new state
		x_ = x_ + (K * y);
		P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	double px = x_(0);
	double py = x_(1);
	double vx = x_(2);
	double vy = x_(3);
 
        double c1 = sqrt(px*px + py*py);
	double c2 = atan2(py, px);
	double c3 = (px*vx + py*vy) / c1;
	VectorXd h = VectorXd(3);
	h << c1, c2, c3;

        int size_x = x_.size();
	MatrixXd I = MatrixXd::Identity(size_x,size_x);
	VectorXd y = z - h;
	while ( y(1) > M_PI || y(1) < -M_PI ) 
	{
    		if ( y(1) > M_PI )
        		y(1) -= M_PI;
       		else
        		y(1) += M_PI;
	}	
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P_ * Ht * Si;

	//new state
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;

}
