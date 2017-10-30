#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
 // x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /**
     TODO:
     * predict the state
     */
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
     TODO:
     * update the state by using Kalman Filter equations
     */
    VectorXd z_pred = H_ * x_;
    UpdateKF(z_pred,z);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
     TODO:
     * update the state by using Extended Kalman Filter equations
     */
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    float rho = sqrt(pow(px,2)+pow(py,2));
    // Limit returning phi in [-pi, pi]
    float phi = atan2(py,px);
    float rho_dot = (px * vx + py * vy)/rho;


    //static_assert(phi<=M_PI && phi >=-M_PI,"out of range");
    if(phi>=M_PI && phi <=-M_PI)
        std::cout << "error: phi is out of range"<< std::endl;

    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;
    UpdateKF(z_pred,z);
}

void KalmanFilter::UpdateKF(const VectorXd &z_pred, const VectorXd &z)
{
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * H_) * P_;
}
