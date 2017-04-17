#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  /**
  TODO:
    * predict the state
  */

  x_ = F_ * x_;

  cout << "F_(0, 2) is " << F_(0, 2) << endl;

  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  // MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd S = H_ * P_ * Ht + R_;
  
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;


  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  double rho;
  double phi;
  double rho_dot;

  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];

  double square_root = sqrt(px*px + py*py);
  double a_tiny_number = 0.00001;
  if (fabs(square_root) > a_tiny_number) {
    rho = square_root;
  } else {
    rho = a_tiny_number;
  }

//  if ((px != 0) && (py != 0)) {
  if (px != 0 ) {
    phi = atan2(py, px);
    rho_dot = ((px*vx + py*vy) / rho);
  } else {
    phi = 0;
    rho_dot = 0;
  }

  VectorXd H_of_x = VectorXd(3);
  H_of_x << rho, phi, rho_dot;

  VectorXd y = z - H_of_x;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
