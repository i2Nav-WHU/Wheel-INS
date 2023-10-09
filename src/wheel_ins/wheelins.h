#pragma once

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <queue>
#include <vector>

#include "common/types.h"

class WheelINS {
 public:
  explicit WheelINS(Paras &options);

  ~WheelINS() = default;

  void addImuData(const IMU &imu) {
    imupre_ = imucur_;
    imucur_ = imu;

    if (imuBuff_.size() < imuBuffsize) {
      imuBuff_.push_back(imucur_);
    } else {
      imuBuff_.pop_front();
      imuBuff_.push_back(imucur_);
    }
  }

  void newImuProcess();

  double timestamp() const { return timestamp_; }

  NavState getNavState();

  Eigen::MatrixXd getCovariance() { return Cov_; }

 private:
  void initialize(const NavState &initstate, const NavState &initstate_std);

  void ODOUpdate();

  Matrix3d computeVehicleRotmat();

  void getWheelVelocity();

  void insPropagation(IMU &imupre, IMU &imucur);

  void EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd);

  void EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R);

  void stateFeedback();

  void checkCov() {
    for (int i = 0; i < RANK; i++) {
      if (Cov_(i, i) < 0) {
        std::cout << "Covariance is negative at " << std::setprecision(10)
                  << timestamp_ << " !" << std::endl;
        std::exit(EXIT_FAILURE);
      }
    }
  }

 private:
  Paras paras_;

  double timestamp_;

  double odo_update_t;

  double wheelVelocity;

  IMU imupre_;
  IMU imucur_;

  PVA pvacur_;
  PVA pvapre_;
  ImuError imuerror_;

  Eigen::MatrixXd Cov_;
  Eigen::MatrixXd Qc_;
  Eigen::MatrixXd delta_x_;

  std::deque<IMU> imuBuff_;

  const int imuBuffsize = 0.1 * IMU_RATE;

  enum StateID {
    P_ID = 0,
    V_ID = 3,
    PHI_ID = 6,
    BG_ID = 9,
    BA_ID = 12,
    SG_ID = 15,
    SA_ID = 18
  };
  enum NoiseID {
    VRW_ID = 0,
    ARW_ID = 3,
    BGSTD_ID = 6,
    BASTD_ID = 9,
    SGSTD_ID = 12,
    SASTD_ID = 15
  };
};
