#include "wheelins.h"

#include <math.h>

#include <fstream>

#include "common/rotation.h"
#include "insmech.h"

WheelINS::WheelINS(Paras &paras) {
  this->paras_ = paras;

  timestamp_ = 0;

  Cov_.resize(RANK, RANK);
  Qc_.resize(NOISERANK, NOISERANK);
  delta_x_.resize(RANK, 1);
  Cov_.setZero();
  Qc_.setZero();
  delta_x_.setZero();

  auto imunoise = paras_.imunoise;
  Qc_.block(ARW_ID, ARW_ID, 3, 3) =
      imunoise.gyr_arw.cwiseProduct(imunoise.gyr_arw).asDiagonal();
  Qc_.block(VRW_ID, VRW_ID, 3, 3) =
      imunoise.acc_vrw.cwiseProduct(imunoise.acc_vrw).asDiagonal();
  Qc_.block(BGSTD_ID, BGSTD_ID, 3, 3) =
      2 / imunoise.corr_time *
      imunoise.gyrbias_std.cwiseProduct(imunoise.gyrbias_std).asDiagonal();
  Qc_.block(BASTD_ID, BASTD_ID, 3, 3) =
      2 / imunoise.corr_time *
      imunoise.accbias_std.cwiseProduct(imunoise.accbias_std).asDiagonal();
  Qc_.block(SGSTD_ID, SGSTD_ID, 3, 3) =
      2 / imunoise.corr_time *
      imunoise.gyrscale_std.cwiseProduct(imunoise.gyrscale_std).asDiagonal();
  Qc_.block(SASTD_ID, SASTD_ID, 3, 3) =
      2 / imunoise.corr_time *
      imunoise.accscale_std.cwiseProduct(imunoise.accscale_std).asDiagonal();

  initialize(paras_.initstate, paras_.initstate_std);

  odo_update_t = paras_.starttime;
}

void WheelINS::initialize(const NavState &initstate,
                          const NavState &initstate_std) {
  pvacur_.pos = initstate.pos;
  pvacur_.vel = initstate.vel;
  pvacur_.att.euler = initstate.euler;
  pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
  pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);

  imuerror_ = initstate.imuerror;

  pvapre_ = pvacur_;

  ImuError imuerror_std = initstate_std.imuerror;
  Cov_.block(P_ID, P_ID, 3, 3) =
      initstate_std.pos.cwiseProduct(initstate_std.pos).asDiagonal();
  Cov_.block(V_ID, V_ID, 3, 3) =
      initstate_std.vel.cwiseProduct(initstate_std.vel).asDiagonal();
  Cov_.block(PHI_ID, PHI_ID, 3, 3) =
      initstate_std.euler.cwiseProduct(initstate_std.euler).asDiagonal();
  Cov_.block(BG_ID, BG_ID, 3, 3) =
      imuerror_std.gyrbias.cwiseProduct(imuerror_std.gyrbias).asDiagonal();
  Cov_.block(BA_ID, BA_ID, 3, 3) =
      imuerror_std.accbias.cwiseProduct(imuerror_std.accbias).asDiagonal();
  Cov_.block(SG_ID, SG_ID, 3, 3) =
      imuerror_std.gyrscale.cwiseProduct(imuerror_std.gyrscale).asDiagonal();
  Cov_.block(SA_ID, SA_ID, 3, 3) =
      imuerror_std.accscale.cwiseProduct(imuerror_std.accscale).asDiagonal();
}

void WheelINS::newImuProcess() {
  timestamp_ = imucur_.timestamp;

  insPropagation(imupre_, imucur_);
  ODOUpdate();

  checkCov();

  pvapre_ = pvacur_;
  imupre_ = imucur_;
}

void WheelINS::insPropagation(IMU &imupre, IMU &imucur) {
  INSMech::imuCompensate(imucur, imuerror_);

  INSMech::insMech(pvapre_, pvacur_, imupre, imucur);

  Eigen::MatrixXd Phi, F, Qd, G;

  Phi.resizeLike(Cov_);
  F.resizeLike(Cov_);
  Qd.resizeLike(Cov_);
  G.resize(RANK, NOISERANK);
  Phi.setIdentity();
  F.setZero();
  Qd.setZero();
  G.setZero();

  F.block(P_ID, V_ID, 3, 3) = Eigen::Matrix3d::Identity();

  F.block(V_ID, PHI_ID, 3, 3) =
      Rotation::skewSymmetric(pvapre_.att.cbn * imucur.acceleration);
  F.block(V_ID, BA_ID, 3, 3) = pvapre_.att.cbn;
  F.block(V_ID, SA_ID, 3, 3) =
      pvapre_.att.cbn * (imucur.acceleration.asDiagonal());

  F.block(PHI_ID, BG_ID, 3, 3) = -pvapre_.att.cbn;
  F.block(PHI_ID, SG_ID, 3, 3) =
      -pvapre_.att.cbn * (imucur.angular_velocity.asDiagonal());

  F.block(BG_ID, BG_ID, 3, 3) =
      -1 / paras_.imunoise.corr_time * Eigen::Matrix3d::Identity();
  F.block(BA_ID, BA_ID, 3, 3) =
      -1 / paras_.imunoise.corr_time * Eigen::Matrix3d::Identity();
  F.block(SG_ID, SG_ID, 3, 3) =
      -1 / paras_.imunoise.corr_time * Eigen::Matrix3d::Identity();
  F.block(SA_ID, SA_ID, 3, 3) =
      -1 / paras_.imunoise.corr_time * Eigen::Matrix3d::Identity();

  G.block(V_ID, VRW_ID, 3, 3) = pvapre_.att.cbn;
  G.block(PHI_ID, ARW_ID, 3, 3) = pvapre_.att.cbn;
  G.block(BG_ID, BGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
  G.block(BA_ID, BASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
  G.block(SG_ID, SGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
  G.block(SA_ID, SASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();

  Phi.setIdentity();
  Phi = Phi + F * imucur.dt;

  Qd = G * Qc_ * G.transpose() * imucur.dt;
  Qd = (Phi * Qd * Phi.transpose() + Qd) / 2;

  EKFPredict(Phi, Qd);
}

void WheelINS::EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd) {
  assert(Phi.rows() == Cov_.rows());
  assert(Qd.rows() == Cov_.rows());

  Cov_ = Phi * Cov_ * Phi.transpose() + Qd;
  delta_x_ = Phi * delta_x_;
}

void WheelINS::EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H,
                         Eigen::MatrixXd &R) {
  assert(H.cols() == Cov_.rows());
  assert(dz.rows() == H.rows());
  assert(dz.rows() == R.rows());
  assert(dz.cols() == 1);

  auto temp = H * Cov_ * H.transpose() + R;
  Eigen::MatrixXd K = Cov_ * H.transpose() * temp.inverse();

  Eigen::MatrixXd I;
  I.resizeLike(Cov_);
  I.setIdentity();
  I = I - K * H;

  delta_x_ = delta_x_ + K * (dz - H * delta_x_);
  Cov_ = I * Cov_ * I.transpose() + K * R * K.transpose();
}

void WheelINS::ODOUpdate() {
  if (imuBuff_.size() < imuBuffsize) return;

  if ((imucur_.timestamp - odo_update_t) < paras_.odo_update_interval) return;

  getWheelVelocity();

  Matrix3d C_nv = computeVehicleRotmat().transpose();
  Matrix3d C_bv = C_nv * pvacur_.att.cbn;
  Vector3d velocity_vframe = C_nv * pvacur_.vel;
  Matrix3d angularVelocity_skew =
      Rotation::skewSymmetric(imucur_.angular_velocity);
  Matrix3d leverarm_skew = Rotation::skewSymmetric(paras_.leverArm);
  Matrix3d velocitySkew_nframe = C_nv * Rotation::skewSymmetric(pvacur_.vel);
  Vector3d velocity_leverarm = C_bv * angularVelocity_skew * paras_.leverArm;

  Eigen::MatrixXd Hv;
  Hv.resize(3, RANK);
  Hv.setZero();

  Hv.block<3, 3>(0, V_ID) = C_nv;
  Hv.block<3, 3>(0, PHI_ID) =
      C_nv * Rotation::skewSymmetric(pvacur_.att.cbn * angularVelocity_skew *
                                     paras_.leverArm);
  Hv.block<3, 3>(0, BG_ID) = -C_bv * leverarm_skew;
  Hv.block<3, 3>(0, SG_ID) =
      -C_bv * leverarm_skew * imucur_.angular_velocity.asDiagonal();

  Hv.block<3, 1>(0, 8) = -velocitySkew_nframe.block<3, 1>(0, 2);

  Eigen::MatrixXd Zv = velocity_vframe + velocity_leverarm;
  Zv(0) -= wheelVelocity;

  Eigen::MatrixXd Rv =
      paras_.odo_measurement_std.cwiseProduct(paras_.odo_measurement_std)
          .asDiagonal();

  EKFUpdate(Zv, Hv, Rv);

  odo_update_t = imucur_.timestamp;

  stateFeedback();
}

void WheelINS::getWheelVelocity() {
  double gyro_x_mean = 0.0;

  int i = 0;
  for (auto it = imuBuff_.begin(); it != imuBuff_.end(); ++it) {
    gyro_x_mean += it->angular_velocity[0];
  }
  gyro_x_mean /= imuBuffsize;

  wheelVelocity = -paras_.wheelradius * (gyro_x_mean);
}

Matrix3d WheelINS::computeVehicleRotmat() {
  Vector3d vehicle_euler = Vector3d::Zero();

  vehicle_euler[2] = pvacur_.att.euler[2] - M_PI / 2.0;

  Matrix3d C_vn = Rotation::euler2matrix(vehicle_euler);

  return C_vn;
}

void WheelINS::stateFeedback() {
  pvacur_.pos -= delta_x_.block(P_ID, 0, 3, 1);
  pvacur_.vel -= delta_x_.block(V_ID, 0, 3, 1);

  Vector3d delta_att = delta_x_.block(PHI_ID, 0, 3, 1);
  Eigen::Quaterniond qpn = Rotation::rotvec2quaternion(delta_att);
  pvacur_.att.qbn = qpn * pvacur_.att.qbn;
  pvacur_.att.cbn = Rotation::quaternion2matrix(pvacur_.att.qbn);
  pvacur_.att.euler = Rotation::matrix2euler(pvacur_.att.cbn);

  imuerror_.gyrbias += delta_x_.block(BG_ID, 0, 3, 1);
  imuerror_.accbias += delta_x_.block(BA_ID, 0, 3, 1);
  imuerror_.gyrscale += delta_x_.block(SG_ID, 0, 3, 1);
  imuerror_.accscale += delta_x_.block(SA_ID, 0, 3, 1);

  delta_x_.setZero();
}

NavState WheelINS::getNavState() {
  NavState state;

  state.pos = pvacur_.pos;
  state.vel = pvacur_.vel;
  state.euler = pvacur_.att.euler;
  state.imuerror = imuerror_;

  return state;
}
