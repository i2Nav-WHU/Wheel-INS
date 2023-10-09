#include "insmech.h"

#include "common/rotation.h"

void INSMech::insMech(const PVA &pvapre, PVA &pvacur, const IMU &imupre,
                      const IMU &imucur) {
  Eigen::Vector3d d_vfb, d_vfn, d_vgn, gl;
  Eigen::Vector3d temp1, temp2, temp3;
  Eigen::Vector3d imucur_dvel, imucur_dtheta, imupre_dvel, imupre_dtheta;

  imucur_dvel = imucur.acceleration * imucur.dt;
  imucur_dtheta = imucur.angular_velocity * imucur.dt;
  imupre_dvel = imupre.acceleration * imupre.dt;
  imupre_dtheta = imupre.angular_velocity * imupre.dt;

  temp1 = imucur_dtheta.cross(imucur_dvel) / 2;
  temp2 = imupre_dtheta.cross(imucur_dvel) / 12;
  temp3 = imupre_dvel.cross(imucur_dtheta) / 12;

  d_vfb = imucur_dvel + temp1 + temp2 + temp3;

  d_vfn = pvapre.att.cbn * d_vfb;

  gl << 0, 0, NormG;
  d_vgn = gl * imucur.dt;

  pvacur.vel = pvapre.vel + d_vfn + d_vgn;

  Eigen::Vector3d midvel;

  midvel = (pvacur.vel + pvapre.vel) / 2;
  pvacur.pos = pvapre.pos + midvel * imucur.dt;

  Eigen::Quaterniond qbb;
  Eigen::Vector3d rot_bframe;

  rot_bframe = imucur_dtheta + imupre_dtheta.cross(imucur_dtheta) / 12;
  qbb = Rotation::rotvec2quaternion(rot_bframe);

  pvacur.att.qbn = pvapre.att.qbn * qbb;
  pvacur.att.cbn = Rotation::quaternion2matrix(pvacur.att.qbn);
  pvacur.att.euler = Rotation::matrix2euler(pvacur.att.cbn);
}

void INSMech::imuCompensate(IMU &imu, ImuError &imuerror) {
  imu.angular_velocity -= imuerror.gyrbias;
  imu.acceleration -= imuerror.accbias;

  Eigen::Vector3d gyrscale, accscale;
  gyrscale = Eigen::Vector3d::Ones() + imuerror.gyrscale;
  accscale = Eigen::Vector3d::Ones() + imuerror.accscale;
  imu.angular_velocity =
      imu.angular_velocity.cwiseProduct(gyrscale.cwiseInverse());
  imu.acceleration = imu.acceleration.cwiseProduct(accscale.cwiseInverse());
}