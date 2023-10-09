#pragma once

#include <math.h>

#include <Eigen/Geometry>

const double D2R = (M_PI / 180.0);
const double R2D = (180.0 / M_PI);

const int IMU_RATE = 200;
const int RANK = 21;
const int NOISERANK = 18;

static constexpr double NormG = 9.782940329221166;

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

typedef struct IMU {
  double timestamp;
  double dt;

  Vector3d angular_velocity;
  Vector3d acceleration;

} IMU;

typedef struct Attitude {
  Eigen::Quaterniond qbn;
  Eigen::Matrix3d cbn;
  Eigen::Vector3d euler;
} Attitude;

typedef struct PVA {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Attitude att;
} PVA;

typedef struct ImuError {
  Eigen::Vector3d gyrbias;
  Eigen::Vector3d accbias;
  Eigen::Vector3d gyrscale;
  Eigen::Vector3d accscale;
} ImuError;

typedef struct NavState {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d euler;

  ImuError imuerror;
} NavState;

typedef struct ImuNoise {
  Eigen::Vector3d gyr_arw;
  Eigen::Vector3d acc_vrw;
  Eigen::Vector3d gyrbias_std;
  Eigen::Vector3d accbias_std;
  Eigen::Vector3d gyrscale_std;
  Eigen::Vector3d accscale_std;
  double corr_time;
} ImuNoise;

typedef struct Paras {
  // initial state and state standard deviation
  NavState initstate;
  NavState initstate_std;

  // imu noise parameters
  ImuNoise imunoise;

  // install parameters
  Eigen::Vector3d mountAngle = {0, 0, 0};

  Eigen::Vector3d leverArm = {0, 0, 0};
  Eigen::Vector3d odo_measurement_std = {0, 0, 0};

  double odo_update_interval;
  double wheelradius;

  bool ifCompensateVelocity;

  double starttime;

} Paras;