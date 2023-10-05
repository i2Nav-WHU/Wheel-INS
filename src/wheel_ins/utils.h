/*
 * KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Liqiang Wang
 *    Contact : wlq@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef KF_GINS_TYPES_H
#define KF_GINS_TYPES_H

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

#include "common/angle.h"

static constexpr double NormG = 9.782940329221166;

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

    // 初始状态和状态标准差
    // initial state and state standard deviation
    NavState initstate;
    NavState initstate_std;

    // IMU噪声参数
    // imu noise parameters
    ImuNoise imunoise;

    // 安装参数
    // install parameters
    Eigen::Vector3d mountAngle = {0, 0, 0};

    Eigen::Vector3d leverArm = {0, 0, 0};
    Eigen::Vector3d odo_measurement_std = {0, 0, 0};


    double odo_update_interval;
    double wheelradius;
    
    bool ifCompensateVelocity;

    double starttime;


} Paras;

#endif // KF_GINS_TYPES_H
