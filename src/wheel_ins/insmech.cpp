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


#include "common/rotation.h"

#include "insmech.h"

void INSMech::insMech(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur) {

    Eigen::Vector3d d_vfb, d_vfn, d_vgn, gl;
    Eigen::Vector3d temp1, temp2, temp3;
    Eigen::Vector3d imucur_dvel, imucur_dtheta, imupre_dvel, imupre_dtheta;

    imucur_dvel = imucur.acceleration * imucur.dt;
    imucur_dtheta = imucur.angular_velocity * imucur.dt;
    imupre_dvel = imupre.acceleration * imupre.dt;
    imupre_dtheta = imupre.angular_velocity * imupre.dt;

    // rotational and sculling motion
    temp1 = imucur_dtheta.cross(imucur_dvel) / 2;
    temp2 = imupre_dtheta.cross(imucur_dvel) / 12;
    temp3 = imupre_dvel.cross(imucur_dtheta) / 12;


    // velocity increment due to the specific force
    d_vfb = imucur_dvel + temp1 + temp2 + temp3;


    // velocity increment dut to the specfic force projected to the n-frame
    d_vfn = pvapre.att.cbn * d_vfb;


    // velocity increment due to the gravity and Coriolis force
    gl << 0, 0, NormG;
    d_vgn = gl * imucur.dt;

    // velocity update finish
    pvacur.vel = pvapre.vel + d_vfn + d_vgn;


    Eigen::Vector3d midvel;

    // recompute velocity and position at k-1/2
    midvel = (pvacur.vel + pvapre.vel) / 2;
    pvacur.pos = pvapre.pos + midvel * imucur.dt;

    Eigen::Quaterniond qbb;
    Eigen::Vector3d rot_bframe;


    // b-frame rotation vector (b(k) with respect to b(k-1)-frame)
    // compensate the second-order coning correction term.
    rot_bframe = imucur_dtheta + imupre_dtheta.cross(imucur_dtheta) / 12;
    qbb   = Rotation::rotvec2quaternion(rot_bframe);

    // attitude update finish
    pvacur.att.qbn   = pvapre.att.qbn * qbb;
    pvacur.att.cbn   = Rotation::quaternion2matrix(pvacur.att.qbn);
    pvacur.att.euler = Rotation::matrix2euler(pvacur.att.cbn);
}