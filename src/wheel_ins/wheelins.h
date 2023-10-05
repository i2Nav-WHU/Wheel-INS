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

#ifndef GI_ENGINE_H
#define GI_ENGINE_H

#include <Eigen/Dense>
#include <vector>
#include <queue>
#include "common/types.h"

#include "utils.h"

class WheelINS {

public:
    explicit WheelINS(Paras &options);

    ~WheelINS() = default;

    /**
     * @brief 添加新的IMU数据，(不)补偿IMU误差
     *        add new imudata, do (not) compensate imu error
     * @param [in] imu        新的IMU原始数据
     *                        new raw imudata
     * @param [in] compensate 是否补偿IMU误差
     *                        if compensate imu error to new imudata
     * */
    void addImuData(const IMU &imu, bool compensate = false) {

        imupre_ = imucur_;
        imucur_ = imu;

        if (compensate) {
            imuCompensate(imucur_);
        }

        if(imuBuff.size()<imuBuffsize)
        {
            imuBuff.push_back(imucur_);
        }
        else {
            imuBuff.pop_front();
            imuBuff.push_back(imucur_);
        }
    }


    /**
     * @brief 处理新的IMU数据
     *        process new imudata
     * */
    void newImuProcess();


    /**
     * @brief 获取当前时间
     *        get current time
     * */
    double timestamp() const {
        return timestamp_;
    }

    /**
     * @brief 获取当前IMU状态
     *        get current navigation state
     * */
    NavState getNavState();

    /**
     * @brief 获取当前状态协方差
     *        get current state covariance
     * */
    Eigen::MatrixXd getCovariance() {
        return Cov_;
    }

private:
    /**
     * @brief 初始化系统状态和协方差
     *        initialize state and state covariance
     * @param [in] initstate     初始状态
     *                           initial state
     * @param [in] initstate_std 初始状态标准差
     *                           initial state std
     * */
    void initialize(const NavState &initstate, const NavState &initstate_std);

    /**
     * @brief 当前IMU误差补偿到IMU数据中
     *        componsate imu error to the imudata
     * @param [in,out] imu 需要补偿的IMU数据
     *                     imudata to be compensated
     * */
    void imuCompensate(IMU &imu);

    void ODOUpdate();

    Matrix3d computeVehicleRotmat();

    void getWheelVelocity();

    /**
     * @brief 进行INS状态更新(IMU机械编排算法), 并计算IMU状态转移矩阵和噪声阵
     *        do INS state update(INS mechanization), and compute state transition matrix and noise matrix
     * @param [in,out] imupre 前一时刻IMU数据
     *                        imudata at the previous epoch
     * @param [in,out] imucur 当前时刻IMU数据
     *                        imudata at the current epoch
     * */
    void insPropagation(IMU &imupre, IMU &imucur);


    /**
     * @brief Kalman 预测,
     *        Kalman Filter Predict process
     * @param [in,out] Phi 状态转移矩阵
     *                     state transition matrix
     * @param [in,out] Qd  传播噪声矩阵
     *                     propagation noise matrix
     * */
    void EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd);

    /**
     * @brief Kalman 更新
     *        Kalman Filter Update process
     * @param [in] dz 观测新息
     *                measurement innovation
     * @param [in] H  观测矩阵
     *                measurement matrix
     * @param [in] R  观测噪声阵
     *                measurement noise matrix
     * */
    void EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R);


    /**
     * @brief 反馈误差状态到当前状态
     *        feedback error state to the current state
     * */
    void stateFeedback();

    /**
     * @brief 检查协方差对角线元素是否都为正
     *        Check if covariance diagonal elements are all positive
     * */
    void checkCov() {

        for (int i = 0; i < RANK; i++) {
            if (Cov_(i, i) < 0) {
                std::cout << "Covariance is negative at " << std::setprecision(10) << timestamp_ << " !" << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
    }

private:
    Paras options_;

    double timestamp_;

    double odo_update_t;

    double wheelVelocity;

    // IMU和GNSS原始数据
    // raw imudata and gnssdata
    IMU imupre_;
    IMU imucur_;

    // IMU状态（位置、速度、姿态和IMU误差）
    // imu state (position, velocity, attitude and imu error)
    PVA pvacur_;
    PVA pvapre_;
    ImuError imuerror_;

    // Kalman滤波相关
    // ekf variables
    Eigen::MatrixXd Cov_;
    Eigen::MatrixXd Qc_;
    Eigen::MatrixXd dx_;

    
    std::deque<IMU> imuBuff;

    const int imuBuffsize = 0.1 * IMU_RATE;

    // 状态ID和噪声ID
    // state ID and noise ID
    enum StateID { P_ID = 0, V_ID = 3, PHI_ID = 6, BG_ID = 9, BA_ID = 12, SG_ID = 15, SA_ID = 18 };
    enum NoiseID { VRW_ID = 0, ARW_ID = 3, BGSTD_ID = 6, BASTD_ID = 9, SGSTD_ID = 12, SASTD_ID = 15 };
};

#endif // GI_ENGINE_H
