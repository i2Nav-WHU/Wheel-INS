#include "common/rotation.h"

#include "wheelins.h"
#include "insmech.h"
#include <math.h>
#include <fstream>
extern std::ofstream debug_;

WheelINS::WheelINS(Paras &options) {

    this->options_ = options;
    
    timestamp_ = 0;


    Cov_.resize(RANK, RANK);
    Qc_.resize(NOISERANK, NOISERANK);
    dx_.resize(RANK, 1);
    Cov_.setZero();
    Qc_.setZero();
    dx_.setZero();

    auto imunoise                   = options_.imunoise;
    Qc_.block(ARW_ID, ARW_ID, 3, 3) = imunoise.gyr_arw.cwiseProduct(imunoise.gyr_arw).asDiagonal();
    Qc_.block(VRW_ID, VRW_ID, 3, 3) = imunoise.acc_vrw.cwiseProduct(imunoise.acc_vrw).asDiagonal();
    Qc_.block(BGSTD_ID, BGSTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.gyrbias_std.cwiseProduct(imunoise.gyrbias_std).asDiagonal();
    Qc_.block(BASTD_ID, BASTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.accbias_std.cwiseProduct(imunoise.accbias_std).asDiagonal();
    Qc_.block(SGSTD_ID, SGSTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.gyrscale_std.cwiseProduct(imunoise.gyrscale_std).asDiagonal();
    Qc_.block(SASTD_ID, SASTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.accscale_std.cwiseProduct(imunoise.accscale_std).asDiagonal();

    // 设置系统状态(位置、速度、姿态和IMU误差)初值和初始协方差
    // set initial state (position, velocity, attitude and IMU error) and covariance
    initialize(options_.initstate, options_.initstate_std);

    odo_update_t = options_.starttime;
}

void WheelINS::initialize(const NavState &initstate, const NavState &initstate_std) {

    // 初始化位置、速度、姿态
    // initialize position, velocity and attitude
    pvacur_.pos       = initstate.pos;
    pvacur_.vel       = initstate.vel;
    pvacur_.att.euler = initstate.euler;
    pvacur_.att.cbn   = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn   = Rotation::euler2quaternion(pvacur_.att.euler);
    // 初始化IMU误差
    // initialize imu error
    imuerror_ = initstate.imuerror;

    // 给上一时刻状态赋同样的初值
    // set the same value to the previous state
    pvapre_ = pvacur_;

    // 初始化协方差
    // initialize covariance
    ImuError imuerror_std            = initstate_std.imuerror;
    Cov_.block(P_ID, P_ID, 3, 3)     = initstate_std.pos.cwiseProduct(initstate_std.pos).asDiagonal();
    Cov_.block(V_ID, V_ID, 3, 3)     = initstate_std.vel.cwiseProduct(initstate_std.vel).asDiagonal();
    Cov_.block(PHI_ID, PHI_ID, 3, 3) = initstate_std.euler.cwiseProduct(initstate_std.euler).asDiagonal();
    Cov_.block(BG_ID, BG_ID, 3, 3)   = imuerror_std.gyrbias.cwiseProduct(imuerror_std.gyrbias).asDiagonal();
    Cov_.block(BA_ID, BA_ID, 3, 3)   = imuerror_std.accbias.cwiseProduct(imuerror_std.accbias).asDiagonal();
    Cov_.block(SG_ID, SG_ID, 3, 3)   = imuerror_std.gyrscale.cwiseProduct(imuerror_std.gyrscale).asDiagonal();
    Cov_.block(SA_ID, SA_ID, 3, 3)   = imuerror_std.accscale.cwiseProduct(imuerror_std.accscale).asDiagonal();
}

void WheelINS::newImuProcess() {

    

    // set current IMU time as the current state time
    timestamp_ = imucur_.timestamp;

    insPropagation(imupre_, imucur_);
    ODOUpdate();

    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();

    // 更新上一时刻的状态和IMU数据
    // update system state and imudata at the previous epoch
    pvapre_ = pvacur_;
    imupre_ = imucur_;
}

void WheelINS::imuCompensate(IMU &imu) {

    // 补偿IMU零偏误差
    // compensate the imu bias error
    imu.angular_velocity -= imuerror_.gyrbias;
    imu.acceleration -= imuerror_.accbias;

    // 补偿IMU比例因子误差
    // compensate the imu scale error
    Eigen::Vector3d gyrscale, accscale;
    gyrscale   = Eigen::Vector3d::Ones() + imuerror_.gyrscale;
    accscale   = Eigen::Vector3d::Ones() + imuerror_.accscale;
    imu.angular_velocity = imu.angular_velocity.cwiseProduct(gyrscale.cwiseInverse());
    imu.acceleration   = imu.acceleration.cwiseProduct(accscale.cwiseInverse());
}

void WheelINS::insPropagation(IMU &imupre, IMU &imucur) {

    // 对当前IMU数据(imucur)补偿误差, 上一IMU数据(imupre)已经补偿过了
    // compensate imu error to 'imucur', 'imupre' has been compensated
    imuCompensate(imucur);
    // IMU状态更新(机械编排算法)

    INSMech::insMech(pvapre_, pvacur_, imupre, imucur);

    // 系统噪声传播，姿态误差采用phi角误差模型
    // system noise propagate, phi-angle error model for attitude error
    Eigen::MatrixXd Phi, F, Qd, G;

    // 初始化Phi阵(状态转移矩阵)，F阵，Qd阵(传播噪声阵)，G阵(噪声驱动阵)
    // initialize Phi (state transition), F matrix, Qd(propagation noise) and G(noise driven) matrix
    Phi.resizeLike(Cov_);
    F.resizeLike(Cov_);
    Qd.resizeLike(Cov_);
    G.resize(RANK, NOISERANK);
    Phi.setIdentity();
    F.setZero();
    Qd.setZero();
    G.setZero();

    // 使用上一历元状态计算状态转移矩阵
    // compute state transition matrix using the previous state

    // 位置误差
    // position error
    F.block(P_ID, V_ID, 3, 3) = Eigen::Matrix3d::Identity();

    // 速度误差
    // velocity error
    F.block(V_ID, PHI_ID, 3, 3) = Rotation::skewSymmetric(pvapre_.att.cbn * imucur.acceleration);
    F.block(V_ID, BA_ID, 3, 3)  = pvapre_.att.cbn;
    F.block(V_ID, SA_ID, 3, 3)  = pvapre_.att.cbn * (imucur.acceleration.asDiagonal());

    // 姿态误差
    // attitude error
    F.block(PHI_ID, BG_ID, 3, 3)  = -pvapre_.att.cbn;
    F.block(PHI_ID, SG_ID, 3, 3)  = -pvapre_.att.cbn * (imucur.angular_velocity.asDiagonal());

    // IMU零偏误差和比例因子误差，建模成一阶高斯-马尔科夫过程
    // imu bias error and scale error, modeled as the first-order Gauss-Markov process
    F.block(BG_ID, BG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(BA_ID, BA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(SG_ID, SG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(SA_ID, SA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();

    // 系统噪声驱动矩阵
    // system noise driven matrix
    G.block(V_ID, VRW_ID, 3, 3)    = pvapre_.att.cbn;
    G.block(PHI_ID, ARW_ID, 3, 3)  = pvapre_.att.cbn;
    G.block(BG_ID, BGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(BA_ID, BASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(SG_ID, SGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(SA_ID, SASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();

    // 状态转移矩阵
    // compute the state transition matrix
    Phi.setIdentity();
    Phi = Phi + F * imucur.dt;

    // 计算系统传播噪声
    // compute system propagation noise
    Qd = G * Qc_ * G.transpose() * imucur.dt;
    Qd = (Phi * Qd * Phi.transpose() + Qd) / 2;

    // EKF预测传播系统协方差和系统误差状态
    // do EKF predict to propagate covariance and error state
    EKFPredict(Phi, Qd);
}


void WheelINS::EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd) {

    assert(Phi.rows() == Cov_.rows());
    assert(Qd.rows() == Cov_.rows());

    // 传播系统协方差和误差状态
    // propagate system covariance and error state
    Cov_ = Phi * Cov_ * Phi.transpose() + Qd;
    dx_  = Phi * dx_;
}

void WheelINS::EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R) {

    assert(H.cols() == Cov_.rows());
    assert(dz.rows() == H.rows());
    assert(dz.rows() == R.rows());
    assert(dz.cols() == 1);

    // 计算Kalman增益
    // Compute Kalman Gain
    auto temp         = H * Cov_ * H.transpose() + R;
    Eigen::MatrixXd K = Cov_ * H.transpose() * temp.inverse();

    // 更新系统误差状态和协方差
    // update system error state and covariance
    Eigen::MatrixXd I;
    I.resizeLike(Cov_);
    I.setIdentity();
    I = I - K * H;
    // 如果每次更新后都进行状态反馈，则更新前dx_一直为0，下式可以简化为：dx_ = K * dz;
    // if state feedback is performed after every update, dx_ is always zero before the update
    // the following formula can be simplified as : dx_ = K * dz;
    dx_  = dx_ + K * (dz - H * dx_);
    Cov_ = I * Cov_ * I.transpose() + K * R * K.transpose();
}


void WheelINS::ODOUpdate() {

    if(imuBuff.size() < imuBuffsize)
        return;

    if ((imucur_.timestamp - odo_update_t) < options_.odo_update_interval)
        return;

    getWheelVelocity();

    Matrix3d C_nv = computeVehicleRotmat().transpose();
    Matrix3d C_bv = C_nv * pvacur_.att.cbn;
    Vector3d velocity_vframe = C_nv * pvacur_.vel;
    Matrix3d angularVelocity_skew = Rotation::skewSymmetric(imucur_.angular_velocity);
    Matrix3d leverarm_skew = Rotation::skewSymmetric(options_.leverArm);
    Matrix3d velocitySkew_nframe = C_nv * Rotation::skewSymmetric(pvacur_.vel);
    Vector3d velocity_leverarm =
        C_bv * angularVelocity_skew * options_.leverArm;

    Eigen::MatrixXd Hv;
    Hv.resize(3, RANK);
    Hv.setZero();

    Hv.block<3, 3>(0, V_ID) = C_nv;
    Hv.block<3, 3>(0, PHI_ID) = C_nv * Rotation::skewSymmetric(pvacur_.att.cbn * angularVelocity_skew * options_.leverArm);
    Hv.block<3, 3>(0, BG_ID) = -C_bv * leverarm_skew;
    Hv.block<3, 3>(0, SG_ID) = -C_bv * leverarm_skew * imucur_.angular_velocity.asDiagonal();

    Hv.block<3, 1>(0, 8) = - velocitySkew_nframe.block<3, 1>(0, 2);

    Eigen::MatrixXd Zv = velocity_vframe + velocity_leverarm;
    Zv(0) -= wheelVelocity;


    Eigen::MatrixXd Rv = options_.odo_measurement_std.cwiseProduct(options_.odo_measurement_std).asDiagonal();

    EKFUpdate(Zv, Hv, Rv);

    odo_update_t = imucur_.timestamp;

    stateFeedback();

}

void WheelINS::getWheelVelocity() {

    double gyro_x_mean = 0.0;

    int i = 0;
    for(auto it = imuBuff.begin(); it != imuBuff.end(); ++it){
        gyro_x_mean += it->angular_velocity[0];
    }
    gyro_x_mean /= imuBuffsize;


	wheelVelocity = -options_.wheelradius * (gyro_x_mean);
	
}

Matrix3d WheelINS::computeVehicleRotmat()
{
    Vector3d vehicle_euler = Vector3d::Zero();

    vehicle_euler[2] = pvacur_.att.euler[2] - M_PI/2.0;

    Matrix3d C_vn = Rotation::euler2matrix(vehicle_euler);

    return C_vn;
}

void WheelINS::stateFeedback() {
    
    pvacur_.pos -= dx_.block(P_ID, 0, 3, 1);
    pvacur_.vel -= dx_.block(V_ID, 0, 3, 1);

    Vector3d delta_att     = dx_.block(PHI_ID, 0, 3, 1);
    Eigen::Quaterniond qpn = Rotation::rotvec2quaternion(delta_att);
    pvacur_.att.qbn        = qpn * pvacur_.att.qbn;
    pvacur_.att.cbn        = Rotation::quaternion2matrix(pvacur_.att.qbn);
    pvacur_.att.euler      = Rotation::matrix2euler(pvacur_.att.cbn);

    imuerror_.gyrbias += dx_.block(BG_ID, 0, 3, 1);
    imuerror_.accbias += dx_.block(BA_ID, 0, 3, 1);
    imuerror_.gyrscale += dx_.block(SG_ID, 0, 3, 1);
    imuerror_.accscale += dx_.block(SA_ID, 0, 3, 1);

    dx_.setZero();
}

NavState WheelINS::getNavState() {

    NavState state;

    state.pos      = pvacur_.pos;
    state.vel      = pvacur_.vel;
    state.euler    = pvacur_.att.euler;
    state.imuerror = imuerror_;

    return state;
}
