# Wheel-INS
## A Wheel-mounted MEMS IMU-based Dead Reckoning System

Unlike the conventional odometer-aided INS (ODO/INS) in which the IMU is placed on the vehicle body, in the proposed Wheel-INS, the IMU is mounted on a ground vehicle wheel to take advantage of the inherent rotation platform of the vehicle to mimic the rotary INS.  The wheel velocity calculated by the gyroscope outputs and the wheel radius is treated as an external observation with non-holonomic constraints (NHCs) to fuse with INS through an extended Kalman filter (EKF).

## Description
The project implementations are in the *Wheel-INS* folder. It is very consice and light-weight which can be run easily in Windows (Some incompatibility issues should be solved for running in Linux). It does not rely on any third-party libraries. We only use OpenCV to read configure file. 

Two sets of example data with ground truth are provided in this repository (see *Data* folder). Please refer to the *ReadMe.pdf* for details. If git clone is too slow, please try to download the *.zip* file directly.

For any questions, please contact Dr. Jian Kuang (kuang@whu.edu.cn) or Yibin Wu (ybwu@whu.edu.cn).
