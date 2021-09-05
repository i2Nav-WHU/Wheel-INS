# Wheel-INS
## A Wheel-mounted MEMS IMU-based Dead Reckoning System

Unlike the conventional odometer-aided INS (ODO/INS) where the IMU is placed either on the vehicle body or in the coach, in Wheel-INS, the IMU is mounted on the wheel of the ground vehicle to 1) obtain the wheel velocity thus replace the traditional odometer (or wheel encoder); 2) take advantages of the rotation modulation to mitigate the error accumulation of INS. The wheel velocity calculated by the gyroscope outputs and the wheel radius is treated as an external observation with non-holonomic constraint (NHC) to fuse with INS through an extended Kalman filter (EKF).

## About this Repository
The source code is in the *Wheel-INS* folder. It is very consice and light-weight which can be run easily in Windows (Some incompatibility issues should be solved for running in Linux). It does not rely on any third-party libraries.(We only use OpenCV to read configure file.)

Two sets of example data with ground truth are also provided (see *Data* folder). Please refer to the *ReadMe.pdf* for details. If *git clone* is too slow, please try to download the *.zip* file directly.

## Papers
X. Niu, Y. Wu and J. Kuang, "Wheel-INS: A Wheel-mounted MEMS IMU-based Dead Reckoning System," IEEE Transactions on Vehicular Technology, doi: 10.1109/TVT.2021.3108008. ([pdf](http://i2nav.cn/ueditor/jsp/upload/file/20210905/1630804325780076093.pdf)) ([IEEE Xplore](https://ieeexplore.ieee.org/document/9524467))

Y. Wu, X. Niu and J. Kuang, "A Comparison of Three Measurement Models for the Wheel-mounted MEMS IMU-based Dead Reckoning System," IEEE Transactions on Vehicular Technology, doi: 10.1109/TVT.2021.3102409. ([pdf](http://i2nav.cn/ueditor/jsp/upload/file/20210905/1630804728655046341.pdf)) ([IEEE Xplore](https://ieeexplore.ieee.org/document/9508199))

Y. Wu, J. Kuang and X. Niu, "Wheel-INS2: Multiple MEMS IMU-based Dead Reckoning System for Wheeled Robots with Evaluation of Different IMU Configurations," arXiv:2012.10593, 2020. ([pdf](https://arxiv.org/abs/2012.10593))

If you find our code and/or dataset helpful to your academic research, please consider citing our related papers.

For any questions, please fell free to contact Dr. Jian Kuang (kuang@whu.edu.cn) or Mr. Yibin Wu (ybwu@whu.edu.cn).
