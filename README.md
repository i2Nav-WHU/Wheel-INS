# Wheel-INS: A Wheel-mounted MEMS IMU-based Dead Reckoning System

Unlike the conventional odometer-aided inertial navigation system (ODO/INS) where the IMU is always placed either on the vehicle body or in the coach, in Wheel-INS, the IMU is mounted on the wheel of the ground vehicle. There are two major advanatges by doing so: 1) the wheel velocity can be obtained by the Wheel-IMU thus replace the traditional odometer (or wheel encoder); 2) the rotation modulation can be leveraged to mitigate the error accumulation of INS. Particularly, the wheel velocity calculated by the gyroscope outputs and the wheel radius is treated as an external observation with non-holonomic constraint (NHC) to fuse with INS through an extended Kalman filter (EKF).

## News :boom:
:tada::tada: Nov. 2022 Our paper on one wheel-mounted IMU-based SLAM ([Wheel-SLAM](https://arxiv.org/pdf/2211.03174.pdf)) is accepted to IEEE Robotics and Automation Letters. The [source code](https://github.com/i2Nav-WHU/Wheel-SLAM) is released. Check it out!

## Introduction
The source code is in the ***Wheel-INS*** folder. It is very consice and light-weight which can be run easily in Windows (Some incompatibility issues should be solved for running in Linux). It does not rely on any third-party libraries.(We only use OpenCV to read configure file.)

Two sets of example data with ground truth are also provided (see ***Data*** folder). Please refer to the ***ReadMe.pdf*** for details. If *git clone* is too slow, please try to download the *.zip* file directly.

## Related Papers
*Based on the study of Wheel-IMU, we have published three papers. 1) The original Wheel-INS paper where we proposed a wheel-mounted IMU-based dead reckoning system and investigated its characteristics. 2) A thorough and complete comparison on three different measurement models (vehicle velocity information obtained from the Wwheel-IMU) in Wheel-INS with both theoretical analysis and experimental illustration. 3) A multiple IMUs-based localization system for wheeled robots by obtaining different dynamic information of the vhehicle and taking advantage of the relative spatial constraints among the inertial sensors with a comparison on different configurations.*

X. Niu, Y. Wu and J. Kuang, "Wheel-INS: A Wheel-mounted MEMS IMU-based Dead Reckoning System," IEEE Transactions on Vehicular Technology, doi: 10.1109/TVT.2021.3108008, 2021. ([pdf](http://i2nav.cn/ueditor/jsp/upload/file/20210905/1630804325780076093.pdf)) ([IEEE Xplore](https://ieeexplore.ieee.org/document/9524467))

Y. Wu, X. Niu and J. Kuang, "A Comparison of Three Measurement Models for the Wheel-mounted MEMS IMU-based Dead Reckoning System," IEEE Transactions on Vehicular Technology, doi: 10.1109/TVT.2021.3102409, 2021. ([pdf](http://i2nav.cn/ueditor/jsp/upload/file/20210905/1630804728655046341.pdf)) ([IEEE Xplore](https://ieeexplore.ieee.org/document/9508199))

Y. Wu, J. Kuang and X. Niu, "Wheel-INS2: Multiple MEMS IMU-based Dead Reckoning System for Wheeled Robots with Evaluation of Different IMU Configurations," IEEE Transactions on Intelligent Transportation Systems, 2022. ([pdf](https://arxiv.org/abs/2012.10593))([IEEE Xplore](https://ieeexplore.ieee.org/document/9950438))

If you find our study helpful to your academic research, please consider citing the related papers.
```bibtex
@ARTICLE{niu2021tvt,
  author={Niu, Xiaoji and Wu, Yibin and Kuang, Jian},
  journal={IEEE Transactions on Vehicular Technology}, 
  title={{Wheel-INS}: A Wheel-Mounted {MEMS IMU}-Based Dead Reckoning System}, 
  year={2021},
  volume={70},
  number={10},
  pages={9814-9825},
  doi={10.1109/TVT.2021.3108008}
}
```
```bibtex
@ARTICLE{wu2021tvt,
  author={Wu, Yibin and Niu, Xiaoji and Kuang, Jian},
  journal={IEEE Transactions on Vehicular Technology}, 
  title={A Comparison of Three Measurement Models for the Wheel-Mounted {MEMS IMU}-Based Dead Reckoning System}, 
  year={2021},
  volume={70},
  number={11},
  pages={11193-11203},
  doi={10.1109/TVT.2021.3102409}}
```
```bibtex
@ARTICLE{wu2022tvt,  
    author={Wu, Yibin and Kuang, Jian and Niu, Xiaoji},  
    journal={IEEE Transactions on Intelligent Transportation Systems},   
    title={{Wheel-INS2}: Multiple {MEMS IMU}-Based Dead Reckoning System With Different Configurations for Wheeled Robots},   
    year={2022}, 
    pages={1-14},  
    doi={10.1109/TITS.2022.3220508}
}
```
For any questions, please feel free to contact Mr. Yibin Wu (ybwu@whu.edu.cn) or Dr. Jian Kuang (kuang@whu.edu.cn).
