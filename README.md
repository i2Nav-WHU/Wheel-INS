<p align="center">
<h1 align="center">Wheel-INS: A Wheel-mounted MEMS IMU-based Dead Reckoning System</h1>
<p align="center">
<a href="http://i2nav.cn/ueditor/jsp/upload/file/20210905/1630804325780076093.pdf"><img src="https://img.shields.io/badge/Paper-pdf-<COLOR>.svg?style=flat-square" /></a>
<a href="https://github.com/i2Nav-WHU/Wheel-INS"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a> <a href="https://github.com/i2Nav-WHU/Wheel-INS"><img src="https://img.shields.io/badge/Windows-0078D6?st&logo=windows&logoColor=white" /></a>
 </p>
 </p>
IMU drifts quickly with time! Well...it's true, but what if we mount it on the wheel❓

In [Wheel-INS](http://i2nav.cn/ueditor/jsp/upload/file/20210905/1630804325780076093.pdf), we proposed a dead reckoning sysytem for the wheeled robots **using only one Wheel-mounted IMU**. There are two major advantages of mounting IMU to the wheel: 1) The vehicle velocity can be directly obtained by the Wheel-IMU without wheel encoder; 2) The continuous rotation of the wheel can somewhat mitigate the error accumulation of the inertial navigation system because of rotation modulation.

## 💥News💥
:tada::tada: Jan. 2025 Wheel-INS has been extended to [Wheel-GINS](https://github.com/i2Nav-WHU/Wheel-GINS) with the fusion with GNSS. Check out our new [paper](https://arxiv.org/pdf/2501.03079) accepted to IEEE Transactions on Intelligent Transportation Systems.  

:tada::tada: Oct. 2023 A completely new version of the code supporting both Linux and Windows is released!

:tada::tada: Nov. 2022 Wheel-INS has been extended to a SLAM solution! Check out our paper of [Wheel-SLAM](https://arxiv.org/pdf/2211.03174.pdf) accepted to IEEE Robotics and Automation Letters. The [source code](https://github.com/i2Nav-WHU/Wheel-SLAM) is also released. 

:tada::tada: Nov. 2022 Our paper on multiple IMUs-based wheeled robot localization ([Wheel-INS2](https://arxiv.org/abs/2012.10593)) has been accepted to IEEE Transactions on Intelligent Transportation Systems. Check it out!

:tada::tada: Aug. 2021 [Wheel-INS](http://i2nav.cn/ueditor/jsp/upload/file/20210905/1630804325780076093.pdf) has been accepted to IEEE Transactions on Vehicular Technology. Check it out!

## Run it
### 1. Linux
We recommend you use the g++ compiler with Ubuntu 20.04. The build-essential libraries should be installed first:
```shell
sudo apt-get install cmake
sudo apt-get install build-essential
```

After preparing the compilation environment, you can clone the repository and run it as follows:

```shell
# Clone the repository
git clone git@github.com:i2Nav-WHU/Wheel-INS.git ~/

# Build Wheel-INS
cd ~/Wheel-INS
mkdir build && cd build

cmake ..
make -j10

# Run demo dataset
cd ~/Wheel-INS
./bin/Wheel-INS config/robot.yaml
```
### 2. Windows
Here we show how to run the code with [Visual Studio Code (VSCode)](https://code.visualstudio.com/), but you can also use other IDEs, e.g., [Visual Studio](https://visualstudio.microsoft.com/).

- Install VSCode and the extensions: **C/C++**, **C/C++ Extension Pack**, **CMake**, and **CMake Tools**.
- Install [CMake](https://cmake.org/download/) and [Microsoft Visual C/C++ Build Tools](https://visualstudio.microsoft.com/downloads/).
- Open Wheel-INS with VSCode.
- Set compiler: open the Command Palette (Ctrl+Shift+P) and type "CMake: Select a Kit", select the correct build tool according to your system.
- Configure CMake: type and click "CMake: Configure" in the Command Palette.
- Compile Project: type and click "CMake: Build" in the Command Palette.

Once an executable file **Wheel-INS.exe** is generated, the compilation is done. Then, you can run it via the terminal in VSCode as follows:

```shell
.\bin\Release\Wheel-INS.exe config/robot.yaml
```
You can then run **plot.py** in utils to plot the trajectory estimated by Wheel-INS as well as the raw Wheel-IMU data. Here is an example plot for the robot dataset.

<img src="https://user-images.githubusercontent.com/25290921/273607073-13f5363a-ff6c-456b-8e7a-2d52cfc60267.png" width = 45% div align=left/> <img src="https://user-images.githubusercontent.com/25290921/273607080-ded7e0b9-8757-45f0-af66-033ac3d2527d.png" width = 45% div align=center/> 

## Datasets
Two sets of example data with ground truth are provided (see [***dataset***](https://github.com/i2Nav-WHU/Wheel-INS/tree/master/dataset)). Please refer to the ***ReadMe.pdf*** for details. If *git clone* is too slow, please try to download the *.zip* file directly.

## Related Papers
*Based on the study of Wheel-IMU, we have published three papers. 1) The original Wheel-INS paper where we proposed a wheel-mounted IMU-based dead reckoning system and investigated its characteristics. 2) A thorough and complete comparison on three different measurement models in Wheel-INS with both theoretical analysis and experimental illustration. 3) A multiple IMUs-based localization system for wheeled robots by obtaining different dynamic information of the vehicle and taking advantage of the relative spatial constraints among the inertial sensors.*

If you find our studies helpful to your academic research, please consider citing the related papers.

- X. Niu, Y. Wu and J. Kuang, "Wheel-INS: A Wheel-mounted MEMS IMU-based Dead Reckoning System," IEEE Transactions on Vehicular Technology, doi: 10.1109/TVT.2021.3108008, 2021. ([pdf](http://i2nav.cn/ueditor/jsp/upload/file/20210905/1630804325780076093.pdf)) ([IEEE Xplore](https://ieeexplore.ieee.org/document/9524467))
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

- Y. Wu, X. Niu and J. Kuang, "A Comparison of Three Measurement Models for the Wheel-mounted MEMS IMU-based Dead Reckoning System," IEEE Transactions on Vehicular Technology, doi: 10.1109/TVT.2021.3102409, 2021. ([pdf](http://i2nav.cn/ueditor/jsp/upload/file/20210905/1630804728655046341.pdf)) ([IEEE Xplore](https://ieeexplore.ieee.org/document/9508199))
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

- Y. Wu, J. Kuang and X. Niu, "Wheel-INS2: Multiple MEMS IMU-based Dead Reckoning System for Wheeled Robots with Evaluation of Different IMU Configurations," IEEE Transactions on Intelligent Transportation Systems, 2022. ([pdf](https://arxiv.org/abs/2012.10593))([IEEE Xplore](https://ieeexplore.ieee.org/document/9950438))
```bibtex
@ARTICLE{wu2022tits,  
    author={Wu, Yibin and Kuang, Jian and Niu, Xiaoji},  
    journal={IEEE Transactions on Intelligent Transportation Systems},   
    title={{Wheel-INS2}: Multiple {MEMS IMU}-Based Dead Reckoning System With Different Configurations for Wheeled Robots},   
    year={2022}, 
    pages={1-14},  
    doi={10.1109/TITS.2022.3220508}
}
```
## License
The code is released under GPLv3 license.

## Acknowledgement
We would like to thank the [i2Nav group](http://i2nav.cn/) for sharing [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS). 

For any questions, please feel free to contact Mr. Yibin Wu (ybwu at whu.edu.cn) or Dr. Jian Kuang (kuang at whu.edu.cn).
