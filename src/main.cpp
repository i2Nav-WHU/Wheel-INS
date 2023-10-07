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

#include <Eigen/Dense>
#include <absl/time/clock.h>
#include <iomanip>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <sys/stat.h>
#include <unistd.h>

#include "common/angle.h"
#include "fileio/filesaver.h"
#include "fileio/imufileloader.h"

#include "wheel_ins/wheelins.h"

#include <fstream>
std::ofstream debug_;


bool loadConfig(YAML::Node &config, Paras &paras);
void writeNavResult(double time, NavState &navstate, FileSaver &navfile, FileSaver &imuerrfile);
void writeSTD(double time, Eigen::MatrixXd &cov, FileSaver &stdfile);

int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cout << "usage: Wheel-INS wheel-ins.yaml" << std::endl;
        return -1;
    }


    // 加载配置文件
    // load configuration file
    YAML::Node config;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to read configuration file. Please check the path and format of the configuration file!"
                  << std::endl;
        return -1;
    }

    // config = YAML::LoadFile("");

    // 读取配置参数到GINSOptioins中，并构造GIEngine
    // load configuration parameters to GINSOptioins
    Paras paras;
    if (!loadConfig(config, paras)) {
        std::cout << "Error occurs in the configuration file!" << std::endl;
        return -1;
    }

    // 读取文件路径配置
    // load filepath configuration
    std::string imupath, outputpath;
    try {
        imupath    = config["imupath"].as<std::string>();
        
        outputpath = config["outputpath"].as<std::string>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check the file path and output path!" << std::endl;
        return -1;
    }
    if (access(outputpath.c_str(), F_OK))
    mkdir(outputpath.c_str(), S_IRWXU);

    // imu数据配置，数据处理区间
    // imudata configuration， data processing interval
    int imudatalen, imudatarate;
    double starttime, endtime;
    try {
        imudatalen  = config["imudatalen"].as<int>();
        imudatarate = config["imudatarate"].as<int>();
        starttime   = config["starttime"].as<double>();
        endtime     = config["endtime"].as<double>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check the data length, data rate, and the process time!"
                  << std::endl;
        return -1;
    }

    // 加载GNSS文件和IMU文件
    // load GNSS file and IMU file
    
    ImuFileLoader imufile(imupath, imudatalen, imudatarate);

    // 构造GIEngine
    // Construct GIEngine
    WheelINS wheelins(paras);

    // 构造输出文件
    // construct output file
    // navfile: gnssweek(1) + time(1) + pos(3) + vel(3) + euler angle(3) = 11
    // imuerrfile: time(1) + gyrbias(3) + accbias(3) + gyrscale(3) + accscale(3) = 13
    // stdfile: time(1) + pva_std(9) + imubias_std(6) + imuscale_std(6) = 22
    const int nav_columns = 11, imuerr_columns = 13, std_columns = 22;
    FileSaver navfile(outputpath + "/wheelins_Navresult.nav", nav_columns, FileSaver::TEXT);
    FileSaver imuerrfile(outputpath + "/wheelins_IMU_ERR.txt", imuerr_columns, FileSaver::TEXT);
    FileSaver stdfile(outputpath + "/wheelins_STD.txt", std_columns, FileSaver::TEXT);

    std::string debugoutputpath = outputpath + "/debug.txt";
    debug_.open(debugoutputpath.c_str());

    // 检查文件是否正确打开
    // check if these files are all opened
    if (!imufile.isOpen() || !navfile.isOpen() || !imuerrfile.isOpen() || !stdfile.isOpen()) {
        std::cout << "Failed to open data file!" << std::endl;
        return -1;
    }

    // 检查处理时间
    // check process time
    if (endtime < 0) {
        endtime = imufile.endtime();
    }
    std::cout<<std::setprecision(16)<<"starttime "<<starttime<<std::endl;
    std::cout<<"imufile.starttime() "<<imufile.starttime()<<std::endl;
    std::cout<<"endtime "<<endtime<<std::endl;
    if (starttime < imufile.starttime() || starttime > endtime) { //endtime > 604800 || in case of ROS timestamp 
        std::cout << "Process time ERROR!" << std::endl;
        return -1;
    }

    // 数据对齐
    // data alignment
    IMU imu_cur;
    do {
        imu_cur = imufile.next();
    } while (imu_cur.timestamp < starttime);

    // 添加IMU数据到GIEngine中，补偿IMU误差
    // add imudata to GIEngine and compensate IMU error
    wheelins.addImuData(imu_cur, true);


    // 用于保存处理结果
    // used to save processing results
    double timestamp;
    NavState navstate;
    Eigen::MatrixXd cov;

    // 用于显示处理进程
    // used to display processing progress
    int percent = 0, lastpercent = 0;
    double interval = endtime - starttime;

    while (true) {


        // 读取并添加新的IMU数据到GIEngine
        // load new imudata and add it to GIEngine
        imu_cur = imufile.next();
        if (imu_cur.timestamp > endtime || imufile.isEof()) {
            break;
        }
        wheelins.addImuData(imu_cur);
        

        // 处理新的IMU数据
        // process new imudata
        wheelins.newImuProcess();

        // 获取当前时间，IMU状态和协方差
        // get current timestamp, navigation state and covariance
        timestamp = wheelins.timestamp();
        navstate  = wheelins.getNavState();
        cov       = wheelins.getCovariance();

        // std::cout << timestamp << " " << navstate.pos.transpose() << " " << navstate.vel.transpose() << " " << navstate.euler.transpose()<<std::endl;

        // 保存处理结果
        // save processing results
        writeNavResult(timestamp, navstate, navfile, imuerrfile);
        writeSTD(timestamp, cov, stdfile);

        
        // 显示处理进展
        // display processing progress
        percent = int((imu_cur.timestamp - starttime) / interval * 100);
        if (percent - lastpercent >= 1) {
            std::cout << " - Processing: " << std::setw(3) << percent << "%\r" << std::flush;
            lastpercent = percent;
        }
    }

    // 关闭打开的文件
    // close opened file
    imufile.close();
    navfile.close();
    imuerrfile.close();
    stdfile.close();


    std::cout << std::endl << std::endl << "KF-GINS Process Finish! ";
    std::cout << "From " << starttime << " s to " << endtime << " s, total " << interval << " s!" << std::endl;
    

    return 0;
}


bool loadConfig(YAML::Node &config, Paras &paras) {

    std::vector<double> initposstd_vec, initvelstd_vec, initattstd_vec;
    // 读取初始位置、速度、姿态(欧拉角)的标准差
    // load initial position std, velocity std and attitude(euler angle) std
    try {
        initposstd_vec = config["initposstd"].as<std::vector<double>>();
        initvelstd_vec = config["initvelstd"].as<std::vector<double>>();
        initattstd_vec = config["initattstd"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check initial std of position, velocity, and attitude!"
                  << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {
        paras.initstate_std.pos[i]   = initposstd_vec[i];
        paras.initstate_std.vel[i]   = initvelstd_vec[i];
        paras.initstate_std.euler[i] = initattstd_vec[i] * D2R;
    }

    double arw, vrw, gbstd, abstd, gsstd, asstd;

    try{
        arw = config["imunoise"]["arw"].as<double>();
        vrw = config["imunoise"]["vrw"].as<double>();
        gbstd = config["imunoise"]["gbstd"].as<double>();
        abstd = config["imunoise"]["abstd"].as<double>();
        gsstd = config["imunoise"]["gsstd"].as<double>();
        asstd = config["imunoise"]["asstd"].as<double>();
        paras.imunoise.corr_time = config["imunoise"]["corrtime"].as<double>();
    }catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check IMU noise!" << std::endl;
        return false;
    }
        for (int i = 0; i < 3; i++) {
        paras.imunoise.gyr_arw[i]      = arw;
        paras.imunoise.acc_vrw[i]      = vrw;
        paras.imunoise.gyrbias_std[i]  = gbstd;
        paras.imunoise.accbias_std[i]  = abstd;
        paras.imunoise.gyrscale_std[i] = gsstd;
        paras.imunoise.accscale_std[i] = asstd;
    }
 

    // IMU噪声参数转换为标准单位
    // convert imu noise parameters' units to standard units
    paras.imunoise.gyr_arw *= (D2R / 60.0);
    paras.imunoise.acc_vrw /= 60.0;
    paras.imunoise.gyrbias_std *= (D2R / 3600.0);
    paras.imunoise.accbias_std *= 1e-5;
    paras.imunoise.gyrscale_std *= 1e-6;
    paras.imunoise.accscale_std *= 1e-6;
    paras.imunoise.corr_time *= 3600;


    std::vector<double> mountAngle, leverArm, odo_measurement_std;

    double odo_update_interval, wheelradius;
    bool ifCompensateVelocity;

    mountAngle = config["MisalignAngle"].as<std::vector<double>>();
    leverArm = config["WheelLA"].as<std::vector<double>>();
    odo_measurement_std = config["ODO_std"].as<std::vector<double>>();


    paras.mountAngle = Eigen::Vector3d(mountAngle.data());
    paras.leverArm = Eigen::Vector3d(leverArm.data());
    paras.odo_measurement_std = Eigen::Vector3d(odo_measurement_std.data());

    paras.odo_update_interval = config["ODO_dt"].as<double>();
    paras.wheelradius = config["Wheel_Radius"].as<double>();

    paras.ifCompensateVelocity = config["ifCompVel"].as<bool>();

    paras.starttime = config["starttime"].as<double>();

    return true;
}

/**
 * @brief 保存导航结果和IMU误差，已转换为常用单位
 *        save navigation result and imu error, converted them to common units
 * */
void writeNavResult(double time, NavState &navstate, FileSaver &navfile, FileSaver &imuerrfile) {

    std::vector<double> result;

    // 保存导航结果
    // save navigation result
    result.clear();
    result.push_back(time);
    result.push_back(navstate.pos[0]);
    result.push_back(navstate.pos[1]);
    result.push_back(navstate.pos[2]);
    result.push_back(navstate.vel[0]);
    result.push_back(navstate.vel[1]);
    result.push_back(navstate.vel[2]);
    result.push_back(navstate.euler[0] * R2D);
    result.push_back(navstate.euler[1] * R2D);
    result.push_back(navstate.euler[2] * R2D);
    navfile.dump(result);

    // 保存IMU误差
    // save IMU error
    auto imuerr = navstate.imuerror;
    result.clear();
    result.push_back(time);
    result.push_back(imuerr.gyrbias[0] * R2D * 3600);
    result.push_back(imuerr.gyrbias[1] * R2D * 3600);
    result.push_back(imuerr.gyrbias[2] * R2D * 3600);
    result.push_back(imuerr.accbias[0] * 1e5);
    result.push_back(imuerr.accbias[1] * 1e5);
    result.push_back(imuerr.accbias[2] * 1e5);
    result.push_back(imuerr.gyrscale[0] * 1e6);
    result.push_back(imuerr.gyrscale[1] * 1e6);
    result.push_back(imuerr.gyrscale[2] * 1e6);
    result.push_back(imuerr.accscale[0] * 1e6);
    result.push_back(imuerr.accscale[1] * 1e6);
    result.push_back(imuerr.accscale[2] * 1e6);
    imuerrfile.dump(result);
}

/**
 * @brief 保存标准差，已转换为常用单位
 *        save standard deviation, converted to common units
 * */
void writeSTD(double time, Eigen::MatrixXd &cov, FileSaver &stdfile) {

    std::vector<double> result;

    result.clear();
    result.push_back(time);
    // 保存位置、速度、姿态标准差
    // save position, velocity and attitude std
    for (int i = 0; i < 6; i++) {
        result.push_back(sqrt(cov(i, i)));
    }
    for (int i = 6; i < 9; i++) {
        result.push_back(sqrt(cov(i, i)) * R2D);
    }

    // 保存IMU误差标准差
    // save imu error std
    for (int i = 9; i < 12; i++) {
        result.push_back(sqrt(cov(i, i)) * R2D * 3600);
    }
    for (int i = 12; i < 15; i++) {
        result.push_back(sqrt(cov(i, i)) * 1e5);
    }
    for (int i = 15; i < 21; i++) {
        result.push_back(sqrt(cov(i, i)) * 1e6);
    }
    stdfile.dump(result);
}