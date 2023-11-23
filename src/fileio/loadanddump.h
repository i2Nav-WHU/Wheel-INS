#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

#include "common/types.h"
#include "filesaver.h"

inline bool loadConfig(YAML::Node &config, Paras &paras) {
  std::vector<double> initposstd_vec, initvelstd_vec, initattstd_vec;

  try {
    initposstd_vec = config["initposstd"].as<std::vector<double>>();
    initvelstd_vec = config["initvelstd"].as<std::vector<double>>();
    initattstd_vec = config["initattstd"].as<std::vector<double>>();
  } catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration. Please check initial std "
                 "of position, velocity, and attitude!"
              << std::endl;
    return false;
  }
  for (int i = 0; i < 3; i++) {
    paras.initstate_std.pos[i] = initposstd_vec[i];
    paras.initstate_std.vel[i] = initvelstd_vec[i];
    paras.initstate_std.euler[i] = initattstd_vec[i] * D2R;
  }

  double arw, vrw, gbstd, abstd, gsstd, asstd;

  try {
    arw = config["imunoise"]["arw"].as<double>();
    vrw = config["imunoise"]["vrw"].as<double>();
    gbstd = config["imunoise"]["gbstd"].as<double>();
    abstd = config["imunoise"]["abstd"].as<double>();
    gsstd = config["imunoise"]["gsstd"].as<double>();
    asstd = config["imunoise"]["asstd"].as<double>();
    paras.imunoise.corr_time = config["imunoise"]["corrtime"].as<double>();
  } catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration. Please check IMU noise!"
              << std::endl;
    return false;
  }
  for (int i = 0; i < 3; i++) {
    paras.imunoise.gyr_arw[i] = arw * (D2R / 60.0);
    paras.imunoise.acc_vrw[i] = vrw / 60.0;
    paras.imunoise.gyrbias_std[i] = gbstd * (D2R / 3600.0);
    paras.imunoise.accbias_std[i] = abstd * 1e-5;
    paras.imunoise.gyrscale_std[i] = gsstd * 1e-6;
    paras.imunoise.accscale_std[i] = asstd * 1e-6;

    paras.initstate_std.imuerror.gyrbias[i] = gbstd * (D2R / 3600.0);
    paras.initstate_std.imuerror.accbias[i] = abstd * 1e-5;
    paras.initstate_std.imuerror.gyrscale[i] = gsstd * 1e-6;
    paras.initstate_std.imuerror.accscale[i] = asstd * 1e-6;
  }

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

inline void writeNavResult(double time, NavState &navstate, FileSaver &navfile,
                           FileSaver &imuerrfile) {
  std::vector<double> result;

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

inline void writeSTD(double time, Eigen::MatrixXd &cov, FileSaver &stdfile) {
  std::vector<double> result;

  result.clear();
  result.push_back(time);

  for (int i = 0; i < 6; i++) {
    result.push_back(sqrt(cov(i, i)));
  }
  for (int i = 6; i < 9; i++) {
    result.push_back(sqrt(cov(i, i)) * R2D);
  }

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
