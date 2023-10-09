#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#define MKDIR(path) CreateDirectoryA(path.c_str(), NULL)
#else
#include <sys/stat.h>
#include <unistd.h>
#define MKDIR(path) mkdir(path.c_str(), S_IRWXU)
#endif

#include "fileio/imufileloader.h"
#include "fileio/loadanddump.h"
#include "wheel_ins/wheelins.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "usage: Wheel-INS config.yaml" << std::endl;
    return -1;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(argv[1]);

  } catch (YAML::Exception &exception) {
    std::cout << "Failed to read configuration file: " << exception.what()
              << std::endl;
    return -1;
  }

  Paras paras;
  if (!loadConfig(config, paras)) {
    std::cout << "Error occured in the configuration file!" << std::endl;
    return -1;
  }

  std::string imupath, outputpath;
  try {
    imupath = config["imupath"].as<std::string>();
    outputpath = config["outputpath"].as<std::string>();
  } catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration: " << exception.what()
              << std::endl;
    return -1;
  }

#ifdef _WIN32
  if (GetFileAttributesA(outputpath.c_str()) == INVALID_FILE_ATTRIBUTES)
#else
  if (access(outputpath.c_str(), F_OK))
#endif
  {
    MKDIR(std::string("output"));
    MKDIR(outputpath);
  }

  int imudatalen, imudatarate;
  double starttime, endtime;
  try {
    imudatalen = config["imudatalen"].as<int>();
    imudatarate = config["imudatarate"].as<int>();
    starttime = config["starttime"].as<double>();
    endtime = config["endtime"].as<double>();
  } catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration. Please check the data "
                 "length, data rate, and the process time!"
              << std::endl;
    return -1;
  }

  ImuFileLoader imufile(imupath, imudatalen, imudatarate);

  WheelINS wheelins(paras);

  const int nav_columns = 11, imuerr_columns = 13, std_columns = 22;
  FileSaver navfile(outputpath + "/traj.txt", nav_columns, FileSaver::TEXT);
  FileSaver imuerrfile(outputpath + "/imuerror.txt", imuerr_columns,
                       FileSaver::TEXT);
  FileSaver stdfile(outputpath + "/std.txt", std_columns, FileSaver::TEXT);

  if (!imufile.isOpen() || !navfile.isOpen() || !imuerrfile.isOpen() ||
      !stdfile.isOpen()) {
    std::cout << "Failed to open data file!" << std::endl;
    return -1;
  }

  if (endtime < 0) {
    endtime = imufile.endtime();
  }

  if (starttime < imufile.starttime() || starttime > endtime) {
    std::cout << "Process time ERROR!" << std::endl;
    return -1;
  }

  IMU imu_cur;
  do {
    imu_cur = imufile.next();
  } while (imu_cur.timestamp < starttime);

  wheelins.addImuData(imu_cur);

  double timestamp;
  NavState navstate;
  Eigen::MatrixXd cov;

  int percent = 0, lastpercent = 0;
  double interval = endtime - starttime;

  while (true) {
    imu_cur = imufile.next();
    if (imu_cur.timestamp > endtime || imufile.isEof()) {
      break;
    }
    wheelins.addImuData(imu_cur);

    wheelins.newImuProcess();

    timestamp = wheelins.timestamp();
    navstate = wheelins.getNavState();
    cov = wheelins.getCovariance();

    writeNavResult(timestamp, navstate, navfile, imuerrfile);
    writeSTD(timestamp, cov, stdfile);

    percent = int((imu_cur.timestamp - starttime) / interval * 100);
    if (percent - lastpercent >= 1) {
      std::cout << "Processing: " << std::setw(3) << percent << "%\r"
                << std::flush;
      lastpercent = percent;
    }
  }

  imufile.close();
  navfile.close();
  imuerrfile.close();
  stdfile.close();

  std::cout << std::endl << std::endl << "Wheel-INS Process Finish! ";
  std::cout << "From " << starttime << " s to " << endtime << " s, total "
            << interval << " s!" << std::endl;

  return 0;
}