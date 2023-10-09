#pragma once

#include "common/types.h"
#include "fileloader.h"

class ImuFileLoader : public FileLoader {
 public:
  ImuFileLoader() = delete;
  ImuFileLoader(const string &filename, int columns, int rate = 200) {
    open(filename, columns, FileLoader::BINARY);

    dt_ = 1.0 / (double)rate;

    imu_.timestamp = 0;
  }

  const IMU &next() {
    imu_pre_ = imu_;

    data_ = load();

    imu_.timestamp = data_[0];

    memcpy(imu_.angular_velocity.data(), &data_[1], 3 * sizeof(double));
    memcpy(imu_.acceleration.data(), &data_[4], 3 * sizeof(double));

    double dt = imu_.timestamp - imu_pre_.timestamp;
    if (dt < 0.1) {
      imu_.dt = dt;
    } else {
      imu_.dt = dt_;
    }

    return imu_;
  }

  double starttime() {
    double starttime;
    std::streampos sp = filefp_.tellg();

    filefp_.seekg(0, std::ios_base::beg);
    starttime = load().front();
    filefp_.seekg(sp, std::ios_base::beg);
    return starttime;
  }

  double endtime() {
    double endtime = -1;
    std::streampos sp = filefp_.tellg();

    if (filetype_ == TEXT) {
      filefp_.seekg(-2, std::ios_base::end);
      char byte = 0;
      auto pos = filefp_.tellg();
      do {
        pos -= 1;
        filefp_.seekg(pos);
        filefp_.read(&byte, 1);
      } while (byte != '\n');
    } else {
      filefp_.seekg(-columns_ * sizeof(double), std::ios_base::end);
    }
    endtime = load().front();
    filefp_.seekg(sp, std::ios_base::beg);
    return endtime;
  }

 private:
  double dt_;

  IMU imu_, imu_pre_;
  vector<double> data_;
};