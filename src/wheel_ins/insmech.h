#pragma once

#include "common/types.h"

class INSMech {
 public:
  static void insMech(const PVA &pvapre, PVA &pvacur, const IMU &imupre,
                      const IMU &imucur);

  static void imuCompensate(IMU &imu, ImuError &imuerror);
};