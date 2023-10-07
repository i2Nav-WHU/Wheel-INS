#ifndef INSMECH_H
#define INSMECH_H

#include "common/types.h"
#include "utils.h"

class INSMech {

public:

    static void insMech(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur);

    static void imuCompensate(IMU &imu, ImuError &imuerror);

};

#endif // INSMECH_H
