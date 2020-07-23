#pragma once

#include "Const.h"

struct S_NavState //Navigation struct
{

	double Q[nStates][nStates];//noise matrix
	double P[nStates][nStates];//Covariance matrix	
	double X[nStates];//State corrections	

	Vec3 BLH;
	Vec3 Pos;
	Vec3 Vel;
	Vec3 Att;
	QuatVec Q_bn;
	Mat3 C_bn;
	Mat3 C_bv;
	Mat3 C_vn;
	Vec3 Bg;
	Vec3 Sg;
	Vec3 Ba;
	Vec3 Sa;
	
	int InitFlag;
	int InitSum;
	double MemoInitDa[6];
	
	double T_Cur;
	double T_Zupt;
	double T_Odo;
	
	double dt;
	double PreMeas[6];
	double Pre_PreMeas[3];

	int FilterUpdate; 

	double Gx_comped;

	double vehiclepitch;

	double SysVodo;
};

struct IMU_ConstPara //IMU paras
{
	std::string filepath;
	std::string filename;
	
	int MountPos;

	double SetIniHeading;  
	double SetMisalign[3]; 							   

	double BgVar;
	double BaVar;
	double SgVar;
	double SaVar;
	double ARW_2;
	double VRW_2;
	double Tao_t;

	bool Odo_IfUse;
	double Odo_dt;
	double Odo_Var[3];

	bool IfcompV;

	double WheelLeverArm[3];
	double Wheel_Radius;
};