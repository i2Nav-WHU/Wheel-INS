#pragma once

#define  nStates 21  

#define  S_2_(a) ((a)*(a))
#define  absf(a) ((a)>0 ? (a):-(a))
#define  NormG (9.782940329221166)
#define  PI (3.14159265358979323846)
#define  D2R (PI/180.0)
#define  R2D (180.0/PI)

#define  Latitude (30.0*D2R)   
#define  DaRate (200) 
#define  Half_dt (0.5f / DaRate)
#define  BufLen (DaRate) 
#define  DaColNum (7)  

#define A_WGS84  6378137.0              
#define F_WGS84  1.0/298.257223563      // Flattening; WGS-84 
#define Earth_e2 0.00669437999013             
#define Omega_WGS  7.2921151467e-5      //[rad/s], the earth rotation rate 


const int GmeanLen = (int)((DaRate / 2) + 1);//101
const int Half_GmeanLen = (int)(GmeanLen / 2.0f);
#define  HisLen (DaRate - Half_GmeanLen - 1)

//===============================Initialization================================
const double AlignLen = 10.0;
const double set_ini_pos[3] = { 0.0, 0.0, 0.0 };
const double set_ini_vel[3] = { 0.0, 0.0, 0.0 };
const double Init_Pos_Var[3] = { S_2_(0.05), S_2_(0.05), S_2_(0.05) };
const double Init_Vel_Var[3] = { S_2_(0.05), S_2_(0.05), S_2_(0.05) };
const double Init_Att_Var[3] = { S_2_(1.0*D2R), S_2_(1.0*D2R), S_2_(0.1*D2R) };

#define MATRIX_INVERSION_ERROR 1

typedef double Vec3[3];
typedef double Vec7[7];
typedef double QuatVec[4];
typedef double Mat3[9];