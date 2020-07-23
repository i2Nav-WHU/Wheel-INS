#ifndef  _NAV_STATE_
#define  _NAV_STATE_

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "DataType.h"
#include "Const.h"

void One_Sys_Main(cv::FileStorage fs);

//Auxiliary functions
double vec_norm(int len, double *da);

void zero_array(int row, double *da);
int Read_oneEpoch(FILE *fp, double da[]);
void Configure_IMU_Para(IMU_ConstPara * IMU, cv::FileStorage fs);
void WriteFile(S_NavState Nav, IMU_ConstPara IMU, FILE *fp_out, FILE *fp_OutCov);
void OpenFile(IMU_ConstPara imu, FILE **fp_IMU, FILE **fp_out, FILE **fp_OutCov, int sysnum);
int charfind(char *buf, char *sub);
void DataBuff(int* RawDa_Sum, double* NowMeas, double BufRawDa[][DaColNum]);


//INS Mechanization
void ConvertPI(double* angle);//[0, 2pi] -> [-pi, pi]
void CrossProduct(Vec3 a, Vec3 b, Vec3 c);
void euler2rotation(Mat3 rotation, Vec3 euler);//euler in rad
void euler2quat(QuatVec quat, Vec3 euler);
void quat2rotation(Mat3 rotation, QuatVec quat);
void quat2euler(Vec3 euler, QuatVec quat);
void rotation2euler(Vec3 euler, Mat3 rotation);
void rotation2quat(QuatVec quat, Mat3 rotation);
void Bias_Compensate(double cur_meas[], IMU_ConstPara *imu, S_NavState *nav);
void INS_Mech(double pre_meas[], double cur_meas[], S_NavState *nav);


//EKF
void Initial_Nav(double stime, double cur_meas[], IMU_ConstPara *imu, S_NavState *nav);
void getVel(double bufRawDa[][DaColNum], IMU_ConstPara *imu, S_NavState *nav);

void States_Feedback(IMU_ConstPara *imu, S_NavState *nav);
void NHC_Odo_Update(IMU_ConstPara *imu, S_NavState *nav);
void EKF_Predict(double cur_meas[], IMU_ConstPara *imu, S_NavState *nav);
void EKF_Update(S_NavState *nav, int nMeas, double inno[], double H[], double R[]);


//Matrix operations
void AddMatrix(double matrix_a[], double matrix_b[], int matrix_length, double result_matrix[]);
void SubtractMatrix(double matrix_a[], double matrix_b[], int matrix_length, double result_matrix[]);
void MultiplyMatrixWithReal(double real_number, double matrix[], int matrix_length, double result_matrix[]);
void MultiplyMatrix(double matrix_a[], double matrix_b[], int matrix_a_row, int matrix_a_column, int matrix_b_column, double result_matrix[]);
int Cholesky(double matrix[], int matrix_row);
void TransposeMatrix(double matrix[], int matrix_row, int matrix_column, double transpose_matrix[]);
void UnitMatrix(int matrix_order, double unit_matrix[]);
void ZeroMatrix(int matrix_row, int matrix_colunm, double zero_matrix[]);
void CopyMatrix(int row, int col, double matrix_result[], double matrix_b[]);
int MatrixInv(int n, double a[], double resault[]);
void diag_matrix(int row, double value, double *da);
bool QuatMul(double Q1[], double Q2[], double result_Q[]);
void Quatinv(double Q[], double Qinv[]);
bool Quat_norm(double Qbn[]);

#endif
