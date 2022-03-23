#include  <fstream>
#include  <iostream>
#include  <iomanip>
#include  <direct.h>
#include  <io.h>
#include  "NavFunc.h"


int Read_oneEpoch(FILE *fp, double da[])
{
	int rc;
	if ((rc = fread(&da[0], sizeof(double), DaColNum, fp)) != 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}		
}


void Initial_Nav(double stime, double cur_meas[], IMU_ConstPara *imu, S_NavState *nav)
{
	nav->T_Cur = cur_meas[0];
	if (nav->T_Cur < stime + AlignLen)
	{
		if (nav->InitSum == 0)
		{
			zero_array(6, nav->MemoInitDa);
		}
		for (int i = 0; i < 6; i++)
		{
			nav->MemoInitDa[i] += cur_meas[i + 1];
		}
		nav->InitSum++;
	}
	else
	{
		nav->InitFlag = 1;
		for (int i = 0; i < 6; i++)
		{
			nav->MemoInitDa[i] /= nav->InitSum;
		}

		//init the EKF
		zero_array(nStates, &nav->X[0]);
		zero_array(nStates*nStates, &nav->P[0][0]);
		zero_array(nStates*nStates, &nav->Q[0][0]);
		
		nav->Q[3][3] = imu->VRW_2;
		nav->Q[4][4] = imu->VRW_2;
		nav->Q[5][5] = imu->VRW_2;
		nav->Q[6][6] = imu->ARW_2;
		nav->Q[7][7] = imu->ARW_2;
		nav->Q[8][8] = imu->ARW_2;
		nav->Q[9][9] = 2.0 * imu->BgVar / imu->Tao_t;
		nav->Q[10][10] = 2.0 * imu->BgVar / imu->Tao_t;
		nav->Q[11][11] = 2.0 * imu->BgVar / imu->Tao_t;
		nav->Q[12][12] = 2.0 * imu->BaVar / imu->Tao_t;
		nav->Q[13][13] = 2.0 * imu->BaVar / imu->Tao_t;
		nav->Q[14][14] = 2.0 * imu->BaVar / imu->Tao_t;
		nav->Q[15][15] = 2.0 * imu->SgVar / imu->Tao_t;
		nav->Q[16][16] = 2.0 * imu->SgVar / imu->Tao_t;
		nav->Q[17][17] = 2.0 * imu->SgVar / imu->Tao_t;
		nav->Q[18][18] = 2.0 * imu->SaVar / imu->Tao_t;
		nav->Q[19][19] = 2.0 * imu->SaVar / imu->Tao_t;
		nav->Q[20][20] = 2.0 * imu->SaVar / imu->Tao_t;

		nav->P[0][0] = Init_Pos_Var[0];
		nav->P[1][1] = Init_Pos_Var[1];
		nav->P[2][2] = Init_Pos_Var[2];
		nav->P[3][3] = Init_Vel_Var[0];
		nav->P[4][4] = Init_Vel_Var[1];
		nav->P[5][5] = Init_Vel_Var[2];
		nav->P[6][6] = Init_Att_Var[0];
		nav->P[7][7] = Init_Att_Var[1];
		nav->P[8][8] = Init_Att_Var[2];
		nav->P[9][9] = imu->BgVar;
		nav->P[10][10] = imu->BgVar;
		nav->P[11][11] = imu->BgVar;
		nav->P[12][12] = imu->BaVar;
		nav->P[13][13] = imu->BaVar;
		nav->P[14][14] = imu->BaVar;
		nav->P[15][15] = imu->SgVar;
		nav->P[16][16] = imu->SgVar;
		nav->P[17][17] = imu->SgVar;
		nav->P[18][18] = imu->SaVar;
		nav->P[19][19] = imu->SaVar;
		nav->P[20][20] = imu->SaVar;
	
		
		nav->Pos[0] = set_ini_pos[0];
		nav->Pos[1] = set_ini_pos[1];
		nav->Pos[2] = set_ini_pos[2];

		nav->Vel[0] = set_ini_vel[0];
		nav->Vel[1] = set_ini_vel[1];
		nav->Vel[2] = set_ini_vel[2];

		nav->Att[0] = atan2(-nav->MemoInitDa[4], -nav->MemoInitDa[5]);
		nav->Att[1] = atan2(nav->MemoInitDa[3], sqrt(nav->MemoInitDa[4] * nav->MemoInitDa[4] + nav->MemoInitDa[5] * nav->MemoInitDa[5]));
		nav->Att[2] = imu->SetIniHeading;

		euler2rotation(nav->C_bn, nav->Att);
		euler2quat(nav->Q_bn, nav->Att);
		
		zero_array(9, nav->C_vn);

		zero_array(3, &nav->Bg[0]);
		zero_array(3, &nav->Ba[0]);
		zero_array(3, &nav->Sg[0]);
		zero_array(3, &nav->Sa[0]);	

		//set initial gyro bias
		nav->Bg[0] = nav->MemoInitDa[0];
		nav->Bg[1] = nav->MemoInitDa[1];
		nav->Bg[2] = nav->MemoInitDa[2];

		nav->T_Odo = cur_meas[0];

		zero_array(3, &nav->Pre_PreMeas[0]);
		zero_array(6, &nav->PreMeas[0]);

		nav->SysVodo = 0;
		nav->FilterUpdate = 0;	
		nav->Gx_comped = 0.0;
		nav->vehiclepitch = 0.0;
	
	}	
}


void Bias_Compensate(double cur_meas[], IMU_ConstPara *imu, S_NavState *nav)
{
	cur_meas[1] = (1 - nav->Sg[0])*(cur_meas[1] - nav->Bg[0]);
	cur_meas[2] = (1 - nav->Sg[1])*(cur_meas[2] - nav->Bg[1]);
	cur_meas[3] = (1 - nav->Sg[2])*(cur_meas[3] - nav->Bg[2]);

	cur_meas[4] = (1 - nav->Sa[0])*(cur_meas[4] - nav->Ba[0]);
	cur_meas[5] = (1 - nav->Sa[1])*(cur_meas[5] - nav->Ba[1]);
	cur_meas[6] = (1 - nav->Sa[2])*(cur_meas[6] - nav->Ba[2]);
}

//The INS Mechanization algorithm adopted in this system is a simplified version. Because we focused on MEMS IMU and local robot positioning applications, we ignored the earth rotation, the change of naviagtion frame and gravity, which are important for high-end IMU and large-scale applications.
void INS_Mech(double pre_meas[], double cur_meas[], S_NavState *nav) 
{

	double dt = nav->dt;
	
	Vec3 acc_dt, an_hat, zeta, gyros_dt;
	QuatVec quat_tmp;
	double cos_v, sin_v;

	
	Vec3 Gzeta, Gzeta1, Gzeta2;
	
	gyros_dt[0] = 0.5 * (pre_meas[1] + cur_meas[1]) * dt;
	gyros_dt[1] = 0.5 * (pre_meas[2] + cur_meas[2]) * dt;
	gyros_dt[2] = 0.5 * (pre_meas[3] + cur_meas[3]) * dt;

	
	CrossProduct(nav->PreMeas, gyros_dt, Gzeta);
	
	//second-order
	Gzeta[0] = gyros_dt[0] + Gzeta[0] / 12.0;
	Gzeta[1] = gyros_dt[1] + Gzeta[1] / 12.0;
	Gzeta[2] = gyros_dt[2] + Gzeta[2] / 12.0;

	//In the following comments, delta_ means the error of a variable, while Delta_ means the increment of a variable in a certain time interval.
	double v = vec_norm(3, Gzeta);
	if (v >1e-12)
	{
		cos_v = cos(v / 2.0);
		sin_v = (sin(v / 2.0) / v);
		//R[delta_q]*Q_bn (Sola,P7) delta_q = Q_bk_bk-1
		quat_tmp[0] = cos_v*nav->Q_bn[0] + sin_v*(Gzeta[2] * nav->Q_bn[1] - Gzeta[1] * nav->Q_bn[2] + Gzeta[0] * nav->Q_bn[3]);
		quat_tmp[1] = cos_v*nav->Q_bn[1] + sin_v*(-Gzeta[2] * nav->Q_bn[0] + Gzeta[0] * nav->Q_bn[2] + Gzeta[1] * nav->Q_bn[3]);
		quat_tmp[2] = cos_v*nav->Q_bn[2] + sin_v*(Gzeta[1] * nav->Q_bn[0] - Gzeta[0] * nav->Q_bn[1] + Gzeta[2] * nav->Q_bn[3]);
		quat_tmp[3] = cos_v*nav->Q_bn[3] + sin_v*(-Gzeta[0] * nav->Q_bn[0] - Gzeta[1] * nav->Q_bn[1] - Gzeta[2] * nav->Q_bn[2]);
																																	  
		v = vec_norm(4, quat_tmp);
		nav->Q_bn[0] = quat_tmp[0] / v;
		nav->Q_bn[1] = quat_tmp[1] / v;
		nav->Q_bn[2] = quat_tmp[2] / v;
		nav->Q_bn[3] = quat_tmp[3] / v;

		
		quat2rotation(nav->C_bn, nav->Q_bn);
		rotation2euler(nav->Att, nav->C_bn);
	}

	//Delta_v_fk_b
	acc_dt[0] = 0.5 * (pre_meas[4] + cur_meas[4]) * dt;
	acc_dt[1] = 0.5 * (pre_meas[5] + cur_meas[5]) * dt;
	acc_dt[2] = 0.5 * (pre_meas[6] + cur_meas[6]) * dt;
	//Delta_v_fk_b + (Delta_theta_k (cross product) Delta_v_fk_b)/2
	zeta[0] = acc_dt[0] + (-gyros_dt[2] * acc_dt[1] + gyros_dt[1] * acc_dt[2]) / 2.0;
	zeta[1] = acc_dt[1] + (gyros_dt[2] * acc_dt[0] - gyros_dt[0] * acc_dt[2]) / 2.0;
	zeta[2] = acc_dt[2] + (-gyros_dt[1] * acc_dt[0] + gyros_dt[0] * acc_dt[1]) / 2.0;
	//Delta_v_fk_b(k-1)= Delta_v_fk_b + (Delta_theta_k (cross product) Delta_v_fk_b)/2 + (Delta_theta_k-1 (cross product) Delta_v_fk_b + Delta_v_fk_b (cross product) Delta_theta_k)/12
	zeta[0] += (-nav->PreMeas[2] * acc_dt[1] + nav->PreMeas[1] * acc_dt[2] - nav->PreMeas[5] * gyros_dt[1] + nav->PreMeas[4] * gyros_dt[2]) / 12.0;
	zeta[1] += (nav->PreMeas[2] * acc_dt[0] - nav->PreMeas[0] * acc_dt[2] + nav->PreMeas[5] * gyros_dt[0] - nav->PreMeas[3] * gyros_dt[2]) / 12.0;
	zeta[2] += (-nav->PreMeas[1] * acc_dt[0] + nav->PreMeas[0] * acc_dt[1] - nav->PreMeas[4] * gyros_dt[0] + nav->PreMeas[3] * gyros_dt[1]) / 12.0;

	Mat3 Half_gros_dt;
	Half_gros_dt[0] = 1.0; Half_gros_dt[1] = 0.5*gyros_dt[2]; Half_gros_dt[2] = -0.5*gyros_dt[1];
	Half_gros_dt[3] = -0.5*gyros_dt[2]; Half_gros_dt[4] = 1.0; Half_gros_dt[5] = 0.5*gyros_dt[0];
	Half_gros_dt[6] = 0.5*gyros_dt[1]; Half_gros_dt[7] = -0.5*gyros_dt[0]; Half_gros_dt[8] = 1.0;
	Mat3 Half_Cbn;
	MultiplyMatrix(nav->C_bn, Half_gros_dt, 3, 3, 3, Half_Cbn); //Cbn*[I - (0.5*skewmatrix(Delta_theta_k))]

	//Delta_v_nk = C_b(k-1)n * Delta_v_b(k-1) + g_n * dt
	an_hat[0] = Half_Cbn[0] * zeta[0] + Half_Cbn[1] * zeta[1] + Half_Cbn[2] * zeta[2];
	an_hat[1] = Half_Cbn[3] * zeta[0] + Half_Cbn[4] * zeta[1] + Half_Cbn[5] * zeta[2];
	an_hat[2] = Half_Cbn[6] * zeta[0] + Half_Cbn[7] * zeta[1] + Half_Cbn[8] * zeta[2] + NormG * dt;

	// P_nk = P_nk-1 + (v_nk-1 + Delta_v_nk/2)*dt
	nav->Pos[0] = nav->Pos[0] + nav->Vel[0] * dt + 0.5*an_hat[0] * dt;
	nav->Pos[1] = nav->Pos[1] + nav->Vel[1] * dt + 0.5*an_hat[1] * dt;
	nav->Pos[2] = nav->Pos[2] + nav->Vel[2] * dt + 0.5*an_hat[2] * dt;

	nav->Vel[0] = nav->Vel[0] + an_hat[0];
	nav->Vel[1] = nav->Vel[1] + an_hat[1];
	nav->Vel[2] = nav->Vel[2] + an_hat[2];

	
	nav->Pre_PreMeas[0] = nav->PreMeas[0];
	nav->Pre_PreMeas[1] = nav->PreMeas[1];
	nav->Pre_PreMeas[2] = nav->PreMeas[2];

	nav->PreMeas[0] = gyros_dt[0];
	nav->PreMeas[1] = gyros_dt[1];
	nav->PreMeas[2] = gyros_dt[2];

	nav->PreMeas[3] = acc_dt[0];
	nav->PreMeas[4] = acc_dt[1];
	nav->PreMeas[5] = acc_dt[2];
}


void EKF_Predict(double cur_meas[], IMU_ConstPara *imu, S_NavState *nav) 
{
	double dt = nav->dt;
	double Pp[nStates][nStates];
	int i = 0, j = 0;
	for (i = 0; i < nStates; i++)
	{
		for (j = 0; j < nStates; j++)
		{
			Pp[i][j] = nav->P[i][j];
		}
	}
	Vec3 f_n; //f_n = (C_bn*f_b)
	f_n[0] = nav->C_bn[0] * cur_meas[4] + nav->C_bn[1] * cur_meas[5] + nav->C_bn[2] * cur_meas[6];
	f_n[1] = nav->C_bn[3] * cur_meas[4] + nav->C_bn[4] * cur_meas[5] + nav->C_bn[5] * cur_meas[6];
	f_n[2] = nav->C_bn[6] * cur_meas[4] + nav->C_bn[7] * cur_meas[5] + nav->C_bn[8] * cur_meas[6];

	double PHI[nStates][nStates] = { 0.0 };
	double PHI_T[nStates][nStates] = { 0.0 };
	double PHI_P[nStates][nStates] = { 0.0 };
	double Q_w[nStates][nStates] = { 0.0 };
	double tem_matrix[nStates][nStates] = { 0.0 };

	//set the value of PHI=I+F*dt
	if (nStates >= 9)
	{
		for (i = 0; i < 9; i++)
		{
			PHI[i][i] = 1.0;
		}
		// I
		PHI[0][3] = dt;
		PHI[1][4] = dt;
		PHI[2][5] = dt;

		// dt*skewmatrix(C_bn*f_b)
		                            PHI[3][7] = -f_n[2] * dt;		PHI[3][8] = f_n[1] * dt;
		PHI[4][6] = f_n[2] * dt;                      				PHI[4][8] = -f_n[0] * dt;
		PHI[5][6] = -f_n[1] * dt;	PHI[5][7] = f_n[0] * dt;

		for (i = 9; i < 15; i++)
		{
			PHI[i][i] = exp(-dt / imu->Tao_t);//exp(F(t)dt)
		}
		// C_bn
		PHI[3][12] = nav->C_bn[0] * dt;	PHI[3][13] = nav->C_bn[1] * dt;	PHI[3][14] = nav->C_bn[2] * dt;
		PHI[4][12] = nav->C_bn[3] * dt;	PHI[4][13] = nav->C_bn[4] * dt;	PHI[4][14] = nav->C_bn[5] * dt;
		PHI[5][12] = nav->C_bn[6] * dt;	PHI[5][13] = nav->C_bn[7] * dt;	PHI[5][14] = nav->C_bn[8] * dt;

		// -C_bn
		PHI[6][9] = -nav->C_bn[0] * dt;	PHI[6][10] = -nav->C_bn[1] * dt;	PHI[6][11] = -nav->C_bn[2] * dt;
		PHI[7][9] = -nav->C_bn[3] * dt;	PHI[7][10] = -nav->C_bn[4] * dt;	PHI[7][11] = -nav->C_bn[5] * dt;
		PHI[8][9] = -nav->C_bn[6] * dt;	PHI[8][10] = -nav->C_bn[7] * dt;	PHI[8][11] = -nav->C_bn[8] * dt;

		for (i = 15; i < 21; i++)
		{
			PHI[i][i] = exp(-dt / imu->Tao_t);//exp(F(t)dt)
		}
		// C_bn*diag(fb)
		PHI[3][18] = nav->C_bn[0] * cur_meas[4] * dt;		PHI[3][19] = nav->C_bn[1] * cur_meas[5] * dt;		PHI[3][20] = nav->C_bn[2] * cur_meas[6] * dt;
		PHI[4][18] = nav->C_bn[3] * cur_meas[4] * dt;		PHI[4][19] = nav->C_bn[4] * cur_meas[5] * dt;		PHI[4][20] = nav->C_bn[5] * cur_meas[6] * dt;
		PHI[5][18] = nav->C_bn[6] * cur_meas[4] * dt;		PHI[5][19] = nav->C_bn[7] * cur_meas[5] * dt;		PHI[5][20] = nav->C_bn[8] * cur_meas[6] * dt;
		// -C_bn*diag(wibb)
		PHI[6][15] = -nav->C_bn[0] * cur_meas[1] * dt;		PHI[6][16] = -nav->C_bn[1] * cur_meas[2] * dt;		PHI[6][17] = -nav->C_bn[2] * cur_meas[3] * dt;
		PHI[7][15] = -nav->C_bn[3] * cur_meas[1] * dt;		PHI[7][16] = -nav->C_bn[4] * cur_meas[2] * dt;		PHI[7][17] = -nav->C_bn[5] * cur_meas[3] * dt;
		PHI[8][15] = -nav->C_bn[6] * cur_meas[1] * dt;		PHI[8][16] = -nav->C_bn[7] * cur_meas[2] * dt;		PHI[8][17] = -nav->C_bn[8] * cur_meas[3] * dt;

	}

	TransposeMatrix(&PHI[0][0], nStates, nStates, &PHI_T[0][0]);
	MultiplyMatrix(&PHI[0][0], &Pp[0][0], nStates, nStates, nStates, &PHI_P[0][0]);  //PHI*P
	MultiplyMatrix(&PHI_P[0][0], &PHI_T[0][0], nStates, nStates, nStates, &Pp[0][0]);   //PHI*P*PHI_T
	MultiplyMatrix(&PHI[0][0], &nav->Q[0][0], nStates, nStates, nStates, &tem_matrix[0][0]);  //PHI*Q_
	MultiplyMatrix(&nav->Q[0][0], &PHI_T[0][0], nStates, nStates, nStates, &Q_w[0][0]);   // Q_*PHI_T

	AddMatrix(&tem_matrix[0][0], &Q_w[0][0], nStates*nStates, &Q_w[0][0]);    // PHI*Q_+Q_*PHI_T
	MultiplyMatrixWithReal(0.5*dt, &Q_w[0][0], nStates*nStates, &Q_w[0][0]);  //Q_w=0.5*(PHI*Q_+Q_*PHI_T)*dt
	AddMatrix(&Pp[0][0], &Q_w[0][0], nStates*nStates, &Pp[0][0]);  //P=PHI*P*PHI_T+Q_w

	for (i = 0; i < nStates; i++)
	{
		for (j = 0; j < nStates; j++)
		{
			nav->P[i][j] = Pp[i][j];
		}
	}
}

//get velocity by gyro data and the wheel radius
void getVel(double bufRawDa[][DaColNum], IMU_ConstPara *imu, S_NavState *nav)
{
	double Gx = 0.0;	 
	double C_vn_angle[3] = { 0.0 };

	for (int i = -Half_GmeanLen; i <= Half_GmeanLen; i++)
	{
		Gx += bufRawDa[HisLen + i][1];
	}
	Gx /= GmeanLen;

	if (imu->IfcompV)//Compensate the gyro errors for Vv or not
	{
		nav->Gx_comped = (1 - nav->Sg[0])*(Gx - nav->Bg[0]);
		nav->SysVodo = - imu->Wheel_Radius * nav->Gx_comped;
	}
	else
	{
		nav->SysVodo = -imu->Wheel_Radius * (Gx);
	}

	if (imu->MountPos != 0)
	{
		C_vn_angle[2] = nav->Att[2] - PI / 2.0;
	}
	
	Mat3 C_nv;
	euler2rotation(nav->C_vn, C_vn_angle);
	TransposeMatrix(nav->C_vn, 3, 3, C_nv);
	MultiplyMatrix(C_nv,nav->C_bn, 3, 3, 3, nav->C_bv);

}

void States_Feedback(IMU_ConstPara *imu, S_NavState *nav)
{
	nav->Pos[0] = nav->Pos[0] - nav->X[0];
	nav->Pos[1] = nav->Pos[1] - nav->X[1];
	nav->Pos[2] = nav->Pos[2] - nav->X[2];

	nav->Vel[0] -= nav->X[3];
	nav->Vel[1] -= nav->X[4];
	nav->Vel[2] -= nav->X[5];

	double nPhi[3] = { 0.0 };
	double Qx[4] = { 0.0 };
	double Phi[3] = { nav->X[6],  nav->X[7],  nav->X[8] };
	double Phi_mag = sqrt(Phi[0] * Phi[0] + Phi[1] * Phi[1] + Phi[2] * Phi[2]);
	double n = sin(0.5*Phi_mag) / Phi_mag;
	MultiplyMatrixWithReal(n, Phi, 3, nPhi);
	Qx[3] = cos(0.5*Phi_mag);
	Qx[0] = nPhi[0];
	Qx[1] = nPhi[1];
	Qx[2] = nPhi[2];
	double Qx_inv[4], Qbn_new[4], deltaCbn[9];
	double Phi_skew[9] = { 0, -Phi[2], Phi[1], Phi[2], 0, -Phi[0], -Phi[1], Phi[0], 0 };

	Quat_norm(Qx);
	CopyMatrix(4, 1, Qx_inv, Qx);
	quat2rotation(deltaCbn, Qx_inv);

	QuatMul(Qx_inv, nav->Q_bn, Qbn_new);
	Quat_norm(Qbn_new);
	CopyMatrix(3, 3, nav->Q_bn, Qbn_new);
	quat2euler(nav->Att, Qbn_new);
	quat2rotation(nav->C_bn, Qbn_new);

	
	nav->Bg[0] += nav->X[9];
	nav->Bg[1] += nav->X[10];
	nav->Bg[2] += nav->X[11];

	nav->Ba[0] += nav->X[12];
	nav->Ba[1] += nav->X[13];
	nav->Ba[2] += nav->X[14];



	nav->Sg[0] += nav->X[15];
	nav->Sg[1] += nav->X[16];
	nav->Sg[2] += nav->X[17];

	nav->Sa[0] += nav->X[18];
	nav->Sa[1] += nav->X[19];
	nav->Sa[2] += nav->X[20];

	zero_array(nStates, nav->X);

	nav->FilterUpdate = 0;
}



void EKF_Update(S_NavState *nav, int nMeas, double inno[], double H[], double R[])
{
	
	int mLen_1 = nStates * nMeas;
	int mLen_2 = nStates * nStates;
	int mLen_3 = nMeas * nMeas;
	double * Pp = (double *)malloc(mLen_2 * sizeof(double));
	int i = 0, j = 0;
	for (i = 0; i < nStates; i++)
	{
		for (j = 0; j < nStates; j++)
		{
			Pp[i*nStates + j] = nav->P[i][j];
		}
		
	}
	
	double * tem_matrix1 = (double *)malloc(mLen_1 * sizeof(double));
	double * tem_matrix2 = (double *)malloc(mLen_1 * sizeof(double));
	double * tem_matrix3 = (double *)malloc(mLen_1 * sizeof(double));

	TransposeMatrix(H, nMeas, nStates, tem_matrix1);//Ht
	MultiplyMatrix(Pp, tem_matrix1, nStates, nStates, nMeas, tem_matrix2);  //P*Ht

	double * HPHTR = (double *)malloc(mLen_3 * sizeof(double));
	MultiplyMatrix(H, tem_matrix2, nMeas, nStates, nMeas, HPHTR);  //H*P*Ht
	AddMatrix(HPHTR, R, mLen_3, HPHTR);//H*P*Ht+R	
	double * U = (double *)malloc(mLen_3 * sizeof(double));
	CopyMatrix(nMeas, nMeas, U, HPHTR);
	int pdf = Cholesky(U, nMeas);
	if (pdf > 0)
	{
		double * invHPHTR = (double *)malloc(mLen_3 * sizeof(double));
		MatrixInv(nMeas, HPHTR, invHPHTR);//inv(H*P*Ht+R)	
		double * K = (double *)malloc(mLen_1 * sizeof(double));
		MultiplyMatrix(tem_matrix2, invHPHTR, nStates, nMeas, nMeas, K);
		
		double * dx = (double *)malloc(nStates * sizeof(double));
		MultiplyMatrix(K, inno, nStates, nMeas, 1, dx);//dx

		
		double * I = (double *)malloc(mLen_2 * sizeof(double));
		UnitMatrix(nStates, I);//I
		double * IKH = (double *)malloc(mLen_2 * sizeof(double));
		double * IKHP = (double *)malloc(mLen_2 * sizeof(double));
		double * tem_matrix4 = (double *)malloc(mLen_2 * sizeof(double));
		MultiplyMatrix(K, H, nStates, nMeas, nStates, IKH);  // K*H
		SubtractMatrix(I, IKH, mLen_2, IKH);  // I-K*H
		MultiplyMatrix(IKH, Pp, nStates, nStates, nStates, IKHP); //(I-K*H)*P
		TransposeMatrix(IKH, nStates, nStates, tem_matrix4);//(I-K*H)_
		MultiplyMatrix(IKHP, tem_matrix4, nStates, nStates, nStates, Pp);  //(I-K*H)*P*(I-K*H)_
		MultiplyMatrix(K, R, nStates, nMeas, nMeas, tem_matrix1);  //K*R
		TransposeMatrix(K, nStates, nMeas, tem_matrix2);//KT
		MultiplyMatrix(tem_matrix1, tem_matrix2, nStates, nMeas, nStates, tem_matrix4); //K*R*K_T
		AddMatrix(Pp, tem_matrix4, mLen_2, Pp);  //P=(I+K*H)*P*(I+K*H)_+K*R*K_T
		for (i = 0; i < nStates; i++)
		{
			for (j = 0; j < nStates; j++)
			{
				nav->P[i][j] = Pp[i*nStates + j];
			}

		}
		
		nav->FilterUpdate += nMeas;

		free(I);
		free(IKH);
		free(IKHP);
		free(tem_matrix4);
		
		AddMatrix(dx, nav->X, nStates, nav->X);

		free(invHPHTR);
		free(K);
		free(dx);

	}

	free(Pp);
	free(tem_matrix1);
	free(tem_matrix2);
	free(tem_matrix3);
	free(U);
	free(HPHTR);
}


void WriteFile(S_NavState Nav, IMU_ConstPara IMU, FILE *fp_out, FILE *fp_OutCov)
{
	
	if (fp_out != NULL)
	{
		fwrite(&Nav.T_Cur, sizeof(double), 1, fp_out);
		fwrite(Nav.Pos, sizeof(double), 3, fp_out);
		fwrite(Nav.Vel, sizeof(double), 3, fp_out);

		double euler[3] = { 0.0 };
		euler[0] = Nav.Att[0];
		euler[1] = Nav.Att[1];
		euler[2] = Nav.Att[2];

		if (IMU.MountPos != 0)
		{
			euler[2] -= PI / 2;//Transform to vehicle heading
			ConvertPI(&euler[2]);
		}
		fwrite(euler, sizeof(double), 3, fp_out);

		fwrite(Nav.Bg, sizeof(double), 3, fp_out);
		fwrite(Nav.Ba, sizeof(double), 3, fp_out);	
		fwrite(Nav.Sg, sizeof(double), 3, fp_out);
		fwrite(Nav.Sa, sizeof(double), 3, fp_out);
		fwrite(&Nav.SysVodo, sizeof(double), 1, fp_out);
		
	}

	if (fp_OutCov != NULL)
	{
		fwrite(&Nav.T_Cur, sizeof(double), 1, fp_OutCov);
		for (int i = 0; i < nStates; i++)
		{
			double Out_P = sqrt(Nav.P[i][i]);
			fwrite(&Out_P, sizeof(double), 1, fp_OutCov);
		}
	}
	
}

void OpenFile(IMU_ConstPara imu, FILE **fp_IMU, FILE **fp_out, FILE **fp_OutCov, int sysnum)
{
	char* FilePath = const_cast<char*>(imu.filepath.c_str());
	char* FileName = const_cast<char*>(imu.filename.c_str());

	//open IMU file
	char Fname_IMU[300] = { "" };
	strcpy_s(Fname_IMU, FilePath);
	strcat_s(Fname_IMU, FileName);
	fopen_s(fp_IMU, Fname_IMU, "rb");

	//create results file
	char optfolder[300] = { "" };
	strcpy_s(optfolder, FilePath);
	strcat_s(optfolder, "output\\");
	if (_access(optfolder, 0) == -1)	
		_mkdir(optfolder);				
	
	char NAVfile[300] = { "" };
	char Covfile[300] = { "" };
	char DRfile[300] = { "" };

	strcpy_s(NAVfile, optfolder);
	strcpy_s(Covfile, optfolder);

	std::string s = std::to_string(nStates);
	strcat_s(NAVfile, s.c_str());
	strcat_s(Covfile, s.c_str());
	
	if (sysnum == 1)
		strcat_s(NAVfile, "-S-NAV.bin");

	if (sysnum == 1)
		strcat_s(Covfile, "-S-Cov.bin");

	fopen_s(fp_out, NAVfile, "wb");
	fopen_s(fp_OutCov, Covfile, "wb");

}

void DataBuff(int* RawDa_Sum, double* NowMeas, double BufRawDa[][DaColNum])
{
	if (*RawDa_Sum < BufLen)
	{
		for (int i = 0; i < DaColNum; i++)
		{
			BufRawDa[(*RawDa_Sum)][i] = NowMeas[i];
		}
		(*RawDa_Sum)++;
	}
	else
	{
		for (int i = 0; i < BufLen - 1; i++)
		{
			for (int j = 0; j < DaColNum; j++)
			{
				BufRawDa[i][j] = BufRawDa[i + 1][j];
			}
		}
		for (int j = 0; j < DaColNum; j++)
		{
			BufRawDa[BufLen - 1][j] = NowMeas[j];
		}
	}
}

int charfind(char *buf, char *sub)
{

	int len = strlen(buf);                 
	char *p = (char *)malloc(len * 2 + 1); 

	memset(p, 0x00, len * 2 + 1); 
	
	strcpy(p, buf);               
	strcat(p, buf);              

	if (strstr(p, sub) == NULL)
	{ 
		return 0;
	}
	else
	{
		return 1;
	}
}

void NHC_Odo_Update(IMU_ConstPara *imu, S_NavState *nav)
{
	if (imu->Odo_IfUse == 1)
	{
		if ((nav->T_Cur - nav->T_Odo) > (imu->Odo_dt - Half_dt))
		{
			double V_vel[3] = { 0.0 };
			double C_nv[9] = { 0.0 };

			TransposeMatrix(nav->C_vn, 3, 3, C_nv);//current navigation result
			MultiplyMatrix(C_nv, nav->Vel, 3, 3, 1, V_vel);//C_bv*C_nb*v_IMUn

			Vec3 tmp2;
			rotation2euler(tmp2, nav->C_vn);


			double vel_skew[9] = { 0, -nav->Vel[2], nav->Vel[1], nav->Vel[2], 0, -nav->Vel[0], -nav->Vel[1], nav->Vel[0], 0 };
			double V_velskew[9] = { 0.0 };
			MultiplyMatrix(C_nv, vel_skew, 3, 3, 3, V_velskew);//C_bv*C_nb*skewmatrix(V_IMUn)

			double CurGyros[3] = { nav->PreMeas[0] / nav->dt,nav->PreMeas[1] / nav->dt,nav->PreMeas[2] / nav->dt };//It was the increment when saved
			double wib_skew[9] = { 0, -CurGyros[2], CurGyros[1], CurGyros[2], 0, -CurGyros[0], -CurGyros[1], CurGyros[0], 0 };


			double Cbvwskew[9] = { 0.0 };
			double cLeverArm_Skew[9] = { 0.0 };
			double vLeverArm[3] = { 0.0 };
			MultiplyMatrix(nav->C_bv, wib_skew, 3, 3, 3, Cbvwskew);//C_bv*skewmatrix(w_ibb)
			MultiplyMatrix(Cbvwskew, imu->WheelLeverArm, 3, 3, 1, vLeverArm);//C_bv*skewmatrix(w_ibb)*LeverArm

			double Cbnwskew[9] = { 0.0 };
			double vnLeverArm[3] = { 0.0 };

			MultiplyMatrix(nav->C_bn, wib_skew, 3, 3, 3, Cbnwskew);
			MultiplyMatrix(Cbnwskew, imu->WheelLeverArm, 3, 3, 1, vnLeverArm);//C_bn*skewmatrix(w_ibb)*LeverArm
			
			double vnLeverArm_skew[9] = { 0, -vnLeverArm[2], vnLeverArm[1], vnLeverArm[2], 0, -vnLeverArm[0], -vnLeverArm[1], vnLeverArm[0], 0 };
			double vVLeverArm_skew[9] = { 0.0 };
			MultiplyMatrix(C_nv, vnLeverArm_skew, 3, 3, 3, vVLeverArm_skew);//C_nv (cross product) skewmatrix(C_bn*skewmatrix(w_ibb)*LA)

			double LeverArm_Skew[9] = { 0, -imu->WheelLeverArm[2], imu->WheelLeverArm[1], imu->WheelLeverArm[2], 0, -imu->WheelLeverArm[0],
				-imu->WheelLeverArm[1], imu->WheelLeverArm[0], 0 };
			MultiplyMatrix(nav->C_bv, LeverArm_Skew, 3, 3, 3, cLeverArm_Skew);//C_bv*skewmatrix(LeverArm)

			double Cbv_LAskew_wibb[9] = { 0.0 };
			double diagwibb[9] = { CurGyros[0], 0.0, 0.0, 0.0, CurGyros[1], 0.0, 0.0, 0.0, CurGyros[2] };
			MultiplyMatrix(cLeverArm_Skew, diagwibb, 3, 3, 3, Cbv_LAskew_wibb);

			double Hv[3][nStates] = { 0.0 };
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					Hv[i][j + 3] = C_nv[i * 3 + j];
					Hv[i][j + 6] = vVLeverArm_skew[i * 3 + j];
				    Hv[i][j + 9] = -cLeverArm_Skew[i * 3 + j];
					Hv[i][j + 15] = -Cbv_LAskew_wibb[i * 3 + j];
				}
			}
			
			Hv[0][8] = -V_velskew[2];
			Hv[1][8] = -V_velskew[5];
			Hv[2][8] = -V_velskew[8];
			
			if (imu->IfcompV)
			{
				Hv[0][9] += -imu->Wheel_Radius;
				Hv[1][10] += -imu->Wheel_Radius;
				Hv[2][11] += -imu->Wheel_Radius;

				Hv[0][15] += -imu->Wheel_Radius * CurGyros[0];
				Hv[1][16] += -imu->Wheel_Radius * CurGyros[1];
				Hv[2][17] += -imu->Wheel_Radius * CurGyros[2];
			}
			

			double Zv[3] = { 0.0 };
			Zv[0] = V_vel[0] + vLeverArm[0] - nav->SysVodo;
			Zv[1] = V_vel[1] + vLeverArm[1];
			Zv[2] = V_vel[2] + vLeverArm[2];


			double Rv[9] = { 0.0 };
			Rv[0] = imu->Odo_Var[0];
			Rv[4] = imu->Odo_Var[1];
			Rv[8] = imu->Odo_Var[2];

			//inno
			double inno[3] = { 0.0 };
			double Hv_x[3] = { 0.0 };
			MultiplyMatrix(&Hv[0][0], nav->X, 3, nStates, 1, Hv_x);
			SubtractMatrix(Zv, Hv_x, 3, inno);

			EKF_Update(nav, 3, inno, &Hv[0][0], Rv);

			nav->T_Odo = nav->T_Cur;
		}
	}
}
