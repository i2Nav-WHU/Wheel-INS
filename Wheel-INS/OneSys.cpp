#include <iomanip>
#include "NavFunc.h"

void One_Sys_Main(cv::FileStorage fs)
{

	IMU_ConstPara IMU;
	Configure_IMU_Para(&IMU, fs);

	double sTime = (double)fs["start_t"];
	double eTime = (double)fs["end_t"];

	//IMU data
	FILE *fp_IMU = NULL;
	//Output navigation results
	FILE *fp_out = NULL;
	//Output covariance
	FILE *fp_OutCov = NULL;

	OpenFile(IMU, &fp_IMU, &fp_out, &fp_OutCov, 1);

	//IMU data buffer
	double BufRawDa[BufLen][DaColNum];

	//Init navigation struct
	S_NavState  Nav;
	Nav.InitSum = 0;
	Nav.InitFlag = 0;

	
	Vec7 NowMeas;
	Vec7 PreMeas;
	Vec7 CurMeas;

	double dt = 0.0;
	
	int RawDa_Sum = 0;
	bool initial_flag = 0;

	while (!feof(fp_IMU))
	{
		Read_oneEpoch(fp_IMU, NowMeas);
		DataBuff(&RawDa_Sum, NowMeas, BufRawDa);
		if (RawDa_Sum == BufLen)
		{
			if (Nav.InitFlag == 0)
			{
				CopyMatrix(1, DaColNum, PreMeas, BufRawDa[HisLen - 1]);
				CopyMatrix(1, DaColNum, CurMeas, BufRawDa[HisLen]);
				if (CurMeas[0] >= sTime)
				{
					Initial_Nav(sTime, CurMeas, &IMU, &Nav);				
				}				
			}
			else if (Nav.InitFlag == 1)
			{
				if (CurMeas[0] > eTime)
					break;


				CopyMatrix(1, DaColNum, PreMeas, CurMeas);
				CopyMatrix(1, DaColNum, CurMeas, BufRawDa[HisLen]);

				Nav.T_Cur = CurMeas[0];
				Nav.dt = CurMeas[0] - PreMeas[0];

				Bias_Compensate(CurMeas, &IMU, &Nav);
				
				//The INS Mechanization algorithm adopted in this system is a simplified version. Because we focused on MEMS IMU and local robot positioning applications, we ignored the earth rotation, the change of naviagtion frame and gravity, which are important for high-end IMU and large-scale applications.
				//reference: E.-H. Shin, “Estimation techniques for low-cost inertial navigation,” Ph.D. dissertation, Dept. Geomatics Eng., Univ. of Calgary, Calgary, Canada, 2005.
				INS_Mech(PreMeas, CurMeas, &Nav);

				EKF_Predict(CurMeas, &IMU, &Nav);
				
				getVel(BufRawDa, &IMU, &Nav);

				NHC_Odo_Update(&IMU, &Nav);

				if (Nav.FilterUpdate >= 1)
				{
					States_Feedback(&IMU, &Nav);
				}

				WriteFile(Nav, IMU, fp_out, fp_OutCov);
			}
		}
	}
	fclose(fp_out);
	fclose(fp_OutCov);
}

