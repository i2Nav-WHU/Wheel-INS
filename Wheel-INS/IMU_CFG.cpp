#include "NavFunc.h"


void getIMUparas(IMU_ConstPara * IMU, cv::FileStorage fs, cv::FileNode IMU_paras)
{
	cv::FileNodeIterator it1 = IMU_paras.begin();
	cv::FileNodeIterator it = ++IMU_paras.begin();
	cv::FileNodeIterator it2 = --IMU_paras.end();
	std::vector<double> leverarm, odo_std;

	IMU->filepath = (std::string)(*it1)["Filepath"];
	IMU->filename = (std::string)(*it1)["Filename"];

	IMU->BgVar = S_2_(((double)(*it)["std_bg"]) / 3600.0*D2R);
	IMU->BaVar = S_2_(((double)(*it)["std_ba"])*1e-5);
	IMU->SgVar = S_2_(((double)(*it)["std_sg"])*1e-5);
	IMU->SaVar = S_2_(((double)(*it)["std_sa"])*1e-5);
	IMU->ARW_2 = S_2_(((double)(*it)["ARW"]) / 60.0 * D2R);
	IMU->VRW_2 = S_2_(((double)(*it)["VRW"]) / 60.0);
	IMU->Tao_t = ((double)(*it)["Tao_t"]);//Correlation time
	
	IMU->Odo_IfUse = (int)(*it)["ifODO"];
	
	IMU->IfcompV = (int)fs["ifCompVel"];

	if (IMU->MountPos != 0)
	{
		(*it2)["WheelLA"] >> leverarm;
	}
	(*it2)["ODO_std"] >> odo_std;

	if (IMU->MountPos != 0)
	{
		IMU->WheelLeverArm[0] = leverarm[0];
		IMU->WheelLeverArm[1] = leverarm[1];
		IMU->WheelLeverArm[2] = leverarm[2];
	}

	IMU->Odo_dt = (double)fs["ODO_dt"];
	IMU->Odo_Var[0] = S_2_(odo_std[0]);	
	IMU->Odo_Var[1] = S_2_(odo_std[1]);
	IMU->Odo_Var[2] = S_2_(odo_std[2]);

	IMU->SetIniHeading = ((double)(*it2)["init_heading"])*D2R;   

	IMU->Wheel_Radius = fs["Wheel_Radius"];
}

void Configure_IMU_Para(IMU_ConstPara * IMU, cv::FileStorage fs)
{
	std::string sysflag = (std::string)fs["sysflag"];
	cv::FileNode IMU_paras;

	if (strcmp(sysflag.c_str(), "r") == 0)
	{		
		IMU_paras = fs["IMU_right"];
		IMU->MountPos = 1;	
	}
	else if (strcmp(sysflag.c_str(), "l") == 0)
	{
		IMU_paras = fs["IMU_left"];
		IMU->MountPos = -1;	
	}
	else
	{
		printf("SYSFLAG ERROR��\n");
		system("pause");
		return;
	}
	getIMUparas(IMU, fs, IMU_paras);

	fs.release();	
}

