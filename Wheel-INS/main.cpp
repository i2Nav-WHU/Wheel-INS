#include "NavFunc.h"


int main()
{

	cv::FileStorage fs("C:\\Users\\10401\\Desktop\\Wheel-INS OS\\Data\\Test2\\config.yaml", cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "Can not open config file!\n" << std::endl;
		system("pause");
		return 0;
	} 

	One_Sys_Main(fs);

	return 1;
}

